//////////////////////////////////////////////////////////////////////////
//
//  Copyright (c) 2023, Image Engine Design Inc. All rights reserved.
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions are
//  met:
//
//      * Redistributions of source code must retain the above
//        copyright notice, this list of conditions and the following
//        disclaimer.
//
//      * Redistributions in binary form must reproduce the above
//        copyright notice, this list of conditions and the following
//        disclaimer in the documentation and/or other materials provided with
//        the distribution.
//
//      * Neither the name of John Haddon nor the names of
//        any other contributors to this software may be used to endorse or
//        promote products derived from this software without specific prior
//        written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
//  IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
//  THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
//  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
//  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
//  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
//  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
//  PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
//  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
//  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//////////////////////////////////////////////////////////////////////////

#include "GafferScene/MeshSubdivide.h"

// TODO check
#include <numeric>

#include "tbb/parallel_for.h"
//#include "tbb/enumerable_thread_specific.h"
//
// TODO Audit
#include <opensubdiv/far/primvarRefiner.h>
#include <opensubdiv/far/topologyDescriptor.h>
#include <opensubdiv/far/patchTableFactory.h>
#include <opensubdiv/far/stencilTableFactory.h>

#include <opensubdiv/bfr/refinerSurfaceFactory.h>
#include <opensubdiv/bfr/surface.h>
#include <opensubdiv/bfr/tessellation.h>

#include "IECore/CompoundParameter.h"
#include "IECore/DataAlgo.h"
#include "IECore/DespatchTypedData.h"

#include "IECoreScene/PrimitiveVariable.h"
#include "IECoreScene/MeshPrimitive.h"

#include "Gaffer/Context.h"

using namespace IECore;
using namespace IECoreScene;
using namespace Gaffer;
using namespace GafferScene;
using namespace Imath;
namespace OSDF = OpenSubdiv::Far;
namespace OSDB = OpenSubdiv::Bfr;

namespace {

template<typename T>
void podVectorResizeUninitialized( std::vector<T> &v, size_t s )
{
	struct TNoInit
	{
		T data;
		TNoInit() noexcept {
		}
	};

// TODO
//#ifdef NDEBUG
	reinterpret_cast< std::vector< TNoInit >* >( &v )->resize( s );
//#else
	// Actually leaving things uninitialized is great for performance, but not so great for reliably
	// tracking down bugs. In debug mode, where we don't care about performance, set everything to a
	// sentinel value that will make it obvious if something uses an uninitialized value.
	//v.resize( s, T( 777 ) );
//#endif
}

template<class F, typename... Args>
typename std::invoke_result_t<F, Data *, Args&&...> dispatchIndexedView( const PrimitiveVariable &var, F &&functor, Args&&... args )
{
	IECore::dispatch( var.data.get(),
		[&]( const auto *typedData )
		{
			using DataType = typename std::remove_cv_t< std::remove_pointer_t< decltype( typedData ) > >;
			if constexpr ( TypeTraits::IsVectorTypedData<DataType>::value )
			{
				using ElementType = typename DataType::ValueType::value_type;
				// TODO
				if constexpr ( !std::is_same_v< ElementType, bool > )
				{
					return functor( PrimitiveVariable::IndexedView<ElementType>( var ), std::forward<Args>( args )... );
				}
			}

			throw IECore::Exception( fmt::format( "Invalid primitive variable type, data is not a vector : {}", typedData->typeName() ) );
		}
	);
}

// is_specialization_of pasted from C++ standards proposal WG21 P2098R0

template< class T, template<class...> class Primary >
struct is_specialization_of : std::false_type{};

template< template<class...> class Primary, class... Args >
struct is_specialization_of< Primary<Args...>, Primary> : std::true_type{};

template< class T , template<class...> class Primary >
inline constexpr bool is_specialization_of_v = is_specialization_of<T, Primary>::value;


template< class T >
constexpr int numFloatsForType()
{
	if constexpr( std::is_arithmetic_v< T > ) return 1;
	else if constexpr( std::is_same_v< T, Imath::half > ) return 1;
	else if constexpr( is_specialization_of_v< T, Imath::Vec2 > ) return 2;
	else if constexpr( is_specialization_of_v< T, Imath::Vec3 > ) return 3;
	else if constexpr( is_specialization_of_v< T, Imath::Color3 > ) return 3;
	else if constexpr( is_specialization_of_v< T, Imath::Color4 > ) return 4;
	else if constexpr( is_specialization_of_v< T, Imath::Quat > ) return 4;
	else if constexpr( is_specialization_of_v< T, Imath::Box > ) return 2 * numFloatsForType< decltype(T::min) >();
	else if constexpr( is_specialization_of_v< T, Imath::Matrix33 > ) return 9;
	else if constexpr( is_specialization_of_v< T, Imath::Matrix44 > ) return 16;
	else if constexpr( std::is_same_v< T, std::string > || std::is_same_v< T, IECore::InternedString > )
	{
		// Trying to interpolate strings is weird enough that I guess I'm OK with just returning
		// empty strings - the user can probably figure out that this means that subdividing varying strings
		// is not supported?
		return 0;
	}
}

template< class T >
void toFloats( const T& src, float *v )
{
	if constexpr( std::is_arithmetic_v< T > )
	{
		v[0] = src;
	}
	else if constexpr( std::is_same_v< T, Imath::half > )
	{
		v[0] = src;
	}
	else if constexpr( is_specialization_of_v< T, Imath::Vec2 > )
	{
		v[0] = src.x;
		v[1] = src.y;
	}
	else if constexpr( is_specialization_of_v< T, Imath::Vec3 > )
	{
		v[0] = src.x;
		v[1] = src.y;
		v[2] = src.z;
	}
	else if constexpr( is_specialization_of_v< T, Imath::Color3 > )
	{
		v[0] = src.x;
		v[1] = src.y;
		v[2] = src.z;
	}
	else if constexpr( is_specialization_of_v< T, Imath::Color4 > )
	{
		v[0] = src.r;
		v[1] = src.g;
		v[2] = src.b;
		v[3] = src.a;
	}
	else if constexpr( is_specialization_of_v< T, Imath::Quat > )
	{
		v[0] = src.r;
		v[1] = src.v.x;
		v[2] = src.v.y;
		v[3] = src.v.z;
	}
	else if constexpr( is_specialization_of_v< T, Imath::Box > )
	{
		using VT = decltype(T::min);
		toFloats<VT>( src.min, v );
		toFloats<VT>( src.max, v + numFloatsForType<VT>() );
	}
	else if constexpr( is_specialization_of_v< T, Imath::Matrix33 > )
	{
		// TODO test
		v[0] = src[0][0];
		v[1] = src[0][1];
		v[2] = src[0][2];
		v[3] = src[1][0];
		v[4] = src[1][1];
		v[5] = src[1][2];
		v[6] = src[2][0];
		v[7] = src[2][1];
		v[8] = src[2][2];
	}
	else if constexpr( is_specialization_of_v< T, Imath::Matrix44 > )
	{
		v[0]  = src[0][0];
		v[1]  = src[0][1];
		v[2]  = src[0][2];
		v[3]  = src[0][3];
		v[4]  = src[1][0];
		v[5]  = src[1][1];
		v[6]  = src[1][2];
		v[7]  = src[1][3];
		v[8]  = src[2][0];
		v[9]  = src[2][1];
		v[10] = src[2][2];
		v[11] = src[2][3];
		v[12] = src[3][0];
		v[13] = src[3][1];
		v[14] = src[3][2];
		v[15] = src[3][3];
	}
	else if constexpr( std::is_same_v< T, std::string > || std::is_same_v< T, IECore::InternedString > )
	{
		// Ignore strings
	}
}

template< class T >
T fromFloats( float *v )
{
	if constexpr( std::is_arithmetic_v< T > ) return T( *v );
	else if constexpr( std::is_same_v< T, Imath::half > ) return T( *v );
	else if constexpr( is_specialization_of_v< T, Imath::Vec2 > ) return T( v[0], v[1] );
	else if constexpr( is_specialization_of_v< T, Imath::Vec3 > ) return T( v[0], v[1], v[2] );
	else if constexpr( is_specialization_of_v< T, Imath::Color3 > ) return T( v[0], v[1], v[2] );
	else if constexpr( is_specialization_of_v< T, Imath::Color4 > ) return T( v[0], v[1], v[2], v[3] );
	else if constexpr( is_specialization_of_v< T, Imath::Quat > ) return T( v[0], v[1], v[2], v[3] );
	else if constexpr( is_specialization_of_v< T, Imath::Box > )
	{
		using VT = decltype(T::min);
		return T( fromFloats<VT>( v ), fromFloats<VT>( v + numFloatsForType<VT>() ) );
	}
	else if constexpr( is_specialization_of_v< T, Imath::Matrix33 > )
	{
		return T( v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8] );
	}
	else if constexpr( is_specialization_of_v< T, Imath::Matrix44 > )
	{
		return T( v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8], v[9], v[10], v[11], v[12], v[13], v[14], v[15] );
	}
	else if constexpr( std::is_same_v< T, std::string > || std::is_same_v< T, IECore::InternedString > )
	{
		return T();
	}
}

struct PrimvarSetup
{
	PrimvarSetup( const std::string &name, const PrimitiveVariable &var )
		: m_name( name ), m_var( &var ), m_outIndicesWritable( nullptr )
	{
	}

	~PrimvarSetup()
	{
	}

	void initialize( int outputSize, int outputIndexSize )
	{
		// Uniform primitive variables are quite different - they never need to be interpolated, so we can
		// just reuse the input data, and only need to write new indices.
		if( m_var->interpolation != PrimitiveVariable::Uniform )
		{
			dispatch(
				m_var->data.get(),
				[&]( const auto *typedData ) -> void
				{
					using DataType = typename std::decay_t< decltype( *typedData ) >;
					if constexpr ( TypeTraits::IsVectorTypedData<DataType>::value )
					{
						typename DataType::Ptr outData = new DataType();
						podVectorResizeUninitialized( outData->writable(), outputSize );
						m_outWritable = &( outData->writable() );
						m_outData = outData;
					}
				}
			);

			IECore::setGeometricInterpretation( m_outData.get(), IECore::getGeometricInterpretation( m_var->data.get() ) );
		}

		if( outputIndexSize > 0 )
		{
			m_outIndicesData = new IntVectorData();
			podVectorResizeUninitialized( m_outIndicesData->writable(), outputIndexSize );
			m_outIndicesWritable = m_outIndicesData->writable().data();
		}
	}

	std::string m_name;
	const PrimitiveVariable *m_var;

	IECore::DataPtr m_outData;
	void* m_outWritable;
	IECore::IntVectorDataPtr m_outIndicesData;
	int* m_outIndicesWritable;
};


int setupVariables(
	const PrimitiveVariableMap &variables, bool calculateNormals,
	PrimvarSetup &posPrimvarSetup,
	std::vector< PrimvarSetup > &vertexPrimvarSetups,
	std::vector< PrimvarSetup > &uniformPrimvarSetups,
	std::vector< PrimvarSetup > &faceVaryingPrimvarSetups
)
{
	int numFaceVarying = 0;
	for( PrimitiveVariableMap::const_iterator it = variables.begin(); it != variables.end(); ++it )
	{
		if( it->first == "P" || ( calculateNormals && it->first == "N" ) )
		{
			// Don't set up variables that are handled by special cases
			continue;
		}

		if( it->second.interpolation == PrimitiveVariable::Constant )
		{
			// No need to do any setup, we just copy them across at the end.
		}
		else if( it->second.interpolation == PrimitiveVariable::FaceVarying )
		{
			faceVaryingPrimvarSetups.push_back( PrimvarSetup( it->first, it->second ) );
		}
		else if( it->second.interpolation == PrimitiveVariable::Vertex )
		{
			vertexPrimvarSetups.push_back( PrimvarSetup( it->first, it->second ) );
		}
		else if( it->second.interpolation == PrimitiveVariable::Uniform )
		{
			uniformPrimvarSetups.push_back( PrimvarSetup( it->first, it->second ) );
		}
	}

	return numFaceVarying;
}

struct ComponentOwner
{
	int face;
	int offset;
};

struct PrimvarTopology
{
	void initialize( const OSDF::TopologyLevel *meshTopology, int faceVaryingChannel = -1 )
	{
		m_mesh = meshTopology;
		m_faceVaryingChannel = faceVaryingChannel;

		m_facePointOffsets.resize( m_mesh->GetNumFaces() );
		m_vertexOwners.resize(
			faceVaryingChannel == -1 ? m_mesh->GetNumVertices() : m_mesh->GetNumFVarValues( faceVaryingChannel ),
			{ -1, -1 }
		);
		m_edgeOwners.resize( m_mesh->GetNumEdges(), { -1, -1 } );
	}

	inline void addFace( int faceIndex, const OSDB::Tessellation &tessPattern, const OSDF::ConstIndexArray &fVerts, const OSDF::ConstIndexArray &fEdges, int tessUniformRate )
	{
		OSDF::ConstIndexArray fvarValues;
		if( m_faceVaryingChannel != -1 )
		{
			fvarValues = m_mesh->GetFaceFVarValues( faceIndex, m_faceVaryingChannel );
		}

		int ownedBoundaryPoints = 0;
		for( int i = 0; i < fVerts.size(); ++i )
		{

			OSDF::Index vertIndex = fVerts[i];

			bool isVertexOwner = true;
			if( m_faceVaryingChannel == -1 || m_mesh->DoesVertexFVarTopologyMatch( vertIndex ) )
			{
				// For vertex primvar, or faceVarying primvars at verts where the faceVarying topology
				// matches vertex topology, the owner is whichever face touching this vertex has the lowest
				// index.
				for( OSDF::Index f : m_mesh->GetVertexFaces( vertIndex ) )
				{
					isVertexOwner &= f >= faceIndex;
				}
			}
			else
			{
				OSDF::ConstIndexArray adjFaces = m_mesh->GetVertexFaces( vertIndex );
				OSDF::ConstLocalIndexArray adjFaceLocalIndices = m_mesh->GetVertexFaceLocalIndices( vertIndex );

				// At a split vertex for a faceVarying primvar, we have to check the faceVarying indices of
				// adjacent faces at this vertex
				for( int j = 0; j < adjFaces.size(); j++ )
				{
					if( fvarValues[i] == m_mesh->GetFaceFVarValues( adjFaces[j], m_faceVaryingChannel )[ adjFaceLocalIndices[j] ] )
					{
						isVertexOwner &= adjFaces[j] >= faceIndex;
					}
				}
			}
		
			if( isVertexOwner )
			{
				// For faceVarying primitive variables, vertex ownership is stored per-faceVarying index
				// ( If there are two different faceVarying values at a Vertex, then we need two different faces
				// to be responsible for computing that vertex )
				int vertOwnerIndex = m_faceVaryingChannel == -1 ? vertIndex : fvarValues[i];

				m_vertexOwners[vertOwnerIndex].face = faceIndex;
				m_vertexOwners[vertOwnerIndex].offset = ownedBoundaryPoints;
				ownedBoundaryPoints++;
			}

			OSDF::Index edgeIndex = fEdges[i];
			int edgeRate = tessUniformRate;
			if( edgeRate > 1 )
			{
				int pointsPerEdge = edgeRate - 1;

				bool isEdgeOwner = true;
				if( m_faceVaryingChannel != -1 && !m_mesh->DoesEdgeFVarTopologyMatch( edgeIndex, m_faceVaryingChannel ) )
				{
					// If the edge is split by a facevarying primvar, we don't set an owner, which actually
					// means every face owns it's own copy of the edge.
					isEdgeOwner = false;
					ownedBoundaryPoints += pointsPerEdge;
				}
				else
				{
					for( OSDF::Index f : m_mesh->GetEdgeFaces( edgeIndex ) )
					{
						isEdgeOwner &= f >= faceIndex;
					}
				}
		
				if( isEdgeOwner )
				{
					m_edgeOwners[edgeIndex].face = faceIndex;
					m_edgeOwners[edgeIndex].offset = ownedBoundaryPoints;
					ownedBoundaryPoints += pointsPerEdge;
				}
			}
		}

		const int unownedBoundaryPoints = tessPattern.GetNumBoundaryCoords() - ownedBoundaryPoints;
		m_facePointOffsets[ faceIndex ] = tessPattern.GetNumCoords() - unownedBoundaryPoints;
	}

	const OSDF::TopologyLevel *m_mesh;
	int m_faceVaryingChannel;

	std::vector<int> m_facePointOffsets;
	std::vector<ComponentOwner> m_vertexOwners;
	std::vector<ComponentOwner> m_edgeOwners;
};

template < class T >
void evaluateSurface(
	const OSDB::Surface<float> &surface, const float *patchPointData, const float *uv,
	int outIndex, std::vector<T> &out, T *outNormals = nullptr
)
{
	constexpr int typeSize = numFloatsForType<T>();

	float buffer[typeSize];

	if( outNormals )
	{
		float du[typeSize];
		float dv[typeSize];

		surface.Evaluate( uv, patchPointData, typeSize, buffer, du, dv );

		// We know that we only pass outNormals for P, which is required to be V3f, so it's not a problem
		// that other types don't define cross or normalized
		if constexpr( std::is_same_v< T, V3f > )
		{
			outNormals[ outIndex ] = fromFloats<T>( du ).cross( fromFloats<T>( dv ) ).normalized();
		}
	}
	else
	{
		surface.Evaluate( uv, patchPointData, typeSize, buffer );
	}

	out[ outIndex ] = fromFloats<T>( buffer );
}


template <class T>
void tesselateVariable(
	const OSDB::Surface<float> &surface, int faceIndex,
	const OSDF::ConstIndexArray &fVerts, const OSDF::ConstIndexArray &fEdges,
	int tessUniformRate, const OSDB::Tessellation &tessPattern, const std::vector< V2f > &coords,
	const PrimvarTopology &primvarTopology,
	std::vector<int> &patchPointIndicesBuffer, std::vector< float > &patchPointsBuffer, std::vector< int > &boundaryIndicesBuffer,
	PrimvarSetup &setup, const PrimitiveVariable::IndexedView<T> &indexedView, int outIndicesIndex, T *outNormals = nullptr
)
{
	std::vector<T> &out = *(std::vector<T>*)(setup.m_outWritable);
	const int typeSize = numFloatsForType<T>();
	patchPointIndicesBuffer.resize( surface.GetNumControlPoints() );
	patchPointsBuffer.resize( surface.GetNumPatchPoints() * typeSize );

	// Populate the patch point array:
	surface.GetControlPointIndices( patchPointIndicesBuffer.data() );
	if( setup.m_var->interpolation != PrimitiveVariable::FaceVarying )
	{
		// Use the IndexedView to get correct value for the control points whether or not they are indexed.
		for( unsigned int i = 0; i < patchPointIndicesBuffer.size(); i++ )
		{
			toFloats<T>( indexedView[ patchPointIndicesBuffer[i] ], &patchPointsBuffer[i * typeSize] );
		}
	}
	else
	{
		// For FaceVarying primitive variables, any indices are handled by OSD's topology, so we need
		// to not apply the indices here.
		for( unsigned int i = 0; i < patchPointIndicesBuffer.size(); i++ )
		{
			toFloats<T>( indexedView.data()[ patchPointIndicesBuffer[i] ], &patchPointsBuffer[i * typeSize] );
		}
	}


	// Some of the patch points come from the control points, the remainder are derived from those by
	// this function.
	surface.ComputePatchPoints( patchPointsBuffer.data(), typeSize );

	const int numOutCoords = tessPattern.GetNumCoords();

	//
	// Evaluate the sample points of the Tessellation:
	//
	// First traverse the boundary of the face to determine whether
	// to evaluate or share points on vertices and edges of the face.
	// Both pre-existing and new boundary points are identified by
	// index in an array for later use. The interior points are all
	// trivially computed after the boundary is dealt with.
	//
	// Identify the boundary and interior coords and initialize the
	// index array for the potentially shared boundary points:
	//
	int numBoundaryCoords = tessPattern.GetNumBoundaryCoords();
	int numInteriorCoords = numOutCoords - numBoundaryCoords;

	const V2f *tessBoundaryCoords = &coords[0];
	const V2f *tessInteriorCoords = &coords[numBoundaryCoords];

	if( setup.m_outIndicesWritable )
	{
		boundaryIndicesBuffer.resize(numBoundaryCoords);
	}

	//
	// Walk around the face, inspecting each vertex and outgoing edge,
	// and populating the index array of boundary points:

	OSDF::ConstIndexArray fvarValues;
	if( primvarTopology.m_faceVaryingChannel != -1 )
	{
		fvarValues = primvarTopology.m_mesh->GetFaceFVarValues( faceIndex, primvarTopology.m_faceVaryingChannel );
	}

	int boundaryIndex = 0;
	int outOffset = primvarTopology.m_facePointOffsets[faceIndex];
	for( int i = 0; i < fVerts.size(); ++i )
	{
		// For faceVarying primitive variables, vertex ownership is stored per-faceVarying index
		int vertOwnerIndex = primvarTopology.m_faceVaryingChannel == -1 ? fVerts[i] : fvarValues[i];

		const ComponentOwner &vOwner = primvarTopology.m_vertexOwners[ vertOwnerIndex ];
		if( vOwner.face == faceIndex )
		{
			evaluateSurface<T>(
				surface, patchPointsBuffer.data(), (float*)&tessBoundaryCoords[ boundaryIndex ],
				outOffset++, out, outNormals
			);
		}

		if( setup.m_outIndicesWritable )
		{
			if( vOwner.face == faceIndex )
			{
				boundaryIndicesBuffer[boundaryIndex] = outOffset - 1;
			}
			else
			{
				// Assign shared vertex point index to boundary:
				boundaryIndicesBuffer[boundaryIndex] = primvarTopology.m_facePointOffsets[ vOwner.face ] + vOwner.offset;
			}
		}

		boundaryIndex++;

		OSDF::Index edgeIndex = fEdges[i];
		int edgeRate = tessUniformRate;

		//
		// Evaluate/assign or retrieve all shared points for the edge:
		//
		// To keep this simple, assume the edge is manifold. So the
		// second face sharing the edge has that edge in the opposite
		// direction in its boundary relative to the first face --
		// making it necessary to reverse the order of shared points
		// for the boundary of the second face.
		//
		// To support a non-manifold edge, all subsequent faces that
		// share the assigned shared edge must determine if their
		// orientation of that edge is reversed relative to the first
		// face for which the shared edge points were evaluated. So a
		// little more book-keeping and/or inspection is required.
		//
		if( edgeRate > 1 )
		{
			int pointsPerEdge = edgeRate - 1;

			const ComponentOwner &eOwner = primvarTopology.m_edgeOwners[ edgeIndex ];
			if( eOwner.face == -1 || eOwner.face == faceIndex )
			{
				// Identify indices of the new shared points in both the
				// mesh and face and increment their inventory:

				for (int j = 0; j < pointsPerEdge; ++j )
				{
					evaluateSurface<T>(
						surface, patchPointsBuffer.data(), (float*)&tessBoundaryCoords[ boundaryIndex + j ],
						outOffset++, out, outNormals
					);
				}
			}
			
			if( setup.m_outIndicesWritable )
			{
				if( eOwner.face == -1 || eOwner.face == faceIndex )
				{
					// Add the edge values we just wrote to the boundary indices
					for (int j = 0; j < pointsPerEdge; ++j)
					{
						boundaryIndicesBuffer[ boundaryIndex + j ] = outOffset - pointsPerEdge + j;
					}
				}
				else
				{
					// Assign shared points to boundary in reverse order:
					int currentPointIndex = primvarTopology.m_facePointOffsets[ eOwner.face ] + eOwner.offset + pointsPerEdge - 1;
					for (int j = 0; j < pointsPerEdge; ++j)
					{
						boundaryIndicesBuffer[ boundaryIndex + j ] = currentPointIndex--;
					}
				}
			}

			boundaryIndex += pointsPerEdge;
		}
	}

	//
	// Evaluate any interior points unique to this face -- appending
	// them to those shared points computed above for the boundary:
	//
	if( numInteriorCoords )
	{
		for (int i = 0; i < numInteriorCoords; ++i)
		{
			evaluateSurface<T>(
				surface, patchPointsBuffer.data(), (float*)&tessInteriorCoords[ i ],
				outOffset++, out, outNormals
			);
		}
	}

	//
	// Identify the faces of the Tessellation:
	//
	// Note that the coordinate indices used by the facets are local
	// to the face (i.e. they range from [0..N-1], where N is the
	// number of coordinates in the pattern) and so need to be offset
	// when writing to Obj format.
	//
	// For more advanced use, the coordinates associated with the
	// boundary and interior of the pattern are distinguishable so
	// that those on the boundary can be easily remapped to refer to
	// shared edge or corner points, while those in the interior can
	// be separately offset or similarly remapped.
	//
	// So transform the indices of the facets here as needed using
	// the indices of shared boundary points assembled above and a
	// suitable offset for the new interior points added:
	//
	if( setup.m_outIndicesWritable )
	{
		int tessInteriorOffset = outOffset - numOutCoords;
		tessPattern.GetFacets( &setup.m_outIndicesWritable[outIndicesIndex] );

		// TransformFacetCoordIndices seems to incorrectly not be labelled as const
		const_cast< OSDB::Tessellation* >( &tessPattern )->TransformFacetCoordIndices(
			&setup.m_outIndicesWritable[outIndicesIndex], boundaryIndicesBuffer.data(), tessInteriorOffset
		);
	}
}

struct MutexReadGuard
{
	MutexReadGuard( tbb::spin_rw_mutex &m )
		: m_guard( m, false )
	{
	}

	tbb::spin_rw_mutex::scoped_lock m_guard;
};

struct MutexWriteGuard
{
	MutexWriteGuard( tbb::spin_rw_mutex &m )
		: m_guard( m, true )
	{
	}

	tbb::spin_rw_mutex::scoped_lock m_guard;
};

int intVectorAccumulate( std::vector<int> &v )
{
	int accum = 0;
	for( int &o : v )
	{
		int prevAccum = accum;
		accum += o;
		o = prevAccum;
	}
	return accum;
}

} // namespace

GAFFER_NODE_DEFINE_TYPE( MeshSubdivide );

size_t MeshSubdivide::g_firstPlugIndex = 0;

MeshSubdivide::MeshSubdivide( const std::string &name )
	:	ObjectProcessor( name )
{
	storeIndexOfNextChild( g_firstPlugIndex );

	addChild( new IntPlug( "levels", Gaffer::Plug::In, 1, 1 ) );
	addChild( new IntPlug( "scheme", Gaffer::Plug::In, (int)Scheme::CatmullClark, (int)Scheme::First, (int)Scheme::Last ) );
	addChild( new BoolPlug( "calculateNormals", Gaffer::Plug::In, false ) );
	addChild( new BoolPlug( "subdividePolygons", Gaffer::Plug::In, false ) );

}

MeshSubdivide::~MeshSubdivide()
{
}

Gaffer::IntPlug *MeshSubdivide::levelsPlug()
{
	return getChild<Gaffer::IntPlug>( g_firstPlugIndex + 0);
}

const Gaffer::IntPlug *MeshSubdivide::levelsPlug() const
{
	return getChild<Gaffer::IntPlug>( g_firstPlugIndex + 0);
}

Gaffer::IntPlug *MeshSubdivide::schemePlug()
{
	return getChild<Gaffer::IntPlug>( g_firstPlugIndex + 1 );
}

const Gaffer::IntPlug *MeshSubdivide::schemePlug() const
{
	return getChild<Gaffer::IntPlug>( g_firstPlugIndex + 1 );
}

Gaffer::BoolPlug *MeshSubdivide::calculateNormalsPlug()
{
	return getChild<Gaffer::BoolPlug>( g_firstPlugIndex + 2 );
}

const Gaffer::BoolPlug *MeshSubdivide::calculateNormalsPlug() const
{
	return getChild<Gaffer::BoolPlug>( g_firstPlugIndex + 2 );
}

Gaffer::BoolPlug *MeshSubdivide::subdividePolygonsPlug()
{
	return getChild<Gaffer::BoolPlug>( g_firstPlugIndex + 3 );
}

const Gaffer::BoolPlug *MeshSubdivide::subdividePolygonsPlug() const
{
	return getChild<Gaffer::BoolPlug>( g_firstPlugIndex + 3 );
}

bool MeshSubdivide::affectsProcessedObject( const Gaffer::Plug *input ) const
{
	return
		ObjectProcessor::affectsProcessedObject( input ) ||
		input == levelsPlug() ||
		input == schemePlug() ||
		input == calculateNormalsPlug() ||
		input == subdividePolygonsPlug();
}



void MeshSubdivide::hashProcessedObject( const ScenePath &path, const Gaffer::Context *context, IECore::MurmurHash &h ) const
{
	ObjectProcessor::hashProcessedObject( path, context, h );

	levelsPlug()->hash( h );
	schemePlug()->hash( h );
	calculateNormalsPlug()->hash( h );
	subdividePolygonsPlug()->hash( h );
}


IECore::ConstObjectPtr MeshSubdivide::computeProcessedObject( const ScenePath &path, const Gaffer::Context *context, const IECore::Object *inputObject ) const
{
	const IECoreScene::MeshPrimitive *inputMesh = runTimeCast<const IECoreScene::MeshPrimitive>( inputObject );
	if( !inputMesh )
	{
		return inputObject;
	}

	if( inputMesh->interpolation() == "linear" && !subdividePolygonsPlug()->getValue() )
	{
		return inputObject;
	}

	IECore::ConstV3fVectorDataPtr pDataIn = inputMesh->variableData<V3fVectorData>( "P", PrimitiveVariable::Vertex );
	if( !pDataIn )
	{
		throw IECore::Exception( "MeshSubdivide::computeProcessedObject: input mesh has no P data." );
	}
	const int tessUniformRate = 1 << levelsPlug()->getValue();

	Scheme schemeValue = (Scheme)schemePlug()->getValue();
	OpenSubdiv::Sdc::SchemeType osScheme = OpenSubdiv::Sdc::SCHEME_CATMARK;
	switch( schemeValue )
	{
		case Scheme::Bilinear:
			osScheme = OpenSubdiv::Sdc::SCHEME_BILINEAR;
			break;
		case Scheme::CatmullClark:
			osScheme = OpenSubdiv::Sdc::SCHEME_CATMARK;
			break;
		case Scheme::Loop:
			osScheme = OpenSubdiv::Sdc::SCHEME_LOOP;
			break;
		default:
			assert( false );
	}

	if( osScheme == OpenSubdiv::Sdc::SCHEME_LOOP && inputMesh->maxVerticesPerFace() > 3 )
	{
		throw Exception( "Loop subdivision can only be applied to triangle meshes ");
	}

	OpenSubdiv::Sdc::Options options;
	options.SetVtxBoundaryInterpolation(OpenSubdiv::Sdc::Options::VTX_BOUNDARY_EDGE_AND_CORNER);
	options.SetFVarLinearInterpolation(OpenSubdiv::Sdc::Options::FVAR_LINEAR_BOUNDARIES);

	typedef OSDF::TopologyDescriptor Descriptor;
	Descriptor desc;

	// enter mesh topology data:
	desc.numVertices = inputMesh->variableSize( PrimitiveVariable::Vertex );
	desc.numFaces = inputMesh->variableSize( PrimitiveVariable::Uniform );
	desc.numVertsPerFace = inputMesh->verticesPerFace()->readable().data();
	desc.vertIndicesPerFace = inputMesh->vertexIds()->readable().data();

	const IECore::IntVectorData *cornerIdsData = inputMesh->cornerIds();
	const std::vector<int> cornerIds = cornerIdsData->readable();

	const IECore::FloatVectorData *cornerSharpnessesData = inputMesh->cornerSharpnesses();
	const std::vector<float> cornerSharpnesses = cornerSharpnessesData->readable();

	if( !cornerIds.empty() && !cornerSharpnesses.empty() )
	{
		desc.numCorners = cornerIds.size();
		desc.cornerVertexIndices = cornerIds.data();
		desc.cornerWeights = cornerSharpnesses.data();
	}

	const IECore::IntVectorData *creaseLengthsData = inputMesh->creaseLengths();
	const std::vector<int> creaseLengths = creaseLengthsData->readable();

	std::vector<int> expandedIds;
	std::vector<float> expandedSharpnesses;
	if( !creaseLengths.empty() )
	{
		const IECore::FloatVectorData *creaseSharpnessesData = inputMesh->creaseSharpnesses();
		const std::vector<float> creaseSharpnesses = creaseSharpnessesData->readable();

		const IECore::IntVectorData *creaseIdsData = inputMesh->creaseIds();
		const std::vector<int> creaseIds = creaseIdsData->readable();

		// Cortex stores crease edges in a compact way where multiple edges can
		// be part of a crease. OpenSubdiv expects us to provide vertex pairs,
		// though, so we need to assemble those from the more compact
		// representation.

		size_t requiredSize = std::accumulate( creaseLengths.begin(), creaseLengths.end(), 0 ) - creaseLengths.size();
		expandedIds.reserve( requiredSize * 2 );
		expandedSharpnesses.reserve( requiredSize );

		int offset = 0;
		int numCreases = 0;
		for( size_t i = 0; i < creaseLengths.size(); ++i )
		{
			int length = creaseLengths[i];

			for( int j = 1; j < length; ++j, ++numCreases )
			{
				expandedIds.push_back( creaseIds[offset + j - 1] );
				expandedIds.push_back( creaseIds[offset + j] );

				expandedSharpnesses.push_back( creaseSharpnesses[i] );
			}

			offset += length;
		}

		desc.numCreases = numCreases;
		desc.creaseVertexIndexPairs = expandedIds.data();
		desc.creaseWeights = expandedSharpnesses.data();
	}

	PrimitiveVariableMap variables = inputMesh->variables;

	bool calculateNormals = calculateNormalsPlug()->getValue();


	PrimvarSetup posPrimvarSetup( "P", variables.at( "P" ) );
	// TODO - test missing/wrong P
	if( posPrimvarSetup.m_var->data->typeId() != V3fVectorData::staticTypeId() )
	{
		throw IECore::Exception( "P PrimitiveVariable must be V3f" );
	}
	std::vector< PrimvarSetup > vertexPrimvarSetups;
	std::vector< PrimvarSetup > uniformPrimvarSetups;
	std::vector< PrimvarSetup > faceVaryingPrimvarSetups;
	setupVariables(
		variables, calculateNormals,
		posPrimvarSetup, vertexPrimvarSetups, uniformPrimvarSetups, faceVaryingPrimvarSetups
	);

	std::vector<Descriptor::FVarChannel> channels( faceVaryingPrimvarSetups.size() );
	desc.numFVarChannels = faceVaryingPrimvarSetups.size();

	std::vector<int> noopFaceVaryingIndices;

	for( unsigned int i = 0; i < faceVaryingPrimvarSetups.size(); i++ )
	{
		PrimvarSetup &s = faceVaryingPrimvarSetups[i];


		channels[i].numValues = IECore::size( s.m_var->data.get() );
		if( s.m_var->indices )
		{
			channels[i].valueIndices = s.m_var->indices->readable().data();
		}
		else
		{
			if( !noopFaceVaryingIndices.size() )
			{
				noopFaceVaryingIndices.reserve( channels[i].numValues );
				for( int j = 0; j < channels[i].numValues; j++ )
				{
					noopFaceVaryingIndices.push_back( j );
				}
			}
			channels[i].valueIndices = noopFaceVaryingIndices.data();
		}
	}
	desc.fvarChannels = channels.data();

	// Instantiate a FarTopologyRefiner from the descriptor
	OSDF::TopologyRefiner * refiner = OSDF::TopologyRefinerFactory<Descriptor>::Create(desc, OSDF::TopologyRefinerFactory<Descriptor>::Options(osScheme, options));


	typedef OSDB::SurfaceFactoryCacheThreaded< tbb::spin_rw_mutex, MutexReadGuard, MutexWriteGuard > SurfaceFactoryCache;
	typedef OSDB::RefinerSurfaceFactory< SurfaceFactoryCache > SurfaceFactory;
	

	SurfaceFactory::Options surfaceOptions;
	//surfaceOptions.EnableCaching( false ); // TODO - Definitely slower

	SurfaceFactory meshSurfaceFactory( *refiner, surfaceOptions);

	OSDB::Tessellation::Options tessOptions;
	// We use quads except for Loop subdivision which uses tris.
	const int tessFacetSize = osScheme != OpenSubdiv::Sdc::SCHEME_LOOP ? 4 : 3;
	//const int tessFacetSize = 4;
	tessOptions.SetFacetSize( tessFacetSize );
	tessOptions.PreserveQuads( tessFacetSize == 4);

	OSDF::TopologyLevel const & baseLevel = refiner->GetLevel(0);

	// If we were doing adaptive subdivision, here would be the place to prepare a list of tesselation rates
	// per edge that would be referenced below, to ensure consistency.


	const int numFaces = baseLevel.GetNumFaces();

	std::vector<int> faceFacetOffsets( numFaces ); // TODO - rename

	PrimvarTopology vertexTopology;
	vertexTopology.initialize( &baseLevel );

	std::vector< PrimvarTopology > faceVaryingTopologies( faceVaryingPrimvarSetups.size() );
	for( unsigned int i = 0; i < faceVaryingPrimvarSetups.size(); i++ )
	{
		faceVaryingTopologies[i].initialize( &baseLevel, i );
	}

	tbb::task_group_context taskGroupContext( tbb::task_group_context::isolated );

	tbb::parallel_for(
		tbb::blocked_range<int>( 0, numFaces ),
		[&]( tbb::blocked_range<int> &range )
		{
			OSDB::Surface<float> faceSurface;

			for( int faceIndex = range.begin(); faceIndex != range.end(); ++faceIndex )
			{
				// Initialize the Surface for this face -- if valid (skipping
				// holes and boundary faces in some rare cases):
				if( !meshSurfaceFactory.InitVertexSurface( faceIndex, &faceSurface ) )
				{
					continue;
				}

				OSDF::ConstIndexArray fVerts = baseLevel.GetFaceVertices(faceIndex);
				OSDF::ConstIndexArray fEdges = baseLevel.GetFaceEdges(faceIndex);

				OSDB::Tessellation tessPattern(
					faceSurface.GetParameterization(), tessUniformRate, tessOptions
				);

				faceFacetOffsets[ faceIndex ] = tessPattern.GetNumFacets() * tessFacetSize;

				vertexTopology.addFace( faceIndex, tessPattern, fVerts, fEdges, tessUniformRate );

				for( PrimvarTopology &t : faceVaryingTopologies )
				{
					t.addFace( faceIndex, tessPattern, fVerts, fEdges, tessUniformRate );
				}
			}
		},
		tbb::static_partitioner(),
		taskGroupContext
	);

	// The part we can't multithread is converting counts to offsets, but this should be very quick.
	// Note that to save doing a new allocation, we reuse the memory, and the count vectors are invalidated.
	const int numOutPoints = intVectorAccumulate( vertexTopology.m_facePointOffsets );
	const int numOutVertexIds = intVectorAccumulate( faceFacetOffsets );
	const int numOutFacets = numOutVertexIds / tessFacetSize;

	posPrimvarSetup.initialize( numOutPoints, numOutVertexIds );
	for( PrimvarSetup &setup : vertexPrimvarSetups )
	{
		setup.initialize( numOutPoints, 0 );
	}

	for( PrimvarSetup &setup : uniformPrimvarSetups )
	{
		setup.initialize( 0, numOutFacets  );
	}

	for( unsigned int i = 0; i < faceVaryingPrimvarSetups.size(); i++ )
	{
		const int numOutFaceVarying = intVectorAccumulate( faceVaryingTopologies[i].m_facePointOffsets );
		faceVaryingPrimvarSetups[i].initialize( numOutFaceVarying, numOutVertexIds );
	}	

	V3fVectorDataPtr outNormalsData = new V3fVectorData();
	outNormalsData->setInterpretation( GeometricData::Normal );
	std::vector<V3f> &outNormals = outNormalsData->writable();
	if( calculateNormals )
	{
		podVectorResizeUninitialized( outNormals, numOutPoints );
	}

	tbb::parallel_for(
		tbb::blocked_range<int>( 0, numFaces ),
		[&]( tbb::blocked_range<int> &range )
		{
			std::vector<V2f> tessCoords;
			std::vector<int> outFacets;
			std::vector<int> tessBoundaryIndices;

			OSDB::Surface<float> vertexSurface;
			OSDB::Surface<float> faceVaryingSurface;
			std::vector<int> patchPointIndicesBuffer;
			std::vector<float> patchPointsBuffer;
			std::vector<int> indicesBuffer;

			for( int faceIndex = range.begin(); faceIndex != range.end(); ++faceIndex )
			{
				// Initialize the Surface for this face -- if valid (skipping
				// holes and boundary faces in some rare cases):
				if( !meshSurfaceFactory.InitVertexSurface( faceIndex, &vertexSurface ) )
				{
					continue;
				}

				//
				// Declare a simple uniform Tessellation for the Parameterization
				// of this face and identify coordinates of the points to evaluate:
				//
				OSDB::Tessellation tessPattern( vertexSurface.GetParameterization(), tessUniformRate, tessOptions );

				int numOutCoords = tessPattern.GetNumCoords();

				tessCoords.resize( numOutCoords );

				tessPattern.GetCoords( (float*)tessCoords.data() );

				//
				// Prepare the patch points for the Surface, then use them to
				// evaluate output points for all identified coordinates:
				//
				// Resize patch point and output arrays:

				OSDF::ConstIndexArray fVerts = baseLevel.GetFaceVertices( faceIndex );
				OSDF::ConstIndexArray fEdges = baseLevel.GetFaceEdges( faceIndex );

				//if( regularPrimvarSetups[-1]->m_uglyWayToStoreTypeForDispatch->typeId() == V3fVectorData::staticTypeId() )
				tesselateVariable<V3f>(
					vertexSurface, faceIndex, fVerts, fEdges, tessUniformRate, tessPattern, tessCoords, 
					vertexTopology, 
					patchPointIndicesBuffer, patchPointsBuffer, indicesBuffer,
					posPrimvarSetup, PrimitiveVariable::IndexedView<V3f>( *posPrimvarSetup.m_var ),
					faceFacetOffsets[faceIndex],
					calculateNormals ? outNormals.data() : nullptr
				);

				for( PrimvarSetup &setup : vertexPrimvarSetups )
				{
					dispatchIndexedView(
						*setup.m_var,
						[&]( const auto &indexedView ) -> void
						{
							// TODO : This weirdness would just be IndexedView::ElementType if IndexedView
							// declared `using ElementType = T` which seems like it would be very reasonable
							// to add.
							using ElementType = typename std::remove_cv_t< std::remove_reference_t< decltype( indexedView[0] ) > >;

							tesselateVariable<ElementType>(
								vertexSurface, faceIndex, fVerts, fEdges, tessUniformRate, tessPattern, tessCoords, 
								vertexTopology, 
								patchPointIndicesBuffer, patchPointsBuffer, indicesBuffer,
								setup, indexedView, faceFacetOffsets[faceIndex]
							);
						}
					);
				}

				for( PrimvarSetup &setup : uniformPrimvarSetups )
				{
					int faceOffset = faceFacetOffsets[faceIndex] / tessFacetSize;
					// TODO - test indexed uniforms
					int uniformIndex = setup.m_var->indices ? setup.m_var->indices->readable()[faceIndex] : faceIndex;
					for( int i = 0; i < tessPattern.GetNumFacets(); i++ )
					{
						setup.m_outIndicesWritable[ faceOffset + i ] = uniformIndex;
					}
				}

				for( unsigned int i = 0; i < faceVaryingPrimvarSetups.size(); i++ )
				{
					if( !meshSurfaceFactory.InitFaceVaryingSurface( faceIndex, &faceVaryingSurface, i ) )
					{
						continue;
					}

					dispatchIndexedView(
						*faceVaryingPrimvarSetups[i].m_var,
						[&]( const auto &indexedView ) -> void
						{
							// TODO : This weirdness would just be IndexedView::ElementType if IndexedView
                            // declared `using ElementType = T` which seems like it would be very reasonable
                            // to add.
							using ElementType = typename std::remove_cv_t< std::remove_reference_t< decltype( indexedView[0] ) > >;
							tesselateVariable<ElementType>(
								faceVaryingSurface, faceIndex, fVerts, fEdges, tessUniformRate, tessPattern, tessCoords, 
								faceVaryingTopologies[i], 
								patchPointIndicesBuffer, patchPointsBuffer, indicesBuffer,
								faceVaryingPrimvarSetups[i], indexedView, faceFacetOffsets[faceIndex]
							);
						}
					);
				}	
			}
		},
		tbb::auto_partitioner(),
		taskGroupContext
	);

	delete refiner; // TODO - use unique_ptr
	IntVectorDataPtr verticesPerFaceData = new IntVectorData();
	verticesPerFaceData->writable().resize( numOutFacets, tessFacetSize );

	MeshPrimitivePtr result = new MeshPrimitive( verticesPerFaceData, posPrimvarSetup.m_outIndicesData, "linear", IECore::runTimeCast<IECore::V3fVectorData>( posPrimvarSetup.m_outData.get() ) );

	if( calculateNormals )
	{
		result->variables["N"] = PrimitiveVariable( PrimitiveVariable::Vertex, outNormalsData );
	}

	for( PrimvarSetup &setup : vertexPrimvarSetups )
	{
		result->variables[setup.m_name] = PrimitiveVariable( PrimitiveVariable::Vertex, setup.m_outData );
	}

	for( PrimvarSetup &setup : uniformPrimvarSetups )
	{
		result->variables[setup.m_name] = PrimitiveVariable( PrimitiveVariable::Uniform, setup.m_var->data, setup.m_outIndicesData );
	}

	for( PrimvarSetup &setup : faceVaryingPrimvarSetups )
	{
		result->variables[setup.m_name] = PrimitiveVariable( PrimitiveVariable::FaceVarying, setup.m_outData, setup.m_outIndicesData );
	}

	// We didn't need to make setups to hold interpolated data during threading for constant primvars,
	// we just copy them across directly.
	for( PrimitiveVariableMap::const_iterator it = variables.begin(); it != variables.end(); ++it )
	{
		if( it->second.interpolation == PrimitiveVariable::Constant )
		{
			result->variables[it->first] = PrimitiveVariable( PrimitiveVariable::Constant, const_cast<Data*>( it->second.data.get() ) );
		}
	}

	return result;
}

Gaffer::ValuePlug::CachePolicy MeshSubdivide::processedObjectComputeCachePolicy() const
{
	return ValuePlug::CachePolicy::Default;
}

