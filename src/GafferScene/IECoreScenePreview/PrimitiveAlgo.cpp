//////////////////////////////////////////////////////////////////////////
//
//  Copyright (c) 2024, Image Engine Design Inc. All rights reserved.
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

#include "GafferScene/Private/IECoreScenePreview/PrimitiveAlgo.h"

#include "IECoreScene/PrimitiveVariable.h"
#include "IECoreScene/MeshPrimitive.h"
#include "IECoreScene/CurvesPrimitive.h"
#include "IECoreScene/PointsPrimitive.h"

#include "IECore/DataAlgo.h"
#include "IECore/TypeTraits.h"

#include <unordered_map>
#include <numeric>

#include "fmt/format.h"

#include "tbb/parallel_for.h"

using namespace IECoreScene;
using namespace IECore;
using namespace IECoreScenePreview;


namespace {

// Copied from Context.inl because it isn't public
template<typename T>
struct DataTraits
{
	using DataType = IECore::TypedData<T>;
};

template<typename T>
struct DataTraits<Imath::Vec2<T> >
{
	using DataType = IECore::GeometricTypedData<Imath::Vec2<T>>;
};

template<typename T>
struct DataTraits<Imath::Vec3<T> >
{
	using DataType = IECore::GeometricTypedData<Imath::Vec3<T>>;
};

template<typename T>
struct DataTraits<std::vector<Imath::Vec2<T> > >
{
	using DataType = IECore::GeometricTypedData<std::vector<Imath::Vec2<T>>>;
};

template<typename T>
struct DataTraits<std::vector<Imath::Vec3<T> > >
{
	using DataType = IECore::GeometricTypedData<std::vector<Imath::Vec3<T>>>;
};


// Return if we have TypedData holder for a vector of T
template< typename T>
constexpr bool supportsVectorTypedData()
{
	// This should probably be a whitelist, not a blacklist. But also it should be defined somewhere
	// central, not here.
	return !(
		std::is_same_v< T, IECore::TransformationMatrixd > ||
		std::is_same_v< T, IECore::TransformationMatrixf > ||
		std::is_same_v< T, IECore::Splineff > ||
		std::is_same_v< T, IECore::SplinefColor3f > ||
		std::is_same_v< T, IECore::SplinefColor4f > ||
		std::is_same_v< T, IECore::Splinedd > ||
		std::is_same_v< T, IECore::PathMatcher > ||
		std::is_same_v< T, boost::posix_time::ptime>
	);
}

Imath::M44f normalTransform( const Imath::M44f &m )
{
	Imath::M44f result = m.inverse();
	result.transpose();
	return result;
}

// \todo : Perhaps belongs in DataAlgo with IECore::size? ( Also, stuff like DataAlgo::size should be
// refactored to use `if constexpr` )
void dataResize( Data *data, size_t size )
{
	IECore::dispatch( data,
		[size] ( auto *typedData ) {
			using DataType = std::remove_pointer_t< decltype( typedData ) >;
			if constexpr( TypeTraits::IsVectorTypedData< DataType >::value )
			{
				// Ideally, we wouldn't initialize anything here, and we would only zero out the memory
				// if needed. ( ie. while we're in the final multithreaded loop of mergePrimitives, and find
				// that one primvar has no data for this primvar, then we could zero out just that segment ...
				// that would be a potentially significant performance win ).
				//
				// However, we currently can't suppress zero-initialization for most types, so it's most
				// consistent if for now we just force everything to to zero-initialize, rather than taking
				// advantage of the extra performance for the types where we could.
				//
				// So we've got a hardcoded list here of imath types where the default constructor doesn't
				// initialize. Note that if there is any type covered by dispatch which doesn't initialize,
				// and isn't listed in this hardcoded list, you will get weird, uninitialized behaviour.
				if constexpr(
					std::is_same_v< DataType, V2iVectorData > || std::is_same_v< DataType, V3iVectorData > ||
					std::is_same_v< DataType, V2fVectorData > || std::is_same_v< DataType, V3fVectorData > ||
					std::is_same_v< DataType, V2dVectorData > || std::is_same_v< DataType, V3dVectorData > ||
					std::is_same_v< DataType, Color3fVectorData > || std::is_same_v< DataType, Color4fVectorData >
				)
				{
					using SingleElementType = typename DataType::ValueType::value_type;
					typedData->writable().resize( size, SingleElementType( 0 ) );
				}
				else
				{
					typedData->writable().resize( size );
				}
			}
			else if( size != 1 )
			{
				throw IECore::Exception( fmt::format(
					"Can't resize, not a vector data type: {}", typedData->typeName()
				) );
			}
		}
	);
}

inline void transformPrimVarValue(
	const Imath::V3f &source, Imath::V3f &dest,
	const Imath::M44f &matrix, const Imath::M44f &normalMatrix, GeometricData::Interpretation interpretation
)
{
	if( interpretation == GeometricData::Point )
	{
		dest = source * matrix;
	}
	else if( interpretation == GeometricData::Vector )
	{
		matrix.multDirMatrix( source, dest );
	}
	else if( interpretation == GeometricData::Normal )
	{
		normalMatrix.multDirMatrix( source, dest );
	}
	else
	{
		dest = source;
	}

}


inline void copyElements( const Data *sourceData, size_t sourceIndex, Data *destData, size_t destIndex, size_t num, const Imath::M44f &matrix, const Imath::M44f &normalMatrix )
{
	IECore::dispatch( destData,
		[&] ( auto *typedDestData ) {
			using DataType = std::remove_pointer_t< decltype( typedDestData ) >;
			if constexpr( TypeTraits::IsVectorTypedData< DataType >::value )
			{
				auto &typedDest = typedDestData->writable();
				auto *typedSourceData = IECore::runTimeCast< const DataType >( sourceData );
				if( !typedSourceData )
				{
					// Failed to cast to destination type ... maybe this is a Constant variable being promoted,
					// and the Data stores a single element instead of a vector?

					using SingleElementDataType = typename DataTraits< typename DataType::ValueType::value_type >::DataType;

					auto *singleElementTypedSourceData = IECore::runTimeCast< const SingleElementDataType >( sourceData );
					if( singleElementTypedSourceData )
					{
						if constexpr( std::is_same_v< SingleElementDataType, V3fData > )
						{
							// Fairly weird corner case, but technically Constant primvars could need transforming too
							GeometricData::Interpretation interp = singleElementTypedSourceData->getInterpretation();
							transformPrimVarValue(
								singleElementTypedSourceData->readable(), typedDest[ destIndex ], matrix, normalMatrix, interp
							);
						}
						else
						{
							typedDest[ destIndex ] = singleElementTypedSourceData->readable();
						}
						return;
					}
					else
					{
						throw IECore::Exception( fmt::format(
							"Can't copy element of type {} to destination of type: {}",
							sourceData->typeName(), destData->typeName()
						) );
					}
				}
				const auto &typedSource = typedSourceData->readable();

				if constexpr( std::is_same_v< DataType, V3fVectorData > )
				{
					GeometricData::Interpretation interp = typedSourceData->getInterpretation();
					for( size_t i = 0; i < num; i++ )
					{
						transformPrimVarValue(
							typedSource[ sourceIndex + i ], typedDest[ destIndex + i ], matrix, normalMatrix, interp
						);
					}
				}
				else
				{
					for( size_t i = 0; i < num; i++ )
					{
						typedDest[ destIndex + i ] = typedSource[ sourceIndex + i ];
					}
				}
			}
			else
			{
				throw IECore::Exception( fmt::format(
					"Can't copy elements, not a vector data type: {}", typedDestData->typeName()
				) );
			}
		}
	);
}

IECore::TypeId vectorDataTypeFromDataType( const Data *data )
{
	return IECore::dispatch( data,
		[] ( auto *typedData ) {
			using DataType = std::remove_pointer_t< decltype( typedData ) >;
			if constexpr( TypeTraits::HasValueType< DataType >::value )
			{
				using ValueType = typename DataType::ValueType;
				if constexpr(
					!TypeTraits::IsVectorTypedData< DataType >::value &&
					supportsVectorTypedData<ValueType>()
				)
				{
					return DataTraits< std::vector<ValueType> >::DataType::staticTypeId();
				}
			}
			return IECore::InvalidTypeId;
		}
	);
}

bool interpolationMatches(
	IECoreScene::TypeId primType, PrimitiveVariable::Interpolation a, PrimitiveVariable::Interpolation b
)
{
	if( a == b )
	{
		return true;
	}

	if( primType == IECoreScene::MeshPrimitiveTypeId )
	{
		auto isVertex = []( PrimitiveVariable::Interpolation x) {
			return x == PrimitiveVariable::Vertex || x == PrimitiveVariable::Varying;
		};
		return isVertex( a ) && isVertex( b );
	}
	else if( primType == IECoreScene::CurvesPrimitiveTypeId )
	{
		auto isVarying = []( PrimitiveVariable::Interpolation x) {
			return x == PrimitiveVariable::Varying || x == PrimitiveVariable::FaceVarying;
		};
		return isVarying( a ) && isVarying( b );
	}
	else
	{
		auto isVertex = []( PrimitiveVariable::Interpolation x) {
			return x == PrimitiveVariable::Vertex || x == PrimitiveVariable::Varying || x == PrimitiveVariable::FaceVarying;
		};
		return isVertex( a ) && isVertex( b );
	}
}

PrimitiveVariable::Interpolation mergeInterpolations(
	IECoreScene::TypeId primType, PrimitiveVariable::Interpolation a, PrimitiveVariable::Interpolation b,
	const IECore::InternedString &msgName
)
{
	PrimitiveVariable::Interpolation result;

	// In general, more specific Interpolations have a higher enum value, so we want to take
	// whichever interpolation is higher. This doesn't always work, so afterwards we have several
	// special cases to clean things up.
	result = std::max( a, b );

	if( primType == IECoreScene::PointsPrimitiveTypeId )
	{
		// On points, everything is output as Vertex
		result = PrimitiveVariable::Vertex;
	}
	else if(
		primType == IECoreScene::MeshPrimitiveTypeId &&
		result >= PrimitiveVariable::Vertex &&
		( a == PrimitiveVariable::Uniform || b == PrimitiveVariable::Uniform )
	)
	{
		// On meshes, if you mix Uniform and Vertex, we need to use FaceVarying to represent both
		result = PrimitiveVariable::FaceVarying;
	}
	else if(
		primType == IECoreScene::CurvesPrimitiveTypeId &&
		result >= PrimitiveVariable::Varying &&
		( a == PrimitiveVariable::Vertex || b == PrimitiveVariable::Vertex )
	)
	{
		// Mixing Vertex/Varying on curves requires a lossy resample that would make things more complex.
		msg( Msg::Warning, "mergePrimitives",
			fmt::format(
				"Discarding variable \"{}\" - Cannot mix Vertex and Varying curve variables.",
				std::string( msgName )
			)
		);
		result = PrimitiveVariable::Invalid;
	}

	// When merging interpolations, if the interpolation has synonymous names, we always choose the canonical one
	if( interpolationMatches( primType, result, PrimitiveVariable::Vertex ) )
	{
		result = PrimitiveVariable::Vertex;
	}
	else if( interpolationMatches( primType, result, PrimitiveVariable::Varying ) )
	{
		result = PrimitiveVariable::Varying;
	}

	return result;
}

// Set up indices on the destination matching the source indices ( includes handling converting interpolations ).
// Note that this only handles interpolations used by mergePrimitives ( ie. only promotion to more specific
// interpolations, like Vertex -> FaceVarying, but it cann't do averaging ).
void copyIndices(
	const std::vector<int> *sourceIndices, int *destIndices,
	IECoreScene::TypeId primTypeId,
	PrimitiveVariable::Interpolation sourceInterp, PrimitiveVariable::Interpolation destInterp,
	size_t numIndices, size_t dataStart,
	const Primitive *sourcePrim
)
{
	// Helper function that translates from an index in the source data to an index in the destination
	// data, based on the data offset, and the source indices ( if present )
	auto translateIndex = [sourceIndices, dataStart]( int j ){
		return sourceIndices ? dataStart + (*sourceIndices)[j] : dataStart + j;
	};

	if( interpolationMatches( primTypeId, sourceInterp, destInterp ) )
	{
		// If the interpolation hasn't changed, we don't need to anything special, just translate
		// each index.
		for( size_t j = 0; j < numIndices; j++ )
		{
			*(destIndices++) = translateIndex( j );
		}
	}
	else if( sourceInterp == PrimitiveVariable::Constant )
	{
		// Constant variables aren't stored as vectors, so they can't have been indexed to start,
		// just set all the output indices to the one element of output data.
		for( size_t j = 0; j < numIndices; j++ )
		{
			*(destIndices++) = dataStart;
		}
	}
	else if( sourceInterp == PrimitiveVariable::Uniform )
	{
		if( primTypeId == IECoreScene::MeshPrimitiveTypeId )
		{
			// On a mesh, if you combine a Uniform with anything it doesn't match, then it gets
			// promoted to FaceVarying.
			assert( destInterp == PrimitiveVariable::FaceVarying );

			const MeshPrimitive *sourceMesh = IECore::runTimeCast< const MeshPrimitive >( sourcePrim );
			const std::vector<int> &sourceVerticesPerFace = sourceMesh->verticesPerFace()->readable();

			int sourceI = 0;
			for( int numVerts : sourceVerticesPerFace )
			{
				for( int k = 0; k < numVerts; k++ )
				{
					*(destIndices++) = translateIndex( sourceI );
				}
				sourceI++;
			}
		}
		else if( primTypeId == IECoreScene::CurvesPrimitiveTypeId )
		{
			const CurvesPrimitive *sourceCurves = IECore::runTimeCast< const CurvesPrimitive >( sourcePrim );

			if( destInterp == PrimitiveVariable::Vertex )
			{
				const std::vector<int> &sourceVerticesPerCurve = sourceCurves->verticesPerCurve()->readable();
				int sourceI = 0;
				for( int numVerts : sourceVerticesPerCurve )
				{
					for( int k = 0; k < numVerts; k++ )
					{
						*(destIndices++) = translateIndex( sourceI );
					}
					sourceI++;
				}
			}
			else
			{
				assert( destInterp == PrimitiveVariable::Varying );
				int sourceI = 0;
				size_t sourceNumCurves = sourceCurves->numCurves();
				for( size_t i = 0; i < sourceNumCurves; i++ )
				{
					int numVarying = sourceCurves->variableSize( PrimitiveVariable::Varying, i );
					for( int k = 0; k < numVarying; k++ )
					{
						*(destIndices++) = translateIndex( sourceI );
					}
					sourceI++;
				}
			}
		}
		else
		{
			int constantIndex = translateIndex( 0 );
			for( size_t j = 0; j < numIndices; j++ )
			{
				*(destIndices++) = constantIndex;
			}
		}
	}
	else
	{
		// The only time we convert to a non-matching interpolation from something that isn't uniform,
		// is when promototing Vertex primvars on meshes.
		assert( destInterp == PrimitiveVariable::FaceVarying );
		assert( primTypeId == IECoreScene::MeshPrimitiveTypeId );
		assert( sourceInterp == PrimitiveVariable::Vertex || sourceInterp == PrimitiveVariable::Varying );

		const MeshPrimitive *sourceMesh = IECore::runTimeCast< const MeshPrimitive >( sourcePrim );
		const std::vector<int> &sourceVertexIds = sourceMesh->vertexIds()->readable();

		for( size_t j = 0; j < numIndices; j++ )
		{
			*(destIndices++) = translateIndex( sourceVertexIds[ j ] );
		}
	}
}


} // namespace


void PrimitiveAlgo::transformPrimitive(
	IECoreScene::Primitive &primitive, Imath::M44f matrix,
	const IECore::Canceller *canceller
)
{
	if( matrix == Imath::M44f() )
	{
		// Early out for identity matrix
		return;
	}

	Imath::M44f normalMatrix = normalTransform( matrix );

	for( const auto &[name, var] : primitive.variables )
	{
		Canceller::check( canceller );
		V3fVectorData *vecVar = IECore::runTimeCast<V3fVectorData>( var.data.get() );
		V3fData *vecConstVar = IECore::runTimeCast<V3fData>( var.data.get() );
		if( !vecVar && !vecConstVar )
		{
			continue;
		}

		GeometricData::Interpretation interp = vecVar ? vecVar->getInterpretation() : vecConstVar->getInterpretation();
		if( !(
			interp == GeometricData::Interpretation::Point ||
			interp == GeometricData::Interpretation::Vector ||
			interp == GeometricData::Interpretation::Normal
		) )
		{
			continue;
		}

		if( vecVar )
		{
			for( Imath::V3f &i : vecVar->writable() )
			{
				Canceller::check( canceller );
				transformPrimVarValue( i, i, matrix, normalMatrix, interp );
			}
		}
		else
		{
			// Fairly weird corner case, but technically Constant primvars could need transforming too
			transformPrimVarValue( vecConstVar->writable(), vecConstVar->writable(), matrix, normalMatrix, interp );
		}
	}
}

IECoreScene::PrimitivePtr PrimitiveAlgo::mergePrimitives(
	const std::vector< std::pair< const IECoreScene::Primitive*, Imath::M44f > > &primitives,
	const IECore::Canceller *canceller
)
{
	IECoreScene::TypeId resultTypeId = (IECoreScene::TypeId)IECore::InvalidTypeId;

	// Mesh specific globals
	std::string resultMeshInterpolation;
	IECore::InternedString resultMeshInterpolateBound;
	IECore::InternedString resultMeshFaceVaryingLI;
	IECore::InternedString resultMeshTriangleSub;

	// Curve specific globals
	const CubicBasisf invalidBasis( Imath::M44f( 0.0f ), 0 );
	CubicBasisf resultCurvesBasis( invalidBasis );
	bool resultCurvesPeriodic( false );

	// Data we need to store for each primvar we output
	struct PrimVarInfo
	{
		PrimVarInfo( PrimitiveVariable::Interpolation interpol, IECore::TypeId t, GeometricData::Interpretation interpretation, int numMeshes )
			: interpolation( interpol ),
			typeId( t ), interpret( interpretation ), interpretInvalid( false ), indexed( false ),
			numData( numMeshes, 0 )
		{
		}

		// Interpolation is set to Invalid if a primitive variable is being ignored.
		PrimitiveVariable::Interpolation interpolation;

		// Need to track typeId so we can make sure everything matches
		IECore::TypeId typeId;

		GeometricData::Interpretation interpret;
		bool interpretInvalid;

		// The only case where we don't index the output is if all the input interpolations match, and none
		// of the inputs are indexed - hopefully this is the common case though.
		bool indexed;

		// We need to collect the data size before we can allocate the output primvars
		std::vector<unsigned int> numData;
		std::vector<unsigned int> accumDataSizes;
	};

	std::unordered_map< IECore::InternedString, PrimVarInfo > varInfos;

	//
	// Before we can even start counting the sizes of things, we need to gather information about what
	// kinds of primitives and primvars we're dealing with.
	//

	for( unsigned int i = 0; i < primitives.size(); i++ )
	{
		if( !primitives[i].first )
		{
			throw IECore::Exception( "Cannot merge null Primitive" );
		}

		IECoreScene::TypeId primTypeId = (IECoreScene::TypeId)primitives[i].first->typeId();
		if( resultTypeId == (IECoreScene::TypeId)IECore::InvalidTypeId )
		{
			// Initializing for first primitive
			if( !(
				primTypeId == IECoreScene::PointsPrimitiveTypeId ||
				primTypeId == IECoreScene::MeshPrimitiveTypeId ||
				primTypeId == IECoreScene::CurvesPrimitiveTypeId
			) )
			{
				throw IECore::Exception( fmt::format(
					"Unsupported Primitive type for merging: {}", primitives[i].first->typeName()
				) );
			}

			resultTypeId = primTypeId;

			if( resultTypeId == IECoreScene::MeshPrimitiveTypeId )
			{
				const MeshPrimitive *sourceMesh = static_cast< const MeshPrimitive *>( primitives[i].first );
				resultMeshInterpolation = sourceMesh->interpolation();
				resultMeshInterpolateBound = sourceMesh->getInterpolateBoundary();
				resultMeshFaceVaryingLI = sourceMesh->getFaceVaryingLinearInterpolation();
				resultMeshTriangleSub = sourceMesh->getTriangleSubdivisionRule();
			}
			else if( resultTypeId == IECoreScene::CurvesPrimitiveTypeId )
			{
				const CurvesPrimitive *sourceCurves = static_cast< const CurvesPrimitive *>( primitives[i].first );
				resultCurvesBasis = sourceCurves->basis();
				resultCurvesPeriodic = sourceCurves->periodic();
			}
		}
		else
		{
			// We already have a primitive, so the types must match
			if( primTypeId != resultTypeId )
			{
				throw IECore::Exception( fmt::format(
					"Primitive type mismatch: {} != {}",
					primitives[i].first->typeName(), RunTimeTyped::typeNameFromTypeId( (IECore::TypeId) resultTypeId )
				) );
			}

			// Check that the primitive type specific globals match. These globals all have a reasonable default
			// value, so we don't throw an exception if they don't match, we just emit a warning, and use the
			// default value.
			if( resultTypeId == IECoreScene::MeshPrimitiveTypeId )
			{
				const MeshPrimitive *sourceMesh = static_cast< const MeshPrimitive *>( primitives[i].first );

				if(
					resultMeshInterpolation != "" &&
					sourceMesh->interpolation() != resultMeshInterpolation
				)
				{
					msg( Msg::Warning, "mergePrimitives",
						fmt::format(
							"Ignoring mismatch between mesh interpolations {} and {} and defaulting to linear",
							resultMeshInterpolation, sourceMesh->interpolation()
						)
					);
					resultMeshInterpolation = "";
				}

				if(
					resultMeshInterpolateBound != "" &&
					sourceMesh->getInterpolateBoundary() != resultMeshInterpolateBound
				)
				{
					msg( Msg::Warning, "mergePrimitives",
						fmt::format(
							"Ignoring mismatch between mesh interpolate bound {} and {} and defaulting to edgeAndCorner",
							resultMeshInterpolateBound.string(), sourceMesh->getInterpolateBoundary().string()
						)
					);
					resultMeshInterpolateBound = "";
				}

				if(
					resultMeshFaceVaryingLI != "" &&
					sourceMesh->getFaceVaryingLinearInterpolation() != resultMeshFaceVaryingLI
				)
				{
					msg( Msg::Warning, "mergePrimitives",
						fmt::format(
							"Ignoring mismatch between mesh face varying linear interpolation {} and {} and defaulting to cornersPlus1",
							resultMeshFaceVaryingLI.string(), sourceMesh->getFaceVaryingLinearInterpolation().string()
						)
					);
					resultMeshFaceVaryingLI = "";
				}

				if(
					resultMeshTriangleSub != "" &&
					sourceMesh->getTriangleSubdivisionRule() != resultMeshTriangleSub
				)
				{
					msg( Msg::Warning, "mergePrimitives",
						fmt::format(
							"Ignoring mismatch between mesh triangle subdivision rule {} and {} and defaulting to catmullClark",
							resultMeshTriangleSub.string(), sourceMesh->getTriangleSubdivisionRule().string()
						)
					);
					resultMeshTriangleSub = "";
				}

			}
			else if( resultTypeId == IECoreScene::CurvesPrimitiveTypeId )
			{
				const CurvesPrimitive *sourceCurves = static_cast< const CurvesPrimitive *>( primitives[i].first );
				CubicBasisf basis = sourceCurves->basis();
				bool periodic = sourceCurves->periodic();
				if( periodic != resultCurvesPeriodic )
				{
					throw IECore::Exception( "Cannot merge periodic and non-periodic curves" );
				}

				if(
					resultCurvesBasis != invalidBasis &&
					basis != resultCurvesBasis
				)
				{
					msg( Msg::Warning, "mergePrimitives",
						"Ignoring mismatch in curve basis and defaulting to linear"
					);
					resultCurvesBasis = invalidBasis;
				}
			}
		}

		// Process all the primvars for this primitive, adding new entries to the varInfo list, or
		// checking that existing entries match correctly
		for( const auto &[name, var] : primitives[i].first->variables )
		{
			GeometricData::Interpretation interpret = IECore::getGeometricInterpretation( var.data.get() );

			IECore::TypeId varTypeId = var.data->typeId();

			if( var.interpolation == PrimitiveVariable::Constant )
			{
				varTypeId = vectorDataTypeFromDataType( var.data.get() );
			}

			PrimVarInfo &varInfo = varInfos.try_emplace( name, var.interpolation, varTypeId, interpret, primitives.size() ).first->second;

			if( varInfo.interpolation == PrimitiveVariable::Invalid )
			{
				continue;
			}

			if( varTypeId == IECore::InvalidTypeId )
			{
				msg( Msg::Warning, "mergePrimitives",
					fmt::format(
						"Discarding variable \"{}\" - Cannot promote Constant primitive variable of type \"{}\".",
						std::string( name ), var.data->typeName()
					)
				);
				varInfo.interpolation = PrimitiveVariable::Invalid;
				continue;
			}

			if( varInfo.typeId != varTypeId )
			{
				msg( Msg::Warning, "mergePrimitives",
					fmt::format(
						"Discarding variable \"{}\" - types don't match: \"{}\" and \"{}\"",
						name,
						IECore::RunTimeTyped::typeNameFromTypeId( varInfo.typeId ),
						IECore::RunTimeTyped::typeNameFromTypeId( var.data->typeId() )
					)
				);
				varInfo.interpolation = PrimitiveVariable::Invalid;
				continue;
			}

			if( !varInfo.interpretInvalid )
			{
				if( interpret != varInfo.interpret )
				{
					varInfo.interpret = GeometricData::Interpretation::Numeric;
					varInfo.interpretInvalid = true;
					msg( Msg::Warning, "mergePrimitives",
						fmt::format(
							"Interpretation mismatch for primitive variable \"{}\", defaulting to \"Numeric\"", name
						)
					);
				}
			}

			// For meshes and curves, the interpolation we need depends on the interpolations of all the variables
			// being combined.
			varInfo.interpolation = mergeInterpolations( resultTypeId, varInfo.interpolation, var.interpolation, name );
		}
	}

	//
	// Now loop over variables collecting the information we'll need to allocate them.
	//

	for( auto &[name, varInfo] : varInfos )
	{
		if( varInfo.interpolation == PrimitiveVariable::Invalid )
		{
			continue;
		}

		// If we've processed all primitives, and this var is still just a Constant, promote it to at least
		// Uniform, so we can represent different values from different primitives.
		if( varInfo.interpolation == PrimitiveVariable::Constant )
		{
			varInfo.interpolation = PrimitiveVariable::Uniform;
		}


		// We also need to count the amount of data for this primvar contributed by each primitive.
		for( unsigned int i = 0; i < primitives.size(); i++ )
		{

			auto it = primitives[i].first->variables.find( name );

			if( it == primitives[i].first->variables.end() )
			{
				// This primitive doesn't have this primvar, we'll just write one data element
				// that will be left uninitialized.
				// Note : It's probably arguable what is most correct here ... is it unexpected that a var that
				// usually isn't indexed would become indexed because one prim is missing it? But there is an
				// efficiency gain in not storing the zero value repeatedly ( in any case where the data type is
				// more than 4 bytes ). I've currently gone with indexing it because it feels simplest to
				// implement - we need to make this work for the indexed case, so it's easy to just always use
				// the indexed case.
				varInfo.numData[i] = 1;
				varInfo.indexed = true;
				continue;
			}

			varInfo.numData[i] = IECore::size( it->second.data.get() );

			// Only if everything is simple and matches can we skip outputting indices ( though this
			// is hopefully the most common case )
			if( it->second.indices || !interpolationMatches( resultTypeId, it->second.interpolation, varInfo.interpolation ) )
			{
				varInfo.indexed = true;
			}
		}
	}

	//
	// Prepare count and offset lists for every interpolation type ( simpler than doing an extra query over
	// all variables to collect which interpolations are used ).
	//

	// There isn't a MaxInterpolation enum, but we don't expect this list to change, and can double check
	// the current maximum
	const int numInterpolations = 6;
	assert( PrimitiveVariable::Constant < numInterpolations );
	assert( PrimitiveVariable::Uniform < numInterpolations );
	assert( PrimitiveVariable::Vertex < numInterpolations );
	assert( PrimitiveVariable::Varying < numInterpolations );
	assert( PrimitiveVariable::FaceVarying < numInterpolations );

	std::vector< std::vector<int> > countInterpolation( numInterpolations );
	std::vector< int > totalInterpolation( numInterpolations );
	std::vector< std::vector<int> > accumInterpolation( numInterpolations );

	for( int interpolation = 0; interpolation < numInterpolations; interpolation++ )
	{
		int accum = 0;
		countInterpolation[interpolation].reserve( primitives.size() );
		accumInterpolation[interpolation].reserve( primitives.size() );
		for( unsigned int i = 0; i < primitives.size(); i++ )
		{
			countInterpolation[interpolation].push_back( primitives[i].first->variableSize( ((PrimitiveVariable::Interpolation)interpolation) ) );
			accumInterpolation[interpolation][i] = accum;
			accum += countInterpolation[interpolation].back();
		}
		totalInterpolation[interpolation] = accum;
	}

	//
	// Allocate the result, together with any topology information needed ( and optionally crease data in the case of
	// meshes.
	//

	IECoreScene::PrimitivePtr result;

	IECoreScene::MeshPrimitivePtr resultMesh;
	IntVectorDataPtr resultVerticesPerFaceData;
	IntVectorDataPtr resultVertexIdsData;

	std::vector<int> countCorners;
	std::vector<int> countCreases;
	std::vector<int> countCreaseIds;
	std::vector< int > accumCorners;
	std::vector< int > accumCreases;
	std::vector< int > accumCreaseIds;

	IntVectorDataPtr resultCornerIdsData;
	FloatVectorDataPtr resultCornerSharpnessesData;
	IntVectorDataPtr resultCreaseLengthsData;
	IntVectorDataPtr resultCreaseIdsData;
	FloatVectorDataPtr resultCreaseSharpnessesData;

	IECoreScene::CurvesPrimitivePtr resultCurves;
	IntVectorDataPtr resultVerticesPerCurveData;

	if( resultTypeId == IECoreScene::MeshPrimitiveTypeId )
	{
		resultMesh = new IECoreScene::MeshPrimitive();

		resultVerticesPerFaceData = new IntVectorData;
		resultVertexIdsData = new IntVectorData;

		resultVerticesPerFaceData->writable().resize( totalInterpolation[ PrimitiveVariable::Uniform ] );
		resultVertexIdsData->writable().resize( totalInterpolation[ PrimitiveVariable::FaceVarying ] );

		int totalAccumCorners = 0;
		int totalAccumCreases = 0;
		int totalAccumCreaseIds = 0;

		countCorners.reserve( primitives.size() );
		countCreases.reserve( primitives.size() );
		countCreaseIds.reserve( primitives.size() );
		accumCorners.reserve( primitives.size() );
		accumCreases.reserve( primitives.size() );
		accumCreaseIds.reserve( primitives.size() );

		for( unsigned int i = 0; i < primitives.size(); i++ )
		{
			const MeshPrimitive *p = static_cast< const MeshPrimitive *>( primitives[i].first );

			countCorners.push_back( p->cornerIds()->readable().size() );
			accumCorners.push_back( totalAccumCorners );
			totalAccumCorners += countCorners.back();

			countCreases.push_back( p->creaseLengths()->readable().size() );
			accumCreases.push_back( totalAccumCreases );
			totalAccumCreases += countCreases.back();

			countCreaseIds.push_back( p->creaseIds()->readable().size() );
			accumCreaseIds.push_back( totalAccumCreaseIds );
			totalAccumCreaseIds += countCreaseIds.back();
		}

		if( totalAccumCorners )
		{
			resultCornerIdsData = new IntVectorData;
			resultCornerIdsData->writable().resize( totalAccumCorners );
			resultCornerSharpnessesData = new FloatVectorData;
			resultCornerSharpnessesData->writable().resize( totalAccumCorners );
		}

		if( totalAccumCreases )
		{
			resultCreaseLengthsData = new IntVectorData;
			resultCreaseLengthsData->writable().resize( totalAccumCreases );
			resultCreaseSharpnessesData = new FloatVectorData;
			resultCreaseSharpnessesData->writable().resize( totalAccumCreases );
			resultCreaseIdsData = new IntVectorData;
			resultCreaseIdsData->writable().resize( totalAccumCreaseIds );
		}

		result = resultMesh;
	}
	else if( resultTypeId == IECoreScene::CurvesPrimitiveTypeId )
	{
		resultCurves = new IECoreScene::CurvesPrimitive();
		result = resultCurves;

		resultVerticesPerCurveData = new IntVectorData;
		resultVerticesPerCurveData->writable().resize( totalInterpolation[ PrimitiveVariable::Uniform ] );
	}
	else
	{
		result = new IECoreScene::PointsPrimitive( totalInterpolation[ PrimitiveVariable::Vertex ] );
	}

	//
	// Allocate storage for the primitives variables
	//

	for( auto &[name, varInfo] : varInfos )
	{
		if( varInfo.interpolation == PrimitiveVariable::Invalid )
		{
			continue;
		}

		varInfo.accumDataSizes.reserve( varInfo.numData.size() );
		size_t accumDataSize = 0;
		for( unsigned int i : varInfo.numData )
		{
			varInfo.accumDataSizes.push_back( accumDataSize );
			accumDataSize += i;
		}

		PrimitiveVariable &p = result->variables.emplace(name, PrimitiveVariable() ).first->second;

		p.data = IECore::runTimeCast<Data>( IECore::Object::create( varInfo.typeId ) );

		IECore::setGeometricInterpretation( p.data.get(), varInfo.interpret );
		p.interpolation = varInfo.interpolation;
		Canceller::check( canceller );
		dataResize( p.data.get(), accumDataSize );

		if( varInfo.indexed )
		{
			p.indices = new IntVectorData();
			Canceller::check( canceller );
			p.indices->writable().resize( totalInterpolation[ varInfo.interpolation ] );
		}
	}

	//
	// Now a big parallel loop where we do the majority of actual work - copying all the primvar and topology
	// data to the destination.
	//

	tbb::task_group_context taskGroupContext( tbb::task_group_context::isolated );

	tbb::parallel_for(
		tbb::blocked_range<size_t>( 0, primitives.size() ),
		[&]( tbb::blocked_range<size_t> &range )
		{
			for( size_t i = range.begin(); i != range.end(); i++ )
			{
				const Primitive &sourcePrim = *primitives[i].first;

				const Imath::M44f &matrix = primitives[i].second;
				const Imath::M44f normalMatrix = normalTransform( matrix );

				// Copy the data ( and indices ) for each prim var for this primitive into
				// the destination primvar.
				for( auto &[name, varInfo] : varInfos )
				{
					if( varInfo.interpolation == PrimitiveVariable::Invalid )
					{
						continue;
					}

					PrimitiveVariable &destVar = result->variables.find( name )->second;

					const size_t numIndices = countInterpolation[ varInfo.interpolation ][i];
					const size_t startIndex = accumInterpolation[ varInfo.interpolation ][i];
					const size_t dataStart = varInfo.accumDataSizes[i];


					auto it = sourcePrim.variables.find( name );
					if( it == sourcePrim.variables.end() || it->second.interpolation == PrimitiveVariable::Invalid )
					{
						// No matching data found in this primitive for this primvar

						// We don't currently have a way to suppress zero-initialization of the data, so
						// we don't need to initialize that here

						if( varInfo.indexed )
						{
							Canceller::check( canceller );

							// We always leave one data element for primitives that don't have the relevant
							// primvar, so just write out all indices pointing to that element.
							int *destIndices = &destVar.indices->writable()[ startIndex ];
							for( size_t j = 0; j < numIndices; j++ )
							{
								*(destIndices++) = dataStart;
							}
						}
					}
					else
					{
						const PrimitiveVariable &sourceVar = it->second;

						Canceller::check( canceller );
						copyElements( sourceVar.data.get(), 0, destVar.data.get(), dataStart, varInfo.numData[i], matrix, normalMatrix );

						if( varInfo.indexed )
						{
							Canceller::check( canceller );
							int *destIndices = &destVar.indices->writable()[ startIndex ];

							copyIndices(
								sourceVar.indices ? &sourceVar.indices->readable() : nullptr, destIndices,
								resultTypeId, sourceVar.interpolation, varInfo.interpolation,
								numIndices, dataStart,
								&sourcePrim
							);
						}
					}
				}

				// Copy the topology information for this primitive into the result topology information

				if( resultTypeId == IECoreScene::MeshPrimitiveTypeId )
				{
					const MeshPrimitive *sourceMesh = IECore::runTimeCast< const MeshPrimitive >( &sourcePrim );

					int startUniform = accumInterpolation[ PrimitiveVariable::Uniform ][i];
					int numUniform = countInterpolation[ PrimitiveVariable::Uniform ][i];
					int startVertex = accumInterpolation[ PrimitiveVariable::Vertex ][i];
					int startFaceVarying = accumInterpolation[ PrimitiveVariable::FaceVarying ][i];
					int numFaceVarying = countInterpolation[ PrimitiveVariable::FaceVarying ][i];

					const int *sourceVerticesPerFace = &sourceMesh->verticesPerFace()->readable()[0];
					int *resultVerticesPerFace = &resultVerticesPerFaceData->writable()[ startUniform ];
					Canceller::check( canceller );
					for( int j = 0; j < numUniform; j++ )
					{
						*(resultVerticesPerFace++) = *(sourceVerticesPerFace++);
					}

					const int* sourceVertexIds = &sourceMesh->vertexIds()->readable()[0];
					int *resultVertexIds = &resultVertexIdsData->writable()[startFaceVarying];
					Canceller::check( canceller );
					for( int j = 0; j < numFaceVarying; j++ )
					{
						*(resultVertexIds++) = *(sourceVertexIds++) + startVertex;
					}

					if( resultCornerIdsData )
					{
						const int *sourceCornerIds = &sourceMesh->cornerIds()->readable()[0];
						const float *sourceCornerSharpnesses = &sourceMesh->cornerSharpnesses()->readable()[0];
						int *resultCornerIds = &resultCornerIdsData->writable()[ accumCorners[i] ];
						float *resultCornerSharpnesses = &resultCornerSharpnessesData->writable()[ accumCorners[i] ];
						Canceller::check( canceller );
						for( int j = 0; j < countCorners[i]; j++ )
						{
							*(resultCornerIds++) = *(sourceCornerIds++) + startVertex;
							*(resultCornerSharpnesses++) = *(sourceCornerSharpnesses++);
						}
					}

					if( resultCreaseLengthsData )
					{
						const int *sourceCreaseLengths = &sourceMesh->creaseLengths()->readable()[0];
						const float *sourceCreaseSharpnesses = &sourceMesh->creaseSharpnesses()->readable()[0];
						int *resultCreaseLengths = &resultCreaseLengthsData->writable()[accumCreases[i]];
						float *resultCreaseSharpnesses = &resultCreaseSharpnessesData->writable()[accumCreases[i]];
						Canceller::check( canceller );
						for( int j = 0; j < countCreases[i]; j++ )
						{
							*(resultCreaseLengths++) = *(sourceCreaseLengths++);
							*(resultCreaseSharpnesses++) = *(sourceCreaseSharpnesses++);
						}

						const int *sourceCreaseIds = &sourceMesh->creaseIds()->readable()[0];
						int *resultCreaseIds = &resultCreaseIdsData->writable()[accumCreaseIds[i]];
						Canceller::check( canceller );
						for( int j = 0; j < countCreaseIds[i]; j++ )
						{
							*(resultCreaseIds++) = *(sourceCreaseIds++) + startVertex;
						}
					}
				}
				else if( resultTypeId == IECoreScene::CurvesPrimitiveTypeId )
				{
					const CurvesPrimitive *sourceCurves = IECore::runTimeCast< const CurvesPrimitive >( &sourcePrim );

					int startUniform = accumInterpolation[ PrimitiveVariable::Uniform ][i];
					int numUniform = countInterpolation[ PrimitiveVariable::Uniform ][i];

					int *resultVerticesPerCurve = &resultVerticesPerCurveData->writable()[ startUniform ];
					const int *sourceVerticesPerCurve = &sourceCurves->verticesPerCurve()->readable()[0];
					Canceller::check( canceller );
					for( int j = 0; j < numUniform; j++ )
					{
						*(resultVerticesPerCurve++) = *(sourceVerticesPerCurve++);
					}
				}
			}
		},
		tbb::auto_partitioner(),
		taskGroupContext
	);

	// Set topology and other primitive type specific globals

	if( resultMesh )
	{
		if( resultMeshInterpolation == "" )
		{
			resultMeshInterpolation = "linear";
		}

		resultMesh->setTopologyUnchecked( resultVerticesPerFaceData, resultVertexIdsData, totalInterpolation[ PrimitiveVariable::Vertex ], resultMeshInterpolation );

		if( resultMeshInterpolateBound == "" )
		{
			resultMeshInterpolateBound = MeshPrimitive::interpolateBoundaryEdgeAndCorner;
		}
		resultMesh->setInterpolateBoundary( resultMeshInterpolateBound );

		if( resultMeshFaceVaryingLI == "" )
		{
			resultMeshFaceVaryingLI = MeshPrimitive::faceVaryingLinearInterpolationCornersPlus1;
		}
		resultMesh->setFaceVaryingLinearInterpolation( resultMeshFaceVaryingLI );

		if( resultMeshTriangleSub == "" )
		{
			resultMeshTriangleSub = MeshPrimitive::triangleSubdivisionRuleCatmullClark;
		}
		resultMesh->setTriangleSubdivisionRule( resultMeshTriangleSub );

		if( resultCornerIdsData )
		{
			resultMesh->setCorners( resultCornerIdsData.get(), resultCornerSharpnessesData.get() );
		}

		if( resultCreaseLengthsData )
		{
			resultMesh->setCreases(
				resultCreaseLengthsData.get(), resultCreaseIdsData.get(), resultCreaseSharpnessesData.get()
			);
		}
	}
	else if( resultCurves )
	{
		if( resultCurvesBasis == invalidBasis )
		{
			resultCurvesBasis = CubicBasisf::linear();
		}

		resultCurves->setTopology( resultVerticesPerCurveData, resultCurvesBasis, resultCurvesPeriodic );
	}

	return result;
}
