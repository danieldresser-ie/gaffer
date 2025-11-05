//////////////////////////////////////////////////////////////////////////
//
//  Copyright (c) 2013, Image Engine Design Inc. All rights reserved.
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

#include "GafferScene/Cube.h"

#include "IECoreScene/MeshPrimitive.h"

using namespace Gaffer;
using namespace GafferScene;
using namespace Imath;
using namespace IECore;
using namespace IECoreScene;

namespace {

void addCorner( const V3f &p, std::vector< V3f > &ps, std::vector< std::vector< int > > &perFaceIndices, const V3i &vertsPer )
{
	int cornerIndex  = ps.size();
	ps.push_back( p );

	perFaceIndices[ p.x != 0.0f ][
		( p.y > 0 ) * ( vertsPer.y - 1 ) +
		( p.z > 0 ) * ( vertsPer.z - 1 ) * vertsPer.y
	] = cornerIndex;
	perFaceIndices[ 2 + ( p.y != 0.0f ) ][
		( p.x > 0 ) * ( vertsPer.x - 1 ) +
		( p.z > 0 ) * ( vertsPer.z - 1 ) * vertsPer.x
	] = cornerIndex;
	perFaceIndices[ 4 + ( p.z != 0.0f ) ][
		( p.x > 0 ) * ( vertsPer.x - 1 ) +
		( p.y > 0 ) * ( vertsPer.y - 1 ) * vertsPer.x
	] = cornerIndex;
}

void addXEdge( const V3f &p, std::vector< V3f > &ps, std::vector< std::vector< int > > &perFaceIndices, const V3i &vertsPer )
{
	for( int i = 1; i < vertsPer.x - 1; i++ )
	{
		int edgeIndex = ps.size();
		ps.push_back( V3f( i / float( vertsPer.x - 1 ), p.y, p.z ) );

		perFaceIndices[ 2 + ( p.y != 0.0f ) ][ i + ( p.z > 0 ) * ( vertsPer.z - 1 ) * vertsPer.x ] = edgeIndex;
		perFaceIndices[ 4 + ( p.z != 0.0f ) ][ i + ( p.y > 0 ) * ( vertsPer.y - 1 ) * vertsPer.x ] = edgeIndex;
	}
}

void addYEdge( const V3f &p, std::vector< V3f > &ps, std::vector< std::vector< int > > &perFaceIndices, const V3i &vertsPer )
{
	for( int i = 1; i < vertsPer.y - 1; i++ )
	{
		int edgeIndex = ps.size();
		ps.push_back( V3f( p.x, i / float( vertsPer.y - 1 ), p.z ) );

		perFaceIndices[ ( p.x != 0.0f ) ][ i + ( p.z > 0 ) * ( vertsPer.z - 1 ) * vertsPer.y ] = edgeIndex;
		perFaceIndices[ 4 + ( p.z != 0.0f ) ][ i * vertsPer.x + ( p.x > 0 ) * ( vertsPer.x - 1 ) ] = edgeIndex;
	}
}

void addZEdge( const V3f &p, std::vector< V3f > &ps, std::vector< std::vector< int > > &perFaceIndices, const V3i &vertsPer )
{
	for( int i = 1; i < vertsPer.z - 1; i++ )
	{
		int edgeIndex = ps.size();
		ps.push_back( V3f( p.x, p.y, i / float( vertsPer.z - 1 ) ) );

		perFaceIndices[ ( p.x != 0.0f ) ][ i * vertsPer.y + ( p.y > 0 ) * ( vertsPer.y - 1 ) ] = edgeIndex;
		perFaceIndices[ 2 + ( p.y != 0.0f ) ][ i * vertsPer.x + ( p.x > 0 ) * ( vertsPer.x - 1 ) ] = edgeIndex;
	}
}

void addXFace( float p, std::vector< V3f > &ps, std::vector< std::vector< int > > &perFaceIndices, const V3i &vertsPer )
{
	for( int j = 1; j < vertsPer.z - 1; j++ )
	{
		for( int i = 1; i < vertsPer.y - 1; i++ )
		{
			perFaceIndices[ ( p != 0.0f ) ][ i + j * vertsPer.y ] = ps.size();
			ps.push_back( V3f( p, i / float( vertsPer.y - 1 ), j / float( vertsPer.z - 1 ) ) );
		}
	}
}

void addYFace( float p, std::vector< V3f > &ps, std::vector< std::vector< int > > &perFaceIndices, const V3i &vertsPer )
{
	for( int j = 1; j < vertsPer.z - 1; j++ )
	{
		for( int i = 1; i < vertsPer.x - 1; i++ )
		{
			perFaceIndices[ 2 + ( p != 0.0f ) ][ i + j * vertsPer.x ] = ps.size();
			ps.push_back( V3f( i / float( vertsPer.x - 1 ), p, j / float( vertsPer.z - 1 ) ) );
		}
	}
}

void addZFace( float p, std::vector< V3f > &ps, std::vector< std::vector< int > > &perFaceIndices, const V3i &vertsPer )
{
	for( int j = 1; j < vertsPer.y - 1; j++ )
	{
		for( int i = 1; i < vertsPer.x - 1; i++ )
		{
			perFaceIndices[ 4 + ( p != 0.0f ) ][ i + j * vertsPer.x ] = ps.size();
			ps.push_back( V3f( i / float( vertsPer.x - 1 ), j / float( vertsPer.y - 1 ), p ) );
		}
	}
}

MeshPrimitivePtr createDividedBox( const Box3f &b, const Imath::V3f &divisions )
{
	V3i vertsPer = divisions + V3i(1);

	std::vector< Imath::V2i > faceSizes;
	faceSizes.push_back( Imath::V2i( vertsPer[1], vertsPer[2] ) );
	faceSizes.push_back( Imath::V2i( vertsPer[1], vertsPer[2] ) );
	faceSizes.push_back( Imath::V2i( vertsPer[0], vertsPer[2] ) );
	faceSizes.push_back( Imath::V2i( vertsPer[0], vertsPer[2] ) );
	faceSizes.push_back( Imath::V2i( vertsPer[0], vertsPer[1] ) );
	faceSizes.push_back( Imath::V2i( vertsPer[0], vertsPer[1] ) );

	std::vector< std::vector< int > > perFaceIndices;

	perFaceIndices.resize( 6 );
	size_t numFaces = 0;
	for( int i = 0; i < 6; i++ )
	{
		perFaceIndices[i].resize( faceSizes[i].x * faceSizes[i].y, -1 ); // TODO - no init
		numFaces += ( faceSizes[i].x - 1 ) * ( faceSizes[i].y - 1 );
	}

	V3fVectorDataPtr pData = new V3fVectorData;
	std::vector< V3f > &ps = pData->writable();

	addCorner( V3f( 0, 0, 0 ), ps, perFaceIndices, vertsPer );
	addCorner( V3f( 1, 0, 0 ), ps, perFaceIndices, vertsPer );
	addCorner( V3f( 1, 1, 0 ), ps, perFaceIndices, vertsPer );
	addCorner( V3f( 0, 1, 0 ), ps, perFaceIndices, vertsPer );
	addCorner( V3f( 1, 0, 1 ), ps, perFaceIndices, vertsPer );
	addCorner( V3f( 1, 1, 1 ), ps, perFaceIndices, vertsPer );
	addCorner( V3f( 0, 0, 1 ), ps, perFaceIndices, vertsPer );
	addCorner( V3f( 0, 1, 1 ), ps, perFaceIndices, vertsPer );

	addXEdge( V3f( 0, 0, 0 ), ps, perFaceIndices, vertsPer );
	addXEdge( V3f( 0, 0, 1 ), ps, perFaceIndices, vertsPer );
	addXEdge( V3f( 0, 1, 0 ), ps, perFaceIndices, vertsPer );
	addXEdge( V3f( 0, 1, 1 ), ps, perFaceIndices, vertsPer );

	addYEdge( V3f( 0, 0, 0 ), ps, perFaceIndices, vertsPer );
	addYEdge( V3f( 0, 0, 1 ), ps, perFaceIndices, vertsPer );
	addYEdge( V3f( 1, 0, 0 ), ps, perFaceIndices, vertsPer );
	addYEdge( V3f( 1, 0, 1 ), ps, perFaceIndices, vertsPer );

	addZEdge( V3f( 0, 0, 0 ), ps, perFaceIndices, vertsPer );
	addZEdge( V3f( 0, 1, 0 ), ps, perFaceIndices, vertsPer );
	addZEdge( V3f( 1, 0, 0 ), ps, perFaceIndices, vertsPer );
	addZEdge( V3f( 1, 1, 0 ), ps, perFaceIndices, vertsPer );

	addXFace( 0, ps, perFaceIndices, vertsPer );
	addXFace( 1, ps, perFaceIndices, vertsPer );

	addYFace( 0, ps, perFaceIndices, vertsPer );
	addYFace( 1, ps, perFaceIndices, vertsPer );

	addZFace( 0, ps, perFaceIndices, vertsPer );
	addZFace( 1, ps, perFaceIndices, vertsPer );

   std::string interpolation = "linear";
	/*std::vector<int> verticesPerFace {
		4, 4, 4, 4, 4, 4
	};
	std::vector<int> vertexIds {
		3,2,1,0,
		1,2,5,4,
		4,5,7,6,
		6,7,3,0,
		2,3,7,5,
		0,1,4,6
	};

	p.push_back( V3f( b.min.x, b.min.y, b.min.z ) );    // 0
	p.push_back( V3f( b.max.x, b.min.y, b.min.z ) );    // 1
	p.push_back( V3f( b.max.x, b.max.y, b.min.z ) );    // 2
	p.push_back( V3f( b.min.x, b.max.y, b.min.z ) );    // 3
	p.push_back( V3f( b.max.x, b.min.y, b.max.z ) );    // 4
	p.push_back( V3f( b.max.x, b.max.y, b.max.z ) );    // 5
	p.push_back( V3f( b.min.x, b.min.y, b.max.z ) );    // 6
	p.push_back( V3f( b.min.x, b.max.y, b.max.z ) );    // 7
	*/

	for( V3f &p : ps )
	{
		p = b.min + b.size() * p ;
	}


	std::cerr << "INTERMEDIATE INDICES";
	for( size_t i = 0; i < perFaceIndices.size(); i++ )
	{
		std::cerr << "\n\n";
		for( int y = 0; y < faceSizes[i].y; y++ )
		{
			for( int x = 0; x < faceSizes[i].x; x++ )
			{
				std::cerr << perFaceIndices[i][ y * faceSizes[i].x + x ] << " ";
			}
			std::cerr << "\n";
		}
	}

	IntVectorDataPtr verticesPerFaceData = new IntVectorData();
	verticesPerFaceData->writable().resize( numFaces, 4 );

	IntVectorDataPtr vertexIdsData = new IntVectorData();
	std::vector<int> &vertexIds = vertexIdsData->writable();
	vertexIds.reserve( numFaces * 4 );

	for( size_t i = 0; i < perFaceIndices.size(); i++ )
    {
		const std::vector<int> &fi = perFaceIndices[i];
        for( int y = 0; y < faceSizes[i].y - 1; y++ )
        {
            for( int x = 0; x < faceSizes[i].x - 1; x++ )
            {
				vertexIds.push_back( fi[y * faceSizes[i].x + x] );
				vertexIds.push_back( fi[y * faceSizes[i].x + x + 1] );
				vertexIds.push_back( fi[( y + 1 ) * faceSizes[i].x + x + 1] );
				vertexIds.push_back( fi[( y + 1 ) * faceSizes[i].x + x] );
            }
        }
    }

	MeshPrimitivePtr result = new MeshPrimitive( verticesPerFaceData, vertexIdsData, interpolation, pData );



	/*V2fVectorDataPtr uvData = new V2fVectorData;
	uvData->setInterpretation( GeometricData::UV );
	std::vector<Imath::V2f> &uvs = uvData->writable();

	for( int i = 0; i < 5; i++ )
	{
		uvs.push_back( Imath::V2f( 0.375f, 0.25f * i ) );
		uvs.push_back( Imath::V2f( 0.625f, 0.25f * i ) );
	}

	for( int i = 0; i < 2; i++ )
	{
		uvs.push_back( Imath::V2f( 0.125f, 0.25f * i ) );
		uvs.push_back( Imath::V2f( 0.875f, 0.25f * i ) );
	}

	std::vector<int> uvIndices {
		4,5,7,6,
		11,13,3,1,
		1,3,2,0,
		0,2,12,10,
		5,4,2,3,
		6,7,9,8,
	};

	result->variables["uv"] = PrimitiveVariable( PrimitiveVariable::FaceVarying, uvData, new IntVectorData ( uvIndices ) );


   std::vector<Imath::V3f> normals {
		Imath::V3f( 0, 0, 1 ),
		Imath::V3f( 0, 0, -1 ),
		Imath::V3f( 0, 1, 0 ),
		Imath::V3f( 0, -1, 0 ),
		Imath::V3f( 1, 0, 0 ),
		Imath::V3f( -1, 0, 0 ),
	};

	std::vector<int> nIndices {
		1,1,1,1,
		4,4,4,4,
		0,0,0,0,
		5,5,5,5,
		2,2,2,2,
		3,3,3,3,
	};

	result->variables["N"] = PrimitiveVariable( PrimitiveVariable::FaceVarying, new V3fVectorData( normals, GeometricData::Normal ), new IntVectorData ( nIndices ) );*/

	return result;
}

} // namespace

GAFFER_NODE_DEFINE_TYPE( Cube );

size_t Cube::g_firstPlugIndex = 0;

Cube::Cube( const std::string &name )
	:	ObjectSource( name, "cube" )
{
	storeIndexOfNextChild( g_firstPlugIndex );
	addChild( new V3fPlug( "dimensions", Plug::In, V3f( 1.0f ), V3f( 0.0f ) ) );
	addChild( new V3iPlug( "divisions", Plug::In, V3i( 1 ), V3i( 1 ) ) );
}

Cube::~Cube()
{
}

Gaffer::V3fPlug *Cube::dimensionsPlug()
{
	return getChild<V3fPlug>( g_firstPlugIndex );
}

const Gaffer::V3fPlug *Cube::dimensionsPlug() const
{
	return getChild<V3fPlug>( g_firstPlugIndex );
}

Gaffer::V3iPlug *Cube::divisionsPlug()
{
	return getChild<V3iPlug>( g_firstPlugIndex + 1 );
}

const Gaffer::V3iPlug *Cube::divisionsPlug() const
{
	return getChild<V3iPlug>( g_firstPlugIndex + 1 );
}

void Cube::affects( const Plug *input, AffectedPlugsContainer &outputs ) const
{
	ObjectSource::affects( input, outputs );

	if(
		input->parent<V3fPlug>() == dimensionsPlug() ||
		input->parent<V3iPlug>() == divisionsPlug()
	)
	{
		outputs.push_back( sourcePlug() );
	}
}

void Cube::hashSource( const Gaffer::Context *context, IECore::MurmurHash &h ) const
{
	dimensionsPlug()->hash( h );
	divisionsPlug()->hash( h );
}

IECore::ConstObjectPtr Cube::computeSource( const Context *context ) const
{
	V3f dimensions = dimensionsPlug()->getValue();
	V3i divisions = divisionsPlug()->getValue();

	return createDividedBox( Box3f( -dimensions / 2.0f, dimensions / 2.0f ), divisions );
}
