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
		input->parent<V3fPlug>() == divisionsPlug()
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

   std::string interpolation = "linear";
	vector< V3f > p;
	std::vector<int> verticesPerFace {
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

	MeshPrimitivePtr result = new MeshPrimitive( new IntVectorData(verticesPerFace), new IntVectorData(vertexIds), interpolation, new V3fVectorData(p) );

	V2fVectorDataPtr uvData = new V2fVectorData;
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

	result->variables["N"] = PrimitiveVariable( PrimitiveVariable::FaceVarying, new V3fVectorData( normals, GeometricData::Normal ), new IntVectorData ( nIndices ) );

	return result;
}
