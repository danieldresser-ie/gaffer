//////////////////////////////////////////////////////////////////////////
//
//  Copyright (c) 2020, Image Engine Design Inc. All rights reserved.
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

#include "Gaffer/Context.h"

#include "GafferImage/ImageAlgo.h"
#include "GafferImageTest/DeepResampleConstraints.h"
#include "GafferImage/DeepAlgo.h"

using namespace std;
using namespace Imath;
using namespace IECore;
using namespace Gaffer;
using namespace GafferImage;
using namespace GafferImageTest;

GAFFER_GRAPHCOMPONENT_DEFINE_TYPE( DeepResampleConstraints );

size_t DeepResampleConstraints::g_firstPlugIndex = 0;

DeepResampleConstraints::DeepResampleConstraints( const std::string &name )
	:	ImageProcessor( name )
{
	storeIndexOfNextChild( g_firstPlugIndex );

	addChild( new FloatPlug( "alphaTolerance", Gaffer::Plug::In, 0.01 ) );
	addChild( new FloatPlug( "depthTolerance", Gaffer::Plug::In, 0.01 ) );
	addChild( new BoolPlug( "upper", Gaffer::Plug::In, false ) );

	addChild( new CompoundObjectPlug( "__constraints", Gaffer::Plug::Out, new IECore::CompoundObject() ) );

	// We don't ever want to change these, so we make pass-through connections.
	outPlug()->dataWindowPlug()->setInput( inPlug()->dataWindowPlug() );
	outPlug()->formatPlug()->setInput( inPlug()->formatPlug() );
	outPlug()->metadataPlug()->setInput( inPlug()->metadataPlug() );
	outPlug()->deepPlug()->setInput( inPlug()->deepPlug() );
}

DeepResampleConstraints::~DeepResampleConstraints()
{
}

Gaffer::FloatPlug *DeepResampleConstraints::alphaTolerancePlug()
{
	return getChild<FloatPlug>( g_firstPlugIndex + 0 );
}

const Gaffer::FloatPlug *DeepResampleConstraints::alphaTolerancePlug() const
{
	return getChild<FloatPlug>( g_firstPlugIndex + 0 );
}

Gaffer::FloatPlug *DeepResampleConstraints::depthTolerancePlug()
{
	return getChild<FloatPlug>( g_firstPlugIndex + 1 );
}

const Gaffer::FloatPlug *DeepResampleConstraints::depthTolerancePlug() const
{
	return getChild<FloatPlug>( g_firstPlugIndex + 1 );
}

Gaffer::BoolPlug *DeepResampleConstraints::upperPlug()
{
	return getChild<BoolPlug>( g_firstPlugIndex + 2 );
}

const Gaffer::BoolPlug *DeepResampleConstraints::upperPlug() const
{
	return getChild<BoolPlug>( g_firstPlugIndex + 2 );
}

Gaffer::CompoundObjectPlug *DeepResampleConstraints::constraintsPlug()
{
	return getChild<CompoundObjectPlug>( g_firstPlugIndex + 3 );
}

const Gaffer::CompoundObjectPlug *DeepResampleConstraints::constraintsPlug() const
{
	return getChild<CompoundObjectPlug>( g_firstPlugIndex + 3 );
}

void DeepResampleConstraints::affects( const Gaffer::Plug *input, AffectedPlugsContainer &outputs ) const
{
	ImageProcessor::affects( input, outputs );

	if(
		input == inPlug()->sampleOffsetsPlug() || input == inPlug()->channelNamesPlug() ||
		input == inPlug()->channelDataPlug() ||
		input == alphaTolerancePlug() || input == depthTolerancePlug() || input == upperPlug()
	)
	{
		outputs.push_back( constraintsPlug() );
	}

	if( input == constraintsPlug() )
	{
		outputs.push_back( outPlug()->channelDataPlug() );
	}

	if( input == constraintsPlug() )
	{
		outputs.push_back( outPlug()->sampleOffsetsPlug() );
	}
}

void DeepResampleConstraints::hash( const Gaffer::ValuePlug *output, const Gaffer::Context *context, IECore::MurmurHash &h ) const
{
	ImageProcessor::hash( output, context, h );

	if( output != constraintsPlug() )
	{
		return;
	}


	alphaTolerancePlug()->hash( h );
	depthTolerancePlug()->hash( h );
	upperPlug()->hash( h );

	ConstStringVectorDataPtr channelNamesData = inPlug()->channelNames();
	std::vector<string> channelNames = channelNamesData->readable();

	inPlug()->sampleOffsetsPlug()->hash( h );

	if(
		ImageAlgo::channelExists( channelNames, "A" ) &&
		ImageAlgo::channelExists( channelNames, "Z" ) 
	)
	{
		ImagePlug::ChannelDataScope channelScope( context );
		channelScope.setChannelName( "A" );
		inPlug()->channelDataPlug()->hash( h );

		channelScope.setChannelName( "Z" );
		inPlug()->channelDataPlug()->hash( h );

		if( ImageAlgo::channelExists( channelNames, "ZBack" ) )
		{
			inPlug()->channelDataPlug()->hash( h );
		}
	}
}

void DeepResampleConstraints::compute( Gaffer::ValuePlug *output, const Gaffer::Context *context ) const
{
	ImageProcessor::compute( output, context );

	if( output != constraintsPlug() )
	{
		return;
	}

	float alphaTolerance = alphaTolerancePlug()->getValue();
	float depthTolerance = depthTolerancePlug()->getValue();
	float upper = upperPlug()->getValue();

	ConstIntVectorDataPtr sampleOffsetsData = inPlug()->sampleOffsetsPlug()->getValue();
	const std::vector<int> &sampleOffsets = sampleOffsetsData->readable();

	CompoundObjectPtr constraintsData = new CompoundObject();

	ConstStringVectorDataPtr channelNamesData = inPlug()->channelNames();
	std::vector<string> channelNames = channelNamesData->readable();

	if(
		ImageAlgo::channelExists( channelNames, "A" ) &&
		ImageAlgo::channelExists( channelNames, "Z" )
	)
	{
		ImagePlug::ChannelDataScope channelScope( context );
		channelScope.setChannelName( "A" );
		ConstFloatVectorDataPtr alphaData = inPlug()->channelDataPlug()->getValue();
		const std::vector<float> &alpha = alphaData->readable();

		channelScope.setChannelName( "Z" );
		ConstFloatVectorDataPtr zData = inPlug()->channelDataPlug()->getValue();
		const std::vector<float> &z = zData->readable();

		ConstFloatVectorDataPtr zBackData;
		if( ImageAlgo::channelExists( channelNames, "ZBack" ) )
		{
			channelScope.setChannelName( "ZBack" );
			zBackData = inPlug()->channelDataPlug()->getValue();
		}
		else
		{
			zBackData = zData;
		}
		const std::vector<float> &zBack = zBackData->readable();

		IntVectorDataPtr outSampleOffsetsData = new IntVectorData();
		std::vector<int> &outSampleOffsets = outSampleOffsetsData->writable();
		outSampleOffsets.resize( sampleOffsets.size() );


		FloatVectorDataPtr outAlphaData = new FloatVectorData();
		std::vector<float> &outAlpha = outAlphaData->writable();
		FloatVectorDataPtr outZData = new FloatVectorData();
		std::vector<float> &outZ = outZData->writable();
		FloatVectorDataPtr outZBackData = new FloatVectorData();
		std::vector<float> &outZBack = outZBackData->writable();

		//std::vector<DeepPixel> outputPixels;
		//outputPixels.resize( sampleOffsets.size() );
		int prev = 0;
		int outputCount = 0;

		std::vector<DeepAlgo::Detail::DeepConstraint> lowerConstraints;
		std::vector<DeepAlgo::Detail::DeepConstraint> upperConstraints;
		std::vector<DeepAlgo::Detail::DeepConstraint> &currentConstraints = upper ? upperConstraints : lowerConstraints;
		
		for( unsigned int i = 0; i < sampleOffsets.size(); i++ )
		{
			int index = sampleOffsets[i];
		
			DeepAlgo::Detail::debugConstraintsForPixel(
				index - prev, &alpha[prev], &z[prev], &zBack[prev],
				alphaTolerance, depthTolerance,
				lowerConstraints, upperConstraints	
			);

			int constraintsCount = 0;
			float currentAlpha = 0;
			float currentDepth = -1e8;
			for( auto &c : currentConstraints )
			{
				if( c.a != currentAlpha )
				{
					float sampleAlpha = 1 - ( 1 - c.a ) / ( 1 - currentAlpha );
					outAlpha.push_back( sampleAlpha );
					outZ.push_back( currentDepth );
					outZBack.push_back( c.z );

					currentAlpha = c.a;
					constraintsCount++;
				}
				currentDepth = c.z;
			}
			
			outputCount += constraintsCount;
			outSampleOffsets[i] = outputCount;

			prev = index;
		}

		constraintsData->members()["sampleOffsets"] = outSampleOffsetsData;
		constraintsData->members()["A"] = outAlphaData;
		constraintsData->members()["Z"] = outZData;
		constraintsData->members()["ZBack"] = outZBackData;
	}

	static_cast<CompoundObjectPlug *>( output )->setValue( constraintsData );
}

void DeepResampleConstraints::hashChannelData( const ImagePlug *output, const Gaffer::Context *context, IECore::MurmurHash &h ) const
{
	ImageProcessor::hashChannelData( output, context, h );
	const std::string &channelName = context->get<std::string>( GafferImage::ImagePlug::channelNameContextName );

	inPlug()->channelDataPlug()->hash( h );

	ImagePlug::ChannelDataScope channelScope( context );
	channelScope.remove( ImagePlug::channelNameContextName );

	constraintsPlug()->hash( h );
	h.append( channelName );
}

IECore::ConstFloatVectorDataPtr DeepResampleConstraints::computeChannelData( const std::string &channelName, const Imath::V2i &tileOrigin, const Gaffer::Context *context, const ImagePlug *parent ) const
{
	ImagePlug::ChannelDataScope channelScope( context );
	channelScope.remove( ImagePlug::channelNameContextName );

	ConstCompoundObjectPtr constraints = constraintsPlug()->getValue();

	if( !constraints->members().size() )
	{
		return ImagePlug::emptyTile();
	}

	if( channelName == "Z" )
	{
		return constraints->member<FloatVectorData>("Z");
	}
	else if( channelName == "ZBack" )
	{
		return constraints->member<FloatVectorData>("ZBack");
	}
	else
	{
		return constraints->member<FloatVectorData>("A");
	}
}

void DeepResampleConstraints::hashSampleOffsets( const ImagePlug *parent, const Gaffer::Context *context, IECore::MurmurHash &h ) const
{
	ImageProcessor::hashSampleOffsets( parent, context, h );

	constraintsPlug()->hash( h );
}

IECore::ConstIntVectorDataPtr DeepResampleConstraints::computeSampleOffsets( const Imath::V2i &tileOrigin, const Gaffer::Context *context, const ImagePlug *parent ) const
{
	ConstCompoundObjectPtr constraints = constraintsPlug()->getValue();

	if( !constraints->members().size() )
	{
		return ImagePlug::emptyTileSampleOffsets();
	}

	return constraints->member<IntVectorData>("sampleOffsets");
}

void DeepResampleConstraints::hashChannelNames( const GafferImage::ImagePlug *parent, const Gaffer::Context *context, IECore::MurmurHash &h ) const
{
	ImageProcessor::hashChannelNames( parent, context, h );
}

IECore::ConstStringVectorDataPtr DeepResampleConstraints::computeChannelNames( const Gaffer::Context *context, const ImagePlug *parent ) const
{
	return new StringVectorData( { "A", "Z", "ZBack" } );
}

