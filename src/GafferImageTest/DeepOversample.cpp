//////////////////////////////////////////////////////////////////////////
//
//  Copyright (c) 2019, Image Engine Design Inc. All rights reserved.
//  Copyright (c) 2015, Nvizible Ltd. All rights reserved.
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
#include "GafferImageTest/DeepOversample.h"

using namespace std;
using namespace Imath;
using namespace IECore;
using namespace Gaffer;
using namespace GafferImageTest;

GAFFER_GRAPHCOMPONENT_DEFINE_TYPE( DeepOversample );

size_t DeepOversample::g_firstPlugIndex = 0;

DeepOversample::DeepOversample( const std::string &name )
	:	ImageProcessor( name )
{
	storeIndexOfNextChild( g_firstPlugIndex );

	addChild( new FloatPlug( "maxSampleAlpha", Gaffer::Plug::In, 0.1 ) );

	addChild( new IntVectorDataPlug( "__subSamples", Gaffer::Plug::Out, GafferImage::ImagePlug::emptyTileSampleOffsets() ) );

	// We don't ever want to change these, so we make pass-through connections.
	outPlug()->channelNamesPlug()->setInput( inPlug()->channelNamesPlug() );
	outPlug()->dataWindowPlug()->setInput( inPlug()->dataWindowPlug() );
	outPlug()->formatPlug()->setInput( inPlug()->formatPlug() );
	outPlug()->metadataPlug()->setInput( inPlug()->metadataPlug() );
	outPlug()->deepPlug()->setInput( inPlug()->deepPlug() );
}

DeepOversample::~DeepOversample()
{
}

Gaffer::FloatPlug *DeepOversample::maxSampleAlphaPlug()
{
	return getChild<FloatPlug>( g_firstPlugIndex + 0 );
}

const Gaffer::FloatPlug *DeepOversample::maxSampleAlphaPlug() const
{
	return getChild<FloatPlug>( g_firstPlugIndex + 0 );
}

Gaffer::IntVectorDataPlug *DeepOversample::subSamplesPlug()
{
	return getChild<IntVectorDataPlug>( g_firstPlugIndex + 1 );
}

const Gaffer::IntVectorDataPlug *DeepOversample::subSamplesPlug() const
{
	return getChild<IntVectorDataPlug>( g_firstPlugIndex + 1 );
}

void DeepOversample::affects( const Gaffer::Plug *input, AffectedPlugsContainer &outputs ) const
{
	ImageProcessor::affects( input, outputs );

	if(
		input == inPlug()->sampleOffsetsPlug() || input == inPlug()->channelNamesPlug() ||
		input == inPlug()->channelDataPlug() || input == maxSampleAlphaPlug()
	)
	{
		outputs.push_back( subSamplesPlug() );
	}
	
	if( input == subSamplesPlug() || input == inPlug()->channelDataPlug() || input == inPlug()->channelNamesPlug() )
	{
		outputs.push_back( outPlug()->channelDataPlug() );
	}

	if( input == subSamplesPlug() || input == inPlug()->sampleOffsetsPlug() )
	{
		outputs.push_back( outPlug()->sampleOffsetsPlug() );
	}
}

void DeepOversample::hash( const Gaffer::ValuePlug *output, const Gaffer::Context *context, IECore::MurmurHash &h ) const
{
	ImageProcessor::hash( output, context, h );

	if( output != subSamplesPlug() )
	{
		return;
	}

	inPlug()->sampleOffsetsPlug()->hash( h );

	if( !GafferImage::ImageAlgo::channelExists( inPlug()->channelNames()->readable(), "A" ) )
	{
		return;
	}

	maxSampleAlphaPlug()->hash( h );

	GafferImage::ImagePlug::ChannelDataScope channelScope( context );
	channelScope.setChannelName( "A" );
	inPlug()->channelDataPlug()->hash( h );
}

void DeepOversample::compute( Gaffer::ValuePlug *output, const Gaffer::Context *context ) const
{
	ImageProcessor::compute( output, context );

	if( output != subSamplesPlug() )
	{
		return;
	}

	ConstIntVectorDataPtr sampleOffsetsData = inPlug()->sampleOffsetsPlug()->getValue();
	const std::vector<int> &sampleOffsets = sampleOffsetsData->readable();

	IntVectorDataPtr subSamplesData = new IntVectorData();
	std::vector<int> &subSamples = subSamplesData->writable();
	if( GafferImage::ImageAlgo::channelExists( inPlug()->channelNames()->readable(), "A" ) )
	{
		float thresh = maxSampleAlphaPlug()->getValue();

		GafferImage::ImagePlug::ChannelDataScope channelScope( context );
		channelScope.setChannelName( "A" );
		ConstFloatVectorDataPtr alphaData = inPlug()->channelDataPlug()->getValue();
		const std::vector<float> &alpha = alphaData->readable();

		subSamples.resize( alpha.size() + 1 );
		int prev = 0;
		int subStepCount = 0;
		for( unsigned int i = 0; i < sampleOffsets.size(); i++ )
		{
			int index = sampleOffsets[i];
			float accumAlpha = 0;
			for( int j = prev; j < index; j++ )
			{
				float sampleThresh = thresh / ( 1 - accumAlpha );
				float a = alpha[j];

				int subSteps = ceil( log( 1 - a ) / log( 1 - sampleThresh ) );
				
				subSamples[j] = subSteps;
				subStepCount += subSteps;
				accumAlpha += a - a * accumAlpha;
			}
			prev = index;
		}
		subSamples.back() = subStepCount;
	}
	else
	{
		subSamples.resize( sampleOffsets.back(), 1 );
	}

	static_cast<IntVectorDataPlug *>( output )->setValue( subSamplesData );
}

void DeepOversample::hashChannelData( const GafferImage::ImagePlug *output, const Gaffer::Context *context, IECore::MurmurHash &h ) const
{
	ImageProcessor::hashChannelData( output, context, h );

	const std::string &channelName = context->get<std::string>( GafferImage::ImagePlug::channelNameContextName );

	ConstStringVectorDataPtr channelNamesData = inPlug()->channelNames();
	std::vector<string> channelNames = channelNamesData->readable();

	GafferImage::ImagePlug::ChannelDataScope channelScope( context );

	channelScope.remove( GafferImage::ImagePlug::channelNameContextName );
	
	subSamplesPlug()->hash( h );

	if( channelName == "Z" || channelName == "ZBack" )
	{
		h.append( channelName != "Z" );

		if( GafferImage::ImageAlgo::channelExists( channelNames, "Z" ) )
		{
			channelScope.setChannelName( "Z" );
			inPlug()->channelDataPlug()->hash( h );
		}
		if( GafferImage::ImageAlgo::channelExists( channelNames, "ZBack" ) )
		{
			channelScope.setChannelName( "ZBack" );
			inPlug()->channelDataPlug()->hash( h );
		}
	}
	else
	{
		if( GafferImage::ImageAlgo::channelExists( channelNames, "A" ) )
		{
			channelScope.setChannelName( "A" );
			inPlug()->channelDataPlug()->hash( h );
		}

		if( channelName != "A" )
		{
			channelScope.setChannelName( channelName );
			inPlug()->channelDataPlug()->hash( h );
		}
	}
}

IECore::ConstFloatVectorDataPtr DeepOversample::computeChannelData( const std::string &channelName, const Imath::V2i &tileOrigin, const Gaffer::Context *context, const GafferImage::ImagePlug *parent ) const
{
	ConstStringVectorDataPtr channelNamesData = inPlug()->channelNames();
	std::vector<string> channelNames = channelNamesData->readable();

	GafferImage::ImagePlug::ChannelDataScope channelScope( context );

	// TODO - check if channel names Z ZBack A don't exist, update hash and affects
	channelScope.remove( GafferImage::ImagePlug::channelNameContextName );
	
	ConstIntVectorDataPtr subSamplesData = subSamplesPlug()->getValue();
	const std::vector<int> &subSamples = subSamplesData->readable();

	FloatVectorDataPtr channelData = new FloatVectorData();
	std::vector<float> &channel = channelData->writable();
	channel.resize( subSamples.back() );

	if( channelName == "Z" || channelName == "ZBack" )
	{
		bool back = channelName != "Z";
		ConstFloatVectorDataPtr zData;
		ConstFloatVectorDataPtr zBackData;

		if( GafferImage::ImageAlgo::channelExists( channelNames, "Z" ) )
		{
			channelScope.setChannelName( "Z" );
			zData = inPlug()->channelDataPlug()->getValue();
		}
		if( GafferImage::ImageAlgo::channelExists( channelNames, "ZBack" ) )
		{
			channelScope.setChannelName( "ZBack" );
			zBackData = inPlug()->channelDataPlug()->getValue();
		}

		zBackData = zBackData ? zBackData : zData;
		zData = zData ? zData : zBackData;

		const std::vector<float> &z = zData->readable();
		const std::vector<float> &zBack = zBackData->readable();
	
		int outIndex = 0;	
		for( unsigned int i = 0; i < z.size(); i++ )
		{
			int s = subSamples[i]; 
			float sInv = 1.0f / float( s );
			for( int j = 0; j < s; j++ )
			{
				channel[outIndex] = z[i] + ( zBack[i] - z[i] ) * ( j + back ) * sInv;
				outIndex++;
			}
		}
	}
	else
	{
		int size = 0;
		ConstFloatVectorDataPtr alphaData;
		const float *alpha = nullptr;
		if( GafferImage::ImageAlgo::channelExists( channelNames, "A" ) )
		{
			channelScope.setChannelName( "A" );
			alphaData = inPlug()->channelDataPlug()->getValue();
			alpha = &( alphaData->readable()[0] );
			size = alphaData->readable().size();
		}

		ConstFloatVectorDataPtr inChannelData;
		const float *inChannel = nullptr;
		if( channelName != "A" )
		{
			channelScope.setChannelName( channelName );
			inChannelData = inPlug()->channelDataPlug()->getValue();
			inChannel = &( inChannelData->readable()[0] );
			size = inChannelData->readable().size();
		}

		int outIndex = 0;	
		for( int i = 0; i < size; i++ )
		{
			int s = subSamples[i];
			float increment = 1.0/float(s);
			float a = alpha ? alpha[i] : 0;
			float subA = a > 0.0f ? 1 - pow( 1 - a, increment ) : increment;
			float v = inChannel ? inChannel[i] / a * subA : subA;
			for( int j = 0; j < s; j++ )
			{
				channel[outIndex] = v;
				outIndex++;
			}
		}
	}

	return channelData;
}

void DeepOversample::hashSampleOffsets( const GafferImage::ImagePlug *parent, const Gaffer::Context *context, IECore::MurmurHash &h ) const
{
	ImageProcessor::hashSampleOffsets( parent, context, h );

	inPlug()->sampleOffsetsPlug()->hash( h);
	subSamplesPlug()->hash( h );
}

IECore::ConstIntVectorDataPtr DeepOversample::computeSampleOffsets( const Imath::V2i &tileOrigin, const Gaffer::Context *context, const GafferImage::ImagePlug *parent ) const
{
	ConstIntVectorDataPtr inSampleOffsetsData = inPlug()->sampleOffsetsPlug()->getValue();
	const std::vector<int> &inSampleOffsets = inSampleOffsetsData->readable();

	ConstIntVectorDataPtr subSamplesData = subSamplesPlug()->getValue();
	const std::vector<int> &subSamples = subSamplesData->readable();

	IntVectorDataPtr outSampleOffsetsData = new IntVectorData();
	std::vector<int> &outSampleOffsets = outSampleOffsetsData->writable();
	outSampleOffsets.resize( inSampleOffsets.size() );

	int prev = 0;
	int outIndex = 0;
	for( unsigned int i = 0; i < inSampleOffsets.size(); i++ )
	{
		int index = inSampleOffsets[i];
		for( int j = prev; j < index; j++ )
		{
			outIndex += subSamples[j];
		}
		outSampleOffsets[i] = outIndex;
		prev = index;
	}

	return outSampleOffsetsData;
}
