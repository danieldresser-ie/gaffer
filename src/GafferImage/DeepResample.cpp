//////////////////////////////////////////////////////////////////////////
//
//  Copyright (c) 2019, Image Engine Design Inc. All rights reserved.
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
#include "GafferImage/DeepResample.h"
#include "GafferImage/DeepAlgo.h"

// TODO
#include <csignal>

using namespace std;
using namespace Imath;
using namespace IECore;
using namespace Gaffer;
using namespace GafferImage;

GAFFER_GRAPHCOMPONENT_DEFINE_TYPE( DeepResample );

size_t DeepResample::g_firstPlugIndex = 0;

DeepResample::DeepResample( const std::string &name )
	:	ImageProcessor( name )
{
	storeIndexOfNextChild( g_firstPlugIndex );

	addChild( new FloatPlug( "alphaTolerance", Gaffer::Plug::In, 0.01 ) );
	addChild( new FloatPlug( "depthTolerance", Gaffer::Plug::In, 0.01 ) );

	addChild( new ImagePlug( "__tidyIn", Plug::In, Plug::Default & ~Plug::Serialisable ) );

	DeepStatePtr deepStateNode = new DeepState( "__deepState" );
	addChild( deepStateNode );

	addChild( new CompoundObjectPlug( "__resampled", Gaffer::Plug::Out, new IECore::CompoundObject() ) );

	deepStateNode->inPlug()->setInput( inPlug() );
	deepStateNode->deepStatePlug()->setValue( int( DeepState::TargetState::Tidy ) );
	tidyInPlug()->setInput( deepStateNode->outPlug() );

	// We don't ever want to change these, so we make pass-through connections.

	// TODO - we should force ZBack into the channelNames if only Z is present
	outPlug()->channelNamesPlug()->setInput( tidyInPlug()->channelNamesPlug() );

	outPlug()->dataWindowPlug()->setInput( tidyInPlug()->dataWindowPlug() );
	outPlug()->formatPlug()->setInput( tidyInPlug()->formatPlug() );
	outPlug()->metadataPlug()->setInput( tidyInPlug()->metadataPlug() );
	outPlug()->deepPlug()->setInput( tidyInPlug()->deepPlug() );
}

DeepResample::~DeepResample()
{
}

Gaffer::FloatPlug *DeepResample::alphaTolerancePlug()
{
	return getChild<FloatPlug>( g_firstPlugIndex + 0 );
}

const Gaffer::FloatPlug *DeepResample::alphaTolerancePlug() const
{
	return getChild<FloatPlug>( g_firstPlugIndex + 0 );
}

Gaffer::FloatPlug *DeepResample::depthTolerancePlug()
{
	return getChild<FloatPlug>( g_firstPlugIndex + 1 );
}

const Gaffer::FloatPlug *DeepResample::depthTolerancePlug() const
{
	return getChild<FloatPlug>( g_firstPlugIndex + 1 );
}

ImagePlug *DeepResample::tidyInPlug()
{
    return getChild<ImagePlug>( g_firstPlugIndex + 2 );
}

const ImagePlug *DeepResample::tidyInPlug() const
{
    return getChild<ImagePlug>( g_firstPlugIndex + 2 );
}

DeepState *DeepResample::deepState()
{
    return getChild<DeepState>( g_firstPlugIndex + 3 );
}

const DeepState *DeepResample::deepState() const
{
    return getChild<DeepState>( g_firstPlugIndex + 3 );
}

Gaffer::CompoundObjectPlug *DeepResample::resampledPlug()
{
	return getChild<CompoundObjectPlug>( g_firstPlugIndex + 4 );
}

const Gaffer::CompoundObjectPlug *DeepResample::resampledPlug() const
{
	return getChild<CompoundObjectPlug>( g_firstPlugIndex + 4 );
}

void DeepResample::affects( const Gaffer::Plug *input, AffectedPlugsContainer &outputs ) const
{
	ImageProcessor::affects( input, outputs );

	if(
		input == tidyInPlug()->sampleOffsetsPlug() || input == tidyInPlug()->channelNamesPlug() ||
		input == tidyInPlug()->channelDataPlug() ||
		input == alphaTolerancePlug() || input == depthTolerancePlug()
	)
	{
		outputs.push_back( resampledPlug() );
	}

	if( input == resampledPlug() || input == tidyInPlug()->channelDataPlug() )
	{
		outputs.push_back( outPlug()->channelDataPlug() );
	}

	if( input == resampledPlug() || input == tidyInPlug()->sampleOffsetsPlug() )
	{
		outputs.push_back( outPlug()->sampleOffsetsPlug() );
	}
}

void DeepResample::hash( const Gaffer::ValuePlug *output, const Gaffer::Context *context, IECore::MurmurHash &h ) const
{
	ImageProcessor::hash( output, context, h );

	if( output != resampledPlug() )
	{
		return;
	}


	alphaTolerancePlug()->hash( h );
	depthTolerancePlug()->hash( h );

	ConstStringVectorDataPtr channelNamesData = tidyInPlug()->channelNames();
	std::vector<string> channelNames = channelNamesData->readable();

	tidyInPlug()->sampleOffsetsPlug()->hash( h );

	if(
		ImageAlgo::channelExists( channelNames, "A" ) &&
		ImageAlgo::channelExists( channelNames, "Z" ) 
	)
	{
		ImagePlug::ChannelDataScope channelScope( context );
		channelScope.setChannelName( "A" );
		tidyInPlug()->channelDataPlug()->hash( h );

		channelScope.setChannelName( "Z" );
		tidyInPlug()->channelDataPlug()->hash( h );

		if( ImageAlgo::channelExists( channelNames, "ZBack" ) )
		{
			tidyInPlug()->channelDataPlug()->hash( h );
		}
	}
}

void DeepResample::compute( Gaffer::ValuePlug *output, const Gaffer::Context *context ) const
{
	ImageProcessor::compute( output, context );

	if( output != resampledPlug() )
	{
		return;
	}

	// TODO
	const V2i tileOrigin = context->get<V2i>( ImagePlug::tileOriginContextName );

	float alphaTolerance = alphaTolerancePlug()->getValue();
	float depthTolerance = depthTolerancePlug()->getValue();

	ConstIntVectorDataPtr sampleOffsetsData = tidyInPlug()->sampleOffsetsPlug()->getValue();
	const std::vector<int> &sampleOffsets = sampleOffsetsData->readable();

	CompoundObjectPtr resampledData = new CompoundObject();

	ConstStringVectorDataPtr channelNamesData = tidyInPlug()->channelNames();
	std::vector<string> channelNames = channelNamesData->readable();

	if(
		ImageAlgo::channelExists( channelNames, "A" ) &&
		ImageAlgo::channelExists( channelNames, "Z" )
	)
	{
		ImagePlug::ChannelDataScope channelScope( context );
		channelScope.setChannelName( "A" );
		ConstFloatVectorDataPtr alphaData = tidyInPlug()->channelDataPlug()->getValue();
		const std::vector<float> &alpha = alphaData->readable();

		channelScope.setChannelName( "Z" );
		ConstFloatVectorDataPtr zData = tidyInPlug()->channelDataPlug()->getValue();
		const std::vector<float> &z = zData->readable();

		ConstFloatVectorDataPtr zBackData;
		if( ImageAlgo::channelExists( channelNames, "ZBack" ) )
		{
			channelScope.setChannelName( "ZBack" );
			zBackData = tidyInPlug()->channelDataPlug()->getValue();
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

		// Start with the input size, resize to fit afterwards
		// TODO - this would cause a crash if resampling increased the sample count
		// Mathematically, I'm reasonably confident this is impossible, but I should
		// have a proper think about whether this could ever happen with floating point
		// error and such
		outAlpha.resize( alpha.size() + sampleOffsets.size() ); // TODO
		outZ.resize( alpha.size() + sampleOffsets.size() );
		outZBack.resize( alpha.size() + sampleOffsets.size() );

		//std::vector<DeepPixel> outputPixels;
		//outputPixels.resize( sampleOffsets.size() );
		int prev = 0;
		int outputCount = 0;
		for( unsigned int i = 0; i < sampleOffsets.size(); i++ )
		{
			int index = sampleOffsets[i];

			/*DeepPixel before( 0, index - prev );
			for( int j = prev; j < index; j++ )
			{
				if( !( alpha[j] >= -0.1 && alpha[j] <= 1.1  ) )
				{
					std::cerr << "BAD IN : " << alpha[j] << "\n";
					before.addSample( z[j], zBack[j], 0.0f, nullptr );
				}
				else
				{
					before.addSample( z[j], zBack[j], alpha[j], nullptr );
				}
			}*/
			int ly = i / ImagePlug::tileSize();
			V2i pixelLocation = tileOrigin + V2i( i - ly * ImagePlug::tileSize(), ly );
			//std::cerr << "P : " << tileOrigin.x + i - ( ly * ImagePlug::tileSize() ) << " , " << tileOrigin.y + ly << "\n"; 

			//bool debug = pixelLocation == V2i( 79, 41 );
			//bool debug = pixelLocation == V2i( 57, 21 );
			//bool debug = pixelLocation == V2i( 54, 85 );
			//bool debug = pixelLocation == V2i( 93, 54 ) || pixelLocation == V2i( 0, 0 );
			//bool debug = pixelLocation == V2i( 87, 59 );
			//bool debug = pixelLocation == V2i( 32, 33 );
			//bool debug = pixelLocation == V2i( 75, 80 );
			bool debug = false;
			if( debug ) std::cerr << "\n\n\nPixel start : " << pixelLocation << "\n";
			int resampledCount;
			try
			{
				DeepAlgo::resampleDeepPixel(
					index - prev, &alpha[prev], &z[prev], &zBack[prev],
					alphaTolerance, depthTolerance,
					resampledCount, &outAlpha[outputCount], &outZ[outputCount], &outZBack[outputCount],
					debug
				);
				outputCount += resampledCount;
			}
			catch( const std::exception &e )
			{
				std::cerr << "Caught : " << e.what() << "\n";
				std::cerr << "On pixel : " << pixelLocation << "\n";
				throw;
			}
			if( debug ) std::cerr << "\nPixel end : " << pixelLocation << "\n\n\n";
			outSampleOffsets[i] = outputCount;

			prev = index;
		}

		// TODO	- this is too late to reliably avoid crash
		if( outSampleOffsets.back() > (int)(alpha.size() + sampleOffsets.size()) )
		{
			throw IECore::Exception( "Sample count increased <TODO> TOO MUCH during resampling" );
		}

		outAlpha.resize( outSampleOffsets.back() );
		outZ.resize( outSampleOffsets.back() );
		outZBack.resize( outSampleOffsets.back() );
		outAlpha.shrink_to_fit();
		outZ.shrink_to_fit();
		outZBack.shrink_to_fit();

		resampledData->members()["sampleOffsets"] = outSampleOffsetsData;
		resampledData->members()["A"] = outAlphaData;
		resampledData->members()["Z"] = outZData;
		resampledData->members()["ZBack"] = outZBackData;
	}

	static_cast<CompoundObjectPlug *>( output )->setValue( resampledData );
}

void DeepResample::hashChannelData( const ImagePlug *output, const Gaffer::Context *context, IECore::MurmurHash &h ) const
{
	ImageProcessor::hashChannelData( output, context, h );
	const std::string &channelName = context->get<std::string>( GafferImage::ImagePlug::channelNameContextName );

	tidyInPlug()->channelDataPlug()->hash( h );

	ImagePlug::ChannelDataScope channelScope( context );
	channelScope.remove( ImagePlug::channelNameContextName );

	resampledPlug()->hash( h );

	if( channelName == "Z" || channelName ==  "ZBack" || channelName == "A" )
	{
		h.append( channelName );
		return;
	}

	tidyInPlug()->sampleOffsetsPlug()->hash( h );

	channelScope.setChannelName( "A" );
	tidyInPlug()->channelDataPlug()->hash( h );
}

IECore::ConstFloatVectorDataPtr DeepResample::computeChannelData( const std::string &channelName, const Imath::V2i &tileOrigin, const Gaffer::Context *context, const ImagePlug *parent ) const
{
	ImagePlug::ChannelDataScope channelScope( context );
	channelScope.remove( ImagePlug::channelNameContextName );

	ConstCompoundObjectPtr resampled = resampledPlug()->getValue();

	if( !resampled->members().size() )
	{
		channelScope.setChannelName( channelName );
		return tidyInPlug()->channelDataPlug()->getValue();
	}

	if( channelName == "Z" )
	{
		return resampled->member<FloatVectorData>("Z");
	}
	else if( channelName == "ZBack" )
	{
		return resampled->member<FloatVectorData>("ZBack");
	}
	else if( channelName == "A" )
	{
		return resampled->member<FloatVectorData>("A");
	}

	IECore::ConstFloatVectorDataPtr resampledAlphaData = resampled->member<FloatVectorData>("A");
	IECore::ConstIntVectorDataPtr resampledOffsetsData = resampled->member<IntVectorData>("sampleOffsets");
	IECore::ConstIntVectorDataPtr origOffsetsData = tidyInPlug()->sampleOffsetsPlug()->getValue();

	channelScope.setChannelName( "A" );
	IECore::ConstFloatVectorDataPtr origAlphaData = tidyInPlug()->channelDataPlug()->getValue();

	channelScope.setChannelName( channelName );
	IECore::ConstFloatVectorDataPtr origChannelData = tidyInPlug()->channelDataPlug()->getValue();

	const std::vector<int> &origOffsets = origOffsetsData->readable();
	const std::vector<int> &resampledOffsets = resampledOffsetsData->readable();
	const std::vector<float> &origAlpha = origAlphaData->readable();
	const std::vector<float> &resampledAlpha = resampledAlphaData->readable();

	const std::vector<float> &origChannel = origChannelData->readable();

	IECore::FloatVectorDataPtr resultData = new IECore::FloatVectorData();
	std::vector<float> &result = resultData->writable();
	result.resize( resampledAlpha.size() );

	/*
	int pixel = 0;
	float resampledAccumAlpha = 0;
	int origIndex = 0;
	float origAccumAlpha = 0;
	*/

	int prevResampledOffset = 0;
	int prevOrigOffset = 0;
	
	for( unsigned int i = 0; i < resampledOffsets.size(); i++ )
	{
		int resampledOffset = resampledOffsets[i];
		int origOffset = origOffsets[i];
		
		DeepAlgo::conformToAlpha(
			origOffset - prevOrigOffset, resampledOffset - prevResampledOffset,
			&origAlpha[prevOrigOffset], &resampledAlpha[prevResampledOffset],
			&origChannel[prevOrigOffset], &result[prevResampledOffset]
		);
	
		prevResampledOffset = resampledOffset;	
		prevOrigOffset = origOffset;	

		/*
		resampledAccumAlpha += resampledAlpha[i] - resampledAccumAlpha * resampledAlpha[i];

		float 
		while( true )
		{
			float newOrigAccumAlpha = origAccumAlpha + origAlpha[origIndex] - origAccumAlpha * origAlpha[origIndex];
		
			if( newOrigAccumAlpha > resampledAccumAlpha )
			{
				break;
			}

			origAccumAlpha = newOrigAccumAlpha;
			origIndex++;
		}

		
		
		while( pixel < ImagePlug::tilePixels() && i == resampledOffsets[ pixel ] )
		{
			pixel++;
			resampledAccumAlpha = 0;
		}
		*/
	}

	return resultData;
}

void DeepResample::hashSampleOffsets( const ImagePlug *parent, const Gaffer::Context *context, IECore::MurmurHash &h ) const
{
	ImageProcessor::hashSampleOffsets( parent, context, h );

	resampledPlug()->hash( h );
	tidyInPlug()->sampleOffsetsPlug()->hash( h );
}

IECore::ConstIntVectorDataPtr DeepResample::computeSampleOffsets( const Imath::V2i &tileOrigin, const Gaffer::Context *context, const ImagePlug *parent ) const
{
	ConstCompoundObjectPtr resampled = resampledPlug()->getValue();

	if( !resampled->members().size() )
	{
		return tidyInPlug()->sampleOffsetsPlug()->getValue();
	}

	return resampled->member<IntVectorData>("sampleOffsets");
}
