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
#include "GafferImageTest/DeepAlphaOffset.h"

using namespace std;
using namespace Imath;
using namespace IECore;
using namespace Gaffer;
using namespace GafferImageTest;

GAFFER_GRAPHCOMPONENT_DEFINE_TYPE( DeepAlphaOffset );

size_t DeepAlphaOffset::g_firstPlugIndex = 0;

DeepAlphaOffset::DeepAlphaOffset( const std::string &name )
	:	ChannelDataProcessor( name )
{
	storeIndexOfNextChild( g_firstPlugIndex );

	addChild( new FloatPlug( "offset", Gaffer::Plug::In ) );
}

DeepAlphaOffset::~DeepAlphaOffset()
{
}

Gaffer::FloatPlug *DeepAlphaOffset::offsetPlug()
{
	return getChild<FloatPlug>( g_firstPlugIndex + 0 );
}

const Gaffer::FloatPlug *DeepAlphaOffset::offsetPlug() const
{
	return getChild<FloatPlug>( g_firstPlugIndex + 0 );
}

void DeepAlphaOffset::affects( const Gaffer::Plug *input, AffectedPlugsContainer &outputs ) const
{
	ImageProcessor::affects( input, outputs );

	if( input == offsetPlug() || input == inPlug()->channelDataPlug() || input == inPlug()->channelNamesPlug() || input == inPlug()->sampleOffsetsPlug() )
	{
		outputs.push_back( outPlug()->channelDataPlug() );
	}
}

void DeepAlphaOffset::hashChannelData( const GafferImage::ImagePlug *output, const Gaffer::Context *context, IECore::MurmurHash &h ) const
{
	ImageProcessor::hashChannelData( output, context, h );

	const std::string &channelName = context->get<std::string>( GafferImage::ImagePlug::channelNameContextName );


	inPlug()->channelDataPlug()->hash( h );
	if( channelName == "Z" || channelName == "ZBack" )
	{
		return;
	}

	GafferImage::ImagePlug::ChannelDataScope channelScope( context );

	if( channelName == "A" )
	{
		channelScope.remove( GafferImage::ImagePlug::channelNameContextName );

		offsetPlug()->hash( h );
		inPlug()->sampleOffsetsPlug()->hash( h );
	}
	else
	{
		ConstStringVectorDataPtr channelNamesData = inPlug()->channelNames();
		std::vector<string> channelNames = channelNamesData->readable();

		if( !GafferImage::ImageAlgo::channelExists( channelNames, "A" ) )
		{
			return;
		}

		channelScope.setChannelName( "A" );

		inPlug()->channelDataPlug()->hash( h );
		outPlug()->channelDataPlug()->hash( h );
	}
}

void DeepAlphaOffset::processChannelData( const Gaffer::Context *context, const GafferImage::ImagePlug *parent, const std::string &channel, IECore::FloatVectorDataPtr outData ) const
{
	std::vector<float> &out = outData->writable();

	if( channel == "Z" || channel == "ZBack" )
	{
		return;
	}

	GafferImage::ImagePlug::ChannelDataScope channelScope( context );

	if( channel == "A" )
	{
		channelScope.remove( GafferImage::ImagePlug::channelNameContextName );

		float offset = offsetPlug()->getValue();

		ConstIntVectorDataPtr sampleOffsetsData = inPlug()->sampleOffsetsPlug()->getValue();
		const std::vector<int> &sampleOffsets = sampleOffsetsData->readable();

		int prev = 0;
		for( unsigned int i = 0; i < sampleOffsets.size(); i++ )
		{
			int index = sampleOffsets[i];
			float accumAlpha = 0;
			for( int j = prev; j < index; j++ )
			{
				float newAccum = accumAlpha + out[j] - accumAlpha * out[j];
				/*newAccum + offset = accumAlpha + offset + out[j]  - ( accumAlpha + offset ) * out[j]
				newAccum + offset = accumAlpha + offset - ( 1 -  accumAlpha - offset ) * out[j]*/
				if( j == prev )
				{
					out[j] += offset;
				}
				else
				{
					float denom = 1 - accumAlpha - offset;
					if( denom > 0 )
					{
						out[j] = std::min( 1.0f, ( newAccum - accumAlpha ) / denom );
					}
					else
					{
						out[j] = 1.0f;
					}
				}
				accumAlpha = newAccum;
			}
			prev = index;
		}
	}
	else
	{
		ConstStringVectorDataPtr channelNamesData = inPlug()->channelNames();
		std::vector<string> channelNames = channelNamesData->readable();

		if( !GafferImage::ImageAlgo::channelExists( channelNames, "A" ) )
		{
			return;
		}

		channelScope.setChannelName( "A" );

		ConstFloatVectorDataPtr aBeforeData = inPlug()->channelDataPlug()->getValue();
		const std::vector<float> &aBefore = aBeforeData->readable();

		ConstFloatVectorDataPtr aAfterData = outPlug()->channelDataPlug()->getValue();
		const std::vector<float> &aAfter = aAfterData->readable();
	
		for( unsigned int j = 0; j < out.size(); j++ )
		{
			if( aBefore[j] > 0.0f )
			{
				out[j] = out[j] * aAfter[j] / aBefore[j];
			}
		}
	}
}
