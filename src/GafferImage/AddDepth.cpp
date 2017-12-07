//////////////////////////////////////////////////////////////////////////
//
//  Copyright (c) 2017, Image Engine Design Inc. All rights reserved.
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

#include "GafferImage/AddDepth.h"
#include "GafferImage/ImageAlgo.h"

using namespace std;
using namespace Imath;
using namespace IECore;
using namespace GafferImage;
using namespace Gaffer;

IE_CORE_DEFINERUNTIMETYPED( AddDepth );

size_t AddDepth::g_firstPlugIndex = 0;

//TODO - inherit from FlatImageProcessor?
AddDepth::AddDepth( const std::string &name )
	:	ImageProcessor( name )
{
	storeIndexOfNextChild( g_firstPlugIndex );
	addChild( new FloatPlug( "depth" ) );
	addChild( new StringPlug( "sourceZChannel" ) );

	// TODO - do we represent 0 thickness by ZBack = Z or no ZBack?
	addChild( new FloatPlug( "thickness" ) );
	addChild( new StringPlug( "sourceZBackChannel" ) );


	// Pass-through the things we don't want to modify.
	outPlug()->formatPlug()->setInput( inPlug()->formatPlug() );
	outPlug()->dataWindowPlug()->setInput( inPlug()->dataWindowPlug() );
	outPlug()->metadataPlug()->setInput( inPlug()->metadataPlug() );
	outPlug()->deepStatePlug()->setInput( inPlug()->deepStatePlug() );
	outPlug()->sampleOffsetsPlug()->setInput( inPlug()->sampleOffsetsPlug() );
}

AddDepth::~AddDepth()
{
}

FloatPlug *AddDepth::depthPlug()
{
	return getChild<FloatPlug>( g_firstPlugIndex );
}

const FloatPlug *AddDepth::depthPlug() const
{
	return getChild<FloatPlug>( g_firstPlugIndex );
}

StringPlug *AddDepth::sourceZChannelPlug()
{
	return getChild<StringPlug>( g_firstPlugIndex+1 );
}

const StringPlug *AddDepth::sourceZChannelPlug() const
{
	return getChild<StringPlug>( g_firstPlugIndex+1 );
}

FloatPlug *AddDepth::thicknessPlug()
{
	return getChild<FloatPlug>( g_firstPlugIndex+2 );
}

const FloatPlug *AddDepth::thicknessPlug() const
{
	return getChild<FloatPlug>( g_firstPlugIndex+2 );
}

StringPlug *AddDepth::sourceZBackChannelPlug()
{
	return getChild<StringPlug>( g_firstPlugIndex+3 );
}

const StringPlug *AddDepth::sourceZBackChannelPlug() const
{
	return getChild<StringPlug>( g_firstPlugIndex+3 );
}

void AddDepth::affects( const Gaffer::Plug *input, AffectedPlugsContainer &outputs ) const
{
	ImageProcessor::affects( input, outputs );

	if( input == inPlug()->channelNamesPlug() )
	{
		outputs.push_back( outPlug()->channelNamesPlug() );
	}
	else if( input == inPlug()->channelDataPlug() )
	{
		outputs.push_back( outPlug()->channelDataPlug() );
	}
	else if( input == sourceZChannelPlug() ||
		input == depthPlug() ||
		input == sourceZBackChannelPlug() ||
		input == thicknessPlug()
	)
	{
		outputs.push_back( outPlug()->channelDataPlug() );
	}
}

void AddDepth::hashChannelNames( const GafferImage::ImagePlug *output, const Gaffer::Context *context, IECore::MurmurHash &h ) const
{
	ImageProcessor::hashChannelNames( output, context, h );
	inPlug()->channelNamesPlug()->hash( h );
}

IECore::ConstStringVectorDataPtr AddDepth::computeChannelNames( const Gaffer::Context *context, const ImagePlug *parent ) const
{
	StringVectorDataPtr resultData = inPlug()->channelNamesPlug()->getValue()->copy();
	vector<string> &result = resultData->writable();

	if( find( result.begin(), result.end(), "Z" ) == result.end() )
	{
		result.push_back( "Z" );
	}

	if( find( result.begin(), result.end(), "ZBack" ) == result.end() )
	{
		result.push_back( "ZBack" );
	}

	return resultData;
}

void AddDepth::hashChannelData( const GafferImage::ImagePlug *output, const Gaffer::Context *context, IECore::MurmurHash &h ) const
{
	ImageProcessor::hashChannelData( output, context, h );
	const std::string channelName = context->get<std::string>( ImagePlug::channelNameContextName );

	const std::string sourceZChannel = sourceZChannelPlug()->getValue();
	const std::string sourceZBackChannel = sourceZBackChannelPlug()->getValue();

	if ( channelName == "Z" )
	{
		if( !sourceZChannel.size() )
		{
			depthPlug()->hash( h );
		}
		else
		{
			ConstStringVectorDataPtr channelNamesData = inPlug()->channelNamesPlug()->getValue();
			const vector<string> &channelNames = channelNamesData->readable();
			
			if( find( channelNames.begin(), channelNames.end(), sourceZChannel ) == channelNames.end() )
			{
				throw( IECore::Exception( "AddDepth : Cannot find requested Z channel - no channel \""
					+ sourceZChannel + "\" found."
				) );
			}

			ImagePlug::ChannelDataScope zScope( context );
			zScope.setChannelName( sourceZChannel );
			h = inPlug()->channelDataPlug()->hash();
		}
	}
	else if ( channelName == "ZBack" )
	{
		if( !sourceZChannel.size() && !sourceZBackChannel.size() )
		{
			depthPlug()->hash( h );
			thicknessPlug()->hash( h );
		}
		else
		{
			ConstStringVectorDataPtr channelNamesData = inPlug()->channelNamesPlug()->getValue();
			const vector<string> &channelNames = channelNamesData->readable();
			
			if( sourceZBackChannel.size() )
			{
				if( find( channelNames.begin(), channelNames.end(), sourceZBackChannel ) == channelNames.end() )
				{
					throw( IECore::Exception( "AddDepth : Cannot find requested ZBack channel - no channel \""
						+ sourceZBackChannel + "\" found."
					) );
				}
				ImagePlug::ChannelDataScope zBackScope( context );
				zBackScope.setChannelName( sourceZBackChannel );
				h = inPlug()->channelDataPlug()->hash();
			}
			else
			{
				thicknessPlug()->hash( h );

				ImagePlug::ChannelDataScope zScope( context );
				zScope.setChannelName( sourceZChannel );
				outPlug()->channelDataPlug()->hash(h);
			}
		}
	}
	else
	{
		h = inPlug()->channelDataPlug()->hash();
	}
}

IECore::ConstFloatVectorDataPtr AddDepth::computeChannelData( const std::string &channelName, const Imath::V2i &tileOrigin, const Gaffer::Context *context, const ImagePlug *parent ) const
{
	const std::string sourceZChannel = sourceZChannelPlug()->getValue();
	const std::string sourceZBackChannel = sourceZBackChannelPlug()->getValue();

	if ( channelName == "Z" )
	{
		if( !sourceZChannel.size() )
		{
			// Set constant Z
			float depth = depthPlug()->getValue();
			FloatVectorDataPtr resultData = new FloatVectorData;
			resultData->writable().resize( ImagePlug::tileSize() * ImagePlug::tileSize(), depth );

			return resultData;
		}
		else
		{
			ConstStringVectorDataPtr channelNamesData = inPlug()->channelNamesPlug()->getValue();
			const vector<string> &channelNames = channelNamesData->readable();

			// Pass through Z
			if( find( channelNames.begin(), channelNames.end(), sourceZChannel ) == channelNames.end() )
			{
				throw( IECore::Exception( "AddDepth : Cannot find requested Z channel - no channel \""
					+ sourceZChannel + "\" found."
				) );
			}

			ImagePlug::ChannelDataScope zScope( context );
			zScope.setChannelName( sourceZChannel );
			return inPlug()->channelDataPlug()->getValue();
		}
	}
	else if ( channelName == "ZBack" )
	{
		if( !sourceZChannel.size() && !sourceZBackChannel.size() )
		{
			// Set constant ZBack from depth and thickness
			float depth = depthPlug()->getValue();
			float thickness = thicknessPlug()->getValue();
			FloatVectorDataPtr resultData = new FloatVectorData;
			resultData->writable().resize( ImagePlug::tileSize() * ImagePlug::tileSize(), depth + thickness );

			return resultData;
		}
		else
		{
			ConstStringVectorDataPtr channelNamesData = inPlug()->channelNamesPlug()->getValue();
			const vector<string> &channelNames = channelNamesData->readable();

			if( sourceZBackChannel.size() )
			{
				// Pass through ZBack
				if( find( channelNames.begin(), channelNames.end(), sourceZBackChannel ) == channelNames.end() )
				{
					throw( IECore::Exception( "AddDepth : Cannot find requested ZBack channel - no channel \""
						+ sourceZBackChannel + "\" found."
					) );
				}

				ImagePlug::ChannelDataScope zBackScope( context );
				zBackScope.setChannelName( sourceZBackChannel );
				return inPlug()->channelDataPlug()->getValue();
			}
			else
			{
				// Compute ZBack by combining incoming Z with thickness
				float thickness = thicknessPlug()->getValue();

				ImagePlug::ChannelDataScope zScope( context );
				zScope.setChannelName( sourceZChannel );
				FloatVectorDataPtr resultData = outPlug()->channelDataPlug()->getValue()->copy();
				vector<float> &result = resultData->writable();
				for( float &i : result )
				{
					i += thickness;
				}
				return resultData;
			}
		}
	}
	else
	{
		return inPlug()->channelDataPlug()->getValue();
	}
}
