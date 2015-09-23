//////////////////////////////////////////////////////////////////////////
//
//  Copyright (c) 2012, John Haddon. All rights reserved.
//  Copyright (c) 2012-2015, Image Engine Design Inc. All rights reserved.
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

#include "GafferImage/FlatImageProcessor.h"
#include "GafferImage/ImageAlgo.h"
#include "Gaffer/ArrayPlug.h"

using namespace Gaffer;
using namespace GafferImage;

IE_CORE_DEFINERUNTIMETYPED( FlatImageProcessor );

FlatImageProcessor::FlatImageProcessor( const std::string &name )
	:	ImageProcessor( name )
{
	// We don't ever want to change these, so we make pass-through connections.
	outPlug()->sampleOffsetsPlug()->setInput( inPlug()->sampleOffsetsPlug() );
	outPlug()->deepStatePlug()->setInput( inPlug()->deepStatePlug() );
}

FlatImageProcessor::FlatImageProcessor( const std::string &name, size_t minInputs, size_t maxInputs )
	:	ImageProcessor( name, minInputs, maxInputs )
{
	// We don't ever want to change these, so we make pass-through connections.
	outPlug()->sampleOffsetsPlug()->setInput( inPlug()->sampleOffsetsPlug() );
	outPlug()->deepStatePlug()->setInput( inPlug()->deepStatePlug() );
}

FlatImageProcessor::~FlatImageProcessor()
{
}

void FlatImageProcessor::checkInputIsFlat( const ImagePlug *in )
{
	if( in->deepStatePlug()->getValue() != ImagePlug::Flat )
	{
		throw IECore::Exception( "Deep data not supported in input \"" + in->relativeName( in->node() ) + "\"" );
	}
}

void FlatImageProcessor::checkAllInputs() const
{
	if( inPlugs() )
	{
		for( ImagePlugIterator it( inPlugs() ); !it.done(); ++it )
		{
			checkInputIsFlat( it->get() );
		}
	}
	else
	{
		checkInputIsFlat( inPlug() );
	}
}

void FlatImageProcessor::affects( const Gaffer::Plug *input, AffectedPlugsContainer &outputs ) const
{
	ImageProcessor::affects( input, outputs );

	if( input == inPlug()->deepStatePlug() )
	{
		outputs.push_back( outPlug()->channelDataPlug() );
	}
}

void FlatImageProcessor::hashChannelData( const GafferImage::ImagePlug *parent, const Gaffer::Context *context, IECore::MurmurHash &h ) const
{
	checkAllInputs();
	hashFlatChannelData( parent, context, h );
}

IECore::ConstFloatVectorDataPtr FlatImageProcessor::computeChannelData( const std::string &channelName, const Imath::V2i &tileOrigin, const Gaffer::Context *context, const ImagePlug *parent ) const
{
	checkAllInputs();
	return computeFlatChannelData( channelName, tileOrigin, context, parent );
}

void FlatImageProcessor::hashFlatChannelData( const GafferImage::ImagePlug *parent, const Gaffer::Context *context, IECore::MurmurHash &h ) const
{
	ImageProcessor::hashChannelData( parent, context, h );
}

IECore::ConstFloatVectorDataPtr FlatImageProcessor::computeFlatChannelData( const std::string &channelName, const Imath::V2i &tileOrigin, const Gaffer::Context *context, const ImagePlug *parent ) const
{
	throw IECore::NotImplementedException( std::string( typeName() ) + "::computeFlatChannelData" );
}
