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

#ifndef GAFFERIMAGE_DEEPOVERSAMPLE_H
#define GAFFERIMAGE_DEEPOVERSAMPLE_H

#include "Gaffer/NumericPlug.h"

#include "GafferImage/ImageProcessor.h"

namespace GafferImageTest
{

class GAFFERIMAGE_API DeepOversample : public GafferImage::ImageProcessor
{

	public :

		DeepOversample( const std::string &name=defaultName<DeepOversample>() );
		~DeepOversample() override;

		GAFFER_GRAPHCOMPONENT_DECLARE_TYPE( GafferImageTest::DeepOversample, GafferImage::DeepOversampleTypeId, GafferImage::ImageProcessor );

		Gaffer::FloatPlug *maxSampleAlphaPlug();
		const Gaffer::FloatPlug *maxSampleAlphaPlug() const;

		void affects( const Gaffer::Plug *input, AffectedPlugsContainer &outputs ) const override;

	protected :

		void hash( const Gaffer::ValuePlug *output, const Gaffer::Context *context, IECore::MurmurHash &h ) const override;
		void compute( Gaffer::ValuePlug *output, const Gaffer::Context *context ) const override;

		void hashChannelData( const GafferImage::ImagePlug *output, const Gaffer::Context *context, IECore::MurmurHash &h ) const override;
		IECore::ConstFloatVectorDataPtr computeChannelData( const std::string &channelName, const Imath::V2i &tileOrigin, const Gaffer::Context *context, const GafferImage::ImagePlug *parent ) const override;

		void hashSampleOffsets( const GafferImage::ImagePlug *parent, const Gaffer::Context *context, IECore::MurmurHash &h ) const override;
		IECore::ConstIntVectorDataPtr computeSampleOffsets( const Imath::V2i &tileOrigin, const Gaffer::Context *context, const GafferImage::ImagePlug *parent ) const override;

	private :

		Gaffer::IntVectorDataPlug *subSamplesPlug();
		const Gaffer::IntVectorDataPlug *subSamplesPlug() const;

		static size_t g_firstPlugIndex;

};

IE_CORE_DECLAREPTR( DeepOversample )

} // namespace GafferImageTest

#endif // GAFFERIMAGE_DEEPOVERSAMPLE_H
