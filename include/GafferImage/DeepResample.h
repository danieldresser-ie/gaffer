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

#ifndef GAFFERIMAGE_DEEPRESAMPLE_H
#define GAFFERIMAGE_DEEPRESAMPLE_H

#include "Gaffer/NumericPlug.h"

#include "GafferImage/ImageProcessor.h"
#include "GafferImage/DeepState.h"

namespace GafferImage
{

class GAFFERIMAGE_API DeepResample : public ImageProcessor
{

	public :

		DeepResample( const std::string &name=defaultName<DeepResample>() );
		~DeepResample() override;

		GAFFER_GRAPHCOMPONENT_DECLARE_TYPE( GafferImage::DeepResample, GafferImage::DeepResampleTypeId, GafferImage::ImageProcessor );

		Gaffer::FloatPlug *alphaTolerancePlug();
		const Gaffer::FloatPlug *alphaTolerancePlug() const;

		Gaffer::FloatPlug *depthTolerancePlug();
		const Gaffer::FloatPlug *depthTolerancePlug() const;

		void affects( const Gaffer::Plug *input, AffectedPlugsContainer &outputs ) const override;

	protected :

		void hash( const Gaffer::ValuePlug *output, const Gaffer::Context *context, IECore::MurmurHash &h ) const override;
		void compute( Gaffer::ValuePlug *output, const Gaffer::Context *context ) const override;

		void hashChannelData( const GafferImage::ImagePlug *output, const Gaffer::Context *context, IECore::MurmurHash &h ) const override;
		IECore::ConstFloatVectorDataPtr computeChannelData( const std::string &channelName, const Imath::V2i &tileOrigin, const Gaffer::Context *context, const GafferImage::ImagePlug *parent ) const override;

		void hashSampleOffsets( const GafferImage::ImagePlug *parent, const Gaffer::Context *context, IECore::MurmurHash &h ) const override;
		IECore::ConstIntVectorDataPtr computeSampleOffsets( const Imath::V2i &tileOrigin, const Gaffer::Context *context, const GafferImage::ImagePlug *parent ) const override;

	private :

		// Input plug to receive the tidy image from the internal deepState node.
		ImagePlug *tidyInPlug();
		const ImagePlug *tidyInPlug() const;

		// The internal DeepState node for tidying before resampling
		GafferImage::DeepState *deepState();
		const GafferImage::DeepState *deepState() const;

		Gaffer::CompoundObjectPlug *resampledPlug();
		const Gaffer::CompoundObjectPlug *resampledPlug() const;

		static size_t g_firstPlugIndex;

};

IE_CORE_DECLAREPTR( DeepResample )

} // namespace GafferImage

#endif // GAFFERIMAGE_DEEPRESAMPLE_H
