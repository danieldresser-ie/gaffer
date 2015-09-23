//////////////////////////////////////////////////////////////////////////
//
//  Copyright (c) 2012, John Haddon. All rights reserved.
//  Copyright (c) 2012-2015, Image Engine Design Inc. All rights reserved.
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

#ifndef GAFFERIMAGE_FLATIMAGEPROCESSOR_H
#define GAFFERIMAGE_FLATIMAGEPROCESSOR_H

#include "GafferImage/ImageProcessor.h"

namespace GafferImage
{

/// The FlatImageProcessor is in most respects like ImageProcessor, but for nodes which
/// cannot handle deep data.  It will throw an exception if you try to evaluate channel data
/// when an input is deep
class FlatImageProcessor : public ImageProcessor
{

	public :

		/// Constructs with a single input ImagePlug named "in". Use inPlug()
		/// to access this plug.
		FlatImageProcessor( const std::string &name=defaultName<FlatImageProcessor>() );

		/// Constructs with an ArrayPlug called "in". Use inPlug() as a
		/// convenience for accessing the first child in the array, and use
		/// inPlugs() to access the array itself.
		FlatImageProcessor( const std::string &name, size_t minInputs, size_t maxInputs = Imath::limits<size_t>::max() );
		virtual ~FlatImageProcessor();

		IE_CORE_DECLARERUNTIMETYPEDEXTENSION( GafferImage::FlatImageProcessor, FlatImageProcessorTypeId, ImageProcessor );

		virtual void affects( const Gaffer::Plug *input, AffectedPlugsContainer &outputs ) const;

		/// Raise an exception if the given input is not flat.
		/// This is done automatically for the default inputs in checkAllInputs()
		static void checkInputIsFlat( const ImagePlug *in );

	protected :

		virtual void hashChannelData( const GafferImage::ImagePlug *parent, const Gaffer::Context *context, IECore::MurmurHash &h ) const;
		virtual IECore::ConstFloatVectorDataPtr computeChannelData( const std::string &channelName, const Imath::V2i &tileOrigin, const Gaffer::Context *context, const ImagePlug *parent ) const;

		virtual void hashFlatChannelData( const GafferImage::ImagePlug *parent, const Gaffer::Context *context, IECore::MurmurHash &h ) const;
		virtual IECore::ConstFloatVectorDataPtr computeFlatChannelData( const std::string &channelName, const Imath::V2i &tileOrigin, const Gaffer::Context *context, const ImagePlug *parent ) const;

		/// Raise an exception if any inputs are not flat.
		/// Called automatically in hashChannelData and computeChannelData, and implemented to check the
		/// standard inPlug() or inPlugs().
		/// Subclasses which add additional inputs should override this to call checkInputIsFlat on the new inputs,
		/// and then call the base implementation.
		virtual void checkAllInputs() const;


};

IE_CORE_DECLAREPTR( FlatImageProcessor )

} // namespace GafferImage

#endif // GAFFERIMAGE_FLATIMAGEPROCESSOR_H
