//////////////////////////////////////////////////////////////////////////
//
//  Copyright (c) 2016, Image Engine Design Inc. All rights reserved.
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
//      * Neither the name of Image Engine Design nor the names of
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

#ifndef GAFFERIMAGE_UVWARP_H
#define GAFFERIMAGE_UVWARP_H

#include "GafferImage/Warp.h"

namespace GafferImage
{

class VectorWarp : public Warp
{
	public :

		VectorWarp( const std::string &name=defaultName<Warp>() );
		~VectorWarp() override;

		IE_CORE_DECLARERUNTIMETYPEDEXTENSION( GafferImage::VectorWarp, VectorWarpTypeId, Warp );

		ImagePlug *vectorPlug();
		const ImagePlug *vectorPlug() const;

		enum VectorMode
		{
			Relative,   // Relative offset
			Absolute,   // Absolute position
		};

		Gaffer::IntPlug *vectorModePlug();
		const Gaffer::IntPlug *vectorModePlug() const;

		enum VectorUnits
		{
			Pixels,  // Vector specified in pixels
			Screen,  // Vector specified as fraction of display window
		};

		Gaffer::IntPlug *vectorUnitsPlug();
		const Gaffer::IntPlug *vectorUnitsPlug() const;

		void affects( const Gaffer::Plug *input, AffectedPlugsContainer &outputs ) const override;

	protected :

		bool affectsEngine( const Gaffer::Plug *input ) const override;
		void hashEngine( const Imath::V2i &tileOrigin, const Gaffer::Context *context, IECore::MurmurHash &h ) const override;
		const Engine *computeEngine( const Imath::V2i &tileOrigin, const Gaffer::Context *context ) const override;

		void checkAllInputs() const override;

	private :

		struct Engine;

		static size_t g_firstPlugIndex;

};

IE_CORE_DECLAREPTR( VectorWarp )

} // namespace GafferImage

#endif // GAFFERIMAGE_UVWARP_H
