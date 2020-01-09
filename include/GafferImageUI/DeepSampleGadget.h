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

#ifndef GAFFERIMAGEUI_DEEPSAMPLEGADGET_H
#define GAFFERIMAGEUI_DEEPSAMPLEGADGET_H

#include "GafferUI/Gadget.h"
#include "GafferUI/Style.h"
#include "GafferUI/GraphGadget.h"

#include "GafferImageUI/TypeIds.h"

#include "GafferImage/ImagePlug.h"

#include "Gaffer/StandardSet.h"

#include "boost/optional.hpp"

namespace Gaffer
{

	IE_CORE_FORWARDDECLARE( Context );

}

namespace GafferImageUI
{

class GAFFERUI_API DeepSampleGadget : public GafferUI::Gadget
{

	public :

		DeepSampleGadget();

		~DeepSampleGadget() override;

		GAFFER_GRAPHCOMPONENT_DECLARE_TYPE( GafferImageUI::DeepSampleGadget, GafferImageUI::DeepSampleGadgetTypeId, GafferUI::Gadget );

		//void setImagePlug( GafferImage::ImagePlug *imagePlug);
		//GafferImage::ImagePlug *getImagePlug();

		void setDeepSamples( IECore::ConstCompoundDataPtr deepSamples );

		std::string getToolTip( const IECore::LineSegment3f &line ) const override;

	protected :

		void doRenderLayer( GafferUI::Gadget::Layer layer, const GafferUI::Style *style ) const override;

	private :

		void frame();

		void plugDirtied( Gaffer::Plug *plug );

		bool keyPress( GafferUI::GadgetPtr gadget, const GafferUI::KeyEvent &event );

		// Find elements at certain positions
		int keyAt( const IECore::LineSegment3f &position ) const;
		IECore::InternedString curveAt( const IECore::LineSegment3f &position ) const;

		void renderCurve( IECore::ConstFloatVectorDataPtr z, IECore::ConstFloatVectorDataPtr zBack, IECore::ConstFloatVectorDataPtr a, IECore::ConstFloatVectorDataPtr v );

		bool onTimeAxis( int y ) const;
		bool onValueAxis( int x ) const;

		Gaffer::Context *m_context;

		Gaffer::StandardSetPtr m_visiblePlugs;
		Gaffer::StandardSetPtr m_editablePlugs;

		IECore::ConstCompoundDataPtr m_deepSampleDicts;

		int m_highlightedKey;
		int m_highlightedCurve;

		bool m_keyPreview;
		Imath::V3f m_keyPreviewLocation;

		// details regarding spacing and layouting
		int m_xMargin;
		int m_yMargin;
		int m_textScale;
		int m_labelPadding;

		boost::optional<int> m_frameIndicatorPreviewFrame;

};

IE_CORE_DECLAREPTR( DeepSampleGadget );

} // namespace GafferImageUI

#endif // GAFFERIMAGEUI_DEEPSAMPLEGADGET_H
