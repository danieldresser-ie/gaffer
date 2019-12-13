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

#include "Gaffer/Animation.h"
#include "Gaffer/StandardSet.h"

#include "boost/optional.hpp"

namespace Gaffer
{

	IE_CORE_FORWARDDECLARE( Context );

}

namespace GaffeImageUI
{

class GAFFERUI_API DeepSampleGadget : public GafferUI::Gadget
{

	public :

		DeepSampleGadget();

		~DeepSampleGadget() override;

		GAFFER_GRAPHCOMPONENT_DECLARE_TYPE( GaffeImageUI::DeepSampleGadget, DeepSampleGadgetTypeId, Gadget );

		void setImagePlug( GafferImage::ImagePlug *imagePlug);
		GafferImage::ImagePlug *getImagePlug();

		void setPixel( Imath::V2f pixel );
		Imath::V2f *getPixel() const;

		void setContext( Gaffer::Context *context );
		Gaffer::Context *getContext() const;

		std::string getToolTip( const IECore::LineSegment3f &line ) const override;

	protected :

		void doRenderLayer( Layer layer, const Style *style ) const override;

	private :

		void frame();

		void plugDirtied( Gaffer::Plug *plug );

		bool buttonPress( GadgetPtr gadget, const ButtonEvent &event );
		bool buttonRelease( GadgetPtr gadget, const ButtonEvent &event );
		bool keyPress( GadgetPtr gadget, const KeyEvent &event );
		bool keyRelease( GadgetPtr gadget, const KeyEvent &event );

		bool mouseMove( GadgetPtr gadget, const ButtonEvent &event );
		IECore::RunTimeTypedPtr dragBegin( GadgetPtr gadget, const DragDropEvent &event );
		bool dragEnter( GadgetPtr gadget, const DragDropEvent &event );
		bool dragMove( GadgetPtr gadget, const DragDropEvent &event );
		bool dragEnd( GadgetPtr gadget, const DragDropEvent &event );
		bool leave();

		// Find elements at certain positions
		Gaffer::DeepSample::ConstKeyPtr keyAt( const IECore::LineSegment3f &position ) const;

		bool frameIndicatorUnderMouse( const IECore::LineSegment3f &position ) const;

		void visiblePlugAdded( Gaffer::Set *set, IECore::RunTimeTyped *member );
		void visiblePlugRemoved( Gaffer::Set *set, IECore::RunTimeTyped *member );

		void editablePlugAdded( Gaffer::Set *set, IECore::RunTimeTyped *member );
		void editablePlugRemoved( Gaffer::Set *set, IECore::RunTimeTyped *member );

		void renderCurve( const Gaffer::DeepSample::CurvePlug *curvePlug, const Style *style ) const;
		void renderFrameIndicator( int frame, const Style *style, bool preview=false, float lineWidth=2.0 ) const;

		bool plugSetAcceptor( const Gaffer::Set *s, const Gaffer::Set::Member *m );
		void updateKeyPreviewLocation( const Gaffer::DeepSample::CurvePlug *curvePlug, float time );

		std::string undoMergeGroup() const;

		bool onTimeAxis( int y ) const;
		bool onValueAxis( int x ) const;

		Gaffer::Context *m_context;

		Gaffer::StandardSetPtr m_visiblePlugs;
		Gaffer::StandardSetPtr m_editablePlugs;

		std::set<Gaffer::DeepSample::KeyPtr> m_selectedKeys;
		std::map<const Gaffer::DeepSample::Key*, std::pair<float, float> > m_originalKeyValues;

		Imath::V2f m_dragStartPosition;
		Imath::V2f m_lastDragPosition;

		enum class DragMode
		{
			None,
			Selecting,
			Moving,
			MoveFrame
		};

		DragMode m_dragMode;

		enum class MoveAxis
		{
			Both,
			Undefined,
			X,
			Y
		};

		MoveAxis m_moveAxis;

		Gaffer::DeepSample::KeyPtr m_snappingClosestKey;
		Gaffer::DeepSample::KeyPtr m_highlightedKey;
		Gaffer::DeepSample::CurvePlugPtr m_highlightedCurve;

		std::set<std::pair<Gaffer::DeepSample::KeyPtr, Gaffer::DeepSample::CurvePlugPtr> > m_overwrittenKeys;

		int m_mergeGroupId;

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

} // namespace GaffeImageUI

#endif // GAFFERIMAGEUI_DEEPSAMPLEGADGET_H
