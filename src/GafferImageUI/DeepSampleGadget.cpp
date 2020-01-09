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

#include "GafferImageUI/DeepSampleGadget.h"

#include "GafferUI/Pointer.h"
#include "GafferUI/Style.h"
#include "GafferUI/ViewportGadget.h"

#include "Gaffer/Animation.h"
#include "Gaffer/Context.h"
#include "Gaffer/Node.h"
#include "Gaffer/Plug.h"
#include "Gaffer/ScriptNode.h"
#include "Gaffer/UndoScope.h"

#include "IECoreGL/Selector.h"

#include "IECore/InternedString.h"
#include "IECore/NullObject.h"

#include "boost/algorithm/string.hpp"
#include "boost/bind.hpp"

#include <cmath>

using namespace Gaffer;
using namespace GafferUI;
using namespace GafferImageUI;
using namespace Imath;

namespace
{

/// Aliases that define the intended use of each
/// Gadget::Layer by the DeepSampleGadget components.
namespace AnimationLayer
{
	constexpr Gadget::Layer Grid = Gadget::Layer::Back;
	constexpr Gadget::Layer Curves = Gadget::Layer::MidBack;
	constexpr Gadget::Layer Keys = Gadget::Layer::Main;
	constexpr Gadget::Layer Axes = Gadget::Layer::MidFront;
	constexpr Gadget::Layer Overlay = Gadget::Layer::Front;
};

template<typename T>
T frameToTime( float fps, T frame )
{
	return frame / fps;
}

template<typename T>
T timeToFrame( float fps, T time )
{
	return time * fps;
}

float snapTimeToFrame( float fps, float time, float threshold=std::numeric_limits<float>::max() )
{
	float frame = timeToFrame( fps, time );
	float rounded = round( frame );

	return frameToTime( fps, std::abs( frame - rounded ) > threshold ? frame : rounded );
}

// \todo: Consider making the colorForAxes function in StandardStyle public?
//        Include names for plugs representing color? (foo.r, foo.g, foo.b)
Color3f colorFromName( std::string name )
{
	if( boost::ends_with( name, ".x" ) )
	{
		return Imath::Color3f( 0.73, 0.17, 0.17 );
	}
	else if( boost::ends_with( name, ".y" ) )
	{
		return Imath::Color3f( 0.2, 0.57, 0.2 );
	}
	else if( boost::ends_with( name, ".z" ) )
	{
		return Imath::Color3f( 0.2, 0.36, 0.74 );
	}
	else
	{
		return Color3f( 1 );
	}
}

// Compute grid line locations. Note that positions are given in raster space so
// that lines can get drawn directly.
// For the time-dimension we limit the computed locations to multiples of one
// frame plus one level of unlabeled dividing lines. Resulting at a minimum
// distance between lines of a fifth of a frame when zoomed in all the way.
// For the value dimension we allow sub-steps as small as 0.001.
struct AxisDefinition
{
	std::vector<std::pair<float, float> > main;
	std::vector<float> secondary;
};

void computeGrid( const ViewportGadget *viewportGadget, float fps, AxisDefinition &x, AxisDefinition &y )
{
	Imath::V2i resolution = viewportGadget->getViewport();

	IECore::LineSegment3f min, max;
	min = viewportGadget->rasterToWorldSpace( V2f( 0 ) );
	max = viewportGadget->rasterToWorldSpace( V2f( resolution.x, resolution.y ) );
	Imath::Box2f viewportBounds = Box2f( V2f( min.p0.x, min.p0.y ), V2f( max.p0.x, max.p0.y ) );

	Box2f viewportBoundsFrames( timeToFrame( fps, viewportBounds.min ), timeToFrame( fps, viewportBounds.max ) );
	V2i labelMinSize( 50, 20 );
	int xStride = 1;
	float yStride = 1;

	// \todo the box's size() is unrealiable because it considers the box empty for the inverted coords we seem to have here
	V2f pxPerUnit = V2f(
		resolution.x / std::abs( viewportBoundsFrames.min.x - viewportBoundsFrames.max.x ),
		resolution.y / std::abs( viewportBounds.min.y - viewportBounds.max.y ) );

	// Compute the stride to use for the time dimension.
	if( pxPerUnit.x < labelMinSize.x )
	{
		xStride = 5;
		pxPerUnit.x *= 5;

		// If there's not enough space for this zoom level, try using every 10th frame.
 		while( pxPerUnit.x < labelMinSize.x && pxPerUnit.x != 0 )
		{
			xStride *= 10;
			pxPerUnit.x *= 10;
		}
	}

	// Compute the stride to use for the value dimension.
	if( pxPerUnit.y < labelMinSize.y )
	{
		yStride = 5;
		pxPerUnit.y *= 5;

		// If there's not enough space for this zoom level, increase the spacing
		// between values to be drawn.
		while( pxPerUnit.y < labelMinSize.y && pxPerUnit.y != 0 )
		{
			yStride *= 10;
			pxPerUnit.y *= 10;
		}
	}
	else
	{
		// If we actually have too much space between values, progressively
		// decrease the stride to show smaller value deltas.
		float scale = 1;
		while( pxPerUnit.y / 10.0 > labelMinSize.y && scale > 0.001 )
		{
			yStride *= 0.1;
			pxPerUnit /= 10.0;
			scale /= 10.0;
		}
	}

	// Compute line locations based on bounds and strides in both dimensions.
	int lowerBoundX = std::floor( viewportBoundsFrames.min.x / xStride ) * xStride - xStride;
	int upperBoundX = std::ceil( viewportBoundsFrames.max.x );
	for( int i = lowerBoundX; i < upperBoundX; i += xStride )
	{
		float time = frameToTime( fps, static_cast<float>( i ) );
		x.main.push_back( std::make_pair( viewportGadget->worldToRasterSpace( V3f( time, 0, 0 ) ).x, i ) );

		float subStride = frameToTime( fps, xStride / 5.0 );
		for( int s = 1; s < 5; ++s )
		{
			x.secondary.push_back( viewportGadget->worldToRasterSpace( V3f( time + s * subStride, 0, 0 ) ).x );
		}
	}

	float lowerBoundY = std::floor( viewportBounds.max.y / yStride ) * yStride - yStride;
	float upperBoundY = viewportBounds.min.y + yStride;
	for( float j = lowerBoundY; j < upperBoundY; j += yStride )
	{
			y.main.push_back( std::make_pair( viewportGadget->worldToRasterSpace( V3f( 0, j, 0 ) ).y, j ) );
	}
}

} // namespace

//////////////////////////////////////////////////////////////////////////
// DeepSampleGadget implementation
//////////////////////////////////////////////////////////////////////////

GAFFER_GRAPHCOMPONENT_DEFINE_TYPE( GafferImageUI::DeepSampleGadget );

DeepSampleGadget::DeepSampleGadget()
	: m_visiblePlugs( new StandardSet() ), m_editablePlugs( new StandardSet() ), m_highlightedKey( -1 ), m_highlightedCurve( -1 ), m_keyPreview( false ), m_keyPreviewLocation( 0 ), m_xMargin( 60 ), m_yMargin( 20 ), m_textScale( 10 ), m_labelPadding( 5 ), m_frameIndicatorPreviewFrame( boost::none )
{
	keyPressSignal().connect( boost::bind( &DeepSampleGadget::keyPress, this, ::_1,  ::_2 ) );
	requestRender();
}

DeepSampleGadget::~DeepSampleGadget()
{
}

void DeepSampleGadget::doRenderLayer( Layer layer, const Style *style ) const
{
	Gadget::doRenderLayer( layer, style );

	glDisable( GL_DEPTH_TEST );

	const ViewportGadget *viewportGadget = ancestor<ViewportGadget>();
	Imath::V2i resolution = viewportGadget->getViewport();

	ViewportGadget::RasterScope rasterScope( viewportGadget );

	switch ( layer )
	{

	case AnimationLayer::Grid :
	{
		AxisDefinition xAxis, yAxis;
		computeGrid( viewportGadget, 0.1, xAxis, yAxis );

		Imath::Color4f axesColor( 60.0 / 255, 60.0 / 255, 60.0 / 255, 1.0f );

		// drawing base grid
		for( const auto &x : xAxis.main )
		{
			style->renderLine( IECore::LineSegment3f( V3f( x.first, 0, 0 ), V3f( x.first, resolution.y, 0 ) ), x.second == 0.0f ? 3.0 : 2.0, &axesColor );
		}

		for( const auto &y : yAxis.main )
		{
			style->renderLine( IECore::LineSegment3f( V3f( 0, y.first, 0 ), V3f( resolution.x, y.first, 0 ) ), y.second == 0.0f ? 3.0 : 2.0, &axesColor );
		}

		// drawing sub grid for frames
		for( float x : xAxis.secondary )
		{
			style->renderLine( IECore::LineSegment3f( V3f( x, 0, 0 ), V3f( x, resolution.y, 0 ) ), 1.0, &axesColor );
		}

		break;
	}

	case AnimationLayer::Curves :
	{
		/*for( const auto &member : *m_visiblePlugs )
		{
			const Animation::CurvePlug *curvePlug = IECore::runTimeCast<const Animation::CurvePlug>( &member );
			//renderCurve( curvePlug, style );
		}*/

		break;
	}

	case AnimationLayer::Keys :
	{
		Imath::Color3f black( 0, 0, 0 );

		Box2f b;

		for( auto const &imageData : m_deepSampleDicts->readable() )
		{
			const IECore::CompoundData *image = IECore::runTimeCast< IECore::CompoundData >( imageData.second.get() );

			const IECore::FloatVectorData *zData = image->member<IECore::FloatVectorData>( "Z" );
			const IECore::FloatVectorData *zBackData = image->member<IECore::FloatVectorData>( "ZBack" );
			const IECore::FloatVectorData *aData = image->member<IECore::FloatVectorData>( "A" );

			if( !zData )
			{
				continue;
			}

			if( !zBackData )
			{
				zBackData = zData;
			}

			for( auto const &channelData : image->readable() )
			{
				if( channelData.first == "Z" || channelData.first == "ZBack" )
				{
					continue;
				}
				const IECore::FloatVectorData *channel = IECore::runTimeCast< IECore::FloatVectorData >( channelData.second.get() );
			
					
				Color3f c( 1.0 );
				if( channelData.first == "R" )
				{
					c = Color3f( 1.0f, 0.0f, 0.0f );
				}
				else if( channelData.first == "G" )
				{
					c = Color3f( 0.0f, 1.0f, 0.0f );
				}
				else if( channelData.first == "B" )
				{
					c = Color3f( 0.0f, 0.0f, 1.0f );
				}
				V2f size( 2.0f );

				IECoreGL::glColor( c );
				float accum = 0;
				float accumAlpha = 0;
				for( unsigned int i = 0; i < channel->readable().size(); i++ )
				{
					V2f startPosition = viewportGadget->worldToRasterSpace( V3f( zData->readable()[i], accum, 0 ) );
					accum += channel->readable()[i] - channel->readable()[i] * accumAlpha;
					if( aData )
					{
						accumAlpha += aData->readable()[i] - aData->readable()[i] * accumAlpha;
					}
					V2f endPosition = viewportGadget->worldToRasterSpace( V3f( zBackData->readable()[i], accum, 0 ) );
					style->renderSolidRectangle( Box2f( startPosition - size, startPosition + size ) );
					style->renderSolidRectangle( Box2f( endPosition - size, endPosition + size ) );
				}
			}
		}
		/*for( auto &runtimeTyped : *m_editablePlugs )
		{
			Animation::CurvePlug *curvePlug = IECore::runTimeCast<Animation::CurvePlug>( &runtimeTyped );

			for( Animation::Key &key : *curvePlug )
			{
				bool isHighlighted = ( m_highlightedKey && key == *m_highlightedKey ) || ( selecting && b.intersects( V2f( key.getTime(), key.getValue() ) ));
				bool isSelected = m_selectedKeys.count( Animation::KeyPtr( &key ) ) > 0;
				V2f keyPosition = viewportGadget->worldToRasterSpace( V3f( key.getTime(), key.getValue(), 0 ) );
				style->renderAnimationKey( keyPosition, isSelected || isHighlighted ? Style::HighlightedState : Style::NormalState, isHighlighted ? 3.0 : 2.0, &black );
			}
		}*/
		break;
	}

	case AnimationLayer::Axes :
	{
		AxisDefinition xAxis, yAxis;
		computeGrid( viewportGadget, 1, xAxis, yAxis );

		// draw axes on top of everything.
		Imath::Color4f axesColor( 60.0 / 255, 60.0 / 255, 60.0 / 255, 1.0 );
		IECoreGL::glColor( axesColor ); // \todo: maybe renderSolidRectangle() should accept a userColor
		style->renderSolidRectangle( Box2f( V2f( 0 ) , V2f( m_xMargin, resolution.y - m_yMargin ) ) );
		style->renderSolidRectangle( Box2f( V2f( 0, resolution.y - m_yMargin ) , V2f( resolution.x, resolution.y ) ) );

		boost::format formatX( "%.2f" );
		boost::format formatY( "%.3f" );

		// \todo: pull matrix stack operations out of the loops.
		for( const auto &x : xAxis.main )
		{
			if( x.first < m_xMargin )
			{
				continue;
			}

			glPushMatrix();

			std::string label = boost::str( formatX % x.second );
			Box3f labelBound = style->textBound( Style::BodyText, label );

			glTranslatef( x.first - labelBound.center().x * m_textScale, resolution.y - m_labelPadding, 0.0f );
			glScalef( m_textScale, -m_textScale, m_textScale );

			style->renderText( Style::BodyText, label );

			glPopMatrix();
		}

		for( const auto &y : yAxis.main )
		{
			if( y.first > resolution.y - m_yMargin )
			{
				continue;
			}

			glPushMatrix();

			std::string label = boost::str( formatY % y.second );
			Box3f labelBound = style->textBound( Style::BodyText, label );

			glTranslatef( ( m_xMargin - m_labelPadding ) - labelBound.size().x * m_textScale, y.first + labelBound.center().y * m_textScale, 0.0f );
			glScalef( m_textScale, -m_textScale, m_textScale );

			style->renderText( Style::BodyText, label );

			glPopMatrix();
		}

		break;

	}

	case AnimationLayer::Overlay :
	{
		if( m_keyPreview )
		{
			V2f keyPosition = viewportGadget->worldToRasterSpace( m_keyPreviewLocation );
			style->renderAnimationKey( keyPosition, Style::HighlightedState, 3.0 );
		}

		break;
	}

	default:
		break;

	}
}

void DeepSampleGadget::plugDirtied( Gaffer::Plug *plug )
{
	requestRender();
}

std::string DeepSampleGadget::getToolTip( const IECore::LineSegment3f &line ) const
{
	if( int key = keyAt( line ) >= 0 )
	{
		//return boost::str( boost::format( "%f -> %f" ) % key->getTime() % key->getValue() );
		return boost::str( boost::format( "%i" ) % key );
	}

	IECore::InternedString curveName = curveAt( line );
	if( curveName.string().size() )
	{
		return curveName.string();
	}

	return "";
}

void DeepSampleGadget::frame()
{
	Box3f b;

	// trying to frame to visible curves next
	/*if( !( m_visiblePlugs->size() == 0 ) )
	{
		for( const auto &runtimeTyped : *m_visiblePlugs )
		{
			const Animation::CurvePlug *curvePlug = IECore::runTimeCast<const Animation::CurvePlug>( &runtimeTyped );

			for( const auto &key : *curvePlug )
 			{
				b.extendBy( V3f( key.getTime(), key.getValue(), 0 ) );
			}
		}
	}
	// setting default framing as last resort
	else
	{
		b = Box3f( V3f( -1, -1, 0), V3f( 1, 1, 0 ) );
	}

	// add some padding in case only a single key was selected
	Box3f bound( b.min - V3f( .1 ), b.max + V3f( .1 ) );

	// scale bounding box so there's some space between keys and the axis
	V3f center = bound.center();
	bound.min = center + ( bound.min - center ) * 1.2;
	bound.max = center + ( bound.max - center ) * 1.2;

	ViewportGadget *viewportGadget = ancestor<ViewportGadget>();
	// \todo: we might have to compensate for the axis we're drawing
	viewportGadget->frame( bound );
	*/

	return;
}

bool DeepSampleGadget::keyPress( GadgetPtr gadget, const KeyEvent &event )
{
	if( event.key == "F" )
	{
		frame();
		return true;
	}

	return false;
}

bool DeepSampleGadget::onTimeAxis( int y ) const
{
	const ViewportGadget *viewportGadget = ancestor<ViewportGadget>();
	Imath::V2i resolution = viewportGadget->getViewport();

	return y >= resolution.y - m_yMargin;
}

bool DeepSampleGadget::onValueAxis( int x ) const
{
	return x <= m_xMargin;
}


int DeepSampleGadget::keyAt( const IECore::LineSegment3f &position ) const
{
	return -1;
	/*
	std::vector<IECoreGL::HitRecord> selection;
	std::vector<Animation::ConstKeyPtr> keys;

	{
		ViewportGadget::SelectionScope selectionScope( position, this, selection, IECoreGL::Selector::IDRender );
		IECoreGL::Selector *selector = IECoreGL::Selector::currentSelector();
		const Style *style = this->style();
		style->bind();
		GLuint name = 1; // Name 0 is invalid, so we start at 1

		const ViewportGadget *viewportGadget = ancestor<ViewportGadget>();
		ViewportGadget::RasterScope rasterScope( viewportGadget );

		for( const auto &member : *m_editablePlugs )
		{
			const Animation::CurvePlug *curvePlug = IECore::runTimeCast<const Animation::CurvePlug>( &member );

			for( const Animation::Key &key : *curvePlug )
			{
				keys.emplace_back( &key );
				selector->loadName( name++ );
				V2f keyPosition = viewportGadget->worldToRasterSpace( V3f( key.getTime(), key.getValue(), 0 ) );
				style->renderAnimationKey( keyPosition, Style::NormalState, 4.0 ); // slightly bigger for easier selection
			}
		}
	}

	if( selection.empty() )
	{
		return nullptr;
	}

	return keys[selection[0].name-1];
	*/
}

IECore::InternedString DeepSampleGadget::curveAt( const IECore::LineSegment3f &position ) const
{
	/*std::vector<IECoreGL::HitRecord> selection;
	std::vector<Animation::ConstCurvePlugPtr> curves;

	{
		ViewportGadget::SelectionScope selectionScope( position, this, selection, IECoreGL::Selector::IDRender );
		IECoreGL::Selector *selector = IECoreGL::Selector::currentSelector();
		const Style *style = this->style();
		style->bind();
		GLuint name = 1; // Name 0 is invalid, so we start at 1

		for( const auto &runtimeTyped : *m_visiblePlugs )
		{
			const Animation::CurvePlug *curvePlug = IECore::runTimeCast<const Animation::CurvePlug>( &runtimeTyped );
			curves.emplace_back( curvePlug );
			selector->loadName( name++ );
			renderCurve( curvePlug, style );
		}
	}

	if( selection.empty() )
	{
		return nullptr;
	}

	return curves[selection[0].name-1];*/
	return "";
}

void DeepSampleGadget::setDeepSamples( IECore::ConstCompoundDataPtr deepSamples )
{
	m_deepSampleDicts = deepSamples;
	requestRender();
}


/*void DeepSampleGadget::renderCurve( const Animation::CurvePlug *curvePlug, const Style *style ) const
{
	const ViewportGadget *viewportGadget = ancestor<ViewportGadget>();
	ViewportGadget::RasterScope rasterScope( viewportGadget );

	Animation::ConstKeyPtr previousKey = nullptr;
	V2f previousKeyPosition = V2f( 0 );

	bool isHighlighted = curvePlug == m_highlightedCurve;

	for( const auto &key : *curvePlug )
	{
		V2f keyPosition = viewportGadget->worldToRasterSpace( V3f( key.getTime(), key.getValue(), 0 ) );

		if( previousKey )
		{
			// \todo: needs tangent computation/hand-off as soon as we support more interpolation modes
			//        consider passing interpolation into renderCurveSegment to handle all drawing there

			const Imath::Color3f color3 = colorFromName( drivenPlugName( curvePlug ) );

			if( key.getType() == Gaffer::Animation::Linear )
			{
				style->renderAnimationCurve( previousKeyPosition, keyPosition, /inTangent/   V2f( 0 ), /outTangent/   V2f( 0 ), isHighlighted ? Style::HighlightedState : Style::NormalState, &color3 );
			}
			else if( key.getType() == Gaffer::Animation::Step )
			{
				const Color4f color4( color3[0], color3[1], color3[2], 1.0f );
				// \todo: replace with linear curve segment to get highlighting
				style->renderLine( IECore::LineSegment3f( V3f( previousKeyPosition.x, previousKeyPosition.y, 0 ), V3f( keyPosition.x, previousKeyPosition.y, 0) ), 0.5, &color4 );
				style->renderLine( IECore::LineSegment3f( V3f( keyPosition.x, previousKeyPosition.y, 0 ), V3f( keyPosition.x, keyPosition.y, 0 ) ), 0.5, &color4 );
			}
		}

		previousKey = &key;
		previousKeyPosition = keyPosition;
	}
}
*/
