//////////////////////////////////////////////////////////////////////////
//
//  Copyright (c) 2017, John Haddon. All rights reserved.
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

#include "boost/bind.hpp"

#include "OpenEXR/ImathEuler.h"

#include "IECore/Exception.h"

#include "GafferUI/RotateHandle.h"

using namespace Imath;
using namespace IECore;
using namespace GafferUI;

//////////////////////////////////////////////////////////////////////////
//
//////////////////////////////////////////////////////////////////////////

namespace
{

float closestRotation( const V2f &p, float targetRotation )
{
	const float r = atan2( p.y, p.x );
	return Eulerf::angleMod( r - targetRotation ) + targetRotation;
}

} // namespace

//////////////////////////////////////////////////////////////////////////
// RotateHandle
//////////////////////////////////////////////////////////////////////////

IE_CORE_DEFINERUNTIMETYPED( RotateHandle );

RotateHandle::RotateHandle( Style::Axes axes )
	:	Handle( defaultName<RotateHandle>() ), m_axes( Style::X )
{
	setAxes( axes );
	dragMoveSignal().connect( boost::bind( &RotateHandle::dragMove, this, ::_2 ) );
}

RotateHandle::~RotateHandle()
{
}

void RotateHandle::setAxes( Style::Axes axes )
{
	if( axes == m_axes )
	{
		return;
	}

	if( axes > Style::Z )
	{
		throw IECore::Exception( "Unsupported axes" );
	}

	m_axes = axes;
	requestRender();
}

Style::Axes RotateHandle::getAxes() const
{
	return m_axes;
}

float RotateHandle::rotation( const DragDropEvent &event ) const
{
	return closestRotation( m_drag.position( event ), m_rotation ) - closestRotation( m_drag.startPosition(), 0.0f );
}

void RotateHandle::renderHandle( const Style *style, Style::State state ) const
{
	style->renderRotateHandle( m_axes, state );
}

void RotateHandle::dragBegin( const DragDropEvent &event )
{
	V3f axis0( 0.0f );
	V3f axis1( 0.0f );
	axis0[(m_axes+1)%3] = 1.0f;
	axis1[(m_axes+2)%3] = 1.0f;
	m_drag = PlanarDrag( this, V3f( 0 ), axis0, axis1, event );
	m_rotation = closestRotation( m_drag.position( event ), 0.0f );
}

bool RotateHandle::dragMove( const DragDropEvent &event )
{
	// We can only recover an angle in the range -PI, PI from the 2d position
	// that our drag gives us, but we want to be able to support continuous
	// values and multiple revolutions. Here we keep track of the current rotation
	// position so that we can adjust correctly in `RotateHandle::rotation()`.
	m_rotation = closestRotation( m_drag.position( event ), m_rotation );
	return false;
}
