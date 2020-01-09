##########################################################################
#
#  Copyright (c) 2020, Image Engine Design Inc. All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are
#  met:
#
#      * Redistributions of source code must retain the above
#        copyright notice, this list of conditions and the following
#        disclaimer.
#
#      * Redistributions in binary form must reproduce the above
#        copyright notice, this list of conditions and the following
#        disclaimer in the documentation and/or other materials provided with
#        the distribution.
#
#      * Neither the name of John Haddon nor the names of
#        any other contributors to this software may be used to endorse or
#        promote products derived from this software without specific prior
#        written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
#  IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
#  THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
#  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
#  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
#  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
#  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
#  PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
#  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
#  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
#  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
##########################################################################

import sys
import imath

import IECore

import Gaffer
import GafferImage
import GafferUI
import GafferImageUI

class DeepPixelInfo( GafferUI.Widget ) :

	def __init__( self, **kw ) :

		self.__column = GafferUI.ListContainer( GafferUI.ListContainer.Orientation.Vertical, spacing = 4 )

		GafferUI.Widget.__init__( self, self.__column, **kw )

		with self.__column :
			self.__gadgetWidget = GafferUI.GadgetWidget(
				bufferOptions = set(
					[ GafferUI.GLWidget.BufferOptions.Depth,
					  GafferUI.GLWidget.BufferOptions.Double, ] ), )

			self.__deepSamplesGadget = GafferImageUI.DeepSampleGadget()
			self.__gadgetWidget.getViewportGadget().setPrimaryChild( self.__deepSamplesGadget )
			self.__gadgetWidget.getViewportGadget().setVariableAspectZoom( True )

		self.__imagePlugs = []
		self.__pixel = imath.V2i( 50, 50 )

	def setImagePlugs( self, imagePlugs ):
		self.__imagePlugs = imagePlugs
		self.__imagePlugsDirtyConnections = [
			p.node().plugDirtiedSignal().connect( self.__plugDirtied ) for p in imagePlugs
		]
		self.updatePixelData()

	def setPixel( self, pixel ):
		self.__pixel = pixel
		self.updatePixelData()

	def __plugDirtied( self, plug ):
		if type( plug ) == GafferImage.ImagePlug:
			if plug.direction() == Gaffer.Plug.Direction.Out:
				self.updatePixelData()

	def updatePixelData( self ):
		print "UPDATE PIXEL DATA"
		print self.__imagePlugs
		print self.__pixel
		deepSampler = GafferImage.DeepSampler()
		deepSampler["pixel"].setValue( self.__pixel )

		allPixelDeepSamples = {}

		for p in self.__imagePlugs:
			deepSampler["image"].setInput( p )
			pixelData = deepSampler["pixelData"].getValue()
			allPixelDeepSamples[p.fullName()] = pixelData

		if len( allPixelDeepSamples ) == 1:
			allPixelDeepSamples = { "" : allPixelDeepSamples.values()[0] }

		print IECore.CompoundData( allPixelDeepSamples )
		self.__deepSamplesGadget.setDeepSamples( IECore.CompoundData( allPixelDeepSamples ) )
