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

		self.__pixel = Gaffer.V2iPlug( "pixelCoordinates" )
		self.__channelMask = Gaffer.StringPlug( "channelMask" )
		self.__autoFrame = Gaffer.BoolPlug( "autoFrame", defaultValue = True )
		self.__logarithmic = Gaffer.BoolPlug( "logarithmic" )

		# HACK for ChannelMaskPlug
		self.__inputPlugs = Gaffer.ArrayPlug( "in", element = GafferImage.ImagePlug() )
	
		self.__dummyNode = Gaffer.Node()
		self.__dummyNode.addChild( self.__pixel )
		self.__dummyNode.addChild( self.__channelMask )
		self.__dummyNode.addChild( self.__autoFrame )
		self.__dummyNode.addChild( self.__logarithmic )
		self.__dummyNode.addChild( self.__inputPlugs )

		self.__uiPlugDirtiedConnection = self.__dummyNode.plugDirtiedSignal().connect( Gaffer.WeakMethod( self.__uiPlugDirtied ) )

		with self.__column :
			with GafferUI.ListContainer( GafferUI.ListContainer.Orientation.Horizontal, spacing = 4 ) :
				self.__p1 = GafferUI.PlugWidget( self.__pixel )
				self.__p2 = GafferUI.PlugWidget( GafferImageUI.ChannelMaskPlugValueWidget( self.__channelMask ) )
				self.__p3 = GafferUI.PlugWidget( self.__autoFrame )
				self.__p4 = GafferUI.PlugWidget( self.__logarithmic )

			self.__gadgetWidget = GafferUI.GadgetWidget(
				bufferOptions = set(
					[ GafferUI.GLWidget.BufferOptions.Depth,
					  GafferUI.GLWidget.BufferOptions.Double, ] ), )

			self.__deepSamplesGadget = GafferImageUI.DeepSampleGadget()
			self.__gadgetWidget.getViewportGadget().setPrimaryChild( self.__deepSamplesGadget )
			self.__gadgetWidget.getViewportGadget().setVariableAspectZoom( True )

		self.__imagePlugs = []

	def __uiPlugDirtied( self, plug ):
		if plug == self.__pixel or plug == self.__channelMask:
			self.updatePixelData()
		elif plug == self.__logarithmic:
			self.__deepSamplesGadget.setLogarithmic( plug.getValue() )
		elif plug == self.__autoFrame:
			self.__deepSamplesGadget.setAutoFrame( plug.getValue() )

	def setImagePlugs( self, imagePlugs ):
		self.__imagePlugs = imagePlugs
		self.__imagePlugsDirtyConnections = [
			p.node().plugDirtiedSignal().connect( self.__plugDirtied ) for p in imagePlugs
		]
		self.__inputPlugs.resize( len( imagePlugs ) )
		for i in range( len( imagePlugs ) ):
			self.__inputPlugs[i].setInput( imagePlugs[ i ] )
		self.updatePixelData()

	def setPixel( self, pixel ):
		self.__pixel.setValue( pixel )

	def __plugDirtied( self, plug ):
		if type( plug ) == GafferImage.ImagePlug:
			if plug.direction() == Gaffer.Plug.Direction.Out:
				self.updatePixelData()

	def updatePixelData( self ):

		allPixelDeepSamples = {}
		channelMask = self.__channelMask.getValue()

		for p in self.__imagePlugs:
			# TODO - repeated construction of DeepSampler is stupid.  Why does it fail to
			# return the correct results if I do a setInput in this loop?
			deepSampler = GafferImage.DeepSampler()
			deepSampler["pixel"].setValue( self.__pixel.getValue() )
			deepSampler["image"].setInput( p )

			with p.node().scriptNode().context():
				pixelData = deepSampler["pixelData"].getValue()

			maskedPixelData = IECore.CompoundData()
			for name, data in pixelData.items():
				if name in [ "Z", "ZBack", "A" ] or IECore.StringAlgo.matchMultiple( name, channelMask ):
					maskedPixelData[name] = data 
			allPixelDeepSamples[p.fullName()] = maskedPixelData

		self.__deepSamplesGadget.setDeepSamples( IECore.CompoundData( allPixelDeepSamples ) )
