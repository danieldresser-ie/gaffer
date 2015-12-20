##########################################################################
#
#  Copyright (c) 2015, Image Engine Design Inc. All rights reserved.
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

import imath

import IECore
import IECoreImage

import Gaffer
import GafferTest
import GafferImage

class ImageTestCase( GafferTest.TestCase ) :

	class DeepImage( GafferImage.ImageNode ) :
		def __init__( self, name = "DeepImage" ) :
			GafferImage.ImageNode.__init__( self, name )
			self.addChild( Gaffer.IntPlug( "deepState", Gaffer.Plug.Direction.In, GafferImage.ImagePlug.DeepState.Messy ) )
			self.addChild( GafferImage.FormatPlug( "format", Gaffer.Plug.Direction.In ) )

			self["__empty"] = GafferImage.Empty()
			self["__empty"]["format"].setInput( self["format"] )

			self["__deepState"] = GafferImage.DeepState()
			self["__deepState"]["in"].setInput( self["__empty"]["out"] )
			self["__deepState"]["deepState"].setInput( self["deepState"] )

			self["out"].setInput( self["__deepState"]["out"] )

	IECore.registerRunTimeTyped( DeepImage )

	def assertImageHashesEqual( self, imageA, imageB ) :

		self.assertEqual( imageA["format"].hash(), imageB["format"].hash() )
		self.assertEqual( imageA["dataWindow"].hash(), imageB["dataWindow"].hash() )
		self.assertEqual( imageA["metadata"].hash(), imageB["metadata"].hash() )
		self.assertEqual( imageA["channelNames"].hash(), imageB["channelNames"].hash() )

		dataWindow = imageA["dataWindow"].getValue()
		self.assertEqual( dataWindow, imageB["dataWindow"].getValue() )

		channelNames = imageA["channelNames"].getValue()
		self.assertEqual( channelNames, imageB["channelNames"].getValue() )

		tileOrigin = GafferImage.ImagePlug.tileOrigin( dataWindow.min() )
		while tileOrigin.y < dataWindow.max().y :
			tileOrigin.x = GafferImage.ImagePlug.tileOrigin( dataWindow.min() ).x
			while tileOrigin.x < dataWindow.max().x :
				for channelName in channelNames :
					self.assertEqual(
						imageA.channelDataHash( channelName, tileOrigin ),
						imageB.channelDataHash( channelName, tileOrigin )
					)
				tileOrigin.x += GafferImage.ImagePlug.tileSize()
			tileOrigin.y += GafferImage.ImagePlug.tileSize()

	def assertImagesEqual( self, imageA, imageB, maxDifference = 0.0, ignoreMetadata = False, ignoreDataWindow = False ) :

		self.assertEqual( imageA["format"].getValue(), imageB["format"].getValue() )
		if not ignoreDataWindow :
			self.assertEqual( imageA["dataWindow"].getValue(), imageB["dataWindow"].getValue() )
		if not ignoreMetadata :
			self.assertEqual( imageA["metadata"].getValue(), imageB["metadata"].getValue() )
		self.assertEqual( imageA["channelNames"].getValue(), imageB["channelNames"].getValue() )

		difference = GafferImage.Merge()
		difference["in"][0].setInput( imageA )
		difference["in"][1].setInput( imageB )
		difference["operation"].setValue( GafferImage.Merge.Operation.Difference )

		stats = GafferImage.ImageStats()
		stats["in"].setInput( difference["out"] )
		stats["area"].setValue( imageA["format"].getValue().getDisplayWindow() )

		for channelName in imageA["channelNames"].getValue() :

			stats["channels"].setValue( IECore.StringVectorData( [ channelName ] * 4 ) )
			self.assertLessEqual( stats["max"]["r"].getValue(), maxDifference, "Channel {0}".format( channelName ) )

	## Returns an image node with an empty data window. This is useful in
	# verifying that nodes deal correctly with such inputs.
	def emptyImage( self ) :

		image = IECoreImage.ImagePrimitive( imath.Box2i(), imath.Box2i( imath.V2i( 0 ), imath.V2i( 100 ) ) )
		image["R"] = IECore.FloatVectorData()
		image["G"] = IECore.FloatVectorData()
		image["B"] = IECore.FloatVectorData()
		image["A"] = IECore.FloatVectorData()

		result = GafferImage.ObjectToImage()
		result["object"].setValue( image )

		self.assertEqual( result["out"]["dataWindow"].getValue(), imath.Box2i() )

		return result

	def deepImage( self, fmt=None ) :

		deep = self.DeepImage()
		if fmt :
			deep["format"].setValue( fmt )

		return deep

	def _testNonFlatThrows( self, node ) :

		deep = self.deepImage()

		node["in"].setInput( deep["out"] )

		deep["deepState"].setValue( GafferImage.ImagePlug.DeepState.Flat )
		self.assertNotEqual( deep["out"].imageHash(), node["out"].imageHash() )

		deep["deepState"].setValue( GafferImage.ImagePlug.DeepState.Messy )
		self.assertRaisesRegexp( RuntimeError, 'Deep data not supported in input "in*', node["out"].imageHash )

