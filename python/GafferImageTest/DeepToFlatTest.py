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

import os
import math
import copy
import sys
import random

import unittest
import imath

import IECore

import Gaffer
import GafferTest
import GafferImage
import GafferOSL
import GafferImageTest

class DeepToFlatTest( GafferImageTest.ImageTestCase ) :

	# Check that my math is correct, and adding extra samples without changing the shape of the curve
	# does not affect the filtered depth
	def testDepthFilterIntegralCorrect( self ):

		constant = GafferImage.Constant()
		constant["format"].setValue( GafferImage.Format( 100, 100, 1.000 ) )
		constant["color"].setValue( imath.Color4f( 0, 0, 0, 0.99000001 ) )

		flatToDeep = GafferImage.FlatToDeep()
		flatToDeep["in"].setInput( constant["out"] )
		flatToDeep["zBackMode"].setValue( GafferImage.FlatToDeep.ZBackMode.Thickness )
		flatToDeep["thickness"].setValue( 1.0 )

		createTwoSamples = GafferImageTest.DeepOversample()
		createTwoSamples["in"].setInput( flatToDeep["out"] )
		createTwoSamples["maxSampleAlpha"].setValue( 0.5 )

		oslCode = GafferOSL.OSLCode()
		oslCode["out"].addChild( Gaffer.FloatPlug( "a", direction = Gaffer.Plug.Direction.Out ) )
		oslCode["code"].setValue( 'a = P[2] > 0.5 ? u : v;' )

		varyAlpha = GafferOSL.OSLImage()
		varyAlpha["channels"].addChild( Gaffer.NameValuePlug( "A", Gaffer.FloatPlug( "value" ), True ) )
		varyAlpha["in"].setInput( createTwoSamples["out"] )
		varyAlpha["channels"][0]["value"].setInput( oslCode["out"]["a"] )

		deepToFlat = GafferImage.DeepToFlat()
		deepToFlat["in"].setInput( varyAlpha["out"] )

		oversample = GafferImageTest.DeepOversample()
		oversample["in"].setInput( varyAlpha["out"] )
		oversample["maxSampleAlpha"].setValue( 0.01 )

		oversampledDeepToFlat = GafferImage.DeepToFlat()
		oversampledDeepToFlat["in"].setInput( oversample["out"] )

		self.assertImagesEqual( deepToFlat["out"], oversampledDeepToFlat["out"], maxDifference = 0.000005 )

if __name__ == "__main__":
	unittest.main()

