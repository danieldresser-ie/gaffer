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

import os
import pathlib
import shutil
import unittest
import random
import subprocess
import time

import imath

import IECore

import Gaffer
import GafferTest
import GafferImage
import GafferImageTest

class ResampleTest( GafferImageTest.ImageTestCase ) :

	def testDataWindow( self ) :

		c = GafferImage.Constant()
		c["format"].setValue( GafferImage.Format( 100, 100 ) )
		c["color"].setValue( imath.Color4f( 1 ) )

		r = GafferImage.Resample()
		r["in"].setInput( c["out"] )
		r["matrix"].setValue(
			imath.M33f().translate( imath.V2f( 10.5, 11.5 ) ).scale( imath.V2f( 0.1 ) )
		)

		self.assertEqual(
			r["out"]["dataWindow"].getValue(),
			imath.Box2i(
				imath.V2i( 10, 11 ),
				imath.V2i( 21, 22 )
			)
		)

	def testExpectedOutput( self ) :

		def __test( fileName, size, filter ) :

			inputFileName = self.imagesPath() / fileName

			reader = GafferImage.ImageReader()
			reader["fileName"].setValue( inputFileName )

			inSize = reader["out"]["format"].getValue().getDisplayWindow().size()
			inSize = imath.V2f( inSize.x, inSize.y )

			resample = GafferImage.Resample()
			resample["in"].setInput( reader["out"] )
			resample["matrix"].setValue(
				imath.M33f().scale( imath.V2f( size.x, size.y ) / inSize )
			)
			resample["filter"].setValue( filter )
			resample["boundingMode"].setValue( GafferImage.Sampler.BoundingMode.Clamp )

			crop = GafferImage.Crop()
			crop["in"].setInput( resample["out"] )
			crop["area"].setValue( imath.Box2i( imath.V2i( 0 ), size ) )

			outputFileName = self.temporaryDirectory() / ( "%s_%dx%d_%s.exr" % ( pathlib.Path( fileName ).with_suffix(""), size.x, size.y, filter ) )
			writer = GafferImage.ImageWriter()
			writer["in"].setInput( crop["out"] )
			writer["channels"].setValue( "[RGB]" )
			writer["fileName"].setValue( outputFileName )
			writer["task"].execute()

			result = GafferImage.ImageReader()
			result["fileName"].setValue( writer["fileName"].getValue() )

			expected = GafferImage.ImageReader()
			expected["fileName"].setValue(
				self.imagesPath() / (
					"%s_%dx%d_%s.exr" % (
						pathlib.Path( fileName ).with_suffix(""),
						size.x,
						size.y,
						filter
					)
				)
			)

			self.assertImagesEqual( result["out"], expected["out"], maxDifference = 0.0005, ignoreMetadata = True )

			# Enable to write out images for visual comparison with OIIO.
			# The images will appear in a "resampleComparison" subdirectory
			# of the current directory.
			if False :

				resampleComparisonDir = pathlib.Path( "resampleComparison" )
				resampleComparisonDir.mkdir( exist_ok=True )

				shutil.copyfile( outputFileName, resampleComparisonDir / ( "gaffer_" + outputFileName.name ) )

				oiioOutputFileName = resampleComparisonDir / ( "oiio_%s_%dx%d_%s.exr" % ( pathlib.Path( fileName ).with_suffix(""), size.x, size.y, filter ) )

				subprocess.check_call(
					"oiiotool --threads 1 %s --ch R,G,B --resize:filter=%s %dx%d  -o %s" %
					(
						inputFileName,
						filter,
						size.x, size.y,
						oiioOutputFileName
					),
					shell = True
				)

		tests = [
			( "resamplePatterns.exr", imath.V2i( 4 ), "lanczos3" ),
			( "resamplePatterns.exr", imath.V2i( 40 ), "box" ),
			( "resamplePatterns.exr", imath.V2i( 101 ), "gaussian" ),
			( "resamplePatterns.exr", imath.V2i( 119 ), "mitchell" ),
		]

		for args in tests :
			with self.subTest( fileName = args[0], size = args[1], ftilter = args[2] ):
				__test( *args )

	def testInseparableFastPath( self ) :

		reader = GafferImage.ImageReader()
		reader["fileName"].setValue( self.imagesPath() / "resamplePatterns.exr" )

		# When applying an inseparable filter with no scaling, we can use a much faster code path.
		# This code path should not have any effect on the result
		resampleFastPath = GafferImage.Resample()
		resampleFastPath["in"].setInput( reader["out"] )
		resampleFastPath['filterScale'].setValue( imath.V2f( 4 ) )
		resampleFastPath["filter"].setValue( "radial-lanczos3" )

		# Force the slow code path using the "debug" parameter
		resampleReference = GafferImage.Resample()
		resampleReference["in"].setInput( reader["out"] )
		resampleReference['filterScale'].setValue( imath.V2f( 4 ) )
		resampleReference["filter"].setValue( "radial-lanczos3" )
		resampleReference["debug"].setValue( GafferImage.Resample.Debug.SinglePass )

		self.assertImagesEqual( resampleFastPath["out"], resampleReference["out"] )

	def testSincUpsize( self ) :

		c = GafferImage.Constant()
		c["format"].setValue( GafferImage.Format( 100, 100 ) )
		c["color"].setValue( imath.Color4f( 1 ) )

		r = GafferImage.Resample()
		r["matrix"].setValue( imath.M33f().scale( imath.V2f( 4 ) ) )
		r["boundingMode"].setValue( GafferImage.Sampler.BoundingMode.Clamp )
		r["filter"].setValue( "sinc" )
		r["in"].setInput( c["out"] )

		i = GafferImage.ImageAlgo.image( r["out"] )
		self.assertEqual( i["R"], IECore.FloatVectorData( [ 1.0 ] * 400 * 400 ) )

	def testExpandDataWindow( self ) :

		d = imath.Box2i( imath.V2i( 5, 6 ), imath.V2i( 101, 304 ) )
		c = GafferImage.Constant()
		c["format"].setValue( GafferImage.Format( d ) )

		r = GafferImage.Resample()
		r["in"].setInput( c["out"] )
		r["filter"].setValue( "box" )
		self.assertEqual( r["out"]["dataWindow"].getValue(), d )

		r["expandDataWindow"].setValue( True )
		self.assertEqual( r["out"]["dataWindow"].getValue(), imath.Box2i( d.min() - imath.V2i( 1 ), d.max() + imath.V2i( 1 ) ) )

		r["filterScale"].setValue( imath.V2f( 10 ) )
		self.assertEqual( r["out"]["dataWindow"].getValue(), imath.Box2i( d.min() - imath.V2i( 5 ), d.max() + imath.V2i( 5 ) ) )

	def testCancellation( self ) :

		script = Gaffer.ScriptNode()

		script["c"] = GafferImage.Constant()

		script["r"] = GafferImage.Resample()
		script["r"]["in"].setInput( script["c"]["out"] )
		script["r"]["filterScale"].setValue( imath.V2f( 2000 ) )

		bt = Gaffer.ParallelAlgo.callOnBackgroundThread( script["r"]["out"], lambda : GafferImageTest.processTiles( script["r"]["out"] ) )
		# Give background tasks time to get into full swing
		time.sleep( 0.1 )

		# Check that we can cancel them in reasonable time
		acceptableCancellationDelay = 4.0 if GafferTest.inCI() else 0.25
		t = time.time()
		bt.cancelAndWait()
		self.assertLess( time.time() - t, acceptableCancellationDelay )

		# Check that we can do the same when using a non-separable filter
		script["r"]["filter"].setValue( "disk" )

		bt = Gaffer.ParallelAlgo.callOnBackgroundThread( script["r"]["out"], lambda : GafferImageTest.processTiles( script["r"]["out"] ) )
		time.sleep( 0.1 )

		t = time.time()
		bt.cancelAndWait()
		self.assertLess( time.time() - t, acceptableCancellationDelay )

	def testNonFlatThrows( self ) :

		resample = GafferImage.Resample()
		resample["matrix"].setValue( imath.M33f().scale( imath.V2f( 0.5 ) ) )

		self.assertRaisesDeepNotSupported( resample )

	@GafferTest.TestRunner.PerformanceTestMethod( repeat = 5 )
	def testPerfHorizontal( self ) :

		imageReader = GafferImage.ImageReader()
		imageReader["fileName"].setValue( self.imagesPath() / 'deepMergeReference.exr' )

		resize = GafferImage.Resize()
		resize["in"].setInput( imageReader["out"] )
		resize["format"].setValue( GafferImage.Format( 1920, 1080, 1.000 ) )

		resample = GafferImage.Resample()
		resample["in"].setInput( resize["out"] )
		resample["filter"].setValue( 'box' )
		resample["filterScale"].setValue( imath.V2f( 300.0 ) )

		GafferImageTest.processTiles( resize["out"] )

		with GafferTest.TestRunner.PerformanceScope() :
			GafferImageTest.processTiles( resample["__horizontalPass"] )

	@GafferTest.TestRunner.PerformanceTestMethod( repeat = 5 )
	def testPerfVertical( self ) :

		imageReader = GafferImage.ImageReader()
		imageReader["fileName"].setValue( self.imagesPath() / 'deepMergeReference.exr' )

		resize = GafferImage.Resize()
		resize["in"].setInput( imageReader["out"] )
		resize["format"].setValue( GafferImage.Format( 1920, 1080, 1.000 ) )

		resample = GafferImage.Resample()
		resample["in"].setInput( resize["out"] )
		resample["filter"].setValue( 'box' )
		resample["filterScale"].setValue( imath.V2f( 300.0 ) )

		GafferImageTest.processTiles( resample["__horizontalPass"] )

		with GafferTest.TestRunner.PerformanceScope() :
			GafferImageTest.processTiles( resample["out"] )

	@GafferTest.TestRunner.PerformanceTestMethod( repeat = 5 )
	def testPerfSmallFilter( self ) :

		imageReader = GafferImage.ImageReader()
		imageReader["fileName"].setValue( self.imagesPath() / 'deepMergeReference.exr' )

		resize = GafferImage.Resize()
		resize["in"].setInput( imageReader["out"] )
		resize["format"].setValue( GafferImage.Format( 4000, 4000, 1.000 ) )

		resample = GafferImage.Resample()
		resample["in"].setInput( resize["out"] )
		resample["filter"].setValue( 'box' )
		resample["filterScale"].setValue( imath.V2f( 4.0 ) )

		GafferImageTest.processTiles( resize["out"] )

		with GafferTest.TestRunner.PerformanceScope() :
			GafferImageTest.processTiles( resample["out"] )

	@GafferTest.TestRunner.PerformanceTestMethod( repeat = 5 )
	def testPerfVerySmallFilter( self ) :

		imageReader = GafferImage.ImageReader()
		imageReader["fileName"].setValue( self.imagesPath() / 'deepMergeReference.exr' )

		resize = GafferImage.Resize()
		resize["in"].setInput( imageReader["out"] )
		resize["format"].setValue( GafferImage.Format( 4000, 4000, 1.000 ) )

		resample = GafferImage.Resample()
		resample["in"].setInput( resize["out"] )
		resample["filter"].setValue( 'box' )
		resample["filterScale"].setValue( imath.V2f( 2.0 ) )

		GafferImageTest.processTiles( resize["out"] )

		with GafferTest.TestRunner.PerformanceScope() :
			GafferImageTest.processTiles( resample["out"] )

	@GafferTest.TestRunner.PerformanceTestMethod( repeat = 1 )
	def testPerfInseparableLanczos( self ) :

		imageReader = GafferImage.ImageReader()
		imageReader["fileName"].setValue( self.imagesPath() / 'deepMergeReference.exr' )

		resize = GafferImage.Resize()
		resize["in"].setInput( imageReader["out"] )
		resize["format"].setValue( GafferImage.Format( 64, 64 ) )

		resample = GafferImage.Resample()
		resample["in"].setInput( resize["out"] )
		resample["filter"].setValue( 'radial-lanczos3' )
		resample["filterScale"].setValue( imath.V2f( 20.0 ) )

		GafferImageTest.processTiles( resize["out"] )

		with GafferTest.TestRunner.PerformanceScope() :
			GafferImageTest.processTiles( resample["out"] )

	@GafferTest.TestRunner.PerformanceTestMethod( repeat = 1 )
	def testPerfInseparableDisk( self ) :

		imageReader = GafferImage.ImageReader()
		imageReader["fileName"].setValue( self.imagesPath() / 'deepMergeReference.exr' )

		resize = GafferImage.Resize()
		resize["in"].setInput( imageReader["out"] )
		resize["format"].setValue( GafferImage.Format( 1920, 1920 ) )

		resample = GafferImage.Resample()
		resample["in"].setInput( resize["out"] )
		resample["filter"].setValue( 'disk' )
		resample["filterScale"].setValue( imath.V2f( 20.0 ) )

		GafferImageTest.processTiles( resize["out"] )

		with GafferTest.TestRunner.PerformanceScope() :
			GafferImageTest.processTiles( resample["out"] )

	@GafferTest.TestRunner.PerformanceTestMethod( repeat = 1 )
	def testPerfInseparableAwkwardSize( self ) :

		imageReader = GafferImage.ImageReader()
		imageReader["fileName"].setValue( self.imagesPath() / 'deepMergeReference.exr' )

		resize = GafferImage.Resize()
		resize["in"].setInput( imageReader["out"] )
		resize["format"].setValue( GafferImage.Format( 6000, 6000 ) )

		resample = GafferImage.Resample()
		resample["in"].setInput( resize["out"] )
		resample["filter"].setValue( 'disk' )
		resample["filterScale"].setValue( imath.V2f( 1.1 ) )

		GafferImageTest.processTiles( resize["out"] )

		with GafferTest.TestRunner.PerformanceScope() :
			GafferImageTest.processTiles( resample["out"] )

	def testBruteForceDeep( self ):

		representativeImagePath = GafferImageTest.ImageTestCase.imagesPath() / "representativeDeepImage.exr"
		deepIntPointsPath = GafferImageTest.ImageTestCase.imagesPath() / "deepIntPoints.exr"
		deepIntVolumesPath = GafferImageTest.ImageTestCase.imagesPath() / "deepIntVolumes.exr"
		deepFloatPointsPath = GafferImageTest.ImageTestCase.imagesPath() / "deepFloatPoints.exr"
		deepFloatVolumesPath = GafferImageTest.ImageTestCase.imagesPath() / "deepFloatVolumes.exr"

		representativeImage = GafferImage.ImageReader( "representativeDeep" )
		representativeImage["fileName"].setValue( representativeImagePath )
		# This image has some high RGB values in it, which make it less meaningful to compare error tolerances.
		# Normalize it so the RGB values are in an approximately 0-1 range like the alpha.
		representativeImageNormalized = GafferImage.Grade()
		representativeImageNormalized["in"].setInput( representativeImage["out"] )
		representativeImageNormalized["multiply"].setValue( imath.Color4f( 0.2, 0.2, 0.2, 1 ) )

		intPoints = GafferImage.ImageReader()
		intPoints["fileName"].setValue( deepIntPointsPath )
		intVolumes = GafferImage.ImageReader()
		intVolumes["fileName"].setValue( deepIntVolumesPath )
		floatPoints = GafferImage.ImageReader()
		floatPoints["fileName"].setValue( deepFloatPointsPath )
		floatVolumes = GafferImage.ImageReader()
		floatVolumes["fileName"].setValue( deepFloatVolumesPath )

		allInts = GafferImage.DeepMerge( "allInts" )
		allInts["in"][0].setInput( intPoints["out"] )
		allInts["in"][1].setInput( intVolumes["out"] )

		allFloats = GafferImage.DeepMerge( "allFloats" )
		allFloats["in"][0].setInput( floatPoints["out"] )
		allFloats["in"][1].setInput( floatVolumes["out"] )

		allCombined = GafferImage.DeepMerge( "allCombined" )
		allCombined["in"][0].setInput( intPoints["out"] )
		allCombined["in"][1].setInput( intVolumes["out"] )
		allCombined["in"][2].setInput( floatPoints["out"] )
		allCombined["in"][3].setInput( floatVolumes["out"] )

		# By upsizing this test image, we produce a test image where there are lots of identical samples
		# in adjacent pixels, which could help exercise special cases when merging samples at identical depths
		allCombinedUpsize = GafferImage.Resize( "allCombinedUpsize" )
		allCombinedUpsize["in"].setInput( allCombined["out"] )
		allCombinedUpsize["format"].setValue( GafferImage.Format( 128, 128, 1.000 ) )
		allCombinedUpsize["filter"].setValue( 'box' )

		testImage = GafferImage.ImagePlug()

		deepResample = GafferImage.Resample()
		deepResample["in"].setInput( testImage )
		deepResample["expandDataWindow"].setValue( True )

		# TODO REMOVE

		deepTidy = GafferImage.DeepTidy()
		deepTidy["in"].setInput( testImage )
		deepResample["in"].setInput( deepTidy["out"] )

		# TODO REMOVE

		flattenAfter = GafferImage.DeepToFlat()
		flattenAfter["in"].setInput( deepResample["out"] )
		# The filtered depth output from Flatten is affected enormously by curve shape discrepancy,
		# so we just won't check Z. The tests using DeepSlice are used to check the depth.
		flattenAfter['depthMode'].setValue( GafferImage.DeepToFlat.DepthMode.None_ )

		flattenBefore = GafferImage.DeepToFlat()
		flattenBefore["in"].setInput( testImage )
		flattenBefore['depthMode'].setValue( GafferImage.DeepToFlat.DepthMode.None_ )

		flatResample = GafferImage.Resample()
		flatResample["in"].setInput( flattenBefore["out"] )
		flatResample["expandDataWindow"].setValue( True )
		flatResample["matrix"].setInput( deepResample["matrix"] )
		flatResample["filter"].setInput( deepResample["filter"] )
		flatResample["filterScale"].setInput( deepResample["filterScale"] )

		sliceAfter = GafferImage.DeepSlice()
		sliceAfter["in"].setInput( deepResample["out"] )
		sliceAfter["farClip"]["enabled"].setValue( True )
		sliceAfter["flatten"].setValue( True )

		sliceAfterNoZ = GafferImage.DeleteChannels()
		sliceAfterNoZ["in"].setInput( sliceAfter["out"] )
		sliceAfterNoZ["channels"].setValue( "Z ZBack" )

		referenceSlice = GafferImage.DeepSlice()
		referenceSlice["in"].setInput( testImage )
		referenceSlice["farClip"]["enabled"].setValue( True )
		referenceSlice["farClip"]["value"].setInput( sliceAfter["farClip"]["value"] )
		referenceSlice["flatten"].setValue( True )

		referenceSliceNoZ = GafferImage.DeleteChannels()
		referenceSliceNoZ["in"].setInput( referenceSlice["out"] )
		referenceSliceNoZ["channels"].setValue( "Z ZBack" )

		referenceSliceResample = GafferImage.Resample()
		referenceSliceResample["in"].setInput( referenceSliceNoZ["out"] )
		referenceSliceResample["expandDataWindow"].setValue( True )
		referenceSliceResample["matrix"].setInput( deepResample["matrix"] )
		referenceSliceResample["filter"].setInput( deepResample["filter"] )
		referenceSliceResample["filterScale"].setInput( deepResample["filterScale"] )

		linearReferenceSliceTidy = GafferImage.DeepTidy()
		linearReferenceSliceTidy["in"].setInput( testImage )

		convertToLinear = GafferImage.Premultiply()
		convertToLinear["in"].setInput( linearReferenceSliceTidy["out"] )
		convertToLinear["channels"].setValue( '[RGBA]' )
		convertToLinear["useDeepVisibility"].setValue( True )

		linearReferenceDontUseA = GafferImage.Shuffle()
		linearReferenceDontUseA["in"].setInput( convertToLinear["out"] )
		linearReferenceDontUseA["shuffles"].addChild( Gaffer.ShufflePlug( "A", "swapA", True ) )

		linearReferenceSlice = GafferImage.DeepSlice()
		linearReferenceSlice["in"].setInput( linearReferenceDontUseA["out"] )
		linearReferenceSlice["farClip"]["enabled"].setValue( True )
		linearReferenceSlice["farClip"]["value"].setInput( sliceAfter["farClip"]["value"] )
		linearReferenceSlice["flatten"].setValue( True )

		linearReferenceRestoreA = GafferImage.Shuffle()
		linearReferenceRestoreA["in"].setInput( linearReferenceSlice["out"] )
		linearReferenceRestoreA["shuffles"].addChild( Gaffer.ShufflePlug( "swapA", "A", True ) )

		linearReferenceSliceNoZ = GafferImage.DeleteChannels()
		linearReferenceSliceNoZ["in"].setInput( linearReferenceRestoreA["out"] )
		linearReferenceSliceNoZ["channels"].setValue( "Z ZBack" )

		linearReferenceSliceResample = GafferImage.Resample()
		linearReferenceSliceResample["in"].setInput( linearReferenceSliceNoZ["out"] )
		linearReferenceSliceResample["expandDataWindow"].setValue( True )
		linearReferenceSliceResample["matrix"].setInput( deepResample["matrix"] )
		linearReferenceSliceResample["filter"].setInput( deepResample["filter"] )
		linearReferenceSliceResample["filterScale"].setInput( deepResample["filterScale"] )

		random.seed( 42 )

		for image, zStart, zEnd in [
			( allInts["out"], 0, 4 ),
			( allFloats["out"], 0, 4 ),
			( allCombined["out"], 0, 4 ),
			#( allCombinedUpsize["out"], 0, 4 ), #TODO
			#( representativeImageNormalized["out"], 4, 11 ),
		]:

			testImage.setInput( image )

			for scale, filter, filterScale in [
				# Results in kernel of [ 0.333333, 1, 0.333333 ]
				# Only samples 9 pixels, moderate weights, reasonable for basic tests
				( 1, "triangle", 1.5 ), 

				# Test a moderate upscale
				( 1.5, "triangle", 1.5 ), 

				# Test a downscale
				( 0.4, "triangle", 1.5 ), 

				# Results in kernel of [ 5.99688e-05, 0.21747, 1, 0.21747, 5.99688e-05 ]
				# Wide, with some very tiny filter weights
				( 1, "blackman-harris", 1.3333334 ),

				# Results in kernel of [ 0.0151259, -0.144255, 1, -0.144255, 0.0151259 ]
				# Fairly wide, with negative lobes
				( 1, "lanczos3", 0.75 ), 
			]:

				deepResample["matrix"].setValue( imath.M33f( ( scale, 0, 0 ), ( 0, scale, 0 ), ( 0, 0, 1 ) ) )
				deepResample["filter"].setValue( filter )
				deepResample["filterScale"].setValue( imath.V2f( filterScale ) )

				with self.subTest( scale = scale, filter = filter, filterScale = filterScale, name = image.node().getName() ) :
					self.assertImagesEqual( flattenAfter["out"], flatResample["out"], maxDifference = 2e-6 )

					# Since some of our tests have samples at integer depths, test specifically in the neighbourhood
					# of integer depths, then test at a bunch of random depths as well
					for depth in (
						[ ( zStart + zEnd ) * 0.5 ] #TODO
						#[ i + o for i in range( zStart, zEnd + 1) for o in [ -5e-7, 0, 5e-7 ] ] +
						#[ random.uniform( zStart, zEnd ) for i in range( 20 ) ]
					):
						with self.subTest( scale = scale, filter = filter, filterScale = filterScale, name = image.node().getName(), depth = depth ) :
							sliceAfter["farClip"]["value"].setValue( depth )

							print( sliceAfterNoZ["out"].channelNames(), linearReferenceSliceResample["out"].channelNames() )
							self.assertImagesEqual(
								referenceSliceResample["out"], sliceAfterNoZ["out"], 
								maxDifference = ( -0.2, 1e-6 )
							)
							self.assertImagesEqual(
								linearReferenceSliceResample["out"], sliceAfterNoZ["out"],
								maxDifference = ( -1e-6, 1 )
							)

if __name__ == "__main__":
	unittest.main()
