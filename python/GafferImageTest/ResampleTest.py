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

		testImage = GafferImage.ImagePlug()

		formatQuery = GafferImage.FormatQuery()
		formatQuery["image"].setInput( testImage )

		sliceNear = GafferImage.DeepSlice()
		sliceNear["in"].setInput( testImage )
		sliceNear["nearClip"]["enabled"].setValue( False )
		sliceNear["farClip"]["enabled"].setValue( True )
		sliceNear["flatten"].setValue( False )

		flattenedNear = GafferImage.DeepToFlat()
		flattenedNear["in"].setInput( sliceNear["out"] )
		flattenedNear["depthMode"].setValue( GafferImage.DeepToFlat.DepthMode.Range )

		flatSliceNear = GafferImage.DeepSlice()
		flatSliceNear["in"].setInput( testImage )
		flatSliceNear["nearClip"]["enabled"].setValue( False )
		flatSliceNear["farClip"]["enabled"].setValue( True )
		flatSliceNear["farClip"]["value"].setInput( sliceNear["farClip"]["value"] )
		flatSliceNear["flatten"].setValue( True )

		sliceFar = GafferImage.DeepSlice()
		sliceFar["in"].setInput( testImage )
		sliceFar["nearClip"]["enabled"].setValue( True )
		sliceFar["farClip"]["enabled"].setValue( False )
		sliceFar["flatten"].setValue( False )


		flattenedFar = GafferImage.DeepToFlat()
		flattenedFar["in"].setInput( sliceFar["out"] )
		flattenedFar["depthMode"].setValue( GafferImage.DeepToFlat.DepthMode.Range )

		flatSliceFar = GafferImage.DeepSlice()
		flatSliceFar["in"].setInput( testImage )
		flatSliceFar["nearClip"]["enabled"].setValue( True )
		flatSliceFar["nearClip"]["value"].setInput( sliceFar["nearClip"]["value"] )
		flatSliceFar["farClip"]["enabled"].setValue( False )
		flatSliceFar["flatten"].setValue( True )

		sliceMiddle = GafferImage.DeepSlice()
		sliceMiddle["in"].setInput( testImage )
		sliceMiddle["nearClip"]["enabled"].setValue( True )
		sliceMiddle["farClip"]["enabled"].setValue( True )
		sliceMiddle["flatten"].setValue( False )

		flattenedMiddle = GafferImage.DeepToFlat()
		flattenedMiddle["in"].setInput( sliceMiddle["out"] )
		flattenedMiddle["depthMode"].setValue( GafferImage.DeepToFlat.DepthMode.Range )

		flatSliceMiddle = GafferImage.DeepSlice()
		flatSliceMiddle["in"].setInput( testImage )
		flatSliceMiddle["nearClip"]["enabled"].setValue( True )
		flatSliceMiddle["nearClip"]["value"].setInput( sliceMiddle["nearClip"]["value"] )
		flatSliceMiddle["farClip"]["enabled"].setValue( True )
		flatSliceMiddle["farClip"]["value"].setInput( sliceMiddle["farClip"]["value"] )
		flatSliceMiddle["flatten"].setValue( True )

		flattenedInput = GafferImage.DeepToFlat()
		flattenedInput["in"].setInput( testImage )
		flattenedInput["depthMode"].setValue( GafferImage.DeepToFlat.DepthMode.None_ )

		flatSliceNearWithoutDepth = GafferImage.DeleteChannels()
		flatSliceNearWithoutDepth["in"].setInput( flatSliceNear["out"] )
		flatSliceNearWithoutDepth["channels"].setValue( "Z ZBack" )

		flatSliceFarWithoutDepth = GafferImage.DeleteChannels()
		flatSliceFarWithoutDepth["in"].setInput( flatSliceFar["out"] )
		flatSliceFarWithoutDepth["channels"].setValue( "Z ZBack" )

		flatSliceMiddleWithoutDepth = GafferImage.DeleteChannels()
		flatSliceMiddleWithoutDepth["in"].setInput( flatSliceMiddle["out"] )
		flatSliceMiddleWithoutDepth["channels"].setValue( "Z ZBack" )

		nearOverFar = GafferImage.Merge()
		nearOverFar["operation"].setValue( GafferImage.Merge.Operation.Over )
		nearOverFar["in"][0].setInput( flatSliceFarWithoutDepth["out"] )
		nearOverFar["in"][1].setInput( flatSliceNearWithoutDepth["out"] )

		nearOverMiddleOverFar = GafferImage.Merge()
		nearOverMiddleOverFar["operation"].setValue( GafferImage.Merge.Operation.Over )
		nearOverMiddleOverFar["in"][0].setInput( flatSliceFarWithoutDepth["out"] )
		nearOverMiddleOverFar["in"][1].setInput( flatSliceMiddleWithoutDepth["out"] )
		nearOverMiddleOverFar["in"][2].setInput( flatSliceNearWithoutDepth["out"] )

		tidyInput = GafferImage.DeepTidy()
		tidyInput["in"].setInput( testImage )

		sampleCountsInput = GafferImage.DeepSampleCounts()
		sampleCountsInput["in"].setInput( tidyInput["out"] )

		sampleCountsNear = GafferImage.DeepSampleCounts()
		sampleCountsNear["in"].setInput( sliceNear["out"] )

		sampleCountsFar = GafferImage.DeepSampleCounts()
		sampleCountsFar["in"].setInput( sliceFar["out"] )

		sampleCountsMiddle = GafferImage.DeepSampleCounts()
		sampleCountsMiddle["in"].setInput( sliceMiddle["out"] )

		sampleCountsNearFar = GafferImage.Merge()
		sampleCountsNearFar["operation"].setValue( GafferImage.Merge.Operation.Add )
		sampleCountsNearFar["in"][0].setInput( sampleCountsNear["out"] )
		sampleCountsNearFar["in"][1].setInput( sampleCountsFar["out"] )

		sampleCountsNearMiddleFar = GafferImage.Merge()
		sampleCountsNearMiddleFar["operation"].setValue( GafferImage.Merge.Operation.Add )
		sampleCountsNearMiddleFar["in"][0].setInput( sampleCountsNear["out"] )
		sampleCountsNearMiddleFar["in"][1].setInput( sampleCountsMiddle["out"] )
		sampleCountsNearMiddleFar["in"][2].setInput( sampleCountsFar["out"] )

		tidyNear = GafferImage.DeepTidy()
		tidyNear["in"].setInput( sliceNear["out"] )

		tidyFar = GafferImage.DeepTidy()
		tidyFar["in"].setInput( sliceFar["out"] )

		tidyMiddle = GafferImage.DeepTidy()
		tidyMiddle["in"].setInput( sliceMiddle["out"] )

		holdoutConstant = GafferImage.Constant()
		holdoutConstant["format"].setInput( formatQuery["format"] )

		holdoutDepth = GafferImage.FlatToDeep()
		holdoutDepth["in"].setInput( holdoutConstant["out"] )

		holdout = GafferImage.DeepHoldout()
		holdout["in"].setInput( testImage )
		holdout["holdout"].setInput( holdoutDepth["out"] )

		holdoutWithoutDepth = GafferImage.DeleteChannels()
		holdoutWithoutDepth["in"].setInput( holdout["out"] )
		holdoutWithoutDepth["channels"].setValue( "Z ZBack" )

		random.seed( 42 )

		for image, zStart, zEnd in [
			( allInts["out"], 0, 4 ),
			( allFloats["out"], 0, 4 ),
			( allCombined["out"], 0, 4 ),
			( representativeImage["out"], 4, 11 ),
		]:

			testImage.setInput( image )

			# Since some of our tests have samples at integer depths, test specifically in the neighbourhood
			# of integer depths, then test at a bunch of random depths as well
			for depth in (
				[ i + o for i in range( zStart, zEnd + 1) for o in [ -5e-7, 0, 5e-7 ] ] +
				[ random.uniform( zStart, zEnd ) for i in range( 20 ) ]
			):
				with self.subTest( mode = "Near/Far", name = image.node().getName(), depth = depth ) :
					sliceNear["farClip"]["value"].setValue( depth )
					sliceFar["nearClip"]["value"].setValue( depth )

					# The output from DeepSlice should always be tidy, which we can validate by making
					# sure tidying has no effect
					self.assertImagesEqual( tidyNear["out"], sliceNear["out"] )
					self.assertImagesEqual( tidyFar["out"], sliceFar["out"] )

					# Check that the flat output from DeepSlice matches with what we get by flattening
					# the deep output
					self.assertImagesEqual( flattenedNear["out"], flatSliceNear["out"], maxDifference = 1e-6 )
					self.assertImagesEqual( flattenedFar["out"], flatSliceFar["out"], maxDifference = 1e-6 )


					# Check that we match with passing an image containing a constant depth into DeepHoldout
					holdoutDepth["depth"].setValue( depth )
					try:
						self.assertImagesEqual( flatSliceNearWithoutDepth["out"], holdoutWithoutDepth["out"], maxDifference = 3e-5 )
					except:
						# We handle point samples exactly at the threshold a little bit differently than
						# this DeepHoldout approach - the holdout is doing a DeepMerge with a black image with
						# a point sample at a fixed depth at each pixel, so the fraction of a point sample
						# exactly at the cutoff depth that comes through depends on the EXR logic for merging
						# samples ( since the cutoff image is opaque, you get 50% of an opaque sample, or 0% of
						# a non-opaque sample ).
						#
						# Our logic is different: we exclude all samples at the cutoff for farClipDepth,
						# and we include samples at the cutoff for nearClipDepth. This ensures that using
						# two DeepSlices to split an image at a particular depth, and then re-compositing it,
						# gives you something that matches the original.
						#
						# Because of this difference, in order to get the tests passing, we just shift the
						# holdout depth slightly nearer in order to get a matching result from the holdout.
						holdoutDepth["depth"].setValue( depth - 5e-7 )
						self.assertImagesEqual( flatSliceNearWithoutDepth["out"], holdoutWithoutDepth["out"], maxDifference = 3e-5 )

					# Check that using DeepSlice to take everything before a depth, and using DeepSlice to
					# take everything after a depth, results in 2 images that composite together to match
					# the original
					self.assertImagesEqual( nearOverFar["out"], flattenedInput["out"], maxDifference = 4e-6 )

					# Check that sample counts of the two slices are reasonable. The sum should be no less than
					# the original sample counts, and no more than 1 greater ( since if a sample is split by
					# the depth, it will appear in both )
					self.assertImagesEqual(
						sampleCountsInput["out"], sampleCountsNearFar["out"], maxDifference = (0,1)
					)

if __name__ == "__main__":
	unittest.main()
