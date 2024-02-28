##########################################################################
#
#  Copyright (c) 2024, Image Engine Design Inc. All rights reserved.
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

import unittest
import imath

import IECore
import IECoreScene

import Gaffer
import GafferTest

import GafferScene
import GafferSceneTest

class MeshSubdivideTest( GafferSceneTest.SceneTestCase ) :

	def test( self ) :

		pass

	@GafferTest.TestRunner.PerformanceTestMethod()
	def testSmallSourcePerf( self ):

		sphere = GafferScene.Sphere()
		sphere["divisions"].setValue( imath.V2i( 100, 100 ) )

		filter = GafferScene.PathFilter()
		filter["paths"].setValue( IECore.StringVectorData( [ '/sphere' ] ) )

		meshType = GafferScene.MeshType()
		meshType["in"].setInput( sphere["out"] )
		meshType["filter"].setInput( filter["out"] )
		meshType["meshType"].setValue( 'catmullClark' )

		subdivide = GafferScene.MeshSubdivide()
		subdivide["in"].setInput( meshType["out"] )
		subdivide["filter"].setInput( filter["out"] )
		subdivide["subdividePolygons"].setValue( True )
		subdivide["levels"].setValue( 3 )

		subdivide["in"].object( "/sphere" )

		with GafferTest.TestRunner.PerformanceScope() :
			subdivide["out"].object( "/sphere" )

	@GafferTest.TestRunner.PerformanceTestMethod()
	def testBigSourcePerfRegular( self ):

		cube = GafferScene.Cube()

		filter = GafferScene.PathFilter()
		filter["paths"].setValue( IECore.StringVectorData( [ '/cube' ] ) )

		meshType = GafferScene.MeshType()
		meshType["in"].setInput( cube["out"] )
		meshType["filter"].setInput( filter["out"] )
		meshType["meshType"].setValue( 'catmullClark' )

		# Our set of primitives is currently so limited that the only way to generate a curved primitive
		# without highly irregular vertices is subdividing a cube.
		preSubdivide = GafferScene.MeshSubdivide()
		preSubdivide["in"].setInput( meshType["out"] )
		preSubdivide["filter"].setInput( filter["out"] )
		preSubdivide["subdividePolygons"].setValue( True )
		preSubdivide["levels"].setValue( 7 )

		# May be unnecessary if we add an option to output subdivs
		remeshType = GafferScene.MeshType()
		remeshType["in"].setInput( preSubdivide["out"] )
		remeshType["filter"].setInput( filter["out"] )
		remeshType["meshType"].setValue( 'catmullClark' )

		subdivide = GafferScene.MeshSubdivide()
		subdivide["in"].setInput( remeshType["out"] )
		subdivide["filter"].setInput( filter["out"] )
		subdivide["subdividePolygons"].setValue( True )
		subdivide["levels"].setValue( 1 )

		subdivide["in"].object( "/cube" )

		with GafferTest.TestRunner.PerformanceScope() :
			subdivide["out"].object( "/cube" )

	@GafferTest.TestRunner.PerformanceTestMethod( repeat = 1)
	def testBigSourcePerfIrregular( self ):

		sphere = GafferScene.Sphere()
		sphere["divisions"].setValue( imath.V2i( 300, 300 ) )

		filter = GafferScene.PathFilter()
		filter["paths"].setValue( IECore.StringVectorData( [ '/sphere' ] ) )

		meshType = GafferScene.MeshType()
		meshType["in"].setInput( sphere["out"] )
		meshType["filter"].setInput( filter["out"] )
		meshType["meshType"].setValue( 'catmullClark' )

		subdivide = GafferScene.MeshSubdivide()
		subdivide["in"].setInput( meshType["out"] )
		subdivide["filter"].setInput( filter["out"] )
		subdivide["subdividePolygons"].setValue( True )
		subdivide["levels"].setValue( 1 )

		subdivide["in"].object( "/sphere" )

		with GafferTest.TestRunner.PerformanceScope() :
			subdivide["out"].object( "/sphere" )

if __name__ == "__main__":
	unittest.main()
