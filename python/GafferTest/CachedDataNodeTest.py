##########################################################################
#
#  Copyright (c) 2026, Image Engine Design Inc. All rights reserved.
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

import IECore

import Gaffer
import GafferTest
import os
import pathlib
import random
import shutil
import tempfile

class CachedDataNodeTest( GafferTest.TestCase ) :

	def setUp( self ):
		GafferTest.TestCase.setUp( self )

		self.__altMountTemporaryDirectory = None

	def tearDown( self ):
		GafferTest.TestCase.tearDown( self )

		if self.__altMountTemporaryDirectory is not None :
			shutil.rmtree( self.__altMountTemporaryDirectory )

	def altMountTemporaryDirectory( self ):
		# Set up a temp directory in /dev/shm, which should be a separate mount, allowing us to test the case
		# where hardlinking fails.
		if self.__altMountTemporaryDirectory is None:
			self.__altMountTemporaryDirectory = pathlib.Path( tempfile.mkdtemp( prefix = "gafferTest", dir = "/dev/shm" ) )
		return self.__altMountTemporaryDirectory

	def setupComparison( self, s ) :
		s["cachedDataNode"] = Gaffer.CachedDataNode()
		s["compareBox"] = Gaffer.Box()
		s["compareBox"]["compound"] = Gaffer.CompoundDataPlug( flags = Gaffer.Plug.Flags.Default | Gaffer.Plug.Flags.Dynamic )

	def comparisonSetEntry( self, s, key, value ) :
		if value is None:
			s["cachedDataNode"].removeEntry( key )
			del s["compareBox"]["compound"][key]
		else:
			s["cachedDataNode"].setEntry( key, value )
			s["compareBox"]["compound"].addMembers( IECore.CompoundData( { key : value } ), True )

	def assertComparisonValid( self, s ) :
		keys = set( s["cachedDataNode"]["keys"].getValue() )

		compoundData = IECore.CompoundData()
		s["compareBox"]["compound"].fillCompoundData( compoundData )

		self.assertEqual( keys, set( compoundData.keys() ) )

		for k in keys:
			self.assertEqual( s["cachedDataNode"].getEntry( k ), compoundData[ k ] )

	def assertSaved( self, s, expectRecycleBin = False ) :

		hashes = set()

		for cachedDataNode in Gaffer.CachedDataNode.RecursiveRange( s ):
			self.assertFalse( cachedDataNode.hasLiveEntries() )

			keys = set( cachedDataNode["keys"].getValue() )
			for k in keys:
				hashes.add( cachedDataNode.getEntry( k ).hash() )

		cacheDir = s["fileName"].getValue() + ".cachedData"
		cacheFiles = set()
		if not os.path.exists( cacheDir ):
			# If the cache dir doesn't exist, that is equivalent to having an empty cache dir
			# One is the situation if a script has never had caches, one is the situation if
			# all caches have been deleted, either is fine, if it corresponds to a script with
			# no caches.
			pass
		else:
			cacheFiles = set( os.listdir( cacheDir ) )

		expectedCacheFiles = set( [ "%s.io" % h.toString() for h in hashes ] )
		#if expectRecycleBin:
		#	expectedCacheFiles.add( ".recycleBin" )
		#TODO - whether or not a recycle bin is expected is about to get more complex
		if ".recycleBin" in cacheFiles:
			cacheFiles.remove( ".recycleBin" )

		self.assertEqual( cacheFiles, expectedCacheFiles )

	def testBasic( self ):

		s = Gaffer.ScriptNode()
		s["cachedDataNode"] = Gaffer.CachedDataNode()
		s["cachedDataNode"].setEntry( "a", IECore.IntData( 7 ) )
		s["cachedDataNode"].setEntry( "b", IECore.FloatData( 123.456 ) )
		s["cachedDataNode"].setEntry( "c", IECore.StringData( "Hello world" ) )

		self.assertEqual( s["cachedDataNode"].getEntry( "a" ), IECore.IntData( 7 ) )
		self.assertEqual( s["cachedDataNode"].getEntry( "b" ), IECore.FloatData( 123.456 ) )
		self.assertEqual( s["cachedDataNode"].getEntry( "c" ), IECore.StringData( "Hello world" ) )

		with self.assertRaisesRegex( Exception, "Unknown key: d" ) :
			self.assertEqual( s["cachedDataNode"].getEntry( "d" ), IECore.StringData( "Hello world" ) )
		self.assertEqual( s["cachedDataNode"].getEntry( "d", throwExceptions = False ), None )

		s["cachedDataNode"]["selector"].setValue( "a")
		self.assertEqual( s["cachedDataNode"]["out"].getValue(), IECore.IntData( 7 ) )
		s["cachedDataNode"]["selector"].setValue( "c")
		self.assertEqual( s["cachedDataNode"]["out"].getValue(), IECore.StringData( "Hello world" ) )
		s["cachedDataNode"]["selector"].setValue( "${contextVar}")

		c = Gaffer.Context()
		with c:
			c["contextVar"] = IECore.StringData( "a" )
			self.assertEqual( s["cachedDataNode"]["out"].getValue(), IECore.IntData( 7 ) )
			c["contextVar"] = IECore.StringData( "b" )
			self.assertEqual( s["cachedDataNode"]["out"].getValue(), IECore.FloatData( 123.456 ) )
			c["contextVar"] = IECore.StringData( "d" )
			with self.assertRaisesRegex( Exception, "Unknown key: d" ) :
				# TODO - should we have an option for doing a plug evaluate that doesn't throw
				# for unknown keys?
				s["cachedDataNode"]["out"].getValue()


		# After saving, the live values will be cleared, and values will be read from disk
		s["fileName"].setValue( self.temporaryDirectory() / "test.gfr" )
		s.save()
		Gaffer.ValuePlug.clearCache()

		self.assertEqual( s["cachedDataNode"].getEntry( "a" ), IECore.IntData( 7 ) )
		self.assertEqual( s["cachedDataNode"].getEntry( "b" ), IECore.FloatData( 123.456 ) )
		self.assertEqual( s["cachedDataNode"].getEntry( "c" ), IECore.StringData( "Hello world" ) )

	def testComparison( self ):
		# Test by comparing against values stored explicitly in the Gaffer script
		s = Gaffer.ScriptNode()
		self.setupComparison( s )
		self.comparisonSetEntry( s, "a", IECore.IntData( 7 ) )
		self.comparisonSetEntry( s, "b", IECore.FloatData( 123.456 ) )
		self.comparisonSetEntry( s, "c", IECore.StringData( "Hello world" ) )
		self.assertComparisonValid( s )

		s["fileName"].setValue( self.temporaryDirectory() / "test.gfr" )
		s.save()
		self.assertComparisonValid( s )
		self.assertSaved( s )

		del s

		self.assertFalse( os.path.exists( self.temporaryDirectory() / "test.gfr.cachedData" / ".recycleBin" ) )

		# Test match after reopening
		s = Gaffer.ScriptNode()
		s["fileName"].setValue( self.temporaryDirectory() / "test.gfr" )
		s.load()
		self.assertComparisonValid( s )
		self.assertSaved( s )

		# Modify two entries, check that the cache files get moved to the recycle bin
		self.comparisonSetEntry( s, "a", IECore.IntData( 1 ) )
		self.comparisonSetEntry( s, "b", IECore.FloatData( 2 ) )

		s.save()
		self.assertComparisonValid( s )
		self.assertSaved( s, expectRecycleBin = True )

		self.assertTrue( os.path.exists( self.temporaryDirectory() / "test.gfr.cachedData" ) )
		self.assertTrue( os.path.exists( self.temporaryDirectory() / "test.gfr.cachedData" / ".recycleBin" ) )
		self.assertEqual( len( os.listdir( self.temporaryDirectory() / "test.gfr.cachedData" / ".recycleBin" ) ), 2 )

		# Do another save, which moves another entry to the recycle bin
		self.comparisonSetEntry( s, "a", IECore.IntData( 10000 ) )
		self.assertComparisonValid( s )
		s.save()
		self.assertComparisonValid( s )
		self.assertSaved( s, expectRecycleBin = True )

		self.assertEqual( len( os.listdir( self.temporaryDirectory() / "test.gfr.cachedData" / ".recycleBin" ) ), 3 )

		del s

		# Closing the script deletes the recycle bin
		self.assertFalse( os.path.exists( self.temporaryDirectory() / "test.gfr.cachedData" / ".recycleBin" ) )

		# Everything still loads fine
		s = Gaffer.ScriptNode()
		s["fileName"].setValue( self.temporaryDirectory() / "test.gfr" )
		s.load()
		self.assertComparisonValid( s )
		self.assertSaved( s )

	def testComparisonFuzz( self ):

		# At any given time, there are 5 main actions that can be taken that affect CachedDataNodes in
		# the current script:
		# A) setting a new entry
		# B) changing an existing entry
		# C) undoing
		# D) redoing
		# E) changing the filename
		# We can achieve a good mix of A and B just by choosing randomly from a small pool of possible options,
		# and alternate between the other options by picking randomly.
		# This allows us to generate a sequence of actions that should theoretically exercise all possibilities,
		# which we can compare against an implementation that just saves values in the script.

		s = Gaffer.ScriptNode()
		self.setupComparison( s )

		random.seed( 42 )

		fileNameCount = 0
		fileName = self.temporaryDirectory() / ( "test%i.gfr" % fileNameCount )
		s["fileName"].setValue( fileName )

		for i in range( 100 ):
			# TODO - remove prints
			#print( i )
			if random.random() < 0.05:
				fileNameCount += 1
				fileName = self.temporaryDirectory() / ( "test%i.gfr" % fileNameCount )
				s["fileName"].setValue( fileName )
			elif s.redoAvailable() and random.random() < 0.7:
				#print( "REDO" )
				s.redo()
			elif s.undoAvailable() and random.random() < 0.3:
				#print( "UNDO" )
				s.undo()
			else:
				with Gaffer.UndoScope( s ) :
					self.comparisonSetEntry( s, "a%i" % random.randint( 0, 6 ), IECore.IntData( random.randint( 0, 10000 ) ) )

			#print( { k : s["cachedDataNode"].getEntry( k ) for k in s["cachedDataNode"]["keys"].getValue() } )

			self.assertComparisonValid( s )
			s.save()
			self.assertComparisonValid( s )
			self.assertSaved( s )

			loadS = Gaffer.ScriptNode()
			loadS["fileName"].setValue( fileName )
			loadS.load()
			self.assertComparisonValid( loadS )
			del loadS

	def testMovingManyEntriesToRecycleBin( self ):
		# Just wanted to double check that iterating the cache directory is working properly, by
		# moving a whole bunch of files at once to the recycle bin
		s = Gaffer.ScriptNode()
		self.setupComparison( s )

		for i in range( 1000 ):
			self.comparisonSetEntry( s, "a%i"%i, IECore.IntData( 7 * i ) )

		s["fileName"].setValue( self.temporaryDirectory() / "test.gfr" )
		self.assertComparisonValid( s )
		s.save()
		self.assertComparisonValid( s )
		self.assertSaved( s )

		for i in range( 1000 ):
			self.comparisonSetEntry( s, "a%i"%i, None )

		self.comparisonSetEntry( s, "b", IECore.IntData( 4 ) )

		self.assertEqual( s["cachedDataNode"]["keys"].getValue(), IECore.StringVectorData( [ "b" ] ) )

		self.assertComparisonValid( s )
		s.save()
		self.assertSaved( s, expectRecycleBin = True )
		self.assertEqual( len( os.listdir( self.temporaryDirectory() / "test.gfr.cachedData" / ".recycleBin" ) ), 1000 )

	def testHardLinks( self ):

		# Test that if we hold a cache value the same through several versions of a file, we
		# keep a link back to the original instead of duplicating the file.
		testValue = IECore.IntVectorData( [i for i in range( 100000 ) ] )
		s = Gaffer.ScriptNode()
		s["cachedDataNode"] = Gaffer.CachedDataNode()
		s["cachedDataNode"].setEntry( "a", testValue )
		s["fileName"].setValue( self.temporaryDirectory() / "file1.gfr" )
		s.save()
		s["fileName"].setValue( self.temporaryDirectory() / "file2.gfr" )
		s.save()
		del s

		s = Gaffer.ScriptNode()
		s["fileName"].setValue( self.temporaryDirectory() / "file2.gfr" )
		s.load()
		s["fileName"].setValue( self.temporaryDirectory() / "file3.gfr" )
		s.save()
		del s

		self.assertEqual( os.stat( self.temporaryDirectory() / "file1.gfr.cachedData" / "2be6e2024a34d8808b87824ac350c907.io" ).st_nlink, 3 )
		self.assertEqual( os.stat( self.temporaryDirectory() / "file2.gfr.cachedData" / "2be6e2024a34d8808b87824ac350c907.io" ).st_nlink, 3 )
		self.assertEqual( os.stat( self.temporaryDirectory() / "file3.gfr.cachedData" / "2be6e2024a34d8808b87824ac350c907.io" ).st_nlink, 3 )

		s = Gaffer.ScriptNode()
		s["fileName"].setValue( self.temporaryDirectory() / "file3.gfr" )
		s.load()
		self.assertEqual( s["cachedDataNode"].getEntry( "a" ), testValue )
		del s

		s = Gaffer.ScriptNode()
		s["fileName"].setValue( self.temporaryDirectory() / "file1.gfr" )
		s.load()
		self.assertEqual( s["cachedDataNode"].getEntry( "a" ), testValue )
		del s

	def testUndo( self ):
		s = Gaffer.ScriptNode()
		s["cachedDataNode"] = Gaffer.CachedDataNode()
		s["cachedDataNode"].setEntry( "a", IECore.IntData( 7 ) )
		self.assertEqual( s["cachedDataNode"].getEntry( "a" ), IECore.IntData( 7 ) )

		s["fileName"].setValue( self.temporaryDirectory() / "source.gfr" )
		s.save()
		del s

		s = Gaffer.ScriptNode()
		s["fileName"].setValue( self.temporaryDirectory() / "source.gfr" )
		s.load()
		s["fileName"].setValue( self.temporaryDirectory() / "test.gfr" )
		self.assertEqual( s["cachedDataNode"].getEntry( "a" ), IECore.IntData( 7 ) )

		with Gaffer.UndoScope( s ) :
			s["cachedDataNode"].setEntry( "a", IECore.IntData( 42 ) )

		self.assertEqual( s["cachedDataNode"].getEntry( "a" ), IECore.IntData( 42 ) )

		s.save()

		s.undo()

		# Test that we can reload the data from disk ( even though it's now coming from
		# the recycle bin )
		Gaffer.ValuePlug.clearCache()

		self.assertEqual( s["cachedDataNode"].getEntry( "a" ), IECore.IntData( 7 ) )

		# Now try saving - we want this to get the hard link back out of the recycle bin,
		# rather than saving a separate copy

		s.save()

		del s

		self.assertEqual( os.stat( self.temporaryDirectory() / "source.gfr.cachedData" / "22b41848d90e4f05d50ab80c68957527.io" ).st_nlink, 2 )
		self.assertEqual( os.stat( self.temporaryDirectory() / "test.gfr.cachedData" / "22b41848d90e4f05d50ab80c68957527.io" ).st_nlink, 2 )

	def testManyRecycleBins( self ):

		s = Gaffer.ScriptNode()
		s["counter"] = Gaffer.IntPlug()
		s["cachedDataNode"] = Gaffer.CachedDataNode()

		for i in range( 10 ):
			# Create a cached entry unique to each script we save as
			with Gaffer.UndoScope( s ) :
				s["cachedDataNode"].setEntry( "a%i"%i, IECore.IntData( 1000 + i ) )

			with Gaffer.UndoScope( s ) :
				s["fileName"].setValue( self.temporaryDirectory() / ( "file%i.gfr" % i ) )
			s.save()

			# Change the entry so we move the previous value to the recycle bin
			with Gaffer.UndoScope( s ) :
				s["cachedDataNode"].setEntry( "a%i"%i, IECore.IntData( 2000 + i ) )
			s.save()

		for i in range( 10 ):
			# Each cache dir should contain the entries so far, plus a recycle bin
			self.assertEqual( len( os.listdir( self.temporaryDirectory() / ( "file%i.gfr.cachedData" % i ) ) ), 2 + i )
			# Each recycle bin should contain one file
			self.assertEqual( len( os.listdir( self.temporaryDirectory() / ( "file%i.gfr.cachedData" % i ) / ".recycleBin" ) ), 1 )

		# Ensure that we're using the values from disk
		Gaffer.ValuePlug.clearCache()

		# Check the final values
		for i in range( 10 ) :
			self.assertEqual( s["cachedDataNode"].getEntry( "a%i"%i ), IECore.IntData( 2000 + i ) )

		# Run back through the undo stack getting all the values from the recycle bins
		for i in reversed( range( 0, 10 ) ):
			s.undo()

			self.assertEqual( s["cachedDataNode"].getEntry( "a%i"%i ), IECore.IntData( 1000 + i ) )

			s.undo()
			s.undo()

		self.assertEqual( s["cachedDataNode"]["keys"].getValue(), IECore.StringVectorData() )

		# Redo everything
		for i in range( 30 ):
			s.redo()

		self.assertEqual( set( s["cachedDataNode"]["keys"].getValue() ), { "a%i"%i for i in range( 10 ) } )

		for i in range( 10 ) :
			self.assertEqual( s["cachedDataNode"].getEntry( "a%i"%i ), IECore.IntData( 2000 + i ) )

		# Run back through the undo stack getting all the values from the recycle bins, but this
		# time we'll put new actions on the undo stack, forcing clearing of the undo stack


		for i in reversed( range( 0, 10 ) ):
			s.undo()

			self.assertEqual( s["cachedDataNode"].getEntry( "a%i"%i ), IECore.IntData( 1000 + i ) )

			s.undo()

			self.assertTrue( os.path.exists( self.temporaryDirectory() / ( "file%i.gfr.cachedData/.recycleBin" % i ) ) )

			# Make a new edit, then immediately undo it, just to force the undo stack to be cleared
			with Gaffer.UndoScope( s ) :
				s["counter"].setValue( 10 + i )
			s.undo()

			s.undo()
			self.assertFalse( os.path.exists( self.temporaryDirectory() / ( "file%i.gfr.cachedData/.recycleBin" % i ) ) )

		self.assertEqual( s["cachedDataNode"]["keys"].getValue(), IECore.StringVectorData() )

	# We now require that you rename the caches to match if you rename a script
	@unittest.expectedFailure
	def testRenameA( self ):
		s = Gaffer.ScriptNode()
		s["cachedDataNode"] = Gaffer.CachedDataNode()
		s["cachedDataNode"].setEntry( "a", IECore.IntData( 7 ) )
		s["fileName"].setValue( self.temporaryDirectory() / "test.gfr" )
		s.save()
		del s

		os.rename( self.temporaryDirectory() / "test.gfr", self.temporaryDirectory() / "renameA.gfr" )

		# The renamed script remembers the absolute path of where the caches were previously saved,
		# and is able to load the entry.
		s = Gaffer.ScriptNode()
		s["fileName"].setValue( self.temporaryDirectory() / "renameA.gfr" )
		s.load()
		self.assertEqual( s["cachedDataNode"].getEntry( "a" ), IECore.IntData( 7 ) )

	def testRenameB( self ):
		s = Gaffer.ScriptNode()
		s["cachedDataNode"] = Gaffer.CachedDataNode()
		s["cachedDataNode"].setEntry( "a", IECore.IntData( 42 ) )
		s["fileName"].setValue( self.temporaryDirectory() / "test.gfr" )
		s.save()
		del s

		os.rename( self.temporaryDirectory() / "test.gfr", self.temporaryDirectory() / "renameB.gfr" )
		os.rename( self.temporaryDirectory() / "test.gfr.cachedData", self.temporaryDirectory() / "renameB.gfr.cachedData" )

		# If we move a file and its caches together, it should be able to find the new cache locations when
		# we load.
		s = Gaffer.ScriptNode()
		s["fileName"].setValue( self.temporaryDirectory() / "renameB.gfr" )
		s.load()
		self.assertEqual( s["cachedDataNode"].getEntry( "a" ), IECore.IntData( 42 ) )

	def testCopyPaste( self ):

		app = Gaffer.ApplicationRoot()

		s = Gaffer.ScriptNode()

		app["scripts"]["s"] = s

		s["cachedDataNode"] = Gaffer.CachedDataNode()
		s["cachedDataNode"].setEntry( "a", IECore.IntData( 7 ) )
		s["cachedDataNode"].setEntry( "b", IECore.FloatData( 123.456 ) )
		s["fileName"].setValue( self.temporaryDirectory() / "test.gfr" )

		with self.assertRaisesRegex( Exception, 'Cannot copy, CachedDataNode "ApplicationRoot.scripts.s.cachedDataNode" is not saved yet.' ) :
			s.copy()

		s.save()
		s.copy()
		del s

		t = Gaffer.ScriptNode()
		app["scripts"]["t"] = t
		t.paste()

		self.assertEqual( t["cachedDataNode"].getEntry( "a" ), IECore.IntData( 7 ) )
		self.assertEqual( t["cachedDataNode"].getEntry( "b" ), IECore.FloatData( 123.456 ) )

		t["fileName"].setValue( self.temporaryDirectory() / "copied.gfr" )
		t.save()

		# Check that we find the source files, and link back to them.
		self.assertEqual( os.stat( self.temporaryDirectory() / "copied.gfr.cachedData" / "5f4ab9972edafa975a49cad56ad69070.io" ).st_nlink, 2 )
		self.assertEqual( os.stat( self.temporaryDirectory() / "copied.gfr.cachedData" / "22b41848d90e4f05d50ab80c68957527.io" ).st_nlink, 2 )

		del t

		shutil.rmtree( self.temporaryDirectory() / "copied.gfr.cachedData" )

		# Try again, but this time we delete the source files after loading. This could happen if the
		# caches were managed by another Gaffer session.
		t = Gaffer.ScriptNode()
		app["scripts"]["t"] = t
		t.paste()

		self.assertEqual( t["cachedDataNode"].getEntry( "a" ), IECore.IntData( 7 ) )
		self.assertEqual( t["cachedDataNode"].getEntry( "b" ), IECore.FloatData( 123.456 ) )

		shutil.rmtree( self.temporaryDirectory() / "test.gfr.cachedData" )
		Gaffer.ValuePlug.clearCache()

		# We force loaded the caches as soon as the paste happened, so the values are safe.
		self.assertEqual( t["cachedDataNode"].getEntry( "a" ), IECore.IntData( 7 ) )
		self.assertEqual( t["cachedDataNode"].getEntry( "b" ), IECore.FloatData( 123.456 ) )

		t["fileName"].setValue( self.temporaryDirectory() / "copied.gfr" )
		t.save()

		# But we can't link any more, since we can't find the sources on disk - but we can still correctly
		# write from the data that was loaded
		self.assertEqual( os.stat( self.temporaryDirectory() / "copied.gfr.cachedData" / "5f4ab9972edafa975a49cad56ad69070.io" ).st_nlink, 1 )
		self.assertEqual( os.stat( self.temporaryDirectory() / "copied.gfr.cachedData" / "22b41848d90e4f05d50ab80c68957527.io" ).st_nlink, 1 )

		del t

		# Now that the source data is gone though, trying to paste won't work
		t = Gaffer.ScriptNode()
		app["scripts"]["t"] = t

		with self.assertRaisesRegex( Exception, "Cannot paste - source file uses data caches which are not accessible, or have been modified." ) :
			t.paste()

	def testReference( self ):

		s = Gaffer.ScriptNode()
		s["b"] = Gaffer.Box()
		s["b"]["cachedDataNode"] = Gaffer.CachedDataNode()
		s["b"]["cachedDataNode"].setEntry( "a", IECore.StringData( "aa" ) )
		s["b"]["cachedDataNode"].setEntry( "b", IECore.StringData( "bb" ) )
		s["b"]["cachedDataNode"].setEntry( "c", IECore.StringData( "cc" ) )

		s["b"].exportForReference( self.temporaryDirectory() / "ref.grf" )

		del s

		# Data is stored with reference
		self.assertEqual( len( os.listdir( self.temporaryDirectory() / "ref.grf.cachedData" ) ), 3 )

		s = Gaffer.ScriptNode()
		s["r"] = Gaffer.Reference()
		s["r"].load( self.temporaryDirectory() / "ref.grf" )

		self.assertEqual( s["r"]["cachedDataNode"].getEntry( "a" ), IECore.StringData( "aa" ) )
		self.assertEqual( s["r"]["cachedDataNode"].getEntry( "b" ), IECore.StringData( "bb" ) )
		self.assertEqual( s["r"]["cachedDataNode"].getEntry( "c" ), IECore.StringData( "cc" ) )

		s["cachedDataNode"] = Gaffer.CachedDataNode()
		s["cachedDataNode"].setEntry( "d", IECore.StringData( "dd" ) )

		s["fileName"].setValue( self.temporaryDirectory() / "test.gfr" )
		s.save()

		del s

		s = Gaffer.ScriptNode()
		s["fileName"].setValue( self.temporaryDirectory() / "test.gfr" )
		s.load()

		self.assertEqual( s["r"]["cachedDataNode"].getEntry( "a" ), IECore.StringData( "aa" ) )
		self.assertEqual( s["r"]["cachedDataNode"].getEntry( "b" ), IECore.StringData( "bb" ) )
		self.assertEqual( s["r"]["cachedDataNode"].getEntry( "c" ), IECore.StringData( "cc" ) )
		self.assertEqual( s["cachedDataNode"].getEntry( "d" ), IECore.StringData( "dd" ) )

		# Check that we're still using the data files from the reference, and we only save out
		# the one data file for the local CachedDataNode
		self.assertEqual( len( os.listdir( self.temporaryDirectory() / "test.gfr.cachedData" ) ), 1 )

	@unittest.skipIf( not os.path.exists( "/dev/shm" ), "No /dev/shm, can't test linking across different mounts." )
	def testHardLinkFailure( self ):

		s = Gaffer.ScriptNode()
		s["cachedDataNode"] = Gaffer.CachedDataNode()
		s["cachedDataNode"].setEntry( "a", IECore.IntData( 7 ) )
		s["cachedDataNode"].setEntry( "b", IECore.FloatData( 123.456 ) )
		s["cachedDataNode"].setEntry( "c", IECore.StringData( "Hello world" ) )
		s["fileName"].setValue( self.temporaryDirectory() / "test.gfr" )
		s.save()

		s["fileName"].setValue( self.altMountTemporaryDirectory() / "test.gfr" )

		# Saving as a new file on a different mount will mean we can't use hardlinks, so we should get a warning.
		with IECore.CapturingMessageHandler() as mh :
			s.save()

		self.assertEqual( len( mh.messages ), 1 )
		self.assertRegex( mh.messages[0].message, 'During saving, could not create hardlink at ".*" pointing to ".*", falling back to copying file.' )

		del s

		Gaffer.ValuePlug.clearCache()

		# But everything should still work

		s = Gaffer.ScriptNode()
		s["fileName"].setValue( self.altMountTemporaryDirectory() / "test.gfr" )
		s.load()

		self.assertEqual( s["cachedDataNode"].getEntry( "a" ), IECore.IntData( 7 ) )
		self.assertEqual( s["cachedDataNode"].getEntry( "b" ), IECore.FloatData( 123.456 ) )
		self.assertEqual( s["cachedDataNode"].getEntry( "c" ), IECore.StringData( "Hello world" ) )





	# TODO : More undo tests
	# TODO : think about backups and render scripts
	# TODO : test save as
	# TODO : Implement/Test takeOwnership for dealing with Reference
	# TODO : Switch to using cob files

if __name__ == "__main__":
	unittest.main()
