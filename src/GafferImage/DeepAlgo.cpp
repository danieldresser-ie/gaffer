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

#include "GafferImage/DeepAlgo.h"

// TODO
#include "IECore/Exception.h"

#include <vector>
#include <cmath>
#include <assert.h>
#include <limits> // TODO
#include <iostream> // TODO
#include <stdexcept> // TODO

using namespace GafferImage::DeepAlgo::Detail;

namespace
{

/// Creates a set of point samples across the range of the pixel's depth for the alpha channel.
/// The deepSamples that are returned are stored in the deepSamples vector, with values for depth and accumulated alpha from front to back

const float maxConvertibleAlpha = 1. - 4 * std::numeric_limits<float>::epsilon();
const float maximumLinearY = -log1pf( - maxConvertibleAlpha );

const float linearOpacityThreshold = -log1pf( - 0.99999 );

/// Given a value in linear space, make it exponential.
inline float linearToExponential( float value )
{
	return value == maximumLinearY ? 1 : -expm1f( -value );
}

/// Given a value in exponential space, make it linear.
inline float exponentialToLinear( float value )
{
	return value <= 0 ? 0 : -log1pf( -std::min( maxConvertibleAlpha, value ) );
}

struct SimplePoint
{
	float x, y;
};

SimplePoint evaluateLineAtY( const SimplePoint &a, const SimplePoint &b, float y )
{
	return {
		a.x + ( ( y - a.y ) / ( b.y - a.y ) ) * ( b.x - a.x ),
		y
	};
}

SimplePoint evaluateLineAtX( const SimplePoint &a, const SimplePoint &b, float x )
{
	return {
		x,
		a.y + ( ( x - a.x ) / ( b.x - a.x ) ) * ( b.y - a.y )
	};
}


SimplePoint segmentIntersect( SimplePoint a1, SimplePoint b1, SimplePoint a2, SimplePoint b2 )
{
	SimplePoint disp = { a2.x - a1.x, a2.y - a1.y };
	SimplePoint dir1 = { b1.x - a1.x, b1.y - a1.y };
	SimplePoint dir2 = { b2.x - a2.x, b2.y - a2.y };

	float denom = dir1.x * dir2.y - dir1.y * dir2.x;
	if( fabs( denom ) < 1e-10 )
	{
		// TODO what if lines not coincident
		if( b1.x < b2.x )
		{
			return b1;
		}
		else
		{
			return b2;
		}
	}

	float t = ( disp.x * dir2.y - disp.y * dir2.x ) / denom;

	SimplePoint r;
	if( a2.x == b2.x )
	{
		r.x = a2.x;
	}
	else
	{
		r.x = a1.x + dir1.x * t;
	}

	if( a2.y == b2.y )
	{
		r.y = a2.y;
	}
	else
	{
		r.y = a1.y + dir1.y * t;
	}

	return r;

	// TODO testing without the special cases above is a good way to test hang handling
	//return { a1.x + dir1.x * t, a1.y + dir1.y * t };
}

struct LinearSegment
{
	SimplePoint a, b;
/*	float X; // The depth of the front of the sample.
	float XBack; // The depth at the back of the sample.
	float YBack; // The alpha at the back of the sample.*/
};


struct ConstraintSearchParams
{
	// The points which impose the shallowest constraints
	// ie:  if the current line was any steeper, it could not pass over lowerConstraint and under upperConstraint
	//SimplePoint lowerConstraint, upperConstraint;
	int lowerConstraintIndex, upperConstraintIndex;
};

/*
inline float measureViolation( const SimplePoint *points, int startIndex, int endIndex, int startAnchor, const SimplePoint &comparePoint, float compareDirection )
{
	int anchor = startAnchor;

	float a = ( points[anchor].y - comparePoint.y ) / ( points[anchor].x - comparePoint.x );

	float violation = -100;
	// Test all constraints to make sure that the current line fufills all of them
	for( int i = startIndex; i < endIndex; i++ )
	{
		if( i == anchor )
		{
			continue;
		}

		float lowerX = points[i].x;
		float minY = points[i].y;

		float yAtLowerX = a * ( lowerX - points[anchor].x ) + points[anchor].y;
		violation = std::max( violation, ( yAtLowerX - minY ) * compareDirection );
	}
	return violation;
}
*/

inline int findAnchor( const SimplePoint *points, int startIndex, int endIndex, int startAnchor, const SimplePoint &comparePoint, float scanDirection, float constraintDirection, float steeperDirection, bool debug )
{
	int anchor = startAnchor;

	float a = ( points[anchor].y - comparePoint.y ) / ( points[anchor].x - comparePoint.x );

	// Test all constraints to make sure that the current line fufills all of them
	for( int i = startIndex; i < endIndex; i++ )
	{
		if( i == anchor )
		{
			continue;
		}

		float lowerX = points[i].x;
		float minY = points[i].y;

		float yAtLowerX = a * ( lowerX - points[anchor].x ) + points[anchor].y;

		// Check if we go underneath the minimum constraint at this index
		if( yAtLowerX * constraintDirection < minY * constraintDirection )
		{
			//if( steeperDirection < 0 ) std::cerr << "VIOLATE : " << i << "\n";
			SimplePoint delta = { comparePoint.x - lowerX, comparePoint.y - minY };
			if( delta.x * scanDirection <= 0 )
			{
				//if( steeperDirection < 0 ) std::cerr << "REVERSED : " << comparePoint.x << " : " << lowerX << "\n";
				// If the violated constraint is to the right of the previous upper constraint, then we would need to get steeper to fufill it
				// But the current line is already the steepest that fufills previous constraints, so there can be no line which fufills all constraints.
				// Fail.
				return -1;
			}
			else
			{
				// Replace the lower constraint with the constraint we violated.  Recalculate the steepest line through these constraints, TODO "steepest"
				// and trigger a rescan to check our new line against previous constraints
				float newA = delta.y / delta.x;
				if( newA * steeperDirection < a * steeperDirection )
				{
					a = newA;
					anchor = i;
				}
			}
		}
	}
	return anchor;
}

/*
 Update a set of constraint search parameters with a new set of constraints
 ( must be a superset of the previous constraints used to set up the search parameters )

 startIndex and endIndex define the first and last constraints used in the constraints array.
 headIndex and tailIndex are used for special conditions for the beginning and end.
 Constraints between startIndex and headIndex (inclusive) are used just for upper constraints, since their lower constrainst have already been fufilled by a previous segment.
 Constraints between tailIndex and endIndex (inclusive) are used just for lower constraints, since the new segment will be cut when it reaches a certain y value, and won't exceed the upper constraints.

 This function either updates masterSearchParams with a line that fufills all constraints and returns true, or returns false and does not alter masterSearchParams.
*/
ConstraintSearchParams steepestSegmentThroughConstraints( const SimplePoint *lowerConstraints, int lowerStart, int lowerStop, const SimplePoint *upperConstraints, int upperStart, int upperStop, bool lowerChecked, bool upperChecked ) // TODO - defaults for checked?
{
	ConstraintSearchParams p = { lowerStart, upperStop - 1 };

	while( !( lowerChecked && upperChecked ) )
	{
		if( !lowerChecked )
		{
			int newLower = findAnchor( lowerConstraints, p.lowerConstraintIndex, lowerStop, p.lowerConstraintIndex, upperConstraints[p.upperConstraintIndex], 1.0, 1.0, 1.0, false );
			if( newLower == -1 )
			{
				return {-1, -1};
			}
			else if( newLower != p.lowerConstraintIndex )
			{
				p.lowerConstraintIndex = newLower;
				upperChecked = false;
			}
			lowerChecked = true;
		}

		if( !upperChecked )
		{
			int newUpper = findAnchor( upperConstraints, upperStart, p.upperConstraintIndex + 1, p.upperConstraintIndex, lowerConstraints[p.lowerConstraintIndex], -1.0, -1.0, 1.0, false );
			if( newUpper == -1 )
			{
				return {-1, -1};
			}
			else if( newUpper != p.upperConstraintIndex )
			{
				p.upperConstraintIndex = newUpper;
				lowerChecked = false;
			}
			upperChecked = true;
		}

		/*if( p.lowerConstraintIndex == newLower && p.upperConstraintIndex == newUpper )
		{
			// No changes made
			break;
		}
		p.lowerConstraintIndex = newLower;
		p.upperConstraintIndex = newUpper;*/
	}

	return p;
}
ConstraintSearchParams steepestSegmentThroughConstraints2( const SimplePoint *lowerConstraints, int lowerStart, int lowerStop, const SimplePoint *upperConstraints, int upperStart, int upperStop, bool startUpper = false )
{
	ConstraintSearchParams p = { lowerStart, upperStop - 1 };

	bool upperChecked = false;
	bool lowerChecked = false;

	while( !( lowerChecked && upperChecked ) )
	{
		if( !lowerChecked && !startUpper)
		{
			int newLower = findAnchor( lowerConstraints, p.lowerConstraintIndex, lowerStop, p.lowerConstraintIndex, upperConstraints[p.upperConstraintIndex], 1.0, 1.0, 1.0, false );
			if( newLower == -1 )
			{
				return {-1, -1};
			}
			else if( newLower != p.lowerConstraintIndex )
			{
				p.lowerConstraintIndex = newLower;
				upperChecked = false;
			}
			lowerChecked = true;
		}

		startUpper = false;

		if( !upperChecked )
		{
			int newUpper = findAnchor( upperConstraints, upperStart, p.upperConstraintIndex + 1, p.upperConstraintIndex, lowerConstraints[p.lowerConstraintIndex], -1.0, -1.0, 1.0, false );
			if( newUpper == -1 )
			{
				return {-1, -1};
			}
			else if( newUpper != p.upperConstraintIndex )
			{
				p.upperConstraintIndex = newUpper;
				lowerChecked = false;
			}
			upperChecked = true;
		}

		/*if( p.lowerConstraintIndex == newLower && p.upperConstraintIndex == newUpper )
		{
			// No changes made
			break;
		}
		p.lowerConstraintIndex = newLower;
		p.upperConstraintIndex = newUpper;*/
	}

	return p;
}

bool shallowestSegmentThroughConstraints( const SimplePoint *lowerConstraints, int lowerStart, int lowerStop, const SimplePoint *upperConstraints, int upperStart, int upperStop, ConstraintSearchParams *masterSearchParams )
{
	ConstraintSearchParams p = *masterSearchParams;

	/*std::cerr << "LOWER\n";
	for( int i = lowerStart; i < lowerStop; i++ )
	{
		std::cerr << lowerConstraints[i].x << "," << lowerConstraints[i].y << "\t";
	}
	std::cerr << "\n";
	std::cerr << "UPPER\n";
	for( int i = upperStart; i < upperStop; i++ )
	{
		std::cerr << upperConstraints[i].x << "," << upperConstraints[i].y << "\t";
	}
	std::cerr << "\n";
	std::cerr << "\n";
	*/

	if( p.upperConstraintIndex == -1 || p.lowerConstraintIndex == -1 ) // TODO
	{
		p.upperConstraintIndex = upperStart;
		p.lowerConstraintIndex = lowerStop - 1;

		/*if( lowerConstraints[ p.lowerConstraintIndex ].x >= upperConstraints[ p.upperConstraintIndex ].x )
		{
			return true;
		}*/
	}

	//std::cerr << "SI: " << p.lowerConstraintIndex << " " << p.upperConstraintIndex << "\n";
	while( true )
	{
		int newLower = findAnchor( lowerConstraints, lowerStart, lowerStop, p.lowerConstraintIndex, upperConstraints[p.upperConstraintIndex], -1.0, 1.0, -1.0, false );

		//std::cerr << "LI: " << newLower << " " << p.upperConstraintIndex << "\n";
		if( newLower == -1 )
		{
			return false;
		}

		int newUpper = findAnchor( upperConstraints, upperStart, upperStop, p.upperConstraintIndex, lowerConstraints[newLower], 1.0, -1.0, -1.0, false );
		//std::cerr << "UI: " << newLower << " " << newUpper << "\n";
		if( newUpper == -1 )
		{
			return false;
		}

		if( p.lowerConstraintIndex == newLower && p.upperConstraintIndex == newUpper )
		{
			// No changes made
			break;
		}
		p.lowerConstraintIndex = newLower;
		p.upperConstraintIndex = newUpper;
	}

	*masterSearchParams = p;
	return true;
}

/*
 Remap from the linear spaced used for resampling to exponential alpha values
*/
void linearSegmentsToExponentialAlpha(
	const std::vector< LinearSegment >& compressedSamples,
	int &outSamples, float *outA, float *outZ, float *outZBack
)
{
	outSamples = compressedSamples.size();

	float prevTargetAlpha = 0;

	for( unsigned int i = 0; i < compressedSamples.size(); ++i )
	{
		// We could potentially get samples squashed down to being zero length and overlapping,
		// which would result in them not being merged in the right order.  Outputting a single
		// sample will yield a correct result.
		if(
			i + 1 < compressedSamples.size() &&
			compressedSamples[i].a.x == compressedSamples[i].b.x &&
			compressedSamples[i].a.x == compressedSamples[i+1].a.x &&
			compressedSamples[i].a.x == compressedSamples[i+1].b.x
		)
		{
			continue;
		}

		float targetAlpha = -expm1f( -compressedSamples[i].b.y );


		// The other situation that could arise from floating point error is a segment that doesn't
		// rise above the previous sample segment.  Rather than outputting a 0 alpha segment, just
		// skip it
		if( targetAlpha <= prevTargetAlpha )
		{
			continue;
		}

		outZ[i] = compressedSamples[i].a.x;
		outZBack[i] = compressedSamples[i].b.x;
		outA[i] = ( targetAlpha - prevTargetAlpha ) / ( 1.0f - prevTargetAlpha );
		prevTargetAlpha = targetAlpha;

		// TODO
		if( ! ( outA[i] > -0.1 && outA[i] < 1.1 ) )
		{
			std::cerr << "BAD CALC: " << outA[i] << "\n";
			outA[i] = 0.0f;
		}

	}
}


/*
 Given a set of constraints in linear space, put togther a set of segments that pass through all of them
 */
// TODO - note that this overwrites the constraints
void minimalSegmentsForConstraints(
	const std::vector<SimplePoint> &constraintsLower, const std::vector<SimplePoint> &constraintsUpper,  // TODO - these names are stupid
	std::vector< LinearSegment > &compressedSamples,
	bool debug
)
{
	compressedSamples.clear();

	float yPrev = 0; // exponentialToLinear( 0 ) == 0

	//ConstraintSearchParams prevSearchParams;
	//float prevLineA = 0;
	//float prevLineB = 0;

	unsigned int lowerStartIndex = 0;
	unsigned int lowerStopIndex = 0;
	unsigned int upperStartIndex = 0;
	unsigned int upperStopIndex = 0;

	if( debug ) std::cerr << "SEGMENT SEARCH START\n";

	if( !constraintsLower.size() )
	{
		throw IECore::Exception( "Empty lower constraints C" );
	}

	//while( lowerStopIndex < constraintsLower.size() )
	while( yPrev  < constraintsLower.back().y )
	{
		// Initial constraint search parameters for not yet having found anything
		ConstraintSearchParams currentSearchParams;
		currentSearchParams.lowerConstraintIndex = -1;
		currentSearchParams.upperConstraintIndex = -1;


		bool advanceUpper = false;
		// Note that the next loop must run at least once, since we wouldn't get in here unless
		// lowerStopIndex starts out valid.  The only reason to initialize advanceUpper is because
		// the compiler isn't smart enough to realize that it reliably gets initialized in the loop
		while( lowerStopIndex < constraintsLower.size() )
		{
			// TODO - what's the x comparison here for?  Remove?
			advanceUpper = upperStopIndex < constraintsUpper.size() &&
				constraintsUpper[ upperStopIndex ].y <= constraintsLower[lowerStopIndex].y && constraintsUpper[ upperStopIndex ].x <= constraintsLower[lowerStopIndex].x;

			unsigned int testUpperStopIndex = upperStopIndex;
			unsigned int testLowerStopIndex = lowerStopIndex;
			if( advanceUpper )
			{
				testUpperStopIndex++;
			}
			else
			{
				testLowerStopIndex++;
			}

			if( debug )
			{
				std::cerr << "CURRENT INDICES : " << currentSearchParams.lowerConstraintIndex << " -> " << currentSearchParams.upperConstraintIndex << "\n";
				std::cerr << "SEARCHING LOWER : " << lowerStartIndex << " -> " << testLowerStopIndex << " of " << constraintsLower.size() << "\n";
				std::cerr << "SEARCHING UPPER : " << upperStartIndex << " -> " << testUpperStopIndex << " of " << constraintsUpper.size() <<  "\n";
			}

			/*if( lowerConstraints[ p.lowerConstraintIndex ].x >= upperConstraints[ p.upperConstraintIndex ].x )
			{
				if( debug ) std::cerr << "SKIPPING SEARCH, NO OVERLAP\n";
				// *masterSearchParams = p;
				return true;
			}*/
			if( !( testUpperStopIndex > upperStartIndex && testLowerStopIndex > lowerStartIndex && constraintsLower[ lowerStartIndex ].x < constraintsUpper[ testUpperStopIndex - 1 ].x ) )
			{
				if( debug ) std::cerr << "NOT ENOUGH CONSTRAINTS FOR SEARCH, OR NO OVERLAP\n";
				upperStopIndex = testUpperStopIndex;
				lowerStopIndex = testLowerStopIndex;
				continue;
			}

			ConstraintSearchParams newSegment;
			ConstraintSearchParams tempParams = currentSearchParams;
			if( currentSearchParams.lowerConstraintIndex == -1 )
			{
				// No valid segment found yet, need to check all constraints
				newSegment = steepestSegmentThroughConstraints( &constraintsLower[0], lowerStartIndex, testLowerStopIndex, &constraintsUpper[0], upperStartIndex, testUpperStopIndex, false, false );
			}
			else
			{
				int searchLowerIndex = currentSearchParams.lowerConstraintIndex;
				int searchUpperIndex = currentSearchParams.upperConstraintIndex;
				if( advanceUpper )
				{
					// Generally, once we have found a valid segment, we don't need to ever recheck
					// upper constraints to the right of the upper constraint we've found.  But when
					// we advance the upper constraint, we do need to switch to the new constraint
					// if it is under the previously found line.
					//
					// TODO explain this general principle more
					searchUpperIndex = findAnchor( &constraintsUpper[0], testUpperStopIndex - 1, testUpperStopIndex, currentSearchParams.upperConstraintIndex, constraintsLower[tempParams.lowerConstraintIndex], -1.0, -1.0, 1.0, false );
					if( searchUpperIndex == currentSearchParams.upperConstraintIndex )
					{
						upperStopIndex = testUpperStopIndex;
						lowerStopIndex = testLowerStopIndex;
						continue;
					}

					/*int possiblyInvalidatedUpperConstraints = upperStartIndex;
					float pivotX = constraintsLower[ currentSearchParams.lowerConstraintIndex ].x;
					while( possiblyInvalidatedUpperConstraints < (int)testUpperStopIndex - 2 && constraintsUpper[possiblyInvalidatedUpperConstraints + 1].x < pivotX )
					{
						possiblyInvalidatedUpperConstraints++;
					}

					if( -1 == findAnchor( &constraintsUpper[0], upperStartIndex, possiblyInvalidatedUpperConstraints + 1, searchUpperIndex, constraintsLower[tempParams.lowerConstraintIndex], -1.0, -1.0, 1.0, false ) )
					{
						//throw IECore::Exception( "Does this ever happen?" );
						break;
					}*/

					// NOTE : At this point, we need to do a search with TODO
				}
				else
				{
					searchLowerIndex = findAnchor( &constraintsLower[0], testLowerStopIndex - 1, testLowerStopIndex, currentSearchParams.lowerConstraintIndex, constraintsUpper[tempParams.upperConstraintIndex], 1.0, 1.0, 1.0, false );
					if( searchLowerIndex == currentSearchParams.lowerConstraintIndex )
					{
						upperStopIndex = testUpperStopIndex;
						lowerStopIndex = testLowerStopIndex;
						continue;
					}
					if( searchLowerIndex == -1 )
					{
						break;
					}
					throw IECore::Exception( "DOH" );
				}
				if( debug ) std::cerr << "\n\n************ SEARCH AFTER ADJUST *********** \n\n";
				//newSegment = steepestSegmentThroughConstraints( &constraintsLower[0], searchLowerIndex, testLowerStopIndex, &constraintsUpper[0], upperStartIndex, searchUpperIndex + 1, false, false );
				newSegment = steepestSegmentThroughConstraints2( &constraintsLower[0], searchLowerIndex, testLowerStopIndex, &constraintsUpper[0], upperStartIndex, searchUpperIndex + 1, true );
				//newSegment = steepestSegmentThroughConstraints( &constraintsLower[0], searchLowerIndex, testLowerStopIndex, &constraintsUpper[0], upperStartIndex, searchUpperIndex + 1, false, false, debug );
				/*if( advanceUpper )
				{
					newSegment = steepestSegmentThroughConstraints( &constraintsLower[0], searchLowerIndex, testLowerStopIndex, &constraintsUpper[0], upperStartIndex, searchUpperIndex + 1, false, false, debug );
				}
				else
				{
					newSegment = { searchLowerIndex, searchUpperIndex };
				}*/

			}
			//
			// Try forming a segment that pass over all lower constraints up to searchIndex, and under any upper
			// constraints up to searchIndex which haven't been previously covered

			// We will create a flat top which just reaches the lower constraint at this index
			//float yFinalTrial = constraintsLower[lowerStopIndex].y;

			// Any upper constraints greater than yFinalTrial can be ignored, because we are going to cut before reaching them
			//tailIndex = searchIndex;
			/*while( upperStopIndex < constraintsUpper.size() - 1 && constraintsUpper[ upperStopIndex + 1 ].y < yFinalTrial )
			{
				upperStopIndex++;
			}*/

			// Try fitting a line to our new set of constraints
			//std::cerr << "CONSTRAINT SEARCH: " << tempConstraintLower.size() << " , " << tempConstraintUpper.size() << "\n";
			//bool success = stepestSegmentThroughConstraints( &constraintsLower[0], lowerStartIndex, testLowerStopIndex, &constraintsUpper[0], upperStartIndex, testUpperStopIndex, &currentSearchParams, debug );
			//ConstraintSearchParams newSegment = stepestSegmentThroughConstraints( &constraintsLower[0], lowerStartIndex, testLowerStopIndex, &constraintsUpper[0], upperStartIndex, testUpperStopIndex, &tempParams, debug );
			//ConstraintSearchParams newSegment = stepestSegmentThroughConstraints( &constraintsLower[0], tempParams.lowerConstraintIndex, testLowerStopIndex, &constraintsUpper[0], upperStartIndex, tempParams.upperConstraintIndex + 1, false, false, debug );

			/*if( debug )
			{
				std::cerr << "UP TO LOWER " << constraintsLower[testLowerStopIndex].x << "\n";
				std::cerr << "UP TO UPPER " << constraintsUpper[testUpperStopIndex].x << "\n";
				std::cerr << "SUCCESS: " << success << "\n";
			}*/
			if( newSegment.lowerConstraintIndex == -1 )
			{
				// It didn't work, so we'll use the previous value left in currentSearchParams
				break;
			}

			currentSearchParams = newSegment;
			if( debug && newSegment.lowerConstraintIndex != -1 )
			{
				std::cerr << "SUCCESSFULLY FOUND INDICES : " << currentSearchParams.lowerConstraintIndex << " -> " << currentSearchParams.upperConstraintIndex << "\n";

			}

			upperStopIndex = testUpperStopIndex;
			lowerStopIndex = testLowerStopIndex;

			// We'll continue looping and try to find a segment that covers more constraints
		}

		//assert( searchIndex > scanIndex );
		//std::cerr << "Segment " << compressedSamples.size() << " : " << currentSearchParams.lowerConstraint.x << "," << currentSearchParams.lowerConstraint.y << "->" << currentSearchParams.upperConstraint.x << "," << currentSearchParams.upperConstraint.y << "\n";

		float xStart;

		if( currentSearchParams.upperConstraintIndex == -1 || currentSearchParams.lowerConstraintIndex == -1 )
		{
			currentSearchParams.upperConstraintIndex = upperStopIndex - 1;
			currentSearchParams.lowerConstraintIndex = lowerStartIndex;
			//throw IECore::Exception( "COULDN'T FIND ANY LINE" );
		}

		// If we didn't manage to reach a higher Y value than the previous segment during search,
		// something has gone badly wrong.  We can assume that this is due to floating point precision error,
		// and just advance until we reach an upper constraint that won't risk stalling in an infinite loop
		while( currentSearchParams.upperConstraintIndex + 1 < (int)constraintsUpper.size() && constraintsUpper[currentSearchParams.upperConstraintIndex].y <= constraintsLower[lowerStartIndex].y )
		{
			currentSearchParams.upperConstraintIndex++;
			upperStopIndex = std::max( upperStopIndex, (unsigned int)currentSearchParams.upperConstraintIndex + 1 );
		}


		//assert( currentSearchParams.lowerConstraintIndex != -1 );

		assert( upperStopIndex > upperStartIndex );
		assert( lowerStopIndex > lowerStartIndex );

		bool foundSlope = constraintsLower[ currentSearchParams.lowerConstraintIndex ].x < constraintsUpper[ currentSearchParams.upperConstraintIndex ].x;
		if( foundSlope )
		{
			if( advanceUpper )
			{
				if( debug )
				{
					std::cerr << "INDICES BEFORE";
					if( currentSearchParams.lowerConstraintIndex >= 0 )
					{
						std::cerr << "\nLOWER : " <<
							constraintsLower[currentSearchParams.lowerConstraintIndex].x << ", " <<
							constraintsLower[currentSearchParams.lowerConstraintIndex].y << "\n";
					}
					if( currentSearchParams.upperConstraintIndex >= 0 )
					{
						std::cerr << "\nUPPER : " <<
							constraintsUpper[currentSearchParams.upperConstraintIndex].x << ", " <<
							constraintsUpper[currentSearchParams.upperConstraintIndex].y << "\n";
					}
					std::cerr << "REFIND SHALLOW\n\n";
					std::cerr << "LOWER : ";
					for( unsigned int i = lowerStartIndex; i < lowerStopIndex; i++ )
					{
						std::cerr << constraintsLower[i].x << "," << constraintsLower[i].y << " ";
					}
					std::cerr << "\n";
					std::cerr << "UPPER : ";
					for( unsigned int i = upperStartIndex; i < upperStopIndex; i++ )
					{
						std::cerr << constraintsUpper[i].x << "," << constraintsUpper[i].y << " ";
					}
					std::cerr << "\n";
				}

				if( debug ) std::cerr << "INDICES BEFORE " << currentSearchParams.lowerConstraintIndex << " " << currentSearchParams.upperConstraintIndex << "\n";
				// Hit upper, sneak a bit farther by taking shallowest line
				//bool refindShallowSuccess =
				ConstraintSearchParams newSearchParams = { -1, -1 };
				shallowestSegmentThroughConstraints( &constraintsLower[0], lowerStartIndex, lowerStopIndex, &constraintsUpper[0], upperStartIndex, upperStopIndex, &newSearchParams );

				// If we found a shallow line, it should get us a little bit farther.
				// We know there must be some line through the constraints because the initial constraint
				// search worked, so if we failed to find a shallower line, it must be due to floating point
				// error - we can just keep using the line we've already found.
				if(
					newSearchParams.upperConstraintIndex >= 0 && newSearchParams.lowerConstraintIndex >= 0 &&
					constraintsUpper[newSearchParams.upperConstraintIndex].x < constraintsLower[newSearchParams.lowerConstraintIndex].x
				)
				{
					currentSearchParams = newSearchParams;
				}
				if( debug ) std::cerr << "INDICES AFTER " << currentSearchParams.lowerConstraintIndex << " " << currentSearchParams.upperConstraintIndex << "\n";
				/*if( !constraintsUpper[currentSearchParams.upperConstraintIndex].x > constraintsLower[currentSearchParams.lowerConstraintIndex].x ) throw IECore::Exception( "SHALLOWEST GOT BAD" );
				if( constraintsUpper[currentSearchParams.upperConstraintIndex].x <= constraintsLower[currentSearchParams.lowerConstraintIndex].x ) throw IECore::Exception( "SHALLOWEST GOT BAD 2" );*/
				//currentSearchParams = { -1, -1 };
				//assert( refindShallowSuccess );
			}
		}



		if( debug )
		{
			std::cerr << "\nFOUND LINE\n";
			if( currentSearchParams.lowerConstraintIndex >= 0 )
			{
				std::cerr << "\nLOWER : " <<
					constraintsLower[currentSearchParams.lowerConstraintIndex].x << ", " <<
					constraintsLower[currentSearchParams.lowerConstraintIndex].y << "\n";
			}
			if( currentSearchParams.upperConstraintIndex >= 0 )
			{
				std::cerr << "\nUPPER : " <<
					constraintsUpper[currentSearchParams.upperConstraintIndex].x << ", " <<
					constraintsUpper[currentSearchParams.upperConstraintIndex].y << "\n";
			}
		}

		// TODO - This matches condition below, clean them up
		if(
			foundSlope &&
			//fabs( constraintsLower[ currentSearchParams.lowerConstraintIndex ].x - constraintsUpper[ currentSearchParams.upperConstraintIndex ].x ) > 1e-8f &&  // TODO?
			constraintsLower[ currentSearchParams.lowerConstraintIndex ].y == constraintsUpper[ currentSearchParams.upperConstraintIndex ].y
		)
		{
			// TODO TODO TODO TODO
			/*std::cerr << "\n\nBAD FLAT\n";
			std::cerr << "SEARCHING LOWER: " << lowerStartIndex << " : " << lowerStopIndex << "\n";
			std::cerr << "SEARCHING UPPER: " << upperStartIndex << " : " << upperStopIndex << "\n";
			std::cerr << "CURRENT: " << currentSearchParams.lowerConstraintIndex << " : " << currentSearchParams.upperConstraintIndex << "\n";
			*/

			lowerStartIndex++;
			lowerStopIndex = std::max( lowerStopIndex, lowerStartIndex );

			continue;
		}

		/*if( currentSearchParams.upperConstraintIndex == -1 || currentSearchParams.lowerConstraintIndex == -1 )
		{
			currentSearchParams.upperConstraintIndex = upperStopIndex - 1;
			currentSearchParams.lowerConstraintIndex = lowerStartIndex;
		}*/


		lowerStartIndex = lowerStopIndex;
		// TODO - max?
		upperStartIndex = currentSearchParams.upperConstraintIndex;

		SimplePoint segmentEnd = evaluateLineAtY(
			constraintsLower[currentSearchParams.lowerConstraintIndex],
			constraintsUpper[currentSearchParams.upperConstraintIndex],
			constraintsLower[lowerStopIndex - 1].y
		);
		if( debug )
		{
			std::cerr << "INITIAL yFinal : " << segmentEnd.y << "\n";
		}


		//SimplePoint cacheLowerBeforeIntersect = constraintsLower[ currentSearchParams.lowerConstraintIndex ];
		//SimplePoint cacheUpperBeforeIntersect = constraintsUpper[ currentSearchParams.upperConstraintIndex ];

		SimplePoint prevIntersect;
		if( foundSlope )
		{
			prevIntersect = evaluateLineAtY(
				constraintsLower[currentSearchParams.lowerConstraintIndex],
				constraintsUpper[currentSearchParams.upperConstraintIndex],
				yPrev
			);

			if( compressedSamples.size() > 0 && prevIntersect.x < compressedSamples.back().b.x )
			{
				LinearSegment &prevSegment = compressedSamples.back();
				// Calculate the intersection of our new line with the previous line
				// If the previous sample was a point sample, intersect the new line segment with a vertical line that passes through it.
				prevIntersect = segmentIntersect(
					constraintsLower[currentSearchParams.lowerConstraintIndex],
					constraintsUpper[currentSearchParams.upperConstraintIndex],
					prevSegment.a, prevSegment.b
				);
				if( prevIntersect.x < prevSegment.a.x )
				{
					// This case may occur due to precision issues.
					prevIntersect.x = prevSegment.a.x;
				}
				if( prevIntersect.x > prevSegment.b.x )
				{
					prevIntersect.x = prevSegment.b.x;
				}
			}
		}

		// Calculate where our new line hits its flat top

		if( lowerStopIndex == constraintsLower.size() )
		{
			// All constraints fulfilled
		}
		else if( advanceUpper )
		{
			if( debug ) std::cerr << "ADVANCE UPPER\n";
			// Hit an upper constraint


			// If the previous line goes over the current constraint, then we are going to be underneath it
			// This means that we don't need to worry about upper constraints that it passed under,
			// because if we are heading towards them, we will intersect it
			upperStartIndex = upperStopIndex;

			if( upperStopIndex != constraintsUpper.size() )
			{
				segmentEnd = evaluateLineAtY(
					constraintsLower[currentSearchParams.lowerConstraintIndex],
					constraintsUpper[currentSearchParams.upperConstraintIndex],
					constraintsUpper[ upperStopIndex ].y
				);
			}

			if( lowerStopIndex >= 2 )
			{
				if( segmentEnd.y < constraintsLower[ lowerStopIndex - 1].y )
				{
					if( constraintsLower[ lowerStopIndex - 1].y - segmentEnd.y > 1e-15 )
					{
						std::cerr << "TERRIBLE: " << constraintsLower[ lowerStopIndex - 1].y - segmentEnd.y << "\n"; // TODO
						//throw IECore::Exception( "TERRIBLE" );
					}
					lowerStartIndex--;
				}
				else if( segmentEnd.y > constraintsLower[ lowerStopIndex ].y )
				{
					std::cerr.precision( 20 );
					std::cerr << "LINEAR: " << ( segmentEnd.y ) << " : " << ( constraintsLower[ lowerStopIndex - 1 ].y ) << " -> " << ( constraintsLower[ lowerStopIndex].y ) << "\n";
					std::cerr << "BAD HORIZ-to-LOWER TEST: " << linearToExponential( segmentEnd.y ) << " : " << linearToExponential( constraintsLower[ lowerStopIndex - 1 ].y ) << " -> " << linearToExponential( constraintsLower[ lowerStopIndex].y ) << "\n";
				}
			}
		}
		else
		{

			// TODO The line we found passes under the next lower constraint,
			// so we can find a point where it intersects the lower constraint line

			segmentEnd = evaluateLineAtX(
				constraintsLower[currentSearchParams.lowerConstraintIndex],
				constraintsUpper[currentSearchParams.upperConstraintIndex],
				constraintsLower[lowerStopIndex].x
			);
		}

		if( debug )
		{
			if( lowerStopIndex < constraintsLower.size() )
			{
				std::cerr.precision( 20 );
				std::cerr << "TRYING LOWER CONSTRAINT FAST FORWARD : " << constraintsLower[lowerStopIndex].y << " : " <<  segmentEnd.y << " : " << ( constraintsLower[lowerStopIndex].y <= segmentEnd.y ) << "\n";
			}
		}
		// If our flat top coincides with horizontal segment, we can fast forward to the end of
		// the horizontal segment when considering constraints, because the previous constraints
		// are under the flat top
		while( lowerStopIndex + 1 < constraintsLower.size() && constraintsLower[lowerStopIndex + 1].y <= segmentEnd.y )
		{
			lowerStopIndex++;
			lowerStartIndex = lowerStopIndex;
		}

		if( debug ) std::cerr << "yFinal for computing xEnd : " << segmentEnd.y << "\n";
		// TODO
		/*if( fabs( segmentEnd.y ) == std::numeric_limits<float>::infinity() )
		{
			std::cerr << "????????NEVER?????????" << "\n";
			// We should never get here but if we do, do something fairly sensible.
			segmentEnd = constraintsLower.back();
			xStart = segmentEnd.x;
			lineA = std::numeric_limits<float>::infinity();
			lowerStopIndex = constraintsLower.size();
		}
		else*/

		if( !foundSlope )
		//if( fabs( lineA ) == std::numeric_limits<float>::infinity() )
		{
			// The line we have found is a point sample.
			xStart = segmentEnd.x = constraintsLower[currentSearchParams.lowerConstraintIndex].x;
			assert( std::isfinite( segmentEnd.x ) && !std::isnan( segmentEnd.x ) );
			// TODO - intersect and update constraints
		}
		else
		{
			if( !std::isfinite( segmentEnd.x ) )
			{
				std::cerr << "THROWING BAD\n";
				//assert( false );
				throw IECore::Exception( "BAD" );
			}
			assert( !std::isnan( segmentEnd.x ) );


			// Calculate where our new line hits the previous flat top
			// Calculate the intersection of our new line with the previous line
			xStart = prevIntersect.x;

			// If there is a previous segment, we need to cover the possibility that we are overlapping with it
			if( compressedSamples.size() > 0 && xStart <= compressedSamples.back().b.x )
			{
				LinearSegment &prevSegment = compressedSamples.back();

				// The start of the new segment falls between the start and end of the previous segment.
				// As a result, intersect the two line segments.
				prevSegment.b = prevIntersect;

				// Clamp so that we ensure floating point error doesn't make the previous segment decreasing
				prevSegment.b.y = std::max( prevSegment.a.y, prevSegment.b.y );
			}
		}

		if( debug ) std::cerr << "xstart/end : " << xStart << " : " << segmentEnd.x << "\n";
		assert( segmentEnd.x >= xStart );
		assert( !std::isnan( segmentEnd.y ) );

		LinearSegment compressedSample;
		compressedSample.a = { xStart, compressedSamples.size() > 0 ? compressedSamples.back().b.y : 0.0f };
		compressedSample.b = segmentEnd;


		if( debug )
		{
			std::cerr << "yFinal: " << segmentEnd.y << "\n";
			std::cerr << "SEGMENT INDICES: " << currentSearchParams.lowerConstraintIndex << " : " << currentSearchParams.upperConstraintIndex << "\n";
			std::cerr << "SEGMENT RAW : " << compressedSample.a.x << "\t" << compressedSample.b.x << "\t" << compressedSample.b.y << "\n";
			std::cerr << "SEGMENT: " << compressedSample.a.x << "\t" << compressedSample.b.x << "\t" << linearToExponential( compressedSample.b.y ) << "\n\n\n";
		}

		if( compressedSamples.size() )
		{
			if( compressedSample.b.y < compressedSamples.back().b.y )
			{

				std::cerr << "BAD CURRENT : " << compressedSamples.back().b.y << " -> " << compressedSample.b.y << "\n";
				//throw IECore::Exception( "VERY BAD CURRENT" );

				//TODO
				yPrev = segmentEnd.y;
				continue;
			}
			if( debug && compressedSample.b.y <= compressedSamples.back().b.y )
			{
				std::cerr << "BAD CURRENT : " << compressedSamples.back().b.y << " -> " << compressedSample.b.y << "\n";
				throw IECore::Exception( "BAD CURRENT" );
			}
		}
		if( !std::isfinite( compressedSample.b.x ) )
		{
			throw IECore::Exception( "NON FINITE XBACK" );
		}
		compressedSamples.push_back( compressedSample );

		yPrev = segmentEnd.y;
	}

	// TODO - adjust XBack too
	/*if( compressedSamples.size() > 1 )
	{
		float prevY = compressedSamples[ compressedSamples.size() - 2 ].YBack;
		compressedSamples.back().XBack = ( constraintsUpper.back().y - prevY ) / ( compressedSamples.back().YBack - prevY ) * ( compressedSamples.back().XBack - compressedSamples.back().X ) + compressedSamples.back().X;
	}
	compressedSamples.back().YBack = constraintsUpper.back().y;*/

	//assert( compressedSamples.back().YBack == constraints.back().minY );
	//

	// As per section on Opaque Volume Samples in "Interpreting OpenEXR Deep Pixels", don't write out a large opaque volume sample
	// Instead, write a large almost opaque sample, followed by a tiny point sample to take it up to opaque
	if( compressedSamples.back().b.y > linearOpacityThreshold && compressedSamples.back().b.x != compressedSamples.back().a.x )
	{
		float YPrev = compressedSamples.size() > 1 ? compressedSamples[ compressedSamples.size() - 2].b.y : 0.0f;

		if( YPrev < linearOpacityThreshold )
		{
			float thresholdX = ( (linearOpacityThreshold - YPrev ) / ( compressedSamples.back().b.y - YPrev ) ) * ( compressedSamples.back().b.x - compressedSamples.back().a.x ) + compressedSamples.back().a.x;

			LinearSegment finalSample;
			finalSample.a = { thresholdX, linearOpacityThreshold };
			finalSample.b = compressedSamples.back().b;

			if( finalSample.a.y > finalSample.b.y )
			{
				throw IECore::Exception( "FINAL SAMPLE INSIDE OUT" );
			}


			// TODO - if sample starts over linearOpacityThreshold, this turns it inside out.  Fix this
			compressedSamples.back().b = finalSample.a;
			if( !std::isfinite( thresholdX ) )
			{
				throw IECore::Exception( "BAD THRESHOLD" );
			}

			compressedSamples.push_back( finalSample );
		}
	}
}

inline float applyZTol( float z, float tol, bool upper )
{
	return z * ( 1 + ( upper ? -tol : tol ) * copysign( 1.0f, z ) );
}

/*
 From a deep pixel, create a list of constraints in linear space that define the path the alpha curve must take to stay within
 the given alpha and z tolerance
*/
void linearConstraintsForPixel(
	const int inSamples, const float *inA, const float *inZ, const float *inZBack,
	float alphaTolerance, float zTolerance, float silhouetteDepth, float quality,
	std::vector< SimplePoint > &lowerConstraints,
	std::vector< SimplePoint > &upperConstraints
)
{
	if( !inSamples )
	{
		return;
	}

	quality = std::min( 0.99f, std::max( 0.5f, quality ) );

	float indexToAlpha = alphaTolerance * ( 1.0f - quality );
	float alphaToIndex = 1.0f / indexToAlpha;
	//int alphaIndices = ceilf( alphaToIndex ) + 1; // TODO - not used yet

	// ---- Set up lower constraints ----
	// ---- Set up upper constraints ----

	lowerConstraints.reserve( inSamples ); // TODO
	upperConstraints.reserve( inSamples );


	// TODO - something weird happens when starting from depth 0 ( constraint isn't offset backwards? )

	float targetAlpha = 0;

	for ( int i = 0; i < inSamples; ++i )
	{
		float Z = inZ[ i ];
		float ZBack = inZBack[ i ];

		float targetSegmentAlpha = inA[i];
		float nextTargetAlpha = targetAlpha + ( targetSegmentAlpha - targetAlpha * targetSegmentAlpha );

		if( nextTargetAlpha - alphaTolerance > 0.0f )
		{
			if( ZBack == Z )
			{
				lowerConstraints.push_back( (SimplePoint){
					applyZTol( Z, zTolerance, false ),
					-log1pf( -( nextTargetAlpha - alphaTolerance ) )
				} );
			}
			else
			{
				float stepDepth = applyZTol( Z, zTolerance, false );

				int nextCross = ceilf( targetAlpha * alphaToIndex );
				int lastCross = floorf( nextTargetAlpha * alphaToIndex );

				float nextZTol = applyZTol( Z, zTolerance * ( 1.0f - quality ), false );

				float zScale = -( ZBack - Z ) / log1pf( -targetSegmentAlpha );
				float zOffset = log1pf( -targetAlpha );

				for( int i = nextCross; i <= lastCross; i++ )
				{
					float ai = indexToAlpha * i;

					float crossZ = zScale * (-log1pf( -ai ) + zOffset ) + Z;
					if( ai - alphaTolerance <= 0.0f )
					{
						stepDepth = applyZTol( crossZ, zTolerance, false );
						continue;
					}

					if( crossZ < nextZTol )
					{
						continue;
					}

					nextZTol = applyZTol( crossZ, zTolerance * ( 1.0f - quality ), false );

					lowerConstraints.push_back( (SimplePoint){
						stepDepth,
						exponentialToLinear( ai - alphaTolerance )
					} );

					stepDepth = applyZTol( crossZ, zTolerance, false );
				}
				
				lowerConstraints.push_back( (SimplePoint){
					stepDepth,
					exponentialToLinear( nextTargetAlpha - alphaTolerance )
				} );
			}
		}
	
		if( targetAlpha + alphaTolerance < 1.0f )
		{
			if( ZBack == Z )
			{
				upperConstraints.push_back( (SimplePoint){
					applyZTol( Z, zTolerance, true ),
					-log1pf( -( targetAlpha + alphaTolerance ) )
				} );
			}
			else
			{
				float stepAlphaLinear = exponentialToLinear( targetAlpha + alphaTolerance );

				int nextCross = ceilf( targetAlpha * alphaToIndex );
				int lastCross = floorf( nextTargetAlpha * alphaToIndex );

				float nextZTol = applyZTol( Z, zTolerance * ( 1.0f - quality ), false );

				float zScale = -( ZBack - Z ) / log1pf( -targetSegmentAlpha );
				float zOffset = log1pf( -targetAlpha );

				for( int i = nextCross; i <= lastCross; i++ )
				{
					float ai = indexToAlpha * i;
					float crossZ = zScale * (-log1pf( -ai ) + zOffset ) + Z;
					if( crossZ < nextZTol )
					{
						continue;
					}
					nextZTol = applyZTol( crossZ, zTolerance * ( 1.0f - quality ), false );

					upperConstraints.push_back( (SimplePoint){
						applyZTol( crossZ, zTolerance, true ),
						stepAlphaLinear
					} );

					stepAlphaLinear = exponentialToLinear( ai + alphaTolerance );
				}
				
				upperConstraints.push_back( (SimplePoint){
					applyZTol( ZBack, zTolerance, true ),
					stepAlphaLinear
				} );
			}
		}

		targetAlpha = nextTargetAlpha;

		//prevLower = nextLower;
		//prevLowerAlpha = nextLowerAlpha;

	}
}

}

namespace GafferImage
{

namespace DeepAlgo
{

namespace Detail
{

void debugConstraintsForPixel(
	const int inSamples, const float *inA, const float *inZ, const float *inZBack,
	float alphaTolerance, float zTolerance, float silhouetteDepth,
	std::vector< std::pair<float,float> > &lowerConstraints,
	std::vector< std::pair<float,float> > &upperConstraints
)
{
	std::vector< SimplePoint > lowerLinear;
	std::vector< SimplePoint > upperLinear;
	linearConstraintsForPixel(
		inSamples, inA, inZ, inZBack, 
		alphaTolerance, zTolerance, silhouetteDepth, 0.75f,
		lowerLinear, upperLinear
	);

	lowerConstraints.clear();
	upperConstraints.clear();
	lowerConstraints.reserve( lowerLinear.size() );
	upperConstraints.reserve( upperLinear.size() );
	for( SimplePoint &c : lowerLinear )
	{
		if( lowerConstraints.size() ) assert( ((float)linearToExponential( c.y )) >= lowerConstraints.back().second );
		lowerConstraints.push_back( { c.x, linearToExponential( c.y ) } );
	}
	for( SimplePoint &c : upperLinear )
	{
		upperConstraints.push_back( { c.x, linearToExponential( c.y ) } );
	}
}

}

// TODO - some way to specify tolerances in other channels
// TODO : Shouldn't really be any need for anything to be in float precision here?
void resampleDeepPixel(
	const int inSamples, const float *inA, const float *inZ, const float *inZBack,
	const std::vector<const float *> &colorChannels,
	float alphaTolerance, float colorTolerance, float zTolerance, float silhouetteDepth,
	int &outSamples, float *outA, float *outZ, float *outZBack,
	bool debug
)
{
	if( inSamples <= 1 )
	{
		outSamples = inSamples;
		if( inSamples == 1 )
		{
			outA[0] = inA[0];
			outZ[0] = inZ[0];
			outZBack[0] = inZBack[0];
		}
		return;
	}

	// TODO - move these allocations outside loop?
	std::vector<SimplePoint> constraintsLower;
	std::vector<SimplePoint> constraintsUpper;
	linearConstraintsForPixel(
		inSamples, inA, inZ, inZBack, 
		alphaTolerance, zTolerance, silhouetteDepth, 0.75f,
		constraintsLower, constraintsUpper
	);

	if( !constraintsLower.size() )
	{
		// If there are no lower constraints, then an empty curve is valid.
		// This only happens when the input segments all have 0 alpha.
		outSamples = 0;
		return;
	}

	if( debug )
	{
		std::cerr << "LOWER\n";
		for( const auto &i : constraintsLower )
		{
			std::cerr << i.x << "," << i.y << " ";
		}
		std::cerr << "\nUPPER\n";
		for( const auto &i : constraintsUpper )
		{
			std::cerr << i.x << "," << i.y << " ";
		}
		std::cerr << "\n\n";
	}


	std::vector< LinearSegment > compressedSamples;
	minimalSegmentsForConstraints( constraintsLower, constraintsUpper, compressedSamples, debug );

	// TODO - runtime check? Plus one?
	assert( compressedSamples.size() <= (size_t)inSamples + 1 );


	for( unsigned int i = 1; i < compressedSamples.size(); ++i )
	{
		if( i > 0 && compressedSamples[i-1].b.y > compressedSamples[i].b.y )
		{
			throw IECore::Exception( "NON-DECREASING PRE CHECK FAILED" );
		}
	}

	if( colorChannels.size() == 0 || true ) //TODO
	{
		linearSegmentsToExponentialAlpha( compressedSamples, outSamples, outA, outZ, outZBack );
	}
	else
	{
		// TODO - this is the place to apply colorThreshold.
		// We need to linearSegmentsToExponentialAlpha into a temp buffer
		// Then do a conformToAlpha of all the color data ( ideally refactored so we can do it
		// one sample at a time ).
		// Then do a pair traverse of the original color/alpha
		// evaluated at each original ZBack, and the resampled point with a matching alpha.
		// Anywhere that there is a discrepancy over the colorThreshold, split the resampled
		// segment at that depth
	}

	

	return;
}

void conformToAlpha( int inSamples, int outSamples, const float *inAlpha, const float *outAlpha, const float *inChannel, float *outChannel )
{

	if( inSamples == 0 || outSamples == 0 )
	{
		return;
	}

	float totalAccumAlpha = 0;

	float alphaRemaining = std::min( 1.0f, inAlpha[ 0 ] );

	int integrateIndex = 0;
	float targetAlpha = 0;
	for( int i = 0; i < outSamples; ++i )
	{
		targetAlpha += outAlpha[i] - targetAlpha * outAlpha[i];

		float segmentAccumChannel = 0;
		float segmentAccumAlpha = 0;

		while( integrateIndex < inSamples )
		{
			float alphaNeeded = totalAccumAlpha < 1.0f ? ( targetAlpha - totalAccumAlpha ) / ( 1 - totalAccumAlpha ) : 0.0f;

			float alphaToTake;
			if( alphaNeeded >= alphaRemaining || alphaNeeded >= 1.0f )
			{
				alphaToTake = alphaRemaining;
				alphaRemaining = 0;
			}
			else
			{
				alphaToTake = alphaNeeded;
				alphaRemaining = ( alphaRemaining - alphaNeeded ) / ( 1 - alphaNeeded );
			}

			totalAccumAlpha += ( 1 - totalAccumAlpha ) * alphaToTake;
			float outputAlpha = ( 1 - segmentAccumAlpha ) * alphaToTake;

			float channelMultiplier = outputAlpha > 0.0f ? outputAlpha / inAlpha[ integrateIndex ] : 0.0f;
			segmentAccumChannel += channelMultiplier * inChannel[ integrateIndex ];
			segmentAccumAlpha += outputAlpha;

			if( alphaRemaining > 0 )
			{
				break;
			}
			else
			{
				integrateIndex++;
				if( !( integrateIndex < inSamples ) )
				{
					break;
				}
				alphaRemaining = std::min( 1.0f, inAlpha[ integrateIndex ] );
			}
		}

		outChannel[i] = segmentAccumChannel;
	}
}

}

}

