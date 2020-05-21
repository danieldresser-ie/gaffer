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
float linearToExponential( float value )
{
	return value == maximumLinearY ? 1 : -expm1f( -value );
}

/// Given a value in exponential space, make it linear.
float exponentialToLinear( float value )
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


SimplePoint intersectWithSegment( SimplePoint p, SimplePoint dir, SimplePoint a2, SimplePoint b2 )
{
	SimplePoint disp = { a2.x - p.x, a2.y - p.y };
	SimplePoint dir2 = { b2.x - a2.x, b2.y - a2.y };

	float denom = dir.x * dir2.y - dir.y * dir2.x;
	if( fabs( denom ) < 1e-10 )
	{
		// TODO what if lines not coincident
		// TODO this logic now looks bad
		// TODO and even worse
		if( p.x + dir.x < b2.x )
		{
			//return {p.x + dir.x, p.y + dir.y};
			return {
				std::max( a2.x, std::min( b2.x, p.x + dir.x ) ),
				std::max( a2.y, std::min( b2.y, p.y + dir.y ) )
			}; 
		}
		else
		{
			return b2;
		}
	}

	float t = ( disp.x * dir2.y - disp.y * dir2.x ) / denom;

	return { p.x + dir.x * t, p.y + dir.y * t }; 
	/*SimplePoint r = {
		std::max( a2.x, std::min( b2.x, p.x + dir.x * t ) ),
		std::max( a2.y, std::min( b2.y, p.y + dir.y * t ) )
	}; 

	/////// TODO TEMP
	//if( a2.x == b2.x )
	//{
		//r.x = a2.x;
	//}
	//if( a2.y == b2.y )
	//{
		//r.y = a2.y;
	//}
	/////// TODO TEMP

	// TODO - note about assuming increasingness
	//if( r.x < a2.x || r.y < a2.y )
	//{
	//	return a2;
	//}
	//else if( r.x > b2.x || r.y > b2.y )
	//{
	//	return b2;
	//}

	return r;*/

	// TODO testing without the special cases above is a good way to test hang handling
}

SimplePoint clampToBox( SimplePoint p, SimplePoint a, SimplePoint b )
{
	return {
		std::max( a.x, std::min( b.x, p.x ) ),
		std::max( a.y, std::min( b.y, p.y ) )
	}; 
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
 Given a set of segments that define an optimal resampling which fufills all constraints in the linear remapped space,
 remap all the color and alpha channels of a deep pixel to match
*/
void conformToSegments( 
	const int inSamples, const float *inA, const float *inZ, const float *inZBack,
	const std::vector< LinearSegment >& compressedSamples,
	int &outSamples, float *outA, float *outZ, float *outZBack
)
{
	outSamples = compressedSamples.size();

	float totalAccumAlpha = 0;

	float curA = 0;
	float accumA = 0;

	float alphaRemaining = 0.0;

	int integrateIndex = 0;
	for( unsigned int i = 0; i < compressedSamples.size(); ++i )
	{
		// We could potentially get samples squashed down to being zero length and overlapping,
		// which would result in them not being merged in the right order.  Outputting a single
		// sample will yield a correct result.
		if(
			i + 1 < compressedSamples.size() &&
			compressedSamples[i].a.x == compressedSamples[i].b.x &&
			compressedSamples[i].a.x == compressedSamples[i+1].a.x &&
			compressedSamples[i+1].a.x == compressedSamples[i+1].b.x
		)
		{
			continue;
		}

		// The other situation that could arise from floating point error is a segment that doesn't
		// rise above the previous sample segment.  Rather than outputting a 0 alpha segment, just
		// skip it
		if( i > 0 && compressedSamples[i].b.y <= compressedSamples[i-1].b.y )
		{
			continue;
		}

		float targetAlpha = -expm1f( -compressedSamples[i].b.y );
		if( ! ( targetAlpha > -0.1 && targetAlpha < 1.1 ) )
		{
			throw IECore::Exception( "BAD TARGET " );
		}

		for( ; integrateIndex < inSamples; ++integrateIndex  )
		{
			if( alphaRemaining == 0.0 )
			{
				curA = inA[ integrateIndex ];
				alphaRemaining = curA;
			}

			if( curA >= 1 )
			{
				if( integrateIndex == inSamples - 1 )
				{
					curA = 1.;
				}
				else
				{
					curA = 1. - std::numeric_limits<float>::epsilon(); // TODO - why?
				}
			}

			float alphaNeeded = ( targetAlpha - totalAccumAlpha ) / ( 1 - totalAccumAlpha );

			float alphaToTake;
			if( alphaNeeded >= alphaRemaining )
			{
				assert( alphaRemaining >= 0 );
				alphaToTake = alphaRemaining;
				alphaRemaining = 0;
				totalAccumAlpha += ( 1 - totalAccumAlpha ) * alphaToTake;
				totalAccumAlpha = std::min( targetAlpha, totalAccumAlpha );
			}
			else
			{
				//assert( alphaNeeded >= 0 );
				if( alphaNeeded < 0 )
				{
					std::cerr << "BAD alphaNeeded : " << alphaNeeded << "\n";
					alphaNeeded = 0;
				}
				alphaToTake = alphaNeeded;
				alphaRemaining = 1 - ( 1 - alphaRemaining ) / ( 1 - alphaNeeded );

				totalAccumAlpha = targetAlpha;
			}

			float curChannelMultiplier = curA > 0 ? ( 1 - accumA ) * alphaToTake / curA : 0.0; // TODO
			if( curChannelMultiplier < 0 )
			{
				std::cerr << "BAD MULTIPLIER :" << curChannelMultiplier << " : " << alphaToTake << "\n";
			}

			accumA += curChannelMultiplier * curA;

			if( alphaRemaining > 0 )
			{
				break;
			}
		}

		if( compressedSamples[i].b.x < compressedSamples[i].a.x )
		{
			std::cerr << "BAD XBACK: " << compressedSamples[i].a.x << " -> " << compressedSamples[i].b.x << "\n";
		}

		if( ! ( accumA > -0.1 && accumA < 1.1 ) )
		{
			std::cerr << "BAD CALC: " << accumA << "\n";
			accumA = 0.0f;
		}

		outZ[i] = compressedSamples[i].a.x;
		outZBack[i] = compressedSamples[i].b.x;
		outA[i] = accumA;

		accumA = 0.0f;
	}
}


/*
 Given a set of constraints in linear space, put togther a set of segments that pass through all of them
 */
// TODO - note that this overwrites the constraints
void minimalSegmentsForConstraints( 
	std::vector<SimplePoint> &constraintsLower, std::vector<SimplePoint> &constraintsUpper,  // TODO - these names are stupid
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

		int origLowerStartIndex = lowerStartIndex;

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

			if( upperStopIndex != constraintsUpper.size() && upperStopIndex >= 2 )
			{ 
				if( debug ) std::cerr << "CHECKING UPPER:" << constraintsUpper[ upperStopIndex - 1].x << " " << constraintsUpper[ upperStopIndex ].x << "\n";
				if( debug ) std::cerr << "CHECKING UPPER Y:" << linearToExponential( constraintsUpper[ upperStopIndex - 1 ].y ) << " " << linearToExponential( constraintsUpper[ upperStopIndex ].y ) << "\n";

				SimplePoint intersection = intersectWithSegment(
					constraintsLower[ currentSearchParams.lowerConstraintIndex ],
					{ constraintsUpper[ currentSearchParams.upperConstraintIndex ].x - constraintsLower[ currentSearchParams.lowerConstraintIndex ].x,
					constraintsUpper[ currentSearchParams.upperConstraintIndex ].y - constraintsLower[ currentSearchParams.lowerConstraintIndex ].y },
					constraintsUpper[ upperStopIndex - 1 ],
					constraintsUpper[ upperStopIndex ]
				);
				
				intersection = clampToBox(
					intersection, constraintsUpper[ upperStopIndex - 1 ], constraintsUpper[ upperStopIndex ]
				);

				if( !(
					intersection.x == constraintsUpper[upperStopIndex ].x && 
					intersection.y == constraintsUpper[upperStopIndex ].y
				) )
				{
					upperStartIndex--;
				}
				if( debug ) std::cerr << "**** CHANGE UPPER << " << intersection.x << " , " << intersection.y << "\n";
				constraintsUpper[upperStartIndex] = intersection;
				segmentEnd = intersection;
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
				else
				{

					if( constraintsLower[ lowerStopIndex ].y != constraintsLower[ lowerStopIndex - 1 ].y )  // TODO
					{
						lowerStartIndex--;
						constraintsLower[lowerStartIndex] = evaluateLineAtY(
							constraintsLower[ lowerStopIndex - 1 ], constraintsLower[ lowerStopIndex ], segmentEnd.y
						);
						//constraintsLower[lowerStartIndex].x = ( segmentEnd.y - constraintsLower[ lowerStopIndex - 1 ].y ) / ( constraintsLower[ lowerStopIndex ].y - constraintsLower[ lowerStopIndex - 1 ].y ) * ( constraintsLower[ lowerStopIndex ].x - constraintsLower[ lowerStopIndex - 1 ].x ) + constraintsLower[ lowerStopIndex - 1 ].x;
						//constraintsLower[lowerStartIndex].y = segmentEnd.y;
					}
				}
			}
		}
		else
		{
	
			// TODO The line we found passes under the next lower constraint,
			// so we can find a point where it intersects the lower constraint line

			if( lowerStopIndex >= 2 )
				//&& constraintsLower[ lowerStopIndex - 1 ].x != constraintsLower[ lowerStopIndex ].x
			{ 
				if( debug ) std::cerr << "CHECKING LOWER:" << constraintsLower[ lowerStopIndex - 1].x << " " << constraintsLower[ lowerStopIndex ].x << "\n";
				if( debug ) std::cerr << "CHECKING LOWER Y:" << linearToExponential( constraintsLower[ lowerStopIndex - 1 ].y ) << " " << linearToExponential( constraintsLower[ lowerStopIndex ].y ) << "\n";
				SimplePoint intersection = intersectWithSegment(
					constraintsLower[ currentSearchParams.lowerConstraintIndex ],
					{ constraintsUpper[ currentSearchParams.upperConstraintIndex ].x - constraintsLower[ currentSearchParams.lowerConstraintIndex ].x,
					constraintsUpper[ currentSearchParams.upperConstraintIndex ].y - constraintsLower[ currentSearchParams.lowerConstraintIndex ].y },
					constraintsLower[ lowerStopIndex - 1 ],
					constraintsLower[ lowerStopIndex ]
				);

				intersection = clampToBox(
					intersection, constraintsLower[ lowerStopIndex - 1 ], constraintsLower[ lowerStopIndex ]
				);

				if( intersection.y <= constraintsLower[origLowerStartIndex].y && intersection.x <= constraintsLower[origLowerStartIndex].x )
				{
					intersection = constraintsLower[ lowerStopIndex ];
					if( debug ) std::cerr << "BAD FP SEGMENT INTERSECT\n";
				}

				if( !(
					intersection.x == constraintsLower[lowerStopIndex ].x && 
					intersection.y == constraintsLower[lowerStopIndex ].y
				) )
				{
					lowerStartIndex--;
				}
				if( debug ) std::cerr << "**** CHANGE LOWER << " << intersection.x << " , " << intersection.y << "\n";
				constraintsLower[lowerStartIndex] = intersection;
				segmentEnd = constraintsLower[lowerStartIndex];
			}
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
			assert( std::isfinite( segmentEnd.x ) && !isnan( segmentEnd.x ) );
			// TODO - intersect and update constraints
		}
		else
		{
			if( std::isnan( segmentEnd.x ) )
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
				throw IECore::Exception( "VERY BAD CURRENT" );
				
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
			throw IECore::Exception( "BAD XBACK" );
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

/*
 Given a deep pixel which may contain a mixture of point and volume samples, produce a set of samples which represent the
 alpha falloff curve, where each sample stores a depth and the integrated alpha up to that point.

 Note that currently volume samples are just split into a fixed number of point samples.  A better approach would be
 to split enough times that the alpha increment for each sample is less than the alpha threshold.
*/
void integratedPointSamplesForPixel(
	const int inSamples, const float *inA, const float *inZ, const float *inZBack,
	std::vector<SimplePoint> &deepSamples
)
{
	float ZBackPrev = -1;
	float accumAlpha = 0;

	for ( int i = 0; i < inSamples; ++i )
	{
		// TODO - should I investigate why this causes weirdness?
		if( inA[i] <= 0.0 )
		{
			continue;
		}

		float Z = inZ[ i ];
		float ZBack = inZBack[ i ];

		if( Z != ZBackPrev )
		{
			deepSamples.push_back( { Z, accumAlpha } );
		}

		float nextAccumAlpha = accumAlpha + ( inA[i] - accumAlpha * inA[i] );

		if( nextAccumAlpha >= maxConvertibleAlpha )
		{
			if( inA[i] >= maxConvertibleAlpha || accumAlpha >= maxConvertibleAlpha || Z == ZBack )
			{
				accumAlpha = 1.0f;
			}
			else
			{
				// Weird special case:  if we convert this to an accumulated alpha, it can't be represented
				// as a floating point number, because it's less than 1, but greater than the next lowest
				// floating point number.  If we convert it to 1, we destroy the exponential curved shape
				// of the sample, which in extreme cases, could be significant.  So we need to round it
				// down to maxConvertibleAlpha, but then we also need to adjust ZBack to preserve the curve
				// shape.
				float alphaTaken = ( maxConvertibleAlpha - accumAlpha ) / ( 1 - accumAlpha );
				float fractionTaken = log1p( -alphaTaken ) / log1p( -inA[i] );
				ZBack = std::max( Z, std::min( ZBack, Z + ( ZBack - Z ) * fractionTaken ) );
				accumAlpha = maxConvertibleAlpha;
			}
		}
		else
		{
			accumAlpha = nextAccumAlpha;
		}
		if( deepSamples.size() ) assert( accumAlpha >= deepSamples.back().y );
		deepSamples.push_back( { ZBack, accumAlpha } );
	
		ZBackPrev = ZBack;	
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
	const std::vector<const float *> &colorChannels,
	float alphaTolerance, float colorTolerance, float zTolerance, float silhouetteDepth,
	std::vector< SimplePoint > &lowerConstraints,
	std::vector< SimplePoint > &upperConstraints
)
{
	if( !inSamples )
	{
		return;
	}

	// The curve defining alpha with respect to depth are exponential curves.
	// While finding the resampled segments, we work in a logarithmic space where the original curve and the resampled curve
	// are made of straight line segments.  This makes everything a lot simpler.  Unfortunately, in order to take
	// advantage of this simplicity, we want to treat the boundary constraints as straight lines too.  However, the
	// boundaries are formed by linearly offsetting exponential curves, which means they aren't actually straight lines in the logarithmic space, though they are fairly close as long as alpha tolerance is low.
	//
	// For a curve segment where the alpha changes from "a" up to "b", the error between a simple exponential curve
	// ( relative to 1.0 ), and a curve with the same endpoint, but is exponential before being offset by "c", is:
	// ( 1 - a + c )^( x * ( ln(1-b+c)/ln(1-a+c) - 1 ) + 1 ) - ( 1 - a )^( x * ( ln(1-b)/ln(1-a) - 1) + 1 ) - c
	//
	// Some experiments in looking at the maximum value of this curve over [0,1], show that
	// choosing "b <= 1 - 0.76 * ( 1 - a )" ensures that the error is less than 1% of the offset c,
	// regardless of the value of c.
	//
	// This shows that by adding extra samples to ensure that no segment of the constraints is too long,
	// combined with reducing the alpha tolerance by 1%, we can ensure we never exceed the given tolerances,
	// while still treating the constraints as linear segments.
	//
	// In log space, the above constraint becomes a constraint that the difference between adjacent samples
	// can never exceed ln( 0.76 )
	const float curveSampleRatio = 0.76;
	const float minimumCurveSample = std::max( 0.01 * alphaTolerance, 0.000001 );
	//const float stepToNextCurveSample = -log( 0.76 );
	//const float maxCurveSample = exponentialToLinear( 1 - 0.01 * alphaTolerance );

	// The alpha tolerance that we actually use is padded to take account of the curve error
	float aTol = 0.99 * alphaTolerance;
	
	// TODO - get rid of this alloc by combining with function above
	std::vector< SimplePoint > deepSamples;
	integratedPointSamplesForPixel( inSamples, inA, inZ, inZBack, deepSamples );
	if( deepSamples.size() == 0 )
	{
		// TODO : Currently, as per other TODO, we skip 0 alpha samples, so this could trigger
		return;
	}

	lowerConstraints.reserve( deepSamples.size() ); // TODO
	upperConstraints.reserve( deepSamples.size() );
	//float lastValidMinY = 0;
	float prevAlpha = 0;
	SimplePoint prevLower = { 0, 0 };
	for( unsigned int j = 0; j < deepSamples.size(); ++j )
	{
		float nextX;
		float nextAlpha;

		if( j == deepSamples.size() - 1 )
		{
			// Ensure that we always reach the exact final value
			nextX = applyZTol( deepSamples[j].x, zTolerance, false );
			nextAlpha = std::min( deepSamples[j].y, maximumLinearY );
		}
		else
		{
			float minAlpha = deepSamples[j].y - aTol;
			if( minAlpha >= 0 )
			{
				/*float min = 0.0f;
				if( minAlpha < 1 )
				{
					min = -log1pf( -minAlpha );
					lastValidMinY = min;
				}
				else
				{
					min = lastValidMinY; // TODO - why?
				}*/

				nextX = applyZTol( deepSamples[j].x, zTolerance, false );
				nextAlpha = std::min( minAlpha, maximumLinearY );
			}
			else
			{
				float nextMinAlpha = deepSamples[j+1].y - aTol;
				if( nextMinAlpha > 0 )
				{
					//float min = -log1pf( -minAlpha );
					//float nextMin = -log1pf( -nextMinAlpha );

					//BLAH // This should look more like the more complex calculation below
					//float lerp = -min / ( nextMin - min );
					//float lerp = ( -exponentialToLinear( aTol ) -min ) / ( nextMin - min );
					float lerp = ( exponentialToLinear( aTol ) - exponentialToLinear( deepSamples[j].y ) ) /
						( exponentialToLinear(deepSamples[j+1].y) - exponentialToLinear(deepSamples[j].y ) );

					if( std::isnan( lerp ) || !std::isfinite( lerp ))
					{
						throw IECore::Exception( "CUR" );
					}
		
					float xIntercept = deepSamples[j].x + ( deepSamples[j+1].x - deepSamples[j].x ) * lerp;
					// Make sure floating point error doesn't violate non-decreasing X
					xIntercept = std::min( deepSamples[j+1].x, xIntercept );
					nextX = applyZTol( xIntercept, zTolerance, false );
					nextAlpha = 0.0;
				}
				else
				{
					continue;
				}
			}
		}

		SimplePoint nextLower = { nextX, exponentialToLinear( nextAlpha ) };

		float nextCurveSampleAlpha = ( prevAlpha < 1 - aTol - minimumCurveSample ) ? 1 - aTol - curveSampleRatio * ( 1 - aTol - prevAlpha ) : 1.0;
		float nextCurveSample = nextCurveSampleAlpha != 1.0 ? exponentialToLinear( nextCurveSampleAlpha ) : std::numeric_limits<float>::infinity();

		//float nextCurveSample = prevLower.y < maxCurveSample ? prevLower.y + stepToNextCurveSample : std::numeric_limits<float>::infinity();
		int qqq = 0;
		while( nextCurveSample < nextLower.y && prevLower.x != nextLower.x )
		{
			qqq++;
			if( qqq % 1000 == 0 )
			{
				std::cerr << "QQQ: " << nextCurveSample << "\n";
			}
			float denom = exponentialToLinear(deepSamples[j].y) - exponentialToLinear(deepSamples[j - 1].y );
			if( denom == 0.0f )
			{
				break; // TODO - don't 100% understand the circumstances this occurs in
			}
			float lerp = ( exponentialToLinear( linearToExponential( nextCurveSample ) + aTol ) - exponentialToLinear( deepSamples[j - 1].y ) ) / denom;
			if( !std::isfinite( lerp ))
			{
				throw IECore::Exception( "WHAT: " + std::to_string( deepSamples[j].y ) + " : " + std::to_string( deepSamples[j - 1].y ) );
			}
			float intersectionX = applyZTol( deepSamples[j - 1].x + lerp * ( deepSamples[j].x - deepSamples[j - 1 ].x ),
				zTolerance, false );
		
			if( intersectionX > nextLower.x )
			{
				break;
			}

			if( !std::isfinite( intersectionX ))
			{
				throw IECore::Exception( "HOOWWWWW??" );
			}

			lowerConstraints.push_back( { intersectionX, nextCurveSample } );

			//nextCurveSample = nextCurveSample < maxCurveSample ? nextCurveSample + stepToNextCurveSample : std::numeric_limits<float>::infinity();
			nextCurveSampleAlpha = ( nextCurveSampleAlpha < 1 - aTol - minimumCurveSample ) ? 1 - aTol - curveSampleRatio * ( 1 - aTol - nextCurveSampleAlpha ) : 1.0;
			nextCurveSample = nextCurveSampleAlpha != 1.0 ? exponentialToLinear( nextCurveSampleAlpha ) : std::numeric_limits<float>::infinity();
		}

		if( std::isnan( nextLower.x ) || !std::isfinite( nextLower.x ))
		{
			throw IECore::Exception( "BLAH" );
		}
		if( lowerConstraints.size() ) assert( nextLower.y >= lowerConstraints.back().y );
		lowerConstraints.push_back( nextLower );
		prevLower = nextLower;
		prevAlpha = nextAlpha;
		
	}

	unsigned int matchingLowerIndex = 0;

	assert( deepSamples.size() >= 1 );
	assert( deepSamples[0].y == 0 );
	assert( !isinf( deepSamples[0].x ) );

	// TODO - something weird happens when starting from depth 0 ( constraint isn't offset backwards? )
	// TODO - incorrect results for giant segment with alpha extremely close to 1 ( output is curved )

	SimplePoint prevUpper = { applyZTol( deepSamples[0].x, zTolerance, true ), 0 };
	prevAlpha = 0;
	upperConstraints.push_back( prevUpper );
	assert( !isinf( prevUpper.x ) );
	assert( prevUpper.x > std::numeric_limits<float>::lowest() );
	
	for( unsigned int j = 1; j < deepSamples.size(); ++j )
	{
		float nextAlpha = deepSamples[j].y + aTol;

		// TODO - comment
		SimplePoint nextUpper = {
			applyZTol( deepSamples[j].x, zTolerance, true ),
			nextAlpha < 1 ? -log1pf( -nextAlpha ) : maximumLinearY
		};

		if( nextUpper.x == prevUpper.x )
		{
			// Vertical constraints are simple to deal with - we don't need extra samples to define the curve
			// shape or to make sure we hit thresholds properly.  Just fast-forward matchingLowerIndex
			while( matchingLowerIndex < lowerConstraints.size() && lowerConstraints[matchingLowerIndex].y <= nextUpper.y )
			{
				matchingLowerIndex++;
			}
		}
		else
		{
			float nextCurveSampleAlpha = ( prevAlpha < 1 - minimumCurveSample ) ? 1 - curveSampleRatio * ( 1 - prevAlpha ) : 1.0;
			float nextCurveSample = nextCurveSampleAlpha != 1.0 ? exponentialToLinear( nextCurveSampleAlpha ) : std::numeric_limits<float>::infinity();
			while(
				matchingLowerIndex < lowerConstraints.size() &&
				( lowerConstraints[matchingLowerIndex].y < nextUpper.y || nextCurveSample < nextUpper.y )
			)
			{
				float yValueToInsert;
				if( lowerConstraints[matchingLowerIndex].y < nextCurveSample )
				{
					yValueToInsert = lowerConstraints[matchingLowerIndex].y;
					matchingLowerIndex++;
				}
				else
				{
					yValueToInsert = nextCurveSample;
					//nextCurveSample = nextCurveSample < maxCurveSample ? nextCurveSample + stepToNextCurveSample : std::numeric_limits<float>::infinity();
					nextCurveSampleAlpha = ( nextCurveSampleAlpha < 1 - minimumCurveSample ) ? 1 - curveSampleRatio * ( 1 - nextCurveSampleAlpha ) : 1.0;
					nextCurveSample = nextCurveSampleAlpha != 1.0 ? exponentialToLinear( nextCurveSampleAlpha ) : std::numeric_limits<float>::infinity();
				}


				upperConstraints.push_back( {
					( exponentialToLinear( linearToExponential( yValueToInsert ) - aTol ) - exponentialToLinear( deepSamples[j - 1].y ) ) / ( exponentialToLinear(deepSamples[j].y) - exponentialToLinear(deepSamples[j - 1].y ) ) *
					( nextUpper.x - prevUpper.x ) + prevUpper.x,
					yValueToInsert
				} );
			}
		}

		if( matchingLowerIndex == lowerConstraints.size() )
		{
			// Once we've matched the last lower constraint, we can stop outputting constraints - this
			// is where the constraint search will reach the final accumulated alpha
			break;
		}

		assert( !isinf( nextUpper.x ) );
		upperConstraints.push_back( nextUpper );

		if( lowerConstraints[matchingLowerIndex].y == nextUpper.y )
		{
			matchingLowerIndex++;
		}

		prevUpper = nextUpper;
		prevAlpha = nextAlpha;
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
	const std::vector<const float *> &colorChannels,
	float alphaTolerance, float colorTolerance, float zTolerance, float silhouetteDepth,
	std::vector< std::pair<float,float> > &lowerConstraints,
	std::vector< std::pair<float,float> > &upperConstraints
)
{
	std::vector< SimplePoint > lowerLinear;
	std::vector< SimplePoint > upperLinear;
	linearConstraintsForPixel(
		inSamples, inA, inZ, inZBack, colorChannels,
		alphaTolerance, colorTolerance, zTolerance, silhouetteDepth,
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
		inSamples, inA, inZ, inZBack, colorChannels,
		alphaTolerance, colorTolerance, zTolerance, silhouetteDepth,
		constraintsLower, constraintsUpper
	);

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

	if( debug ) std::cerr << "START CONFORM\n";
	conformToSegments(
		inSamples, inA, inZ, inZBack,
		compressedSamples,
		outSamples, outA, outZ, outZBack
	);
	if( debug ) std::cerr << "END CONFORM\n";
	return;
}

void conformToAlpha( int inSamples, int outSamples, const float *inAlpha, const float *outAlpha, const float *inChannel, float *outChannel )
{
	float totalAccumAlpha = 0;


	float alphaRemaining = 0.0;

	float curChannelData = 0;
	float curChannelAlpha = 0;

	int integrateIndex = 0;
	float targetAlpha = 0;
	for( int i = 0; i < outSamples; ++i )
	{
		targetAlpha += outAlpha[i] - targetAlpha * outAlpha[i];
		if( ! ( targetAlpha > -0.1 && targetAlpha < 1.1 ) )
		{
			std::cerr << "BAD TARGET: " << targetAlpha << "\n";
			targetAlpha = 0.0f;
		}

		float segmentAccumChannel = 0;
		float segmentAccumAlpha = 0;

		for( ; integrateIndex < inSamples; integrateIndex++ )
		{
			if( alphaRemaining == 0.0 )
			{
				curChannelData = inChannel[ integrateIndex ];
				curChannelAlpha = std::min( 1.0f, inAlpha[ integrateIndex ] );
				alphaRemaining = inAlpha[ integrateIndex ];
			}

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
				alphaRemaining = 1 - ( 1 - alphaRemaining ) / ( 1 - alphaNeeded );
			}

			totalAccumAlpha += ( 1 - totalAccumAlpha ) * alphaToTake;
			float curChannelMultiplier = curChannelAlpha > 0 ? ( 1 - segmentAccumAlpha ) * alphaToTake / curChannelAlpha : 0.0; // TODO

			segmentAccumChannel += curChannelMultiplier * curChannelData;
			segmentAccumAlpha += curChannelMultiplier * curChannelAlpha;

			if( alphaRemaining > 0 )
			{
				break;
			}
		}

		/*if( compressedSamples[i].XBack < compressedSamples[i].X )
		{
			std::cerr << "BAD XBACK: " << compressedSamples[i].X << " -> " << compressedSamples[i].XBack << "\n";
		}

		if( ! ( segmentAccumChannels[2] > -0.1 && segmentAccumChannels[2] < 1.1 ) )
		{
			std::cerr << "BAD CALC: " << segmentAccumChannels[2] << "\n";
			segmentAccumChannels[2] = 0.0f;
		}*/

		outChannel[i] = segmentAccumChannel;
	}
}

}

}

