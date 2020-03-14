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

#include <vector>
#include <cmath>
#include <assert.h>
#include <limits> // TODO
#include <iostream> // TODO
#include <iostream> // TODO

using namespace GafferImage::DeepAlgo::Detail;

namespace
{

/// Creates a set of point samples across the range of the pixel's depth for the alpha channel.
/// The deepSamples that are returned are stored in the deepSamples vector, with values for depth and accumulated alpha from front to back

const double maximumLinearY = -log1p( - ( 1. - std::numeric_limits<float>::epsilon() ) );
const double linearOpacityThreshold = -log1p( - 0.9999 );

/// Given a value in linear space, make it exponential.
double linearToExponential( double value )
{
	return value == maximumLinearY ? 1 : -expm1( -value );
}

/// Given a value in exponential space, make it linear.
double exponentialToLinear( double value )
{
	return value <= 0 ? 0 : value >= 1 ? maximumLinearY : -log1p( -value );
}

struct SimplePoint
{
	double x, y;
};

SimplePoint segmentIntersect( SimplePoint a1, SimplePoint b1, SimplePoint a2, SimplePoint b2 )
{
	SimplePoint disp = { a2.x - a1.x, a2.y - a1.y };
	SimplePoint dir1 = { b1.x - a1.x, b1.y - a1.y };
	SimplePoint dir2 = { b2.x - a2.x, b2.y - a2.y };

	double denom = dir1.x * dir2.y - dir1.y * dir2.x;
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

	double t = ( disp.x * dir2.y - disp.y * dir2.x ) / denom;

	return { a1.x + dir1.x * t, a1.y + dir1.y * t };
}


struct LinearSegment
{
	double X; // The depth of the front of the sample.
	double XBack; // The depth at the back of the sample.
	double YBack; // The alpha at the back of the sample.
};


struct ConstraintSearchParams
{
	// Parameters of the most steeply up line which fits constraints
	double a;
	double b;

    // The points which impose the shallowest constraints
    // ie:  if the current line was any steeper, it could not pass over lowerConstraint and under upperConstraint
	//SimplePoint lowerConstraint, upperConstraint;
	int lowerConstraintIndex, upperConstraintIndex;

	ConstraintSearchParams() : a( 0. ), b( 0. ) {};
};

/*
 Update a set of constraint search parameters with a new set of constraints
 ( must be a superset of the previous constraints used to set up the search parameters )

 startIndex and endIndex define the first and last constraints used in the constraints array.
 headIndex and tailIndex are used for special conditions for the beginning and end.
 Constraints between startIndex and headIndex (inclusive) are used just for upper constraints, since their lower constrainst have already been fufilled by a previous segment.
 Constraints between tailIndex and endIndex (inclusive) are used just for lower constraints, since the new segment will be cut when it reaches a certain y value, and won't exceed the upper constraints.

 This function either updates masterSearchParams with a line that fufills all constraints and returns true, or returns false and does not alter masterSearchParams.
*/
bool updateConstraintsSimple( const SimplePoint *lowerConstraints, int lowerStart, int lowerStop, const SimplePoint *upperConstraints, int upperStart, int upperStop, ConstraintSearchParams *masterSearchParams )
{
	ConstraintSearchParams p = *masterSearchParams;

	if( p.a == std::numeric_limits<double>::infinity() )
	{
		if( upperStop > upperStart )
		{
			p.upperConstraintIndex = upperStop - 1;
		}

		if( lowerStop > lowerStart )
		{
			p.lowerConstraintIndex = lowerStart;
		}

		// TODO - currently relying on initialization to infinity
		if( p.upperConstraintIndex != -1 && p.lowerConstraintIndex != -1 && lowerConstraints[ p.lowerConstraintIndex ].x >= upperConstraints[ p.upperConstraintIndex ].x )
		{
			*masterSearchParams = p;
			return true;
		}

		p.a = ( upperConstraints[p.upperConstraintIndex].y - lowerConstraints[p.lowerConstraintIndex].y ) / ( upperConstraints[p.upperConstraintIndex].x - lowerConstraints[p.lowerConstraintIndex].x );
		if( p.a == 0 )
		{
			std::cerr << "X: " << upperConstraints[p.upperConstraintIndex].x << " : " << lowerConstraints[p.lowerConstraintIndex].x << "\n";
			std::cerr << "Y: " << upperConstraints[p.upperConstraintIndex].y << " : " << lowerConstraints[p.lowerConstraintIndex].y << "\n";
			assert( p.a != 0.0f );
		}
		p.b = lowerConstraints[p.lowerConstraintIndex].y - lowerConstraints[p.lowerConstraintIndex].x * p.a;
	}

	// Use a number of max iterations just in case numerical precision gets us stuck
	// ( I've never actually seen this happen ) TODO
	for( int iteration = 0; iteration < lowerStop - lowerStart + upperStop - upperStart; ++iteration )
	{
		bool needRescan = false;

		// Test all constraints to make sure that the current line fufills all of them
		for( int i = lowerStop - 1; i >= lowerStart; i-- )
		{
			double lowerX = lowerConstraints[i].x;
			double minY = lowerConstraints[i].y;

			double yAtLowerX = p.a * lowerX + p.b;

			// Check if we go underneath the minimum constraint at this index
			if( yAtLowerX < minY )
			{
				if( lowerX > upperConstraints[p.upperConstraintIndex].x )
				{
					// If the violated constraint is to the right of the previous upper constraint, then we would need to get steeper to fufill it
					// But the current line is already the steepest that fufills previous constraints, so there can be no line which fufills all constraints.
					// Fail.
					return false;
				}
				else if( i != p.lowerConstraintIndex )
				{
					// Replace the lower constraint with the constraint we violated.  Recalculate the steepest line through these constraints,
					// and trigger a rescan to check our new line against previous constraints
					double newA = (upperConstraints[p.upperConstraintIndex].y - minY) / ( upperConstraints[p.upperConstraintIndex].x - lowerX );
					if( newA < p.a )
					{
						p.a = newA;
						assert( p.a != 0.0f );
						p.b = minY - lowerX * p.a;
						p.lowerConstraintIndex = i;
						needRescan = true;
					}
				}
			}
		}

		// Test all constraints to make sure that the current line fufills all of them
		for( int i = upperStop - 1; i >= upperStart; i-- )
		{

			double upperX = upperConstraints[i].x;
			double maxY = upperConstraints[i].y;

			double yAtUpperX = p.a * upperX + p.b;

			// Check if we go above the maxmimum constraint at this index
			if( yAtUpperX > maxY )
			{
				if( upperX < lowerConstraints[p.lowerConstraintIndex].x )
				{
					// If the violated constraint is to the left of the previous lower constraint, then we would need to get steeper to fufill it
					// But the current line is already the steepest that fufills previous constraints, so there can be no line which fufills all constraints.
					// Fail.
					return false;
				}
				else if( i != p.upperConstraintIndex )
				{
					// Replace the upper constraint with the constraint we violated.  Recalculate the steepest line through these constraints,
					// and trigger a rescan to check our new line against previous constraints
					double newA = (maxY - lowerConstraints[p.lowerConstraintIndex].y) / ( upperX - lowerConstraints[p.lowerConstraintIndex].x );
					if( newA < p.a )
					{
						p.a = newA;
						assert( p.a != 0.0f );
						p.b = maxY - upperX * p.a;
						p.upperConstraintIndex = i;
						needRescan = true;
					}
				}
			}
		}

		if( !needRescan )
		{
			break;
		}
	}

	//std::cerr << "SUCCESS : " << p.a << " , " << p.b << "\n";
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

	double totalAccumAlpha = 0;

	double curA = 0;
	double accumA = 0;

	double alphaRemaining = 0.0;

	int integrateIndex = 0;
	for( unsigned int i = 0; i < compressedSamples.size(); ++i )
	{
		double targetAlpha = -expm1( -compressedSamples[i].YBack );
		if( ! ( targetAlpha > -0.1 && targetAlpha < 1.1 ) )
		{
			std::cerr << "BAD TARGET: " << targetAlpha << "\n";
			targetAlpha = 0.0f;
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

			double alphaNeeded = ( targetAlpha - totalAccumAlpha ) / ( 1 - totalAccumAlpha );

			double alphaToTake;
			if( alphaNeeded >= alphaRemaining )
			{
				assert( alphaRemaining >= 0 );
				alphaToTake = alphaRemaining;
				alphaRemaining = 0;
			}
			else
			{
				assert( alphaNeeded >= 0 );
				alphaToTake = alphaNeeded;
				alphaRemaining = 1 - ( 1 - alphaRemaining ) / ( 1 - alphaNeeded );
			}

			totalAccumAlpha += ( 1 - totalAccumAlpha ) * alphaToTake;
			double curChannelMultiplier = curA > 0 ? ( 1 - accumA ) * alphaToTake / curA : 0.0; // TODO
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

		if( compressedSamples[i].XBack < compressedSamples[i].X )
		{
			std::cerr << "BAD XBACK: " << compressedSamples[i].X << " -> " << compressedSamples[i].XBack << "\n";
		}

		if( ! ( accumA > -0.1 && accumA < 1.1 ) )
		{
			std::cerr << "BAD CALC: " << accumA << "\n";
			accumA = 0.0f;
		}


		outZ[i] = compressedSamples[i].X;
		outZBack[i] = compressedSamples[i].XBack;
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

	double yPrev = 0; // exponentialToLinear( 0 ) == 0

	ConstraintSearchParams prevSearchParams;

	unsigned int lowerStartIndex = 0;
	unsigned int lowerStopIndex = 0;
	unsigned int upperStartIndex = 0;
	unsigned int upperStopIndex = 0;

	if( debug ) std::cerr << "SEGMENT SEARCH START\n";

	while( lowerStopIndex < constraintsLower.size() )
	{
		// Initial constraint search parameters for not yet having found anything
		ConstraintSearchParams currentSearchParams;
		currentSearchParams.a = std::numeric_limits<double>::infinity();
		currentSearchParams.b = 0;

		currentSearchParams.lowerConstraintIndex = -1;
		currentSearchParams.upperConstraintIndex = -1;


		bool advanceUpper = false;
		// Note that the next loop must run at least once, since we wouldn't get in here unless
		// lowerStopIndex starts out valid.  The only reason to initialize advanceUpper is because
		// the compiler isn't smart enough to realize that it reliably gets initialized in the loop
		while( lowerStopIndex < constraintsLower.size() )
		{
			advanceUpper = upperStopIndex < constraintsUpper.size() &&
				constraintsUpper[ upperStopIndex ].y <= constraintsLower[lowerStopIndex].y;
			/*if( debug && upperStopIndex < constraintsUpper.size() )
			{
				std::cerr.precision( 20 );
				std::cerr << "TEST UPPER: " << constraintsUpper[ upperStopIndex ].y << " : " << constraintsLower[lowerStopIndex].y << "\n";
				std::cerr << "ADV UPPER: " << advanceUpper << "\n";
			}*/

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

			if( !( testUpperStopIndex > upperStartIndex && testLowerStopIndex > lowerStartIndex ) )
			{
				upperStopIndex = testUpperStopIndex;
				lowerStopIndex = testLowerStopIndex;
				continue;
			}
			//
			// Try forming a segment that pass over all lower constraints up to searchIndex, and under any upper
			// constraints up to searchIndex which haven't been previously covered

			// We will create a flat top which just reaches the lower constraint at this index
			//double yFinalTrial = constraintsLower[lowerStopIndex].y;

			// Any upper constraints greater than yFinalTrial can be ignored, because we are going to cut before reaching them
			//tailIndex = searchIndex;
			/*while( upperStopIndex < constraintsUpper.size() - 1 && constraintsUpper[ upperStopIndex + 1 ].y < yFinalTrial )
			{
				upperStopIndex++;
			}*/

			// Try fitting a line to our new set of constraints
			//std::cerr << "CONSTRAINT SEARCH: " << tempConstraintLower.size() << " , " << tempConstraintUpper.size() << "\n";
			bool success = updateConstraintsSimple( &constraintsLower[0], lowerStartIndex, testLowerStopIndex, &constraintsUpper[0], upperStartIndex, testUpperStopIndex, &currentSearchParams );

			if( !success )
			{
				// It didn't work, so we'll use the previous value left in currentSearchParams
				break;
			}

			upperStopIndex = testUpperStopIndex;
			lowerStopIndex = testLowerStopIndex;

			// We'll continue looping and try to find a segment that covers more constraints
		}


		
		
		//assert( searchIndex > scanIndex );
		//std::cerr << "Segment " << compressedSamples.size() << " : " << currentSearchParams.lowerConstraint.x << "," << currentSearchParams.lowerConstraint.y << "->" << currentSearchParams.upperConstraint.x << "," << currentSearchParams.upperConstraint.y << "\n";

		double xStart;


		/*if( currentSearchParams.lowerConstraintIndex == -1 )
		{
			std::cerr << "BEFORE"; 
			std::cerr << "UPPER : "; 
			for( unsigned int i = upperStartIndex; i < upperStopIndex; i++ )
			{
				std::cerr << constraintsUpper[i] << " "; 
			}
			std::cerr << "\n"; 
			std::cerr << "LOWER : "; 
			for( unsigned int i = lowerStartIndex; i < lowerStopIndex; i++ )
			{
				std::cerr << constraintsLower[i] << " "; 
			}

			if( advanceUpper ) upperStopIndex--;
			else lowerStopIndex--;

			std::cerr << "NOW"; 
			std::cerr << "UPPER : "; 
			for( unsigned int i = upperStartIndex; i < upperStopIndex; i++ )
			{
				std::cerr << constraintsUpper[i] << " "; 
			}
			std::cerr << "\n"; 
			std::cerr << "LOWER : "; 
			for( unsigned int i = lowerStartIndex; i < lowerStopIndex; i++ )
			{
				std::cerr << constraintsLower[i] << " "; 
			}
			std::cerr << "\n"; 
		}*/
		assert( currentSearchParams.lowerConstraintIndex != -1 );

		lowerStartIndex = lowerStopIndex;

		// TODO - max?
		if( currentSearchParams.upperConstraintIndex != -1 )
		{
			upperStartIndex = currentSearchParams.upperConstraintIndex;
		}


		double yFinal = constraintsLower[lowerStopIndex - 1].y;

		if( lowerStopIndex == constraintsLower.size() )
		{
			// All constraints fulfilled
		}
		else if( advanceUpper )
		{
			// Hit an upper constraint

			// If the previous line goes over the current constraint, then we are going to be underneath it
			// This means that we don't need to worry about upper constraints that it passed under,
			// because if we are heading towards them, we will intersect it
			upperStartIndex = upperStopIndex;

			if( upperStopIndex != constraintsUpper.size() && upperStopIndex >= 2 && currentSearchParams.upperConstraintIndex != -1 )
			{ 
				if( debug ) std::cerr << "CHECKING UPPER:" << constraintsUpper[ upperStopIndex - 1].x << " " << constraintsUpper[ upperStopIndex ].x << "\n";
				if( debug ) std::cerr << "CHECKING UPPER Y:" << linearToExponential( constraintsUpper[ upperStopIndex - 1 ].y ) << " " << linearToExponential( constraintsUpper[ upperStopIndex ].y ) << "\n";
				SimplePoint intersection = segmentIntersect(
					constraintsLower[ currentSearchParams.lowerConstraintIndex ],
					constraintsUpper[ currentSearchParams.upperConstraintIndex ],
					constraintsUpper[ upperStopIndex - 1 ],
					constraintsUpper[ upperStopIndex ]
				);


				double intersectionEpsilon = 1e-10;
				if(
					intersection.x >= constraintsUpper[upperStopIndex - 1].x - intersectionEpsilon &&
					intersection.x <= constraintsUpper[upperStopIndex ].x + intersectionEpsilon &&
					intersection.y >= constraintsUpper[upperStopIndex - 1].y - intersectionEpsilon && // BLEH
					intersection.y <= constraintsUpper[upperStopIndex ].y + intersectionEpsilon
				)
				{
					if(debug) std::cerr << "UPPER : CHANGING Y FROM " << linearToExponential( yFinal ) << " to " << linearToExponential( intersection.y ) << "\n";
					
					upperStartIndex--;
					constraintsUpper[upperStartIndex].x = intersection.x;
					constraintsUpper[upperStartIndex].y = intersection.y;
					
					yFinal = intersection.y;
				}
				else
				{
					std::cerr.precision( 20 );
					std::cerr << "UPPER: BAD CONSTRAINT X : " << intersection.x << " : " << constraintsUpper[upperStopIndex - 1].x << " -> " << constraintsUpper[upperStopIndex].x << "\n";
					std::cerr << "UPPER: BAD CONSTRAINT Y : " << intersection.y << " : " << constraintsUpper[upperStopIndex - 1].y << " -> " << constraintsUpper[upperStopIndex].y << "\n";
					std::cerr << "A : " << ( intersection.x >= constraintsUpper[upperStopIndex - 1].x ) << "\n";
					std::cerr << "B : " << ( intersection.x <= constraintsUpper[upperStopIndex ].x ) << "\n";
					std::cerr << "C : " << ( intersection.y >= constraintsUpper[upperStopIndex - 1].y ) << "\n";
					std::cerr << "D : " << ( intersection.y <= constraintsUpper[upperStopIndex ].y ) << "\n";
				}
			}

			
			if( lowerStopIndex >= 2 && currentSearchParams.upperConstraintIndex != -1 )
			{
				if( yFinal < constraintsLower[ lowerStopIndex - 1].y )
				{
					if( constraintsLower[ lowerStopIndex - 1].y - yFinal > 1e-15 )
					{
						std::cerr << "TERRIBLE: " << constraintsLower[ lowerStopIndex - 1].y - yFinal << "\n"; // TODO
					}
					lowerStartIndex--;
				}
				else if( yFinal < constraintsLower[ lowerStopIndex - 1].y || yFinal > constraintsLower[ lowerStopIndex ].y )
				{
					std::cerr.precision( 20 );
					std::cerr << "LINEAR: " << ( yFinal ) << " : " << ( constraintsLower[ lowerStopIndex - 1 ].y ) << " -> " << ( constraintsLower[ lowerStopIndex].y ) << "\n";
					std::cerr << "BAD HORIZ-to-LOWER TEST: " << linearToExponential( yFinal ) << " : " << linearToExponential( constraintsLower[ lowerStopIndex - 1 ].y ) << " -> " << linearToExponential( constraintsLower[ lowerStopIndex].y ) << "\n";
				}
				else
				{
					lowerStartIndex--;
					constraintsLower[lowerStartIndex].x = ( yFinal - constraintsLower[ lowerStopIndex - 1 ].y ) / ( constraintsLower[ lowerStopIndex ].y - constraintsLower[ lowerStopIndex - 1 ].y ) * ( constraintsLower[ lowerStopIndex ].x - constraintsLower[ lowerStopIndex - 1 ].x ) + constraintsLower[ lowerStopIndex - 1 ].x;
					constraintsLower[lowerStartIndex].y = yFinal;
				}
			}

			
		}
		else
		{
	
			// TODO The line we found passes under the next lower constraint,
			// so we can find a point where it intersects the lower constraint line

			if( lowerStopIndex >= 2 && currentSearchParams.upperConstraintIndex != -1
				//&& constraintsLower[ lowerStopIndex - 1 ].x != constraintsLower[ lowerStopIndex ].x
			 )
			{ 
				if( debug ) std::cerr << "CHECKING LOWER:" << constraintsLower[ lowerStopIndex - 1].x << " " << constraintsLower[ lowerStopIndex ].x << "\n";
				if( debug ) std::cerr << "CHECKING LOWER Y:" << linearToExponential( constraintsLower[ lowerStopIndex - 1 ].y ) << " " << linearToExponential( constraintsLower[ lowerStopIndex ].y ) << "\n";
				SimplePoint intersection = segmentIntersect(
					constraintsLower[ currentSearchParams.lowerConstraintIndex ],
					constraintsUpper[ currentSearchParams.upperConstraintIndex ],
					constraintsLower[ lowerStopIndex - 1 ],
					constraintsLower[ lowerStopIndex ]
				);
				/*SimplePoint curConstraint = constraintsLower[lowerStopIndex];
				SimplePoint prevConstraint = constraintsLower[lowerStopIndex - 1];
				SimplePoint constraintDirection = curConstraint - prevConstraint;
				if( constraintDirection.x != 0 ) // TODO - vertical constraints
				{
					double ca = constraintDirection.y / constraintDirection.x;
					double cb = prevConstraint.y - ca * prevConstraint.x;
					double denom = currentSearchParams.a - ca;
					if( denom != 0.0 )
					{
						double intersectionX = -( currentSearchParams.b - cb ) / denom;
						double intersectionY = currentSearchParams.a * intersectionX + currentSearchParams.b;*/

				if(
					intersection.x >= constraintsLower[lowerStopIndex - 1].x &&
					intersection.x <= constraintsLower[lowerStopIndex ].x &&
					intersection.y >= constraintsLower[lowerStopIndex - 1].y && // BLEH
					intersection.y <= constraintsLower[lowerStopIndex ].y
				)
				{
					if(debug) std::cerr << "CHANGING Y FROM " << linearToExponential( yFinal ) << " to " << linearToExponential( intersection.y ) << "\n";

					lowerStartIndex--;
					constraintsLower[lowerStartIndex].x = intersection.x;
					constraintsLower[lowerStartIndex].y = intersection.y;

					yFinal = intersection.y;
				}
				else
				{
					std::cerr << "BAD CONSTRAINT : " << intersection.x << " : " << constraintsLower[lowerStopIndex - 1].x << " -> " << constraintsLower[lowerStopIndex].x << "\n";
				}
			}
		}

		//std::cerr << "CALC : " << yFinal << " - " << currentSearchParams.b << " / " << currentSearchParams.a << "\n";
		// Calculate where our new line hits its flat top
		double xEnd = ( yFinal - currentSearchParams.b ) / currentSearchParams.a;
		assert( !std::isnan( xEnd ) );

		// TODO
		if( currentSearchParams.lowerConstraintIndex == -1 || fabs( yFinal ) == std::numeric_limits<double>::infinity() )
		{
			std::cerr << "????????NEVER?????????" << "\n";
			// We should never get here but if we do, do something fairly sensible.
			yFinal = constraintsLower.back().y;
			xStart = xEnd = constraintsLower.back().x;
			currentSearchParams.a = std::numeric_limits<double>::infinity();
			lowerStopIndex = constraintsLower.size();
		}
		else if( fabs( currentSearchParams.a ) == std::numeric_limits<double>::infinity() )
		{
			// The line we have found is a point sample.
			xStart = xEnd = constraintsLower[currentSearchParams.lowerConstraintIndex].x;
			// TODO - intersect and update constraints
		}
		else
		{
			

			// Calculate where our new line hits the previous flat top
			xStart = ( yPrev - currentSearchParams.b ) / currentSearchParams.a;

			// If there is a previous segment, we need to cover the possibility that we are overlapping with it
			if( compressedSamples.size() > 0 && xStart < compressedSamples.back().XBack )
			{
				LinearSegment &prevSegment = compressedSamples.back();
				// If the previous sample was a point sample, intersect the new line segment with a vertical line that passes through it.
				if( fabs( prevSearchParams.a ) == std::numeric_limits<double>::infinity() )
				{
					xStart = compressedSamples.back().XBack;
					prevSegment.YBack = prevSegment.XBack * currentSearchParams.a + currentSearchParams.b;
				}
				else
				{
					// Calculate the intersection of our new line with the previous line
					xStart = ( currentSearchParams.b - prevSearchParams.b ) / ( prevSearchParams.a - currentSearchParams.a );
					if( xStart < prevSegment.X )
					{
						// This case may occur due to precision issues.
						xStart = prevSegment.XBack;
					}
					else if( xStart > xEnd )
					{
						// This case may occur due to precision issues.
						xStart = xEnd;
					}
					else
					{
						//std::cerr << "BEFORE : " << prevSegment.XBack << " , " << prevSegment.YBack << "\n";
						// The start of the new segment falls between the start and end of the previous segment.
						// As a result, intersect the two line segments.
						prevSegment.XBack = xStart;
						if( prevSegment.X > prevSegment.XBack )
						{
							std::cerr << "MESSING THINGS UP\n";
						}
						prevSegment.YBack = xStart * prevSearchParams.a + prevSearchParams.b;
						//std::cerr << "AFTER : " << prevSegment.XBack << " , " << prevSegment.YBack << "\n";
					}
				}
			}
		}

		assert( xEnd >= xStart );
		assert( !std::isnan( yFinal ) );

		LinearSegment compressedSample;
		compressedSample.X = xStart;
		compressedSample.XBack = xEnd;
		compressedSample.YBack = yFinal;
		compressedSamples.push_back( compressedSample );

		if( debug )
		{
			std::cerr << "yFinal: " << yFinal << "\n";
			std::cerr << "SEGMENT: " << compressedSample.X << "\t" << compressedSample.XBack << "\t" << linearToExponential( compressedSample.YBack ) << "\n";
		}

		yPrev = yFinal;


		prevSearchParams = currentSearchParams;
	}

	// TODO - adjust XBack too
	/*if( compressedSamples.size() > 1 ) 
	{
		float prevY = compressedSamples[ compressedSamples.size() - 2 ].YBack;
		compressedSamples.back().XBack = ( constraintsUpper.back().y - prevY ) / ( compressedSamples.back().YBack - prevY ) * ( compressedSamples.back().XBack - compressedSamples.back().X ) + compressedSamples.back().X;
	}
	compressedSamples.back().YBack = constraintsUpper.back().y;*/

	//assert( compressedSamples.back().YBack == constraints.back().minY );

	// As per section on Opaque Volume Samples in "Interpreting OpenEXR Deep Pixels", don't write out a large opaque volume sample
	// Instead, write a large almost opaque sample, followed by a tiny point sample to take it up to opaque
	if( compressedSamples.back().YBack > linearOpacityThreshold && compressedSamples.back().XBack != compressedSamples.back().X )
	{
		LinearSegment finalSample;
		finalSample.X = finalSample.XBack = compressedSamples.back().XBack;
		finalSample.YBack = compressedSamples.back().YBack;

		double YPrev = compressedSamples[ compressedSamples.size() - 2].YBack;
		compressedSamples.back().XBack = ( (linearOpacityThreshold - YPrev ) / ( compressedSamples.back().YBack - YPrev ) ) * ( compressedSamples.back().XBack - compressedSamples.back().X ) + compressedSamples.back().X;
		compressedSamples.back().YBack = linearOpacityThreshold;

		compressedSamples.push_back( finalSample );
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
	// Create an additional sample just before the first to force
	// the algorithm to start from the first sample.  // TODO - why is this necessary?
	//double depthOfFirstPoint = inZ[ 0 ] * 0.99999;
	//deepSamples.push_back( SimplePoint( depthOfFirstPoint, 0 ) );

	float ZBackPrev = -1;
	double accumAlpha = 0;
	//double accumAlphaLinear = 0;
	// Now add the remaining samples.
	for ( int i = 0; i < inSamples; ++i )
	{
		double Z = inZ[ i ];
		double ZBack = inZBack[ i ];

		if( Z != ZBackPrev )
		{
			deepSamples.push_back( { Z, accumAlpha } );
		}

		accumAlpha += inA[i] - accumAlpha * inA[i];
		if( accumAlpha >= 1.0f )
		{
			accumAlpha = 1.0f;
		}
		deepSamples.push_back( { ZBack, accumAlpha } );
	
		ZBackPrev = ZBack;	
	}
}

/*
 From a deep pixel, create a list of constraints in linear space that define the path the alpha curve must take to stay within
 the given alpha and z tolerance
*/
void linearConstraintsForPixel(
	const int inSamples, const float *inA, const float *inZ, const float *inZBack,
	double alphaTolerance,
	double zTolerance,
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
	const double curveSampleRatio = 0.76;
	const double minimumCurveSample = 0.01 * alphaTolerance;
	//const double stepToNextCurveSample = -log( 0.76 );
	//const double maxCurveSample = exponentialToLinear( 1 - 0.01 * alphaTolerance );

	// The alpha tolerance that we actually use is padded to take account of the curve error
	double aTol = 0.99 * alphaTolerance;
	
	// TODO - get rid of this alloc by combining with function above
	std::vector< SimplePoint > deepSamples;
	integratedPointSamplesForPixel( inSamples, inA, inZ, inZBack, deepSamples );

	lowerConstraints.reserve( deepSamples.size() ); // TODO
	upperConstraints.reserve( deepSamples.size() );
	//double lastValidMinY = 0;
	double prevAlpha = 0;
	SimplePoint prevLower = { 0, 0 };
	for( unsigned int j = 0; j < deepSamples.size(); ++j )
	{
		double nextX;
		double nextAlpha;

		if( j == deepSamples.size() - 1 )
		{
			// Ensure that we always reach the exact final value
			nextX = deepSamples[j].x * ( 1 + zTolerance );
			nextAlpha = deepSamples[j].y;
		}
		else
		{
			double minAlpha = deepSamples[j].y - aTol;
			if( minAlpha >= 0 )
			{
				/*double min = 0.0f;
				if( minAlpha < 1 )
				{
					min = -log1p( -minAlpha );
					lastValidMinY = min;
				}
				else
				{
					min = lastValidMinY; // TODO - why?
				}*/

				nextX = deepSamples[j].x * ( 1 + zTolerance );
				nextAlpha = std::min( minAlpha, maximumLinearY );
			}
			else
			{
				double nextMinAlpha = deepSamples[j+1].y - aTol;
				if( nextMinAlpha > 0 )
				{
					double min = -log1p( -minAlpha );
					double nextMin = -log1p( -nextMinAlpha );
					double lerp = -min / ( nextMin - min );
					double xIntercept = deepSamples[j].x + ( deepSamples[j+1].x - deepSamples[j].x ) * lerp;
					nextX = xIntercept * ( 1 + zTolerance );
					nextAlpha = 0.0;
				}
				else
				{
					continue;
				}
			}
		}

		SimplePoint nextLower = { nextX, exponentialToLinear( nextAlpha ) };

		double nextCurveSampleAlpha = ( prevAlpha < 1 - aTol - minimumCurveSample ) ? 1 - aTol - curveSampleRatio * ( 1 - aTol - prevAlpha ) : 1.0;
		double nextCurveSample = nextCurveSampleAlpha != 1.0 ? exponentialToLinear( nextCurveSampleAlpha ) : std::numeric_limits<double>::infinity();

		//double nextCurveSample = prevLower.y < maxCurveSample ? prevLower.y + stepToNextCurveSample : std::numeric_limits<double>::infinity();
		while( nextCurveSample < nextLower.y && prevLower.x != nextLower.x)
		{
			double intersectionX =
				( exponentialToLinear( linearToExponential( nextCurveSample ) + aTol ) - exponentialToLinear( deepSamples[j - 1].y ) ) /
				( exponentialToLinear(deepSamples[j].y) - exponentialToLinear(deepSamples[j - 1].y ) ) *
                ( deepSamples[j].x - deepSamples[j - 1].x ) * ( 1 + zTolerance ) + deepSamples[j - 1].x * ( 1 + zTolerance );
		
			if( intersectionX > nextLower.x )
			{
				break;
			}

			lowerConstraints.push_back( { intersectionX, nextCurveSample } );

			//nextCurveSample = nextCurveSample < maxCurveSample ? nextCurveSample + stepToNextCurveSample : std::numeric_limits<double>::infinity();
			nextCurveSampleAlpha = ( nextCurveSampleAlpha < 1 - aTol - minimumCurveSample ) ? 1 - aTol - curveSampleRatio * ( 1 - aTol - nextCurveSampleAlpha ) : 1.0;
			nextCurveSample = nextCurveSampleAlpha != 1.0 ? exponentialToLinear( nextCurveSampleAlpha ) : std::numeric_limits<double>::infinity();
		}

		lowerConstraints.push_back( nextLower );
		prevLower = nextLower;
		prevAlpha = nextAlpha;
		
	}

	unsigned int matchingLowerIndex = 0;

	assert( deepSamples.size() >= 1 );
	assert( deepSamples[0].y == 0  );

	SimplePoint prevUpper = { deepSamples[0].x * ( 1 - zTolerance ), 0 };
	prevAlpha = 0;
	upperConstraints.push_back( prevUpper );
	
	for( unsigned int j = 1; j < deepSamples.size(); ++j )
	{
		double nextAlpha = deepSamples[j].y + aTol;

		// TODO - comment
		SimplePoint nextUpper = {
			deepSamples[j].x * ( 1 - zTolerance ),
			nextAlpha < 1 ? -log1p( -nextAlpha ) : maximumLinearY
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
			double nextCurveSampleAlpha = ( prevAlpha < 1 - minimumCurveSample ) ? 1 - curveSampleRatio * ( 1 - prevAlpha ) : 1.0;
			double nextCurveSample = nextCurveSampleAlpha != 1.0 ? exponentialToLinear( nextCurveSampleAlpha ) : std::numeric_limits<double>::infinity();
			while(
				matchingLowerIndex < lowerConstraints.size() &&
				( lowerConstraints[matchingLowerIndex].y < nextUpper.y || nextCurveSample < nextUpper.y )
			)
			{
				double yValueToInsert;
				if( lowerConstraints[matchingLowerIndex].y < nextCurveSample )
				{
					yValueToInsert = lowerConstraints[matchingLowerIndex].y;
					matchingLowerIndex++;
				}
				else
				{
					yValueToInsert = nextCurveSample;
					//nextCurveSample = nextCurveSample < maxCurveSample ? nextCurveSample + stepToNextCurveSample : std::numeric_limits<double>::infinity();
					nextCurveSampleAlpha = ( nextCurveSampleAlpha < 1 - minimumCurveSample ) ? 1 - curveSampleRatio * ( 1 - nextCurveSampleAlpha ) : 1.0;
					nextCurveSample = nextCurveSampleAlpha != 1.0 ? exponentialToLinear( nextCurveSampleAlpha ) : std::numeric_limits<double>::infinity();
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
    double alphaTolerance,
    double zTolerance,
    std::vector< DeepConstraint > &lowerConstraints,
    std::vector< DeepConstraint > &upperConstraints
)
{
	std::vector< SimplePoint > lowerLinear;
	std::vector< SimplePoint > upperLinear;
	linearConstraintsForPixel(
		inSamples, inA, inZ, inZBack, alphaTolerance, zTolerance,
		lowerLinear, upperLinear
	);

	lowerConstraints.clear();
	upperConstraints.clear();
	lowerConstraints.reserve( lowerLinear.size() );
	upperConstraints.reserve( upperLinear.size() );
	for( SimplePoint &c : lowerLinear )
	{
		lowerConstraints.push_back( { c.x, linearToExponential( c.y ) } );
	}
	for( SimplePoint &c : upperLinear )
	{
		upperConstraints.push_back( { c.x, linearToExponential( c.y ) } );
	}
}

}

// TODO - some way to specify tolerances in other channels
// TODO : Shouldn't really be any need for anything to be in double precision here?
void resampleDeepPixel(
	const int inSamples, const float *inA, const float *inZ, const float *inZBack,
	double alphaTolerance, double zTolerance,
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
		alphaTolerance, zTolerance,
		constraintsLower, constraintsUpper
	);

	std::vector< LinearSegment > compressedSamples;
	minimalSegmentsForConstraints( constraintsLower, constraintsUpper, compressedSamples, debug );
	conformToSegments(
		inSamples, inA, inZ, inZBack,
		compressedSamples,
		outSamples, outA, outZ, outZBack
	);
	return;
}

void conformToAlpha( int inSamples, int outSamples, const float *inAlpha, const float *outAlpha, const float *inChannel, float *outChannel )
{
	double totalAccumAlpha = 0;


	double alphaRemaining = 0.0;

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

		for( ; integrateIndex < inSamples; integrateIndex++  )
		{
			if( alphaRemaining == 0.0 )
			{
				curChannelData = inChannel[ integrateIndex ];
				curChannelAlpha = std::min( 1.0f, inAlpha[ integrateIndex ] );
				alphaRemaining = inAlpha[ integrateIndex ];
			}

			double alphaNeeded = totalAccumAlpha < 1.0f ? ( targetAlpha - totalAccumAlpha ) / ( 1 - totalAccumAlpha ) : 0.0f;

			double alphaToTake;
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
			double curChannelMultiplier = curChannelAlpha > 0 ? ( 1 - segmentAccumAlpha ) * alphaToTake / curChannelAlpha : 0.0; // TODO

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

