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

#include "Gaffer/Context.h"

#include "GafferImage/ImageAlgo.h"
#include "GafferImage/DeepResample.h"

// TODO
#include <csignal>


using namespace std;
using namespace Imath;
using namespace IECore;
using namespace Gaffer;
using namespace GafferImage;

namespace
{

/// Creates a set of point samples across the range of the pixel's depth for the alpha channel.
/// The deepSamples that are returned are stored in the deepSamples vector, with values for depth and accumulated alpha from front to back

const double maximumLinearY = -log1p( - ( 1. - std::numeric_limits<double>::epsilon() ) );
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

V2d segmentIntersect( V2d a1, V2d b1, V2d a2, V2d b2 )
{
	V2d disp = a2 - a1;
	V2d dir1 = b1 - a1;
	V2d dir2 = b2 - a2;

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

	return a1 + dir1 * t;
}


struct LinearSegment
{
	double X; // The depth of the front of the sample.
	double XBack; // The depth at the back of the sample.
	double YBack; // The alpha at the back of the sample.
};

struct SimplePoint2D
{
	double x, y;
};

struct ConstraintSearchParams
{
	// Parameters of the most steeply up line which fits constraints
	double a;
	double b;

    // The points which impose the shallowest constraints
    // ie:  if the current line was any steeper, it could not pass over lowerConstraint and under upperConstraint
	//V2d lowerConstraint, upperConstraint;
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
bool updateConstraintsSimple( const V2d *lowerConstraints, int lowerStart, int lowerStop, const V2d *upperConstraints, int upperStart, int upperStop, ConstraintSearchParams *masterSearchParams )
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
					//std::cerr << "REPLACE\n";
					if( upperX != lowerConstraints[p.lowerConstraintIndex].x )
					{
						double newA = (maxY - lowerConstraints[p.lowerConstraintIndex].y) / ( upperX - lowerConstraints[p.lowerConstraintIndex].x );
						if( newA < p.a )
						{
							p.a = newA;
							assert( p.a != 0.0f );
							p.b = maxY - upperX * p.a;
						}
					}
					p.upperConstraintIndex = i;
					needRescan = true;
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

/*
 Given a set of constraints in linear space, put togther a set of segments that pass through all of them
 */
// TODO - note that this overwrites the constraints
void minimalSegmentsForConstraints( 
	std::vector<V2d> &constraintsLower, std::vector<V2d> &constraintsUpper,
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

		double yFinal = 0;

		bool advanceUpper = false;
		// Note that the next loop must run at least once, since we wouldn't get in here unless
		// lowerStopIndex starts out valid.  The only reason to initialize advanceUpper is because
		// the compiler isn't smart enough to realize that it reliably gets initialized in the loop
		while( lowerStopIndex < constraintsLower.size() )
		{
			advanceUpper = upperStopIndex < constraintsUpper.size() &&
				constraintsUpper[ upperStopIndex ].y < constraintsLower[lowerStopIndex].y;

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

			// It worked, we've got a new segment
			if( lowerStopIndex > 0 )
			{
				yFinal = constraintsLower[lowerStopIndex - 1].y;
			}

			// We'll continue looping and try to find a segment that covers more constraints
		}

		
		
		//assert( searchIndex > scanIndex );
		//std::cerr << "Segment " << compressedSamples.size() << " : " << currentSearchParams.lowerConstraint.x << "," << currentSearchParams.lowerConstraint.y << "->" << currentSearchParams.upperConstraint.x << "," << currentSearchParams.upperConstraint.y << "\n";

		double xStart;


		if( currentSearchParams.lowerConstraintIndex == -1 )
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
		}
		assert( currentSearchParams.lowerConstraintIndex != -1 );

		lowerStartIndex = lowerStopIndex;

		// TODO - max?
		if( currentSearchParams.upperConstraintIndex != -1 )
		{
			upperStartIndex = currentSearchParams.upperConstraintIndex;
		}


		if( lowerStopIndex == constraintsLower.size() )
		{
			// All constraints fulfilled
		}
		else if( !advanceUpper )
		{
	
			// The line we found passes under the next lower constraint,
			// so we can find a point where it intersects the lower constraint line
		
			if( lowerStopIndex >= 2 && currentSearchParams.upperConstraintIndex != -1
				//&& constraintsLower[ lowerStopIndex - 1 ].x != constraintsLower[ lowerStopIndex ].x
			 )
			{ 
				if( debug ) std::cerr << "CHECKING LOWER:" << constraintsLower[ lowerStopIndex - 1].x << " " << constraintsLower[ lowerStopIndex ].x << "\n";
				if( debug ) std::cerr << "CHECKING LOWER Y:" << linearToExponential( constraintsLower[ lowerStopIndex - 1 ].y ) << " " << linearToExponential( constraintsLower[ lowerStopIndex ].y ) << "\n";
				V2d intersection = segmentIntersect(
					constraintsLower[ currentSearchParams.lowerConstraintIndex ],
					constraintsUpper[ currentSearchParams.upperConstraintIndex ],
					constraintsLower[ lowerStopIndex - 1 ],
					constraintsLower[ lowerStopIndex ]
				);
				/*V2d curConstraint = constraintsLower[lowerStopIndex];
				V2d prevConstraint = constraintsLower[lowerStopIndex - 1];
				V2d constraintDirection = curConstraint - prevConstraint;
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
					
					lowerStartIndex = lowerStopIndex - 1;
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
		else
		{
			// We didn't violate a lower constraint, but the search stopped for some reason
			// before reaching the end, so we can conclude we hit an upper constraint


			
			// If the previous line goes over the current constraint, then we are going to be underneath it
			// This means that we don't need to worry about upper constraints that it passed under,
			// because if we are heading towards them, we will intersect it
			// TODO TODO TODO
			while( upperStartIndex + 1 < constraintsUpper.size() && constraintsUpper[upperStartIndex + 1].y < yFinal )
			{
				upperStartIndex++;
			}
		}

		//std::cerr << "CALC : " << yFinal << " - " << currentSearchParams.b << " / " << currentSearchParams.a << "\n";
		// Calculate where our new line hits its flat top
		double xEnd = ( yFinal - currentSearchParams.b ) / currentSearchParams.a;
		assert( !std::isnan( xEnd ) );

		// TODO
		if( currentSearchParams.lowerConstraintIndex == -1 || fabs( yFinal ) == std::numeric_limits<double>::infinity() )
		{
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
	std::vector<V2d> &deepSamples
)
{
	// Create an additional sample just before the first to force
	// the algorithm to start from the first sample.  // TODO - why is this necessary?
	//double depthOfFirstPoint = inZ[ 0 ] * 0.99999;
	//deepSamples.push_back( V2d( depthOfFirstPoint, 0 ) );

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
			deepSamples.push_back( V2d( Z, accumAlpha ) );
		}

		accumAlpha += inA[i] - accumAlpha * inA[i];
		if( accumAlpha >= 1.0f )
		{
			accumAlpha = 1.0f;
		}
		deepSamples.push_back( V2d( ZBack, accumAlpha ) );
	
		ZBackPrev = ZBack;	
	}
}

/*
 From a deep pixel, create a list of constraints in linear space that define the path the alpha curve must take to stay within
 the given alpha and z tolerance
*/
void constraintSamplesForPixel(
	const int inSamples, const float *inA, const float *inZ, const float *inZBack,
	double alphaTolerance,
	double zTolerance,
	std::vector< V2d > &lowerConstraints,
	std::vector< V2d > &upperConstraints
)
{
	// TODO - get rid of this alloc by combining with function above
	std::vector< V2d > deepSamples;
	integratedPointSamplesForPixel( inSamples, inA, inZ, inZBack, deepSamples );

	lowerConstraints.reserve( deepSamples.size() );
	upperConstraints.reserve( deepSamples.size() );
	double lastValidMinY = 0;
	for( unsigned int j = 0; j < deepSamples.size(); ++j )
	{
		if( j == deepSamples.size() - 1 )
		{
			// Ensure that we always reach the exact final value
			lowerConstraints.push_back( V2d( deepSamples[j].x * ( 1 + zTolerance ), exponentialToLinear( deepSamples[j].y ) ) );
		}
		else
		{
			double minAlpha = deepSamples[j].y - alphaTolerance;
			if( minAlpha >= 0 )
			{
				double min = 0.0f;
				if( minAlpha < 1 )
				{
					min = -log1p( -minAlpha );
					lastValidMinY = min;
				}
				else
				{
					min = lastValidMinY;
				}
				lowerConstraints.push_back( V2d( deepSamples[j].x * ( 1 + zTolerance ), min ) );
			}
			else
			{
				float nextMinAlpha = deepSamples[j+1].y - alphaTolerance;
				if( nextMinAlpha > 0 )
				{
					float min = -log1p( -minAlpha );
					float nextMin = -log1p( -nextMinAlpha );
					float lerp = -min / ( nextMin - min );
					float xIntercept = deepSamples[j].x + ( deepSamples[j+1].x - deepSamples[j].x ) * lerp;
					lowerConstraints.push_back( V2d( xIntercept * ( 1 + zTolerance ), 0 ) );
				}
			}
		}

		double maxAlpha = deepSamples[j].y + ( j == 0 ? 0 : alphaTolerance );
		double max = maxAlpha < 1 ? -log1p( -maxAlpha ) : maximumLinearY;
		upperConstraints.push_back( V2d( deepSamples[j].x * ( 1 - zTolerance ), max ) );
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
	std::vector<V2d> constraintsLower;
	std::vector<V2d> constraintsUpper;
	constraintSamplesForPixel(
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

}

GAFFER_GRAPHCOMPONENT_DEFINE_TYPE( DeepResample );

size_t DeepResample::g_firstPlugIndex = 0;

DeepResample::DeepResample( const std::string &name )
	:	ImageProcessor( name )
{
	storeIndexOfNextChild( g_firstPlugIndex );

	addChild( new FloatPlug( "alphaTolerance", Gaffer::Plug::In, 0.01 ) );
	addChild( new FloatPlug( "depthTolerance", Gaffer::Plug::In, 0.01 ) );

	addChild( new CompoundObjectPlug( "__resampled", Gaffer::Plug::Out, new IECore::CompoundObject() ) );

	// We don't ever want to change these, so we make pass-through connections.


	// TODO - we should force ZBack into the channelNames if only Z is present
	outPlug()->channelNamesPlug()->setInput( inPlug()->channelNamesPlug() );

	outPlug()->dataWindowPlug()->setInput( inPlug()->dataWindowPlug() );
	outPlug()->formatPlug()->setInput( inPlug()->formatPlug() );
	outPlug()->metadataPlug()->setInput( inPlug()->metadataPlug() );
	outPlug()->deepPlug()->setInput( inPlug()->deepPlug() );
}

DeepResample::~DeepResample()
{
}

Gaffer::FloatPlug *DeepResample::alphaTolerancePlug()
{
	return getChild<FloatPlug>( g_firstPlugIndex + 0 );
}

const Gaffer::FloatPlug *DeepResample::alphaTolerancePlug() const
{
	return getChild<FloatPlug>( g_firstPlugIndex + 0 );
}

Gaffer::FloatPlug *DeepResample::depthTolerancePlug()
{
	return getChild<FloatPlug>( g_firstPlugIndex + 1 );
}

const Gaffer::FloatPlug *DeepResample::depthTolerancePlug() const
{
	return getChild<FloatPlug>( g_firstPlugIndex + 1 );
}

Gaffer::CompoundObjectPlug *DeepResample::resampledPlug()
{
	return getChild<CompoundObjectPlug>( g_firstPlugIndex + 2 );
}

const Gaffer::CompoundObjectPlug *DeepResample::resampledPlug() const
{
	return getChild<CompoundObjectPlug>( g_firstPlugIndex + 2 );
}

void DeepResample::affects( const Gaffer::Plug *input, AffectedPlugsContainer &outputs ) const
{
	ImageProcessor::affects( input, outputs );

	if(
		input == inPlug()->sampleOffsetsPlug() || input == inPlug()->channelNamesPlug() ||
		input == inPlug()->channelDataPlug() ||
		input == alphaTolerancePlug() || input == depthTolerancePlug()
	)
	{
		outputs.push_back( resampledPlug() );
	}

	if( input == resampledPlug() || input == inPlug()->channelDataPlug() )
	{
		outputs.push_back( outPlug()->channelDataPlug() );
	}

	if( input == resampledPlug() || input == inPlug()->sampleOffsetsPlug() )
	{
		outputs.push_back( outPlug()->sampleOffsetsPlug() );
	}
}

void DeepResample::hash( const Gaffer::ValuePlug *output, const Gaffer::Context *context, IECore::MurmurHash &h ) const
{
	ImageProcessor::hash( output, context, h );

	if( output != resampledPlug() )
	{
		return;
	}


	alphaTolerancePlug()->hash( h );
	depthTolerancePlug()->hash( h );

	ConstStringVectorDataPtr channelNamesData = inPlug()->channelNames();
	std::vector<string> channelNames = channelNamesData->readable();

	inPlug()->sampleOffsetsPlug()->hash( h );

	if(
		ImageAlgo::channelExists( channelNames, "A" ) &&
		ImageAlgo::channelExists( channelNames, "Z" ) 
	)
	{
		ImagePlug::ChannelDataScope channelScope( context );
		channelScope.setChannelName( "A" );
		inPlug()->channelDataPlug()->hash( h );

		channelScope.setChannelName( "Z" );
		inPlug()->channelDataPlug()->hash( h );

		if( ImageAlgo::channelExists( channelNames, "ZBack" ) )
		{
			inPlug()->channelDataPlug()->hash( h );
		}
	}
}

void DeepResample::compute( Gaffer::ValuePlug *output, const Gaffer::Context *context ) const
{
	ImageProcessor::compute( output, context );

	if( output != resampledPlug() )
	{
		return;
	}

	// TODO
	const V2i tileOrigin = context->get<V2i>( ImagePlug::tileOriginContextName );

	float alphaTolerance = alphaTolerancePlug()->getValue();
	float depthTolerance = depthTolerancePlug()->getValue();

	ConstIntVectorDataPtr sampleOffsetsData = inPlug()->sampleOffsetsPlug()->getValue();
	const std::vector<int> &sampleOffsets = sampleOffsetsData->readable();

	CompoundObjectPtr resampledData = new CompoundObject();

	ConstStringVectorDataPtr channelNamesData = inPlug()->channelNames();
	std::vector<string> channelNames = channelNamesData->readable();

	if(
		ImageAlgo::channelExists( channelNames, "A" ) &&
		ImageAlgo::channelExists( channelNames, "Z" )
	)
	{
		ImagePlug::ChannelDataScope channelScope( context );
		channelScope.setChannelName( "A" );
		ConstFloatVectorDataPtr alphaData = inPlug()->channelDataPlug()->getValue();
		const std::vector<float> &alpha = alphaData->readable();

		channelScope.setChannelName( "Z" );
		ConstFloatVectorDataPtr zData = inPlug()->channelDataPlug()->getValue();
		const std::vector<float> &z = zData->readable();

		ConstFloatVectorDataPtr zBackData;
		if( ImageAlgo::channelExists( channelNames, "ZBack" ) )
		{
			channelScope.setChannelName( "ZBack" );
			zBackData = inPlug()->channelDataPlug()->getValue();
		}
		else
		{
			zBackData = zData;
		}
		const std::vector<float> &zBack = zBackData->readable();

		IntVectorDataPtr outSampleOffsetsData = new IntVectorData();
		std::vector<int> &outSampleOffsets = outSampleOffsetsData->writable();
		outSampleOffsets.resize( sampleOffsets.size() );


		FloatVectorDataPtr outAlphaData = new FloatVectorData();
		std::vector<float> &outAlpha = outAlphaData->writable();
		FloatVectorDataPtr outZData = new FloatVectorData();
		std::vector<float> &outZ = outZData->writable();
		FloatVectorDataPtr outZBackData = new FloatVectorData();
		std::vector<float> &outZBack = outZBackData->writable();

		// Start with the input size, resize to fit afterwards
		// TODO - this would cause a crash if resampling increased the sample count
		// Mathematically, I'm reasonably confident this is impossible, but I should
		// have a proper think about whether this could ever happen with floating point
		// error and such
		outAlpha.resize( alpha.size() );
		outZ.resize( alpha.size() );
		outZBack.resize( alpha.size() );

		//std::vector<DeepPixel> outputPixels;
		//outputPixels.resize( sampleOffsets.size() );
		int prev = 0;
		int outputCount = 0;
		for( unsigned int i = 0; i < sampleOffsets.size(); i++ )
		{
			int index = sampleOffsets[i];

			/*DeepPixel before( 0, index - prev );
			for( int j = prev; j < index; j++ )
			{
				if( !( alpha[j] >= -0.1 && alpha[j] <= 1.1  ) )
				{
					std::cerr << "BAD IN : " << alpha[j] << "\n";
					before.addSample( z[j], zBack[j], 0.0f, nullptr );
				}
				else
				{
					before.addSample( z[j], zBack[j], alpha[j], nullptr );
				}
			}*/
			int ly = i / ImagePlug::tileSize();
			V2i pixelLocation = tileOrigin + V2i( i - ly * ImagePlug::tileSize(), ly );
			//std::cerr << "P : " << tileOrigin.x + i - ( ly * ImagePlug::tileSize() ) << " , " << tileOrigin.y + ly << "\n"; 
			bool debug = pixelLocation == V2i( 93, 65 );
			int resampledCount;
			resampleDeepPixel(
				index - prev, &alpha[prev], &z[prev], &zBack[prev],
				alphaTolerance, depthTolerance,
				resampledCount, &outAlpha[outputCount], &outZ[outputCount], &outZBack[outputCount],
				debug
			);
			outputCount += resampledCount;
			outSampleOffsets[i] = outputCount;

			prev = index;
		}

		// TODO	- this is too late to reliably avoid crash
		if( outSampleOffsets.back() > ((int)alpha.size()) )
		{
			throw IECore::Exception( "Sample count increased during resampling" );
		}

		outAlpha.resize( outSampleOffsets.back() );
		outZ.resize( outSampleOffsets.back() );
		outZBack.resize( outSampleOffsets.back() );
		outAlpha.shrink_to_fit();
		outZ.shrink_to_fit();
		outZBack.shrink_to_fit();

		resampledData->members()["sampleOffsets"] = outSampleOffsetsData;
		resampledData->members()["A"] = outAlphaData;
		resampledData->members()["Z"] = outZData;
		resampledData->members()["ZBack"] = outZBackData;
	}

	static_cast<CompoundObjectPlug *>( output )->setValue( resampledData );
}

void DeepResample::hashChannelData( const ImagePlug *output, const Gaffer::Context *context, IECore::MurmurHash &h ) const
{
	ImageProcessor::hashChannelData( output, context, h );
	const std::string &channelName = context->get<std::string>( GafferImage::ImagePlug::channelNameContextName );


	inPlug()->channelDataPlug()->hash( h );

	ImagePlug::ChannelDataScope channelScope( context );
	channelScope.remove( ImagePlug::channelNameContextName );

	resampledPlug()->hash( h );

	if( channelName == "Z" || channelName ==  "ZBack" || channelName == "A" )
	{
		h.append( channelName );
		return;
	}

	inPlug()->sampleOffsetsPlug()->hash( h );

	channelScope.setChannelName( "A" );
	inPlug()->channelDataPlug()->hash( h );

	channelScope.setChannelName( channelName );
	inPlug()->channelDataPlug()->hash( h );
}

IECore::ConstFloatVectorDataPtr DeepResample::computeChannelData( const std::string &channelName, const Imath::V2i &tileOrigin, const Gaffer::Context *context, const ImagePlug *parent ) const
{
	ConstFloatVectorDataPtr inChannelData = inPlug()->channelDataPlug()->getValue();

	ImagePlug::ChannelDataScope channelScope( context );
	channelScope.remove( ImagePlug::channelNameContextName );

	ConstCompoundObjectPtr resampled = resampledPlug()->getValue();

	if( !resampled->members().size() )
	{
		return inChannelData;
	}

	if( channelName == "Z" )
	{
		return resampled->member<FloatVectorData>("Z");
	}
	else if( channelName == "ZBack" )
	{
		return resampled->member<FloatVectorData>("ZBack");
	}
	else if( channelName == "A" )
	{
		return resampled->member<FloatVectorData>("A");
	}

	IECore::ConstFloatVectorDataPtr resampledAlphaData = resampled->member<FloatVectorData>("A");
	IECore::ConstIntVectorDataPtr resampledOffsetsData = resampled->member<IntVectorData>("sampleOffsets");
	IECore::ConstIntVectorDataPtr origOffsetsData = inPlug()->sampleOffsetsPlug()->getValue();

	channelScope.setChannelName( "A" );
	IECore::ConstFloatVectorDataPtr origAlphaData = inPlug()->channelDataPlug()->getValue();

	channelScope.setChannelName( channelName );
	IECore::ConstFloatVectorDataPtr origChannelData = inPlug()->channelDataPlug()->getValue();

	const std::vector<int> &origOffsets = origOffsetsData->readable();
	const std::vector<int> &resampledOffsets = resampledOffsetsData->readable();
	const std::vector<float> &origAlpha = origAlphaData->readable();
	const std::vector<float> &resampledAlpha = resampledAlphaData->readable();

	const std::vector<float> &origChannel = origChannelData->readable();

	IECore::FloatVectorDataPtr resultData = new IECore::FloatVectorData();
	std::vector<float> &result = resultData->writable();
	result.resize( resampledAlpha.size() );

	/*
	int pixel = 0;
	float resampledAccumAlpha = 0;
	int origIndex = 0;
	float origAccumAlpha = 0;
	*/

	int prevResampledOffset = 0;
	int prevOrigOffset = 0;
	
	for( unsigned int i = 0; i < resampledOffsets.size(); i++ )
	{
		int resampledOffset = resampledOffsets[i];
		int origOffset = origOffsets[i];
		
		conformToAlpha(
			origOffset - prevOrigOffset, resampledOffset - prevResampledOffset,
			&origAlpha[prevOrigOffset], &resampledAlpha[prevResampledOffset],
			&origChannel[prevOrigOffset], &result[prevResampledOffset]
		);
	
		prevResampledOffset = resampledOffset;	
		prevOrigOffset = origOffset;	

		/*
		resampledAccumAlpha += resampledAlpha[i] - resampledAccumAlpha * resampledAlpha[i];

		float 
		while( true )
		{
			float newOrigAccumAlpha = origAccumAlpha + origAlpha[origIndex] - origAccumAlpha * origAlpha[origIndex];
		
			if( newOrigAccumAlpha > resampledAccumAlpha )
			{
				break;
			}

			origAccumAlpha = newOrigAccumAlpha;
			origIndex++;
		}

		
		
		while( pixel < ImagePlug::tilePixels() && i == resampledOffsets[ pixel ] )
		{
			pixel++;
			resampledAccumAlpha = 0;
		}
		*/
	}

	return resultData;
}

void DeepResample::hashSampleOffsets( const ImagePlug *parent, const Gaffer::Context *context, IECore::MurmurHash &h ) const
{
	ImageProcessor::hashSampleOffsets( parent, context, h );

	resampledPlug()->hash( h );
	inPlug()->sampleOffsetsPlug()->hash( h );
}

IECore::ConstIntVectorDataPtr DeepResample::computeSampleOffsets( const Imath::V2i &tileOrigin, const Gaffer::Context *context, const ImagePlug *parent ) const
{
	ConstCompoundObjectPtr resampled = resampledPlug()->getValue();

	if( !resampled->members().size() )
	{
		return inPlug()->sampleOffsetsPlug()->getValue();
	}

	return resampled->member<IntVectorData>("sampleOffsets");
}
