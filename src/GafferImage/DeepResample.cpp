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

struct ToleranceConstraint
{
	double lowerX;
	double minY;
	double upperX;
	double maxY;
};

struct DepthPointSample
{
	DepthPointSample( double d, double a ) : depth(d), alpha(a) {}

	double depth;
	double alpha;
};

/// Defines how many samples the compression algorithm takes along the length of a volume sample.
int numberOfSplitsPerSample();

/// Creates a set of point samples across the range of the pixel's depth for the alpha channel.
/// The deepSamples that are returned are stored in the deepSamples vector, with values for depth and accumulated alpha from front to back
/// The splits parameter defines how many point samples each volume sample is split into.

/// Given a value in exponential space, make it linear.
double exponentialToLinear( double value );

/// Given a value in linear space, make it exponential.
double linearToExponential( double value );

const double maximumLinearY = -log1p( - ( 1. - std::numeric_limits<double>::epsilon() ) );
const double linearOpacityThreshold = -log1p( - 0.9999 );

double linearToExponential( double value )
{
	return value == maximumLinearY ? 1 : -expm1( -value );
}

double exponentialToLinear( double value )
{
	return value <= 0 ? 0 : value >= 1 ? maximumLinearY : -log1p( -value );
}

int numberOfSplitsPerSample()
{
	return 1;
}

/*
 Given a deep pixel which may contain a mixture of point and volume samples, produce a set of samples which represent the
 alpha falloff curve, where each sample stores a depth and the integrated alpha up to that point.

 Note that currently volume samples are just split into a fixed number of point samples.  A better approach would be
 to split enough times that the alpha increment for each sample is less than the alpha threshold.
*/
void integratedPointSamplesForPixel(
	const int inSamples, const float *inA, const float *inZ, const float *inZBack,
	std::vector<DepthPointSample> &deepSamples
)
{
	const unsigned int split = numberOfSplitsPerSample();

	// Create an additional sample just before the first to force
	// the algorithm to start from the first sample.  // TODO - why is this necessary?
	double depthOfFirstPoint = inZ[ 0 ] * 0.99999;
	deepSamples.push_back( DepthPointSample( depthOfFirstPoint, 0 ) );

	// Now add the remaining samples.
	for ( int i = 0; i < inSamples; ++i )
	{
		double Z = inZ[ i ];
		double ZBack = inZBack[ i ];

		if( Z == ZBack ) // If this is a point sample.
		{
			deepSamples.push_back( DepthPointSample( Z, inA[ i ] ) );
		}
		else
		{
			double alpha = std::min( 1.0f - 1e-6f, inA[ i ] ); // TODO
			double splitAlpha = -expm1( 1.0 / double( split ) * log1p( -alpha ) );

			for( unsigned int k = 0; k < split; ++k )
			{
				deepSamples.push_back( DepthPointSample( Z + ( ZBack - Z ) * ( double( k + 1. ) / double( split ) ), splitAlpha ) );
			}
		}
	}

	double accumAlpha = 0;
	for ( unsigned int i = 0; i < deepSamples.size(); ++i )
	{
		accumAlpha += ( 1 - accumAlpha ) * deepSamples[i].alpha;
		deepSamples[i].alpha = accumAlpha >= 1 ? 1. : accumAlpha;
	}
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
/*bool updateConstraints( const ToleranceConstraint *constraints, int startIndex, int endIndex, int headIndex, int tailIndex, ConstraintSearchParams *masterSearchParams )
{
	ConstraintSearchParams p = *masterSearchParams;

	//std::cerr << "CONSTRAINT SEARCH : " << startIndex << " , " << endIndex << " : " << headIndex << " , " << tailIndex << "\n";

	// Use a number of max iterations just in case numerical precision gets us stuck
	// ( I've never actually seen this happen )
	for( int iteration = 0; iteration < endIndex - startIndex + 1; ++iteration )
	{
		bool needRescan = false;

		// Test all constraints to make sure that the current line fufills all of them
		for( int i = endIndex; i >= startIndex; i-- )
		{

			double lowerX = constraints[i].lowerX;
			double minY = constraints[i].minY;
			double upperX = constraints[i].upperX;
			double maxY = constraints[i].maxY;

			if( i <= headIndex )
			{
				minY = -std::numeric_limits<double>::infinity();
			}

			if( i >= tailIndex || maxY == maximumLinearY )
			{
				maxY = std::numeric_limits<double>::infinity();
			}

			if( p.a == std::numeric_limits<double>::infinity() )
			{
				// We don't have enough constraints to form a slope yet.
				// We are still in initialization mode.

				if( lowerX < p.lowerConstraint.x && minY != -std::numeric_limits<double>::infinity() )
				{
					p.lowerConstraint.x = lowerX;
					p.lowerConstraint.y = minY;
				}
				if( upperX > p.upperConstraint.x && maxY != std::numeric_limits<double>::infinity() )
				{
					p.upperConstraint.x = upperX;
					p.upperConstraint.y = maxY;
				}
				if( p.lowerConstraint.x < p.upperConstraint.x )
				{
					//std::cerr << "SET : " << p.lowerConstraint.x << " , " << p.lowerConstraint.y << " : " << p.upperConstraint.x << " , " << p.upperConstraint.y << "\n";
					p.a = ( p.upperConstraint.y - p.lowerConstraint.y ) / ( p.upperConstraint.x - p.lowerConstraint.x );
					p.b = p.lowerConstraint.y - p.lowerConstraint.x * p.a;

					// OK, now we have enough constraints to form a line.
					// We now need to rescan in case our new line violates any previously considered constraints
					needRescan = true;
				}
			}
			else
			{
				double yAtLowerX = p.a * lowerX + p.b;
				double yAtUpperX = p.a * upperX + p.b;

				// Check if we go underneath the minimum constraint at this index
				if( yAtLowerX < minY )
				{
					if( lowerX > p.upperConstraint.x )
					{
						// If the violated constraint is to the right of the previous upper constraint, then we would need to get steeper to fufill it
						// But the current line is already the steepest that fufills previous constraints, so there can be no line which fufills all constraints.
						// Fail.
						return false;
					}
					else if( lowerX != p.lowerConstraint.x || minY != p.lowerConstraint.y )
					{
						// Replace the lower constraint with the constraint we violated.  Recalculate the steepest line through these constraints,
						// and trigger a rescan to check our new line against previous constraints
						double newA = (p.upperConstraint.y - minY) / ( p.upperConstraint.x - lowerX );
						if( newA < p.a )
						{
							p.a = newA;
							p.b = minY - lowerX * p.a;
							p.lowerConstraint.x = lowerX;
							p.lowerConstraint.y = minY;
							needRescan = true;
						}
					}
				}

				// Check if we go above the maxmimum constraint at this index
				if( yAtUpperX > maxY )
				{
					if( upperX < p.lowerConstraint.x )
					{
						// If the violated constraint is to the left of the previous lower constraint, then we would need to get steeper to fufill it
						// But the current line is already the steepest that fufills previous constraints, so there can be no line which fufills all constraints.
						// Fail.
						return false;
					}
					else if( upperX != p.upperConstraint.x || maxY != p.upperConstraint.y )
					{
						// Replace the upper constraint with the constraint we violated.  Recalculate the steepest line through these constraints,
						// and trigger a rescan to check our new line against previous constraints
						//std::cerr << "REPLACE\n";
						if( upperX != p.lowerConstraint.x )
						{
							double newA = (maxY - p.lowerConstraint.y) / ( upperX - p.lowerConstraint.x );
							if( newA < p.a )
							{
								p.a = newA;
								p.b = maxY - upperX * p.a;
							}
						}
						p.upperConstraint.x = upperX;
						p.upperConstraint.y = maxY;
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
}*/

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
void minimalSegmentsForConstraints( std::vector< LinearSegment > &compressedSamples, const std::vector< ToleranceConstraint > &constraints )
{
	compressedSamples.clear();

	double yPrev = 0; // exponentialToLinear( 0 ) == 0

	ConstraintSearchParams prevSearchParams;

	std::vector<V2d> tempConstraintLower;
	std::vector<V2d> tempConstraintUpper;

	std::vector<V2d> constraintsLower;
	std::vector<V2d> constraintsUpper;
	constraintsLower.reserve( constraints.size() );
	constraintsUpper.reserve( constraints.size() );

	for( unsigned int i = 0; i < constraints.size(); i++ )
	{
		if( constraints[i].minY != -std::numeric_limits<double>::infinity() )
		{
			constraintsLower.push_back( V2d( constraints[i].lowerX, constraints[i].minY ) );
		}
		constraintsUpper.push_back( V2d( constraints[i].upperX, constraints[i].maxY ) );
	}

	unsigned int lowerStartIndex = 0;
	unsigned int lowerStopIndex = 0;
	unsigned int upperStartIndex = 0;
	unsigned int upperStopIndex = 0;

	//unsigned int preScanIndex = 0;
	//unsigned int scanIndex = 0;
	while( lowerStopIndex < constraintsLower.size() )
	//while( scanIndex < constraints.size() )
	{
		// Initial constraint search parameters for not yet having found anything
		ConstraintSearchParams currentSearchParams;
		currentSearchParams.a = std::numeric_limits<double>::infinity();
		currentSearchParams.b = 0;

		currentSearchParams.lowerConstraintIndex = -1;
		currentSearchParams.upperConstraintIndex = -1;

		double yFinal = 0;
		//unsigned int searchIndex = scanIndex;
		//unsigned int tailIndex = scanIndex;

		//for( ;;searchIndex = scanIndex; searchIndex < constraints.size(); ++searchIndex )

		for( ;lowerStopIndex < constraintsLower.size(); lowerStopIndex++ )
		{
			// Try forming a segment that pass over all lower constraints up to searchIndex, and under any upper
			// constraints up to searchIndex which haven't been previously covered

			// We will create a flat top which just reaches the lower constraint at this index
			double yFinalTrial = constraintsLower[lowerStopIndex].y;

			// Any upper constraints greater than yFinalTrial can be ignored, because we are going to cut before reaching them
			//tailIndex = searchIndex;
			while( upperStopIndex < constraintsUpper.size() - 1 && constraintsUpper[ upperStopIndex + 1 ].y < yFinalTrial )
			{
				upperStopIndex++;
			}

			//tempConstraintLower.clear();
			//tempConstraintUpper.clear();
			/*start = preScanIndex;	
			end = searchIndex;
			head = int( scanIndex ) - 1;
			tail = tailIndex;*/
			/*for( unsigned int i = scanIndex; i <= searchIndex; i++ )
			{
				if( constraints[i].minY != -std::numeric_limits<double>::infinity() )
				{
					tempConstraintLower.push_back( V2d( constraints[i].lowerX, constraints[i].minY ) );
				}
			}
			for( unsigned int i = preScanIndex; i < tailIndex; i++ )
			{
				tempConstraintUpper.push_back( V2d( constraints[i].upperX, constraints[i].maxY ) );
			}*/
			/*for( int i = endIndex; i >= startIndex; i-- )
			{

				double lowerX = constraints[i].lowerX;
				double minY = constraints[i].minY;
				double upperX = constraints[i].upperX;
				double maxY = constraints[i].maxY;

				if( i <= headIndex )
				{
					minY = -std::numeric_limits<double>::infinity();
				}

				if( i >= tailIndex || maxY == maximumLinearY )
				{
					maxY = std::numeric_limits<double>::infinity();
				}
			}*/
			// Try fitting a line to our new set of constraints
			//bool success = updateConstraints( &constraints[0], preScanIndex, searchIndex, int( scanIndex ) - 1, tailIndex, &currentSearchParams );
			//std::cerr << "CONSTRAINT SEARCH: " << tempConstraintLower.size() << " , " << tempConstraintUpper.size() << "\n";
			bool success = updateConstraintsSimple( &constraintsLower[0], lowerStartIndex, lowerStopIndex + 1, &constraintsUpper[0], upperStartIndex, upperStopIndex + 1, &currentSearchParams );

			if( !success )
			{
				// It didn't work, so we'll use the previous value left in currentSearchParams
				break;
			}

			// It worked, we've got a new segment
			yFinal = yFinalTrial;

			// We'll continue looping and try to find a segment that covers more constraints
		}
		
		//assert( searchIndex > scanIndex );
		//std::cerr << "Segment " << compressedSamples.size() << " : " << currentSearchParams.lowerConstraint.x << "," << currentSearchParams.lowerConstraint.y << "->" << currentSearchParams.upperConstraint.x << "," << currentSearchParams.upperConstraint.y << "\n";

		double xStart;

		// Calculate where our new line hits its flat top
		double xEnd = ( yFinal - currentSearchParams.b ) / currentSearchParams.a;
		assert( !std::isnan( xEnd ) );

		assert( currentSearchParams.lowerConstraint.x != std::numeric_limits<double>::infinity() );

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
			/*if( currentSearchParams.lowerConstraint.x == std::numeric_limits<double>::infinity() )
			{
				currentSearchParams.lowerConstraint.x = constraints.back().lowerX;
			}*/
			xStart = xEnd = constraintsLower[currentSearchParams.lowerConstraintIndex].x;
			assert( xStart < 1e10 ); // TODO
		}
		else
		{
			// Calculate where our new line hits the previous flat top
			xStart = ( yPrev - currentSearchParams.b ) / currentSearchParams.a;

			// If there is a previous segment, we need to cover the possibility that we are overlapping with it
			if( compressedSamples.size() > 0 )
			{
				LinearSegment &prevSegment = compressedSamples.back();
				if( xStart < prevSegment.XBack )
				{
					// If the previous sample was a point sample, intersect the new line segment with a vertical line that passes through it.
					if( fabs( prevSearchParams.a ) == std::numeric_limits<double>::infinity() || xStart < prevSegment.X )
					{
						xStart = compressedSamples.back().XBack;
						//compressedSamples.back().XBack = xStart;
						compressedSamples.back().YBack = prevSegment.XBack * currentSearchParams.a + currentSearchParams.b;
					}
					else
					{
						// Calculate the intersection of our new line with the previous line
						xStart = ( currentSearchParams.b - prevSearchParams.b ) / ( prevSearchParams.a - currentSearchParams.a );
						if( xStart < prevSegment.X )
						{
							// This case may occur due to precision issues.
							xStart = prevSegment.XBack;
							//compressedSamples.back().XBack = xStart;
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
		}

		assert( xEnd >= xStart );
		assert( !std::isnan( yFinal ) );

		LinearSegment compressedSample;
		compressedSample.X = xStart;
		compressedSample.XBack = xEnd;
		compressedSample.YBack = yFinal;
		compressedSamples.push_back( compressedSample );

		yPrev = yFinal;

		/*if( scanIndex < constraints.size() && constraints[scanIndex].upperX * prevSearchParams.a + prevSearchParams.b > constraints[scanIndex].maxY )
		{
			while( preScanIndex + 1 < constraints.size() && constraints[preScanIndex + 1].maxY < yFinal ) preScanIndex++;
		}*/
		//scanIndex = searchIndex;
		
		//lowerStopIndex++; // TODO TODO TODO

		//lowerStartIndex = lowerStopIndex;
		//lowerStartIndex++;
		/*if( currentSearchParams.lowerConstraint.x != std::numeric_limits<double>::infinity() )
		{
			while( lowerStartIndex < constraintsLower.size() && constraintsLower[lowerStartIndex].x < currentSearchParams.lowerConstraint.x )
			{
				lowerStartIndex++;
			}
		}*/
		lowerStartIndex = lowerStopIndex;
		//upperStartIndex = upperStopIndex + 1;
		// TODO - silly inefficient
		if( currentSearchParams.upperConstraintIndex != -1 )
		{
			upperStartIndex = currentSearchParams.upperConstraintIndex;
		}
		//upperStopIndex++;
		

		prevSearchParams = currentSearchParams;

		if( upperStopIndex < constraintsUpper.size() && constraintsUpper[upperStopIndex].x * prevSearchParams.a + prevSearchParams.b > constraintsUpper[upperStopIndex].y )
		{
			while( upperStartIndex + 1 < constraints.size() && constraintsUpper[upperStartIndex + 1].y < yFinal )
			{
				upperStartIndex++;
			}
		}


		// If the previous line goes over the current constraint, then we are going to be underneath it
		// This means that we don't need to worry about upper constraints that it passed under,
		// because if we are heading towards them, we will intersect it
		// TODO TODO TODO
		/*if( scanIndex < constraints.size() && constraints[scanIndex].upperX * prevSearchParams.a + prevSearchParams.b > constraints[scanIndex].maxY )
		{
			while( preScanIndex + 1 < constraints.size() && constraints[preScanIndex + 1].maxY < yFinal ) preScanIndex++;
		}*/
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
 From a deep pixel, create a list of constraints in linear space that define the path the alpha curve must take to stay within
 the given alpha and z tolerance
*/
void constraintSamplesForPixel(
	const int inSamples, const float *inA, const float *inZ, const float *inZBack,
	double alphaTolerance,
	double zTolerance,
	std::vector< ToleranceConstraint > &constraints
	)
{
	std::vector< DepthPointSample > deepSamples;
	integratedPointSamplesForPixel( inSamples, inA, inZ, inZBack, deepSamples );

	constraints.resize( deepSamples.size() );
	double lastValidMinY = 0;
	for( unsigned int j = 0; j < deepSamples.size(); ++j )
	{
		double minAlpha = deepSamples[j].alpha - ( ( j == deepSamples.size() - 1 ) ? 0 : alphaTolerance );
		double maxAlpha = deepSamples[j].alpha + ( j == 0 ? 0 : alphaTolerance );

		double max = maxAlpha < 1 ? -log1p( -maxAlpha ) : maximumLinearY;
		double min = -std::numeric_limits<double>::infinity();
		if( minAlpha > 0 )
		{
			if( minAlpha < 1 )
			{
				min = -log1p( -minAlpha );
				lastValidMinY = min;
			}
			else
			{
				min = lastValidMinY;
			}
		}

		constraints[j].lowerX = deepSamples[j].depth * ( 1 + zTolerance );
		constraints[j].minY = min;
		constraints[j].upperX = deepSamples[j].depth * ( 1 - zTolerance );
		constraints[j].maxY = max;
	}

	// Ensure that we always reach the exact final value
	constraints.back().minY = exponentialToLinear( deepSamples.back().alpha );
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
	//if( debug ) raise( SIGABRT ); // TODO

	std::vector< ToleranceConstraint > constraints;
	constraintSamplesForPixel(
		inSamples, inA, inZ, inZBack,
		alphaTolerance, zTolerance,
		constraints
	);

	std::vector< LinearSegment > compressedSamples;
	minimalSegmentsForConstraints( compressedSamples, constraints );
	conformToSegments(
		inSamples, inA, inZ, inZBack,
		compressedSamples,
		outSamples, outA, outZ, outZBack
	);
	return;
}

/*void resampleDeepPixelAdaptive( DeepPixel &out, const DeepPixel &pixel, double alphaTolerance, double zTolerance )
{
	if( pixel.numSamples() < 2 )
	{
		out = pixel;
		return;
	}

	double step = 1.;
	double bestTolerance = alphaTolerance;
	int bestSamples = -1;
	std::vector< LinearSegment > bestCompressedSamples;
	for( unsigned int i = 0; i < 10; ++i )
	{
		std::vector< ToleranceConstraint > constraints;
		constraintSamplesForPixel( constraints, pixel, alphaTolerance, zTolerance );

		// Holds a list of the resampled points.
		std::vector< LinearSegment > compressedSamples;
		minimalSegmentsForConstraints( compressedSamples, constraints );

		if( int( compressedSamples.size() ) == bestSamples || bestSamples == -1 )
		{
			bestTolerance = alphaTolerance;
			bestCompressedSamples = compressedSamples;
			bestSamples = compressedSamples.size();
		}
		else
		{
			alphaTolerance = bestTolerance;
		}

		step *= .5;
		alphaTolerance -= alphaTolerance * step;
	}

	conformToSegments( out, pixel, bestCompressedSamples );
}*/

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
			int resampledCount;
			resampleDeepPixel(
				index - prev, &alpha[prev], &z[prev], &zBack[prev],
				alphaTolerance, depthTolerance,
				resampledCount, &outAlpha[outputCount], &outZ[outputCount], &outZBack[outputCount],
				pixelLocation == V2i( 112, 32 )
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