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


using namespace std;
using namespace Imath;
using namespace IECore;
using namespace Gaffer;
using namespace GafferImage;

namespace
{

/// A DeepPixel represents arbitrary channel data stored at varying depths in space.
/// By convention, depth is measured as distance from the eye plane
/// \ingroup deepCompositingGroup
class DeepPixel
{

    public :
        /// Constructs a new DeepPixel, with a given number of extra channels
        //  ( in addition to the standard Z, ZBack, and alpha ).
        /// numSamples is only used to reserve the appropriate amount of space.
        /// It does not actually initialize data or add default samples.
		DeepPixel( unsigned numUserChannels = 0, unsigned numSamples = 0 )
			: m_numChannels( 3 + numUserChannels ), m_numUserChannels( numUserChannels)
		{
			m_samples.reserve( numSamples * m_numChannels );
		}

        /// The number of depth samples
        unsigned numSamples() const
		{
			return m_samples.size() / m_numChannels;
		}

        /// Adds a new depth sample
        void addSample( float z, float zback, float alpha, const float *userChannelData )
		{
			m_samples.reserve( m_samples.size() + m_numChannels );

			m_samples.push_back( z );
			m_samples.push_back( zback );
			m_samples.push_back( alpha );
			for ( unsigned i=0; i < m_numUserChannels; ++i )
			{
				m_samples.push_back( userChannelData[i] );
			}
		}

        /// The standard channels
        float getZ( unsigned index ) const
		{
			return m_samples[ index * m_numChannels + 0 ];
		}
        float getZBack( unsigned index ) const
		{
			return m_samples[ index * m_numChannels + 1 ];
		}
        float getAlpha( unsigned index ) const
		{
			return m_samples[ index * m_numChannels + 2 ];
		}

        const float *getUserChannelData( unsigned index ) const
		{
			return &m_samples[ index * m_numChannels + 3 ];
		}

        unsigned numUserChannels() const
		{
			return m_numUserChannels;
		}

    private :
        std::vector<float> m_samples;
        unsigned m_numChannels;
        unsigned m_numUserChannels;

};

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

/// Compresses a DeepPixel given an alpha and a depth tolerance.
void resampleDeepPixel( DeepPixel &out, const DeepPixel &p, double alphaTolerance, double zTolerance );

/// Defines how many samples the compression algorithm takes along the length of a volume sample.
int numberOfSplitsPerSample();

void constraintSamplesForPixel( std::vector< ToleranceConstraint > &constraints, const DeepPixel &pixel, double alphaTolerance, double zTolerance );

/// Creates a set of point samples across the range of the pixel's depth for the alpha channel.
/// The deepSamples that are returned are stored in the deepSamples vector, with values for depth and accumulated alpha from front to back
/// The splits parameter defines how many point samples each volume sample is split into.
void integratedPointSamplesForPixel( std::vector<DepthPointSample> &deepSamples, const DeepPixel &pixel, int splits = numberOfSplitsPerSample() );

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
	return 5;
}

/*
 Given a deep pixel which may contain a mixture of point and volume samples, produce a set of samples which represent the
 alpha falloff curve, where each sample stores a depth and the integrated alpha up to that point.

 Note that currently volume samples are just split into a fixed number of point samples.  A better approach would be
 to split enough times that the alpha increment for each sample is less than the alpha threshold.
*/
void integratedPointSamplesForPixel( std::vector<DepthPointSample> &deepSamples, const DeepPixel &pixel, int splits )
{
	const unsigned int split = numberOfSplitsPerSample();

	unsigned int numSamples = pixel.numSamples();

	// Create an additional sample just before the first to force
	// the algorithm to start from the first sample.
	double depthOfFirstPoint = pixel.getZ( 0 ) * 0.99999;
	deepSamples.push_back( DepthPointSample( depthOfFirstPoint, 0 ) );

	// Now add the remaining samples.
	for ( unsigned int i = 0; i < numSamples; ++i )
	{
		double Z = pixel.getZ( i );
		double ZBack = pixel.getZBack( i );

		if( Z == ZBack ) // If this is a point sample.
		{
			deepSamples.push_back( DepthPointSample( Z, pixel.getAlpha( i ) ) );
		}
		else
		{
			double alpha = std::min( 1.0f - 1e-6f, pixel.getAlpha( i ) ); // TODO
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
	SimplePoint2D lowerConstraint, upperConstraint;

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
bool updateConstraints( const ToleranceConstraint *constraints, int startIndex, int endIndex, int headIndex, int tailIndex, ConstraintSearchParams *masterSearchParams )
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
}

/*
 Given a set of segments that define an optimal resampling which fufills all constraints in the linear remapped space,
 remap all the color and alpha channels of a deep pixel to match
*/
void conformToSegments( DeepPixel &resampledPixel, const DeepPixel& pixel, const std::vector< LinearSegment >& compressedSamples )
{
	resampledPixel = DeepPixel( pixel.numUserChannels(), compressedSamples.size() );

	double totalAccumAlpha = 0;
	unsigned numChannels = 3 + resampledPixel.numUserChannels();

	std::vector<double> segmentAccumChannels( numChannels, 0 );
	std::vector<double> curChannelData( numChannels );

	double alphaRemaining = 0.0;

	unsigned int integrateIndex = 0;
	for( unsigned int i = 0; i < compressedSamples.size(); ++i )
	{
		double targetAlpha = -expm1( -compressedSamples[i].YBack );
		if( ! ( targetAlpha > -0.1 && targetAlpha < 1.1 ) )
		{
			std::cerr << "BAD TARGET: " << targetAlpha << "\n";
			targetAlpha = 0.0f;
		}

		for( ; integrateIndex < pixel.numSamples(); ++integrateIndex  )
		{
			if( alphaRemaining == 0.0 )
			{
				curChannelData[0] = pixel.getZ( integrateIndex );
				curChannelData[1] = pixel.getZBack( integrateIndex );
				curChannelData[2] = pixel.getAlpha( integrateIndex );
				for( unsigned j = 0; j < resampledPixel.numUserChannels(); j++ )
				{
					curChannelData[3 + j] = pixel.getUserChannelData( integrateIndex )[j];
				}
				alphaRemaining = curChannelData[ 2 ];
			}

			if( curChannelData[ 2 ] >= 1 )
			{
				if( integrateIndex == pixel.numSamples()-1 )
				{
					curChannelData[ 2 ] = 1.;
				}
				else
				{
					curChannelData[ 2 ] = 1. - std::numeric_limits<float>::epsilon();
				}
			}

			double alphaNeeded = ( targetAlpha - totalAccumAlpha ) / ( 1 - totalAccumAlpha );

			double alphaToTake;
			if( alphaNeeded >= alphaRemaining )
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
			double curChannelMultiplier = curChannelData[2] > 0 ? ( 1 - segmentAccumChannels[ 2 ] ) * alphaToTake / curChannelData[ 2 ] : 0.0; // TODO


			for( unsigned int j = 0; j < numChannels; ++j )
			{
				segmentAccumChannels[j] += curChannelMultiplier * curChannelData[j];
			}

			if( alphaRemaining > 0 )
			{
				break;
			}
		}

		segmentAccumChannels[ 1 ] = compressedSamples[i].XBack;

		float userChannels[ resampledPixel.numUserChannels() ];
		for( unsigned int j = 0; j < resampledPixel.numUserChannels(); ++j )
		{
			userChannels[j] = segmentAccumChannels[3+j];
		}

		if( compressedSamples[i].XBack < compressedSamples[i].X )
		{
			std::cerr << "BAD XBACK: " << compressedSamples[i].X << " -> " << compressedSamples[i].XBack << "\n";
		}

		if( ! ( segmentAccumChannels[2] > -0.1 && segmentAccumChannels[2] < 1.1 ) )
		{
			std::cerr << "BAD CALC: " << segmentAccumChannels[2] << "\n";
			segmentAccumChannels[2] = 0.0f;
		}

		resampledPixel.addSample( compressedSamples[i].X, compressedSamples[i].XBack, segmentAccumChannels[2], userChannels );

		for( unsigned int j = 0; j < numChannels; ++j )
		{
			segmentAccumChannels[j] = 0.0;
		}

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

	unsigned int preScanIndex = 0;
	unsigned int scanIndex = 0;
	while( scanIndex < constraints.size() )
	{
		// Initial constraint search parameters for not yet having found anything
		ConstraintSearchParams currentSearchParams;
		currentSearchParams.a = std::numeric_limits<double>::infinity();
		currentSearchParams.b = 0;

		currentSearchParams.lowerConstraint.x = std::numeric_limits<double>::infinity();
		currentSearchParams.upperConstraint.x = -std::numeric_limits<double>::infinity();

		// TODO - needed to add this initialize to compile, why?
		currentSearchParams.lowerConstraint.y = 0.0f;
		currentSearchParams.upperConstraint.y = 0.0f;

		double yFinal = 0;
		unsigned int searchIndex = scanIndex;
		unsigned int tailIndex = scanIndex;

		for( searchIndex = scanIndex; searchIndex < constraints.size(); ++searchIndex )
		{
			// Try forming a segment that pass over all lower constraints up to searchIndex, and under any upper
			// constraints up to searchIndex which haven't been previously covered

			// We will create a flat top which just reaches the lower constraint at this index
			double yFinalTrial = constraints[searchIndex].minY;

			// Any upper constraints greater than yFinalTrial can be ignored, because we are going to cut before reaching them
			tailIndex = searchIndex;
			while( tailIndex > scanIndex + 1 && constraints[tailIndex - 1].maxY > yFinalTrial )
			{
				--tailIndex;
			}

			// Try fitting a line to our new set of constraints
			bool success = updateConstraints( &constraints[0], preScanIndex, searchIndex, int( scanIndex ) - 1, tailIndex, &currentSearchParams );

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


		if( fabs( yFinal ) == std::numeric_limits<double>::infinity() )
		{
			// We should never get here but if we do, do something fairly sensible.
			yFinal = constraints[ tailIndex - 1 ].minY;
			xStart = xEnd = constraints[ tailIndex-1 ].lowerX;
			currentSearchParams.a = std::numeric_limits<double>::infinity();
		}
		else if( fabs( currentSearchParams.a ) == std::numeric_limits<double>::infinity() )
		{
			// The line we have found is a point sample.
			xStart = xEnd = currentSearchParams.lowerConstraint.x;
		}
		else
		{
			// Calculate where our new line hits the previous flat top
			xStart = ( yPrev - currentSearchParams.b ) / currentSearchParams.a;

			// If there is a previous segment, we need to cover the possibility that we are overlapping with it
			if( compressedSamples.size() > 0 )
			{
				double xPrev = compressedSamples[compressedSamples.size() - 1].X;
				double xBackPrev = compressedSamples[compressedSamples.size() - 1].XBack;
				if( xStart < xBackPrev )
				{
					// If the previous sample was a point sample, intersect the new line segment with a vertical line that passes through it.
					if( fabs( prevSearchParams.a ) == std::numeric_limits<double>::infinity() || xStart < xPrev )
					{
						xStart = compressedSamples[compressedSamples.size() - 1].XBack;
						//compressedSamples[compressedSamples.size() - 1].XBack = xStart;
						compressedSamples[compressedSamples.size() - 1].YBack = xBackPrev * currentSearchParams.a + currentSearchParams.b;
					}
					else
					{
						// Calculate the intersection of our new line with the previous line
						xStart = ( currentSearchParams.b - prevSearchParams.b ) / ( prevSearchParams.a - currentSearchParams.a );
						if( xStart < xPrev )
						{
							// This case may occur due to precision issues.
							xStart = xBackPrev;
							//compressedSamples[compressedSamples.size() - 1].XBack = xStart;
						}
						else if( xStart > xEnd )
						{
							// This case may occur due to precision issues.
							xStart = xEnd;
						}
						else
						{
							// The start of the new segment falls between the start and end of the previous segment.
							// As a result, intersect the two line segments.
							compressedSamples[compressedSamples.size() - 1].XBack = xStart;
							compressedSamples[compressedSamples.size() - 1].YBack = xStart * prevSearchParams.a + prevSearchParams.b;
						}
					}
				}
			}
		}

		LinearSegment compressedSample;
		compressedSample.X = xStart;
		compressedSample.XBack = xEnd;
		compressedSample.YBack = yFinal;
		compressedSamples.push_back( compressedSample );

		yPrev = yFinal;

		scanIndex = searchIndex;

		prevSearchParams = currentSearchParams;


		// If the previous line goes over the current constraint, then we are going to be underneath it
		// This means that we don't need to worry about upper constraints that it passed under,
		// because if we are heading towards them, we will intersect it
		if( scanIndex < constraints.size() && constraints[scanIndex].upperX * prevSearchParams.a + prevSearchParams.b > constraints[scanIndex].maxY )
		{
			while( preScanIndex + 1 < constraints.size() && constraints[preScanIndex + 1].maxY < yFinal ) preScanIndex++;
		}
	}

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
	std::vector< ToleranceConstraint > &constraints,
	const DeepPixel &pixel,
	double alphaTolerance,
	double zTolerance
	)
{
	std::vector< DepthPointSample > deepSamples;
	integratedPointSamplesForPixel( deepSamples, pixel );

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

void resampleDeepPixel( DeepPixel &out, const DeepPixel &pixel, double alphaTolerance, double zTolerance )
{
	if( pixel.numSamples() < 2 )
	{
		out = pixel;
		return;
	}

	std::vector< ToleranceConstraint > constraints;
	constraintSamplesForPixel( constraints, pixel, alphaTolerance, zTolerance );

	std::vector< LinearSegment > compressedSamples;
	minimalSegmentsForConstraints( compressedSamples, constraints );
	conformToSegments( out, pixel, compressedSamples );
	return;
}

void resampleDeepPixelAdaptive( DeepPixel &out, const DeepPixel &pixel, double alphaTolerance, double zTolerance )
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

		std::vector<DeepPixel> outputPixels;
		outputPixels.resize( sampleOffsets.size() );
		int prev = 0;
		int outputCount = 0;
		for( unsigned int i = 0; i < sampleOffsets.size(); i++ )
		{
			int index = sampleOffsets[i];

			DeepPixel before( 0, index - prev );
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
			}
			resampleDeepPixel( outputPixels[i], before, alphaTolerance, depthTolerance );
			outputCount += outputPixels[i].numSamples();
			outSampleOffsets[i] = outputCount;

			prev = index;
		}

		FloatVectorDataPtr outAlphaData = new FloatVectorData();
		std::vector<float> &outAlpha = outAlphaData->writable();
		outAlpha.resize( outputCount );
		FloatVectorDataPtr outZData = new FloatVectorData();
		std::vector<float> &outZ = outZData->writable();
		outZ.resize( outputCount );
		FloatVectorDataPtr outZBackData = new FloatVectorData();
		std::vector<float> &outZBack = outZBackData->writable();
		outZBack.resize( outputCount );

		int outIndex = 0;
		for( unsigned int i = 0; i < sampleOffsets.size(); i++ )
		{
			int j = 0;
			while( outIndex < outSampleOffsets[i] )
			{
				outAlpha[outIndex] = outputPixels[i].getAlpha( j );
				if( !( outAlpha[outIndex] >= -0.1 && outAlpha[outIndex] <= 1.1  ) )
				{
					std::cerr << "BAD A : " << outAlpha[outIndex] << "\n";
					outAlpha[outIndex] = 1.0f;
				}
				outZ[outIndex] = outputPixels[i].getZ( j );
				outZBack[outIndex] = outputPixels[i].getZBack( j );
				outIndex++;
				j++;
			}
		}

		std::cerr << "TEST : " << outAlpha.size() << " : " << outSampleOffsets.back() << "\n";

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
