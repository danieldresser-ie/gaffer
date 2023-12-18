//////////////////////////////////////////////////////////////////////////
//
//  Copyright (c) 2015, Image Engine Design Inc. All rights reserved.
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
//      * Neither the name of Image Engine Design nor the names of
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

#include "GafferImage/Resample.h"

#include "GafferImage/DeepTileAccessor.h"
#include "GafferImage/ImageAlgo.h"
#include "GafferImage/FilterAlgo.h"
#include "GafferImage/Sampler.h"

#include "Gaffer/Context.h"
#include "Gaffer/StringPlug.h"

#include "OpenImageIO/filter.h"
#include "OpenImageIO/fmath.h"

#include <boost/heap/d_ary_heap.hpp>
#include <iostream>
#include <limits>

using namespace Imath;
using namespace IECore;
using namespace Gaffer;
using namespace GafferImage;

//////////////////////////////////////////////////////////////////////////
// Utilities
//////////////////////////////////////////////////////////////////////////

namespace
{

const std::string g_nearestString( "nearest" );

// Used as a bitmask to say which filter pass(es) we're computing.
enum Passes
{
	Horizontal = 1,
	Vertical = 2,
	Both = Horizontal | Vertical,

	// Special pass label when we must compute both passes in one, but there is no scaling.
	// This allows a special code path which is up to 6X faster.
	BothOptimized = Both | 4
};

Passes requiredPasses( const Resample *resample, const ImagePlug *image, const OIIO::Filter2D *filter, V2f &ratio )
{
	int debug = resample->debugPlug()->getValue();
	if( debug == Resample::HorizontalPass )
	{
		return Horizontal;
	}
	else if( debug == Resample::SinglePass )
	{
		// For a SinglePass debug mode, we always use Both.
		// Note that we don't use the optimized pass here, even if the ratio is 1 - we want debug to always
		// use the same path.
		return (Passes)( Horizontal | Vertical );
	}

	if( image == image->parent<ImageNode>()->outPlug() )
	{
		if( !filter )
		{
			// For the nearest filter, we use a separate code path that doesn't consider RequiredPasses anyway.
			return Both;
		}
		if( filter->separable() )
		{
			return Vertical;
		}
		else
		{
			// The filter isn't separable, so we must process everything at once. If the ratio has no
			// scaling though, we can use the optimized path.
			return ( ratio == V2f( 1.0 ) ) ? BothOptimized : Both;
		}
	}
	return Horizontal;
}

// Rounds min down, and max up, while converting from float to int.
Box2i box2fToBox2i( const Box2f &b )
{
	return Box2i(
		V2i( floor( b.min.x ), floor( b.min.y ) ),
		V2i( ceil( b.max.x ), ceil( b.max.y ) )
	);
}

// Calculates the scale and offset needed to convert from output
// coordinates to input coordinates.
void ratioAndOffset( const M33f &matrix, V2f &ratio, V2f &offset )
{
	ratio = V2f( matrix[0][0], matrix[1][1] );
	offset = -V2f( matrix[2][0], matrix[2][1] ) / ratio;
}

// The radius for the filter is specified in the output space. This
// method returns it as a number of pixels in the input space.
V2f inputFilterRadius( const OIIO::Filter2D *filter, const V2f &inputFilterScale )
{
	if( !filter )
	{
		return V2f( 0.0f );
	}

	return V2f(
		filter->width() * inputFilterScale.x * 0.5f,
		filter->height() * inputFilterScale.y * 0.5f
	);
}

// Returns the input region that will need to be sampled when
// generating a given output tile.
Box2i inputRegion( const V2i &tileOrigin, unsigned passes, const V2f &ratio, const V2f &offset, const OIIO::Filter2D *filter, const V2f &inputFilterScale )
{
	Box2f outputRegion( V2f( tileOrigin ), tileOrigin + V2f( ImagePlug::tileSize() ) );
	V2f filterRadius = inputFilterRadius( filter, inputFilterScale );
	V2i filterRadiusCeil( ceilf( filterRadius.x ), ceilf( filterRadius.y ) );

	Box2f result = outputRegion;
	if( passes & Horizontal )
	{
		result.min.x = result.min.x / ratio.x + offset.x;
		result.max.x = result.max.x / ratio.x + offset.x;
		if( result.min.x > result.max.x )
		{
			// Correct for negative scaling inverting
			// the relationship between min and max.
			std::swap( result.min.x, result.max.x );
		}
		result.min.x -= filterRadiusCeil.x;
		result.max.x += filterRadiusCeil.x;
	}
	if( passes & Vertical )
	{
		result.min.y = result.min.y / ratio.y + offset.y;
		result.max.y = result.max.y / ratio.y + offset.y;
		if( result.min.y > result.max.y )
		{
			std::swap( result.min.y, result.max.y );
		}
		result.min.y -= filterRadiusCeil.y;
		result.max.y += filterRadiusCeil.y;
	}

	return box2fToBox2i( result );
}

// Given a filter name, the current scaling ratio of input size / output size, and the desired filter scale in
// output space, return a filter and the correct filter scale in input space
const OIIO::Filter2D *filterAndScale( const std::string &name, V2f ratio, V2f &inputFilterScale )
{
	ratio.x = fabs( ratio.x );
	ratio.y = fabs( ratio.y );

	const OIIO::Filter2D *result;
	if( name == "" )
	{
		if( ratio.x > 1.0f || ratio.y > 1.0f )
		{
			// Upsizing
			result = FilterAlgo::acquireFilter( "blackman-harris" );
		}
		else
		{
			// Downsizing
			result = FilterAlgo::acquireFilter( "lanczos3" );
		}
	}
	else if( name == g_nearestString )
	{
		inputFilterScale = V2f( 0 );
		return nullptr;
	}
	else
	{
		result = FilterAlgo::acquireFilter( name );
	}

	// Convert the filter scale into input space
	inputFilterScale = V2f( 1.0f ) / ratio;

	// Don't allow the filter scale to cover less than 1 pixel in input space
	inputFilterScale = V2f( std::max( 1.0f, inputFilterScale.x ), std::max( 1.0f, inputFilterScale.y ) );

	return result;
}

// Precomputes all the filter weights for a whole row or column of a tile. For separable
// filters these weights can then be reused across all rows/columns in the same tile.
/// \todo The weights computed for a particular tile could also be reused for all
/// tiles in the same tile column or row. We could achieve this by outputting
/// the weights on an internal plug, and using Gaffer's caching to ensure they are
/// only computed once and then reused. At the time of writing, profiles indicate that
/// accessing pixels via the Sampler is the main bottleneck, but once that is optimised
/// perhaps cached filter weights could have a benefit.
void filterWeights1D( const OIIO::Filter2D *filter, const float inputFilterScale, const float filterRadius, const int x, const float ratio, const float offset, Passes pass, std::vector<int> &supportRanges, std::vector<float> &weights )
{
	weights.reserve( ( 2 * ceilf( filterRadius ) + 1 ) * ImagePlug::tileSize() );
	supportRanges.reserve( 2 * ImagePlug::tileSize() );

	const float filterCoordinateMult = 1.0f / inputFilterScale;

	for( int oX = x, eX = x + ImagePlug::tileSize(); oX < eX; ++oX )
	{
		// input pixel position (floating point)
		float iX = ( oX + 0.5 ) / ratio + offset;

		int minX = ceilf( iX - 0.5f - filterRadius );
		int maxX = floorf( iX + 0.5f + filterRadius );

		supportRanges.push_back( minX );
		supportRanges.push_back( maxX );

		if( oX == x )std::cerr << "Weights:";
		for( int fX = minX; fX < maxX; ++fX )
		{
			const float f = filterCoordinateMult * ( float( fX ) + 0.5f - iX );
			// TODO - supportRanges should only include values != 0. Address at same time as
			// moving normalization in here.
			const float w = pass == Horizontal ? filter->xfilt( f ) : filter->yfilt( f );
			weights.push_back( w );
			if( oX == x )std::cerr << w << " ";
		}
		if( oX == x )std::cerr << "\n";

	}
}

// For the inseparable case, we can't always reuse the weights for an adjacent row or column.
// There are a lot of possible scaling factors where the ratio can be represented as a fraction,
// and the weights needed would repeat after a certain number of pixels, and we could compute weights
// for a limited section of pixels, and reuse them in a tiling way.
// That's a bit complicated though, so we're just handling the simplest case currently ( since it is
// a common case ):
// if there is no scaling, then we only need to compute the weights for one pixel, and we can reuse them
// for all pixels. This means we don't loop over output pixels at all here - we just compute the weights
// for one output pixel, and return one 2D support for this pixel - it just gets shifted for each adjacent
// pixel.
void filterWeights2D( const OIIO::Filter2D *filter, const V2f inputFilterScale, const V2f filterRadius, const V2i p, const V2f ratio, const V2f offset, Box2i &support, std::vector<float> &weights )
{
	weights.reserve( ( 2 * ceilf( filterRadius.x ) + 1 ) * ( 2 * ceilf( filterRadius.y ) + 1 ) );
	weights.resize( 0 );

	const V2f filterCoordinateMult( 1.0f / inputFilterScale.x, 1.0f / inputFilterScale.y );

	// input pixel position (floating point)
	V2f i = ( V2f( p ) + V2f( 0.5 ) ) / ratio + offset;

	support = Box2i(
		V2i( ceilf( i.x - 0.5f - filterRadius.x ), ceilf( i.y - 0.5f - filterRadius.y ) ),
		V2i( floorf( i.x + 0.5f + filterRadius.x ), floorf( i.y + 0.5f + filterRadius.y ) )
	);

	//std::cerr << "Weights:";
	for( int fY = support.min.y; fY < support.max.y; ++fY )
	{
		const float fy = filterCoordinateMult.y * ( float( fY ) + 0.5 - i.y );
		for( int fX = support.min.x; fX < support.max.x; ++fX )
		{
			const float fx = filterCoordinateMult.x * ( float( fX ) + 0.5f - i.x );
			const float w = (*filter)( fx, fy );
			weights.push_back( w );
			//std::cerr << w << " ";
		}
		//std::cerr << "\n";
	}
}

void nearestInputPixelX( const int x, const float ratio, const float offset, std::vector<int> &result )
{
	result.reserve( ImagePlug::tileSize() );

	for( int oX = x, eX = x + ImagePlug::tileSize(); oX < eX; ++oX )
	{
		result.push_back( floorf( ( oX + 0.5 ) / ratio + offset ) );
	}
}

Box2f transform( const Box2f &b, const M33f &m )
{
	if( b.isEmpty() )
	{
		return b;
	}

	Box2f r;
	r.extendBy( V2f( b.min.x, b.min.y ) * m );
	r.extendBy( V2f( b.max.x, b.min.y ) * m );
	r.extendBy( V2f( b.max.x, b.max.y ) * m );
	r.extendBy( V2f( b.min.x, b.max.y ) * m );
	return r;
}

const IECore::InternedString g_AName = "A";
const IECore::InternedString g_ZName = "Z";
const IECore::InternedString g_ZBackName = "ZBack";
const IECore::InternedString g_sampleOffsetsName = "sampleOffsets";
const IECore::InternedString g_contributionSupportsName = "contributionSupports";
const IECore::InternedString g_contributionCountsName = "contributionCounts";
const IECore::InternedString g_contributionPixelIndicesName = "contributionPixelIndices";
const IECore::InternedString g_contributionSampleIndicesName = "contributionSampleIndices";
const IECore::InternedString g_contributionWeightsName = "contributionWeights";

class DeepLinearCombiner
{

public:
	DeepLinearCombiner( const std::vector<unsigned int> &countStreams, const std::vector< const float *> &alphaStreams, const std::vector< const float* > &zStreams, const std::vector< const float* > &zBackStreams, const std::vector< float > &weights ) :
		m_countStreams( countStreams ), m_alphaStreams( alphaStreams ), m_zStreams( zStreams ), m_zBackStreams( zBackStreams ), m_weights( weights )
	{
		//std::cerr << "\nSTART PIXEL\n";
		m_streamSampleIndex.resize( countStreams.size(), -1 );
		m_alphaAccumStreams.resize( countStreams.size(), 0.0f );

		for( unsigned int i = 0; i < countStreams.size(); i++ )
		{
			if( countStreams[i] )
			{
				m_heap.push( { zStreams[i][0], zBackStreams[i][0], i } );
			}
		}

		m_totalAccumAlpha = 0;
		m_prevTotalAccumAlpha = 0;
		m_totalClosedSegmentAccumAlpha = 0;
		nextSegment();
	}

	bool hasSegment()
	{
		return m_openSegments.size() > 0;
	}

	float z()
	{
		return m_z;
	}

	float zBack()
	{
		return m_zBack;
	}

	float alpha()
	{
		return m_alpha;
	}

	void nextSegment()
	{
		if( m_totalAccumAlpha == 1.0f )
		{
			m_openSegments.resize( 0 );
			return;
		}

		m_z = m_zBack;
		m_zBack = std::numeric_limits<float>::infinity();
		for( int i = m_openSegments.size() - 1; i >= 0; i-- )
		{
			int stream = m_openSegments[i].stream;
			int sampleIndex = m_streamSampleIndex[stream];
			if( m_zBackStreams[stream][sampleIndex] <= m_z )
			{
				float inc = ( 1 - m_alphaAccumStreams[stream] ) * m_alphaStreams[stream][sampleIndex];
				m_alphaAccumStreams[stream] += inc;
				m_totalClosedSegmentAccumAlpha += inc * m_weights[stream];
				m_openSegments.erase( m_openSegments.begin() + i );
			}
			else
			{
				m_openSegments[i].prevAlpha = m_openSegments[i].alpha;
				/*if( m_zBackStreams[stream][sampleIndex] < m_zBack )
				{
					std::cerr << "TRIMMING ALREADY OPEN : " << m_zBackStreams[stream][sampleIndex] << "\n";
				}*/
				m_zBack = std::min( m_zBackStreams[stream][sampleIndex], m_zBack );
			}
		}

		if( m_heap.size() )
		{
			if( !m_openSegments.size() )
			{
				assert( m_zBack == std::numeric_limits<float>::infinity() );
				m_z = m_heap.top().z;
			}

			while(
				// Basic criteria, we want any segments that have started at the depth we're starting at
				m_heap.size() && m_heap.top().z <= m_z &&
				// And a kinda tricky criteria: if we're not making a volume sample ( ie. it's a point sample ),
				// we're only interested in inputs that are also point samples. Volume samples starting at this
				// depth will end up in the next sample we output, but don't open them yet, because that
				// could result in two open segments from the same stream, which our data structures don't support.
				( ( m_zBack > m_z ) || ( m_heap.top().zBack == m_z ) )
			)
			{
				//float tempTest = m_heap.top().first;
				//auto beginIt = m_heap.begin();
				//std::cerr << "Begin:Top" << beginIt->first << " : " << tempTest << "\n";
				unsigned int stream = m_heap.top().stream;
				// TODO - is there a good way to skip segments with 0 alpha without increasing complexity?
				m_openSegments.push_back( { stream, 0.0f, 0.0f } );
				m_streamSampleIndex[stream]++;
				if( m_zBackStreams[stream][ m_streamSampleIndex[stream] ] < m_z )
				{
					std::cerr << "SAMPLE SHOULD HAVE BEEN CLOSED ALREADY : " << m_zStreams[stream][ m_streamSampleIndex[stream] ] << " -> " << m_zBackStreams[stream][ m_streamSampleIndex[stream] ] << " it had priority " << m_heap.top().z << " and index " << m_streamSampleIndex[stream] << " of " << m_countStreams[stream] << "\n";
				}
				m_zBack = std::min( m_zBack, m_zBackStreams[stream][ m_streamSampleIndex[stream] ] );
				if( m_streamSampleIndex[stream] < m_countStreams[stream] - 1 )
				{
					// Put this stream back in the heap, ready to find the next sample when it's needed.
					// TODO update -> decrease ( note reversal of order )
					/*m_heap.update(
						Heap::s_handle_from_iterator( beginIt ),
						std::make_pair( m_zStreams[stream][ m_streamSampleIndex[stream] + 1 ], stream )
					);*/
					// TODO - performance : Try : std::priority_queue, single op
					m_heap.pop();
					int nextSampleIndex = m_streamSampleIndex[stream] + 1;
					m_heap.push(
						{ m_zStreams[stream][nextSampleIndex], m_zBackStreams[stream][nextSampleIndex], stream }
					);
				}
				else
				{
					// We've reached the end of this stream, and can remove it from the heap.
					m_heap.pop();
				}
			}

			if( m_heap.size() )
			{
				/*if( m_heap.top().first < m_zBack )
				{
					std::cerr << "TRIMMING TO NEXT START : " << m_zBack << " : " << m_heap.top().first << "\n";
				}*/
				m_zBack = std::min( m_zBack, m_heap.top().z );
			}
		}

		
		/*if( m_openSegments.size() )
		{
			std::cerr << "SEGMENT : " << m_z << " -> " << m_zBack << "\n";
		}*/

		m_prevTotalAccumAlpha = m_totalAccumAlpha;
		m_totalAccumAlpha = m_totalClosedSegmentAccumAlpha;
		/*if( m_totalClosedSegmentAccumAlpha != 0.0f )
		{
			float resum = 0;
			float resumWeights = 0;
			for( unsigned int i = 0; i < m_alphaAccumStreams.size(); i++ )
			{
				resum += m_alphaAccumStreams[i] * m_weights[i];
				resumWeights += m_weights[i];
			}
			std::cerr << "CLOSED SEGS : " << m_totalClosedSegmentAccumAlpha << " : " << resum << " ? " << resumWeights << "\n";
			std::cerr << "SIZE : " << m_openSegments.size() << "\n";
		}*/

		for( OpenSegment &i : m_openSegments )
		{
			// TODO - we're not actually using stored i.alpha?
			int sampleIndex = m_streamSampleIndex[i.stream];
			float sampleZ = m_zStreams[i.stream][sampleIndex];
			float sampleZBack = m_zBackStreams[i.stream][sampleIndex];
			float sampleAlpha = m_alphaStreams[i.stream][sampleIndex];
			if( sampleAlpha == 0.0f )
			{
				// We don't need to do anything with a sample with zero alpha
				// TODO - how does this affect color channels?
			}
			if( sampleZ == m_z && sampleZBack == m_zBack )
			{
				i.alpha = sampleAlpha;
			}
			else if( m_zBack == sampleZ )
			{
				i.alpha = 0;
			}
			else if( m_zBack == m_z )
			{
				i.alpha = i.prevAlpha;
			}
			else if( sampleAlpha == 1.0f )
			{
				i.alpha = 1.0f;
			}
			else
			{
				//std::cerr << "SEGMENT FRAC : " << sampleZ << " -> " << m_zBack << " of " << sampleZ << " -> " << sampleZBack << "\n";
				i.alpha = -expm1( ( ( m_zBack - sampleZ ) / ( sampleZBack - sampleZ ) ) * log1p( -sampleAlpha ) );
				//std::cerr << "ALPHA FRAC : " << ( m_zBack - sampleZ ) / ( sampleZBack - sampleZ ) << " : " << i.alpha << "\n";
			}
			m_totalAccumAlpha += ( 1 - m_alphaAccumStreams[i.stream] ) * i.alpha * m_weights[i.stream];
		}
		m_alpha = ( m_totalAccumAlpha - m_prevTotalAccumAlpha ) / ( 1 - m_prevTotalAccumAlpha );
	}

	void addSampleContributions( int &count, std::vector< int > &streamIndices, std::vector< int > &sampleIndices, std::vector< float > &weights )
	{
		count = m_openSegments.size();
		for( OpenSegment &i : m_openSegments )
		{
			unsigned int sampleIndex = m_streamSampleIndex[i.stream];
			streamIndices.push_back( i.stream );
			sampleIndices.push_back( sampleIndex );
			float frac;
			if( i.prevAlpha == 1.0f )
			{
				frac = 0.0f;
			}
			else if( i.alpha == 0.0f )
			{
				float sampleZ = m_zStreams[i.stream][ sampleIndex ];
				float sampleZBack = m_zBackStreams[i.stream][ sampleIndex ];
				if( sampleZ == sampleZBack )
				{
					if( sampleZ == m_z && sampleZBack == m_zBack )
					{
						frac = 1.0f;
					}
					else
					{
						frac = 0.0f;
					}
				}
				else
				{
					// TODO - test this
					frac = ( m_zBack - sampleZ ) / ( sampleZBack - sampleZ );
				}
			}
			else
			{
				frac = ( 1.0f - m_alphaAccumStreams[i.stream] ) * ( i.alpha - i.prevAlpha )
					/ ( 1.0f - m_prevTotalAccumAlpha ) / m_alphaStreams[i.stream][ sampleIndex ];
			}
			weights.push_back( m_weights[i.stream] * frac );
		}
	}

	void addFlatContributions( std::vector< float * > &weights, float fraction )
	{
	}

private:
	// TODO - rename to streamCounts
	const std::vector< unsigned int > &m_countStreams;
	const std::vector< const float* > &m_alphaStreams;
	const std::vector< const float* > &m_zStreams;
	const std::vector< const float* > &m_zBackStreams;
	const std::vector< float > &m_weights;

	std::vector<unsigned int> m_streamSampleIndex;
	std::vector<float> m_alphaAccumStreams;

	float m_z;
	float m_zBack;
	float m_alpha;
	bool m_hasSegment;

	float m_totalClosedSegmentAccumAlpha;
	float m_totalAccumAlpha;
	float m_prevTotalAccumAlpha;


	// Each entry in the heap stores a minimum depth, and a stream index.
	struct HeapEntry
	{
		float z;
		float zBack;
		unsigned int stream;
	};

	// Compare based on start depth, with ties broken by end depth
	struct Compare
	{
		bool operator()(const HeapEntry &a, const HeapEntry &b) const
		{
			if( a.z != b.z )
			{
				return a.z > b.z;
			}
			else
			{
				return a.zBack > b.zBack;
			}
		}
	};
	using Heap = boost::heap::d_ary_heap< HeapEntry, boost::heap::arity<2>, boost::heap::mutable_<true>, boost::heap::compare<Compare> >;
	Heap m_heap;

	struct OpenSegment
	{
		unsigned int stream;
		float prevAlpha;
		float alpha;
	};

	std::vector<OpenSegment> m_openSegments;
};

} // namespace

//////////////////////////////////////////////////////////////////////////
// Resample
//////////////////////////////////////////////////////////////////////////

GAFFER_NODE_DEFINE_TYPE( Resample );

size_t Resample::g_firstPlugIndex = 0;

Resample::Resample( const std::string &name )
	:   ImageProcessor( name )
{
	storeIndexOfNextChild( g_firstPlugIndex );
	addChild( new M33fPlug( "matrix" ) );
	addChild( new StringPlug( "filter" ) );
	addChild( new V2fPlug( "filterScale", Plug::In, V2f( 1 ), V2f( 0 ) ) );
	addChild( new IntPlug( "boundingMode", Plug::In, Sampler::Black, Sampler::Black, Sampler::Clamp ) );
	addChild( new BoolPlug( "expandDataWindow" ) );
	addChild( new IntPlug( "debug", Plug::In, Off, Off, SinglePass ) );
	addChild( new ImagePlug( "__horizontalPass", Plug::Out ) );
	addChild( new CompoundObjectPlug( "__deepResizeData", Gaffer::Plug::Out, new IECore::CompoundObject ) );


	// We don't ever want to change these, so we make pass-through connections.

	outPlug()->viewNamesPlug()->setInput( inPlug()->viewNamesPlug() );
	outPlug()->formatPlug()->setInput( inPlug()->formatPlug() );
	outPlug()->metadataPlug()->setInput( inPlug()->metadataPlug() );
	outPlug()->channelNamesPlug()->setInput( inPlug()->channelNamesPlug() );

	horizontalPassPlug()->viewNamesPlug()->setInput( inPlug()->viewNamesPlug() );
	horizontalPassPlug()->formatPlug()->setInput( inPlug()->formatPlug() );
	horizontalPassPlug()->metadataPlug()->setInput( inPlug()->metadataPlug() );
	horizontalPassPlug()->channelNamesPlug()->setInput( inPlug()->channelNamesPlug() );

	// TODO - tidy input when processing deep

	outPlug()->deepPlug()->setInput( inPlug()->deepPlug() );
	horizontalPassPlug()->deepPlug()->setInput( inPlug()->deepPlug() );
}

Resample::~Resample()
{
}

Gaffer::M33fPlug *Resample::matrixPlug()
{
	return getChild<M33fPlug>( g_firstPlugIndex );
}

const Gaffer::M33fPlug *Resample::matrixPlug() const
{
	return getChild<M33fPlug>( g_firstPlugIndex );
}

Gaffer::StringPlug *Resample::filterPlug()
{
	return getChild<StringPlug>( g_firstPlugIndex + 1 );
}

const Gaffer::StringPlug *Resample::filterPlug() const
{
	return getChild<StringPlug>( g_firstPlugIndex + 1 );
}

Gaffer::V2fPlug *Resample::filterScalePlug()
{
	return getChild<V2fPlug>( g_firstPlugIndex + 2 );
}

const Gaffer::V2fPlug *Resample::filterScalePlug() const
{
	return getChild<V2fPlug>( g_firstPlugIndex + 2 );
}

Gaffer::IntPlug *Resample::boundingModePlug()
{
	return getChild<IntPlug>( g_firstPlugIndex + 3 );
}

const Gaffer::IntPlug *Resample::boundingModePlug() const
{
	return getChild<IntPlug>( g_firstPlugIndex + 3 );
}

Gaffer::BoolPlug *Resample::expandDataWindowPlug()
{
	return getChild<BoolPlug>( g_firstPlugIndex + 4 );
}

const Gaffer::BoolPlug *Resample::expandDataWindowPlug() const
{
	return getChild<BoolPlug>( g_firstPlugIndex + 4 );
}

Gaffer::IntPlug *Resample::debugPlug()
{
	return getChild<IntPlug>( g_firstPlugIndex + 5 );
}

const Gaffer::IntPlug *Resample::debugPlug() const
{
	return getChild<IntPlug>( g_firstPlugIndex + 5 );
}

ImagePlug *Resample::horizontalPassPlug()
{
	return getChild<ImagePlug>( g_firstPlugIndex + 6 );
}

const ImagePlug *Resample::horizontalPassPlug() const
{
	return getChild<ImagePlug>( g_firstPlugIndex + 6 );
}

CompoundObjectPlug *Resample::deepResizeDataPlug()
{
	return getChild<CompoundObjectPlug>( g_firstPlugIndex + 7 );
}

const CompoundObjectPlug *Resample::deepResizeDataPlug() const
{
	return getChild<CompoundObjectPlug>( g_firstPlugIndex + 7 );
}

void Resample::affects( const Gaffer::Plug *input, AffectedPlugsContainer &outputs ) const
{
	ImageProcessor::affects( input, outputs );

	// TODO - audit affects
	// unrelated TODO : stress test mixture of point and volume samples

	if(
		input == inPlug()->dataWindowPlug() ||
		input == matrixPlug() ||
		input == expandDataWindowPlug() ||
		input == filterPlug() ||
		input->parent<V2fPlug>() == filterScalePlug() ||
		input == debugPlug()
	)
	{
		outputs.push_back( outPlug()->dataWindowPlug() );
		outputs.push_back( horizontalPassPlug()->dataWindowPlug() );
	}

	if(
		input == inPlug()->dataWindowPlug() ||
		input == matrixPlug() ||
		input == filterPlug() ||
		input->parent<V2fPlug>() == filterScalePlug() ||
		input == inPlug()->channelDataPlug() ||
		input == boundingModePlug() ||
		input == debugPlug()
	)
	{
		outputs.push_back( outPlug()->channelDataPlug() );
		outputs.push_back( horizontalPassPlug()->channelDataPlug() );
	}

	if(
		input == inPlug()->channelNamesPlug() ||
		input == inPlug()->dataWindowPlug() ||
		input == matrixPlug() ||
		input == filterPlug() ||
		input->parent<V2fPlug>() == filterScalePlug() ||
		input == inPlug()->channelDataPlug() ||
		input == boundingModePlug()
	)
	{
		outputs.push_back( deepResizeDataPlug() );
	}

	if(
		input == inPlug()->deepPlug() ||
		input == deepResizeDataPlug() ||
		input == matrixPlug() ||
		input == filterPlug() ||
		input == boundingModePlug()
	)
	{	
		outputs.push_back( outPlug()->channelDataPlug() );
		outputs.push_back( outPlug()->sampleOffsetsPlug() );
	}
}

void Resample::hash( const Gaffer::ValuePlug *output, const Gaffer::Context *context, IECore::MurmurHash &h ) const
{
	ImageProcessor::hash( output, context, h );

	if( output != deepResizeDataPlug() )
	{
		return;
	}

	ConstStringVectorDataPtr channelNamesData;
	V2f ratio, offset;
	Sampler::BoundingMode boundingMode;
	V2f filterScale;

	{
		ImagePlug::GlobalScope s( context );
		channelNamesData = inPlug()->channelNamesPlug()->getValue();
		ratioAndOffset( matrixPlug()->getValue(), ratio, offset );
		boundingMode = (Sampler::BoundingMode)boundingModePlug()->getValue();
		filterScale = filterScalePlug()->getValue();
	}

	V2f inputFilterScale;
	const OIIO::Filter2D *filter = filterAndScale( filterPlug()->getValue(), ratio, inputFilterScale );
	inputFilterScale *= filterScale;

	filterPlug()->hash( h );
	h.append( inputFilterScale );
	h.append( ratio );
	h.append( offset );

	const V2i tileOrigin = context->get<V2i>( ImagePlug::tileOriginContextName );
	Box2i ir = inputRegion( tileOrigin, Both, ratio, offset, filter, inputFilterScale );

	bool hasA = false;
	bool hasZ = false;
	bool hasZBack = false;
	bool hasArbitraryChannel = false;
	for( const std::string &c : channelNamesData->readable() )
	{
		if( c == ImageAlgo::channelNameA )
		{
			hasA = true;
		}
		else if( c == ImageAlgo::channelNameZ )
		{
			hasZ = true;
		}
		else if( c == ImageAlgo::channelNameZBack )
		{
			hasZBack = true;
		}
		else
		{
			hasArbitraryChannel = true;
		}
	}

	if( hasZ )
	{
		DeepTileAccessor( inPlug(), ImageAlgo::channelNameZ, ir, boundingMode ).hash( h );
	}
	else
	{
		h.append( false );
	}
	if( hasZBack )
	{
		DeepTileAccessor( inPlug(), ImageAlgo::channelNameZBack, ir, boundingMode ).hash( h );
	}
	else
	{
		h.append( false );
	}
	if( hasA )
	{
		DeepTileAccessor( inPlug(), ImageAlgo::channelNameA, ir, boundingMode ).hash( h );
	}
	else
	{
		h.append( false );
	}

	h.append( hasArbitraryChannel );

	// Another tile might happen to need to filter over the same input
	// tiles as this one, so we must include the tile origin to make sure
	// each tile has a unique hash.
	h.append( tileOrigin );
}

void Resample::compute( Gaffer::ValuePlug *output, const Gaffer::Context *context ) const
{
	ImageProcessor::compute( output, context );

	if( output != deepResizeDataPlug() )
	{
		return;
	}

	ConstStringVectorDataPtr channelNamesData;
	V2f ratio, offset;
	Sampler::BoundingMode boundingMode;
	V2f filterScale;

	{
		ImagePlug::GlobalScope s( context );
		channelNamesData = inPlug()->channelNamesPlug()->getValue();
		ratioAndOffset( matrixPlug()->getValue(), ratio, offset );
		boundingMode = (Sampler::BoundingMode)boundingModePlug()->getValue();
		filterScale = filterScalePlug()->getValue();
	}

	V2f inputFilterScale;
	const OIIO::Filter2D *filter = filterAndScale( filterPlug()->getValue(), ratio, inputFilterScale );
	inputFilterScale *= filterScale;

	const V2i tileOrigin = context->get<V2i>( ImagePlug::tileOriginContextName );
	Box2i ir = inputRegion( tileOrigin, Both, ratio, offset, filter, inputFilterScale );

	// Don't think I like this API for requesting a DeepTileAccessor with just sample offsets by passing an
	// empty channel name, but it should be good enough to get things working.
	DeepTileAccessor sampleOffsetsSampler( inPlug(), "", ir, boundingMode );

	bool hasA = false;
	bool hasZ = false;
	bool hasZBack = false;
	bool hasArbitraryChannel = false;
	for( const std::string &c : channelNamesData->readable() )
	{
		if( c == ImageAlgo::channelNameA )
		{
			hasA = true;
		}
		else if( c == ImageAlgo::channelNameZ )
		{
			hasZ = true;
		}
		else if( c == ImageAlgo::channelNameZBack )
		{
			hasZBack = true;
		}
		else
		{
			hasArbitraryChannel = true;
		}
	}
	std::optional<DeepTileAccessor> zSampler, zBackSampler, alphaSampler;
	if( hasZ )
	{
		zSampler.emplace( sampleOffsetsSampler, ImageAlgo::channelNameZ );
	}
	if( hasZBack )
	{
		zBackSampler.emplace( sampleOffsetsSampler, ImageAlgo::channelNameZBack );
	}
	if( hasA )
	{
		alphaSampler.emplace( sampleOffsetsSampler, ImageAlgo::channelNameA );
	}

	std::vector<float> zeroDataBuffer;
	if( !zSampler || !alphaSampler )
	{
		// If we are missing a channel, we will fill it with zeros, so we need a buffer of zeros large enough
		// to supply all the samples for any pixel. It would be nice to just use a fixed global buffer of zeros
		// ... I was tempted to just use &ImagePlug::blackTile()->readable()[0], but someone could make a deep
		// with an arbitrarily large number of samples in one pixel.
		unsigned int maxCount = 0;
		const float *unused;
		unsigned int count;
		for( int y = ir.min.y; y < ir.max.y; ++y )
		{
			for( int x = ir.min.x; x < ir.max.x; ++x )
			{
				sampleOffsetsSampler.sample( x, y, unused, count );
				maxCount = std::max( count, maxCount );
			}
		}

		zeroDataBuffer.resize( maxCount, 0.0f );
	}

	const V2f filterRadius = inputFilterRadius( filter, inputFilterScale );
	const Box2i tileBound( tileOrigin, tileOrigin + V2i( ImagePlug::tileSize() ) );

	Box2i support;
	std::vector<float> weights;
	if( ratio == V2f(1) )
	{
		filterWeights2D( filter, inputFilterScale, filterRadius, tileBound.min, V2f( 1 ), offset, support, weights );
		// TODO - why the heck isn't this being done in filterWeights*?
		float total = 0;
		for( float w : weights )
		{
			total += w;
		}
		// TODO w == 0, assert?
		for( float &w : weights )
		{
			w /= total;
		}
	}

	IntVectorDataPtr outputSampleOffsetsData = new IntVectorData();
	std::vector<int> &outputSampleOffsets = outputSampleOffsetsData->writable();
	FloatVectorDataPtr outputAlphaData = new FloatVectorData();
	std::vector<float> &outputAlpha = outputAlphaData->writable();
	FloatVectorDataPtr outputZData = new FloatVectorData();
	std::vector<float> &outputZ = outputZData->writable();
	FloatVectorDataPtr outputZBackData = new FloatVectorData();
	std::vector<float> &outputZBack = outputZBackData->writable();
	unsigned int currentOutputSampleOffset = 0;

	Box2iVectorDataPtr outputContributionSupportsData;
	std::vector<Box2i> *outputContributionSupports = nullptr;
	IntVectorDataPtr outputContributionCountsData;
	std::vector<int> *outputContributionCounts = nullptr;
	IntVectorDataPtr outputContributionPixelIndicesData;
	std::vector<int> *outputContributionPixelIndices = nullptr;
	IntVectorDataPtr outputContributionSampleIndicesData;
	std::vector<int> *outputContributionSampleIndices = nullptr;
	FloatVectorDataPtr outputContributionWeightsData;
	std::vector<float> *outputContributionWeights = nullptr;

	if( hasArbitraryChannel )
	{	
		outputContributionSupportsData = new Box2iVectorData();
		outputContributionSupports = &outputContributionSupportsData->writable();
		outputContributionSupports->reserve( ImagePlug::tilePixels() );
		outputContributionCountsData = new IntVectorData();
		outputContributionCounts = &outputContributionCountsData->writable();
		outputContributionPixelIndicesData = new IntVectorData();
		outputContributionPixelIndices = &outputContributionPixelIndicesData->writable();
		outputContributionSampleIndicesData = new IntVectorData();
		outputContributionSampleIndices = &outputContributionSampleIndicesData->writable();
		outputContributionWeightsData = new FloatVectorData();
		outputContributionWeights = &outputContributionWeightsData->writable();
	}

	std::vector<unsigned int> counts;
	std::vector<const float *> alphaSamples;
	std::vector<const float *> zSamples;
	std::vector<const float *> zBackSamples;

	V2i oP; // output pixel position
	V2i supportOffset( 0 );
	for( oP.y = tileBound.min.y; oP.y < tileBound.max.y; ++oP.y )
	{
		for( oP.x = tileBound.min.x; oP.x < tileBound.max.x; ++oP.x )
		{
			if( ratio != V2f(1) )
			{
				filterWeights2D( filter, inputFilterScale, filterRadius, oP, ratio, offset, support, weights );
				// TODO - why the heck isn't this being done in filterWeights*?
				float total = 0;
				for( float w : weights )
				{
					total += w;
				}
				// TODO w == 0, assert?
				for( float &w : weights )
				{
					w /= total;
				}
			}
			else
			{
				supportOffset = oP - tileBound.min;
			}
			Canceller::check( context->canceller() );

			unsigned int unused;
			const float *unusedPtr;
			counts.resize( weights.size() );
			alphaSamples.resize( weights.size() );
			zSamples.resize( weights.size() );
			zBackSamples.resize( weights.size());

			if( hasArbitraryChannel )
			{	
				outputContributionSupports->push_back( Imath::Box2i( support.min + supportOffset, support.max + supportOffset ) );
			}

			int i = 0;
			for( int iy = support.min.y + supportOffset.y; iy < support.max.y + supportOffset.y; ++iy )
			{
				for( int ix = support.min.x + supportOffset.x; ix < support.max.x + supportOffset.x; ++ix )
				{
					if( weights[i] == 0.0f )
					{
						counts[i] = 0;
						zSamples[i] = nullptr;
						zBackSamples[i] = nullptr;
						alphaSamples[i] = nullptr;
					}
					else
					{
						sampleOffsetsSampler.sample( ix, iy, unusedPtr, counts[i] );
						if( zSampler )
						{
							zSampler->sample( ix, iy, zSamples[i], unused );
						}
						else
						{
							zSamples[i] = &zeroDataBuffer[0];
						}

						if( zBackSampler )
						{
							zBackSampler->sample( ix, iy, zBackSamples[i], unused );
						}
						else
						{
							zBackSamples[i] = zSamples[i];
						}

						if( alphaSampler )
						{
							alphaSampler->sample( ix, iy, alphaSamples[i], unused );
						}
						else
						{
							alphaSamples[i] = &zeroDataBuffer[0];
						}
					}

					i++;
				}
			}

			DeepLinearCombiner linearCombiner( counts, alphaSamples, zSamples, zBackSamples, weights );

			while( linearCombiner.hasSegment() )
			{
				currentOutputSampleOffset++;
				outputAlpha.push_back( linearCombiner.alpha() );
				outputZ.push_back( linearCombiner.z() );
				outputZBack.push_back( linearCombiner.zBack() );

				if( hasArbitraryChannel )
				{
					int contributionCount = 0;
					linearCombiner.addSampleContributions( contributionCount, *outputContributionPixelIndices, *outputContributionSampleIndices, *outputContributionWeights );
					outputContributionCounts->push_back( contributionCount );
				}

				linearCombiner.nextSegment();
			}
			outputSampleOffsets.push_back( currentOutputSampleOffset );
		}
	}

	CompoundObjectPtr result = new CompoundObject;
	result->members()[ g_sampleOffsetsName ] = outputSampleOffsetsData;
	result->members()[ g_AName ] = outputAlphaData;
	result->members()[ g_ZName ] = outputZData;
	result->members()[ g_ZBackName ] = outputZBackData;

	if( hasArbitraryChannel )
	{
		result->members()[ g_contributionSupportsName ] = outputContributionSupportsData;
		result->members()[ g_contributionCountsName ] = outputContributionCountsData;
		result->members()[ g_contributionPixelIndicesName ] = outputContributionPixelIndicesData;
		result->members()[ g_contributionSampleIndicesName ] = outputContributionSampleIndicesData;
		result->members()[ g_contributionWeightsName ] = outputContributionWeightsData;
	}
	static_cast<CompoundObjectPlug *>( output )->setValue( result );
}

void Resample::hashDataWindow( const GafferImage::ImagePlug *parent, const Gaffer::Context *context, IECore::MurmurHash &h ) const
{
	ImageProcessor::hashDataWindow( parent, context, h );

	inPlug()->dataWindowPlug()->hash( h );
	matrixPlug()->hash( h );
	expandDataWindowPlug()->hash( h );
	filterPlug()->hash( h );
	filterScalePlug()->hash( h );
	debugPlug()->hash( h );
}

Imath::Box2i Resample::computeDataWindow( const Gaffer::Context *context, const ImagePlug *parent ) const
{
	const Box2i srcDataWindow = inPlug()->dataWindowPlug()->getValue();
	if( BufferAlgo::empty( srcDataWindow ) )
	{
		return srcDataWindow;
	}

	// Figure out our data window as a Box2f with fractional
	// pixel values.

	const M33f matrix = matrixPlug()->getValue();
	Box2f dstDataWindow = transform( Box2f( srcDataWindow.min, srcDataWindow.max ), matrix );

	if( expandDataWindowPlug()->getValue() )
	{
		V2f ratio, offset;
		ratioAndOffset( matrix, ratio, offset );

		V2f inputFilterScale;
		const OIIO::Filter2D *filter = filterAndScale( filterPlug()->getValue(), ratio, inputFilterScale );
		inputFilterScale *= filterScalePlug()->getValue();

		const V2f filterRadius = V2f( filter->width(), filter->height() ) * inputFilterScale * 0.5f;

		dstDataWindow.min -= filterRadius * ratio;
		dstDataWindow.max += filterRadius * ratio;
	}

	// Convert that Box2f to a Box2i that fully encloses it.
	// Cheat a little to avoid adding additional pixels when
	// we're really close to the edge. This is primarily to
	// meet user expectations in the Resize node, where it is
	// expected that the dataWindow will exactly match the format.

	const float eps = 1e-4;
	if( ceilf( dstDataWindow.min.x ) - dstDataWindow.min.x < eps )
	{
		dstDataWindow.min.x = ceilf( dstDataWindow.min.x );
	}
	if( dstDataWindow.max.x - floorf( dstDataWindow.max.x ) < eps )
	{
		dstDataWindow.max.x = floorf( dstDataWindow.max.x );
	}
	if( ceilf( dstDataWindow.min.y ) - dstDataWindow.min.y < eps )
	{
		dstDataWindow.min.y = ceilf( dstDataWindow.min.y );
	}
	if( dstDataWindow.max.y - floorf( dstDataWindow.max.y ) < eps )
	{
		dstDataWindow.max.y = floorf( dstDataWindow.max.y );
	}

	Box2i dataWindow = box2fToBox2i( dstDataWindow );

	// If we're outputting the horizontal pass, then replace
	// the vertical range with the original.

	if( parent  == horizontalPassPlug() || debugPlug()->getValue() == HorizontalPass )
	{
		dataWindow.min.y = srcDataWindow.min.y;
		dataWindow.max.y = srcDataWindow.max.y;
	}

	return dataWindow;
}

void Resample::hashChannelData( const GafferImage::ImagePlug *parent, const Gaffer::Context *context, IECore::MurmurHash &h ) const
{
	ImageProcessor::hashChannelData( parent, context, h );


	bool deep;
	V2f ratio, offset;
	V2f filterScale;
	Sampler::BoundingMode boundingMode;
	{
		ImagePlug::GlobalScope c( context );
		deep = inPlug()->deepPlug()->getValue();
		ratioAndOffset( matrixPlug()->getValue(), ratio, offset );
		filterScale = filterScalePlug()->getValue();
		boundingMode = (Sampler::BoundingMode)boundingModePlug()->getValue();
	}

	V2f inputFilterScale;
	const OIIO::Filter2D *filter = filterAndScale( filterPlug()->getValue(), ratio, inputFilterScale );
	inputFilterScale *= filterScale;

	filterPlug()->hash( h );

	const V2i tileOrigin = context->get<V2i>( ImagePlug::tileOriginContextName );
	const std::string &channelName = context->get<std::string>( ImagePlug::channelNameContextName );
	Passes passes = requiredPasses( this, parent, filter, ratio );
	Box2i ir = inputRegion( tileOrigin, deep ? Both : passes, ratio, offset, filter, inputFilterScale );

	if( deep )
	{
		ImagePlug::ChannelDataScope deepResizeDataScope( Context::current() );

		if( filter )
		{
			deepResizeDataScope.remove( ImagePlug::channelNameContextName );
			deepResizeDataPlug()->hash( h );
		}
		else
		{
			h.append( inputFilterScale );
			h.append( ratio );
			h.append( offset );
			outPlug()->sampleOffsetsPlug()->hash( h );
		}

		if( filter && ( channelName == ImageAlgo::channelNameZ || channelName == ImageAlgo::channelNameZBack || channelName == ImageAlgo::channelNameA ) )
		{
			h.append( channelName );
		}
		else
		{
			DeepTileAccessor( inPlug(), channelName, ir, boundingMode ).hash( h );
		}

		return;
	}

	if( passes & Horizontal )
	{
		h.append( inputFilterScale.x );
		h.append( ratio.x );
		h.append( offset.x );
	}
	if( passes & Vertical )
	{
		h.append( inputFilterScale.y );
		h.append( ratio.y );
		h.append( offset.y );
	}

	if( passes == BothOptimized )
	{
		// Append an extra flag so our hash reflects that we are going to take the optimized path
		h.append( true );
	}

	Sampler sampler(
		passes == Vertical ? horizontalPassPlug() : inPlug(),
		channelName,
		ir,
		boundingMode
	);
	sampler.hash( h );

	// Another tile might happen to need to filter over the same input
	// tiles as this one, so we must include the tile origin to make sure
	// each tile has a unique hash.
	h.append( tileOrigin );
}

IECore::ConstFloatVectorDataPtr Resample::computeChannelData( const std::string &channelName, const Imath::V2i &tileOrigin, const Gaffer::Context *context, const ImagePlug *parent ) const
{
	bool deep;
	V2f ratio, offset;
	V2f filterScale;
	Sampler::BoundingMode boundingMode;
	{
		ImagePlug::GlobalScope c( context );
		ratioAndOffset( matrixPlug()->getValue(), ratio, offset );
		deep = inPlug()->deepPlug()->getValue();
		filterScale = filterScalePlug()->getValue();
		boundingMode = (Sampler::BoundingMode)boundingModePlug()->getValue();
	}

	V2f inputFilterScale;
	const OIIO::Filter2D *filter = filterAndScale( filterPlug()->getValue(), ratio, inputFilterScale );
	inputFilterScale *= filterScale;

	Passes passes = requiredPasses( this, parent, filter, ratio );
	Box2i ir = inputRegion( tileOrigin, deep ? Both : passes, ratio, offset, filter, inputFilterScale );

	const Box2i tileBound( tileOrigin, tileOrigin + V2i( ImagePlug::tileSize() ) );

	if( deep )
	{
		ImagePlug::ChannelDataScope deepResizeDataScope( Context::current() );
		deepResizeDataScope.remove( ImagePlug::channelNameContextName );

		if( !filter )
		{
			// We only use the output sample offsets so that we know ahead of time how many points
			// we're going to generate. Bit of a shame to add this complexity, but it does seem
			// like a worthwhile win to not need to reallocate the output buffer.
			ConstIntVectorDataPtr outputSampleOffsetsData = outPlug()->sampleOffsetsPlug()->getValue();
			
			FloatVectorDataPtr resultData = new FloatVectorData();
			std::vector<float> &result = resultData->writable();
			result.resize( outputSampleOffsetsData->readable().back() );

			DeepTileAccessor sampleOffsetsSampler( inPlug(), channelName, ir, boundingMode );
			
			std::vector<int> iPx;
			nearestInputPixelX( tileBound.min.x, ratio.x, offset.x, iPx );

			V2i oP; // output pixel position
			int iPy; // input pixel position Y

			int outputSamplePosition = 0;

			for( oP.y = tileBound.min.y; oP.y < tileBound.max.y; ++oP.y )
			{
				iPy = floorf( ( oP.y + 0.5 ) / ratio.y + offset.y );
				std::vector<int>::const_iterator iPxIt = iPx.begin();

				Canceller::check( context->canceller() );
				for( oP.x = tileBound.min.x; oP.x < tileBound.max.x; ++oP.x )
				{
					const float *channelSamples;
					unsigned int count;
					sampleOffsetsSampler.sample( *iPxIt, iPy, channelSamples, count );
					memcpy( &result[outputSamplePosition], channelSamples, count * sizeof( float ) );
					outputSamplePosition += count;
					++iPxIt;
				}
			}

			return resultData;
			
		}


		ConstCompoundObjectPtr deepResizeData = deepResizeDataPlug()->getValue();

		if( channelName == ImageAlgo::channelNameZ )
		{
			return deepResizeData->member<FloatVectorData>( g_ZName );
		}
		else if( channelName == ImageAlgo::channelNameZBack )
		{
			return deepResizeData->member<FloatVectorData>( g_ZBackName );
		}
		else if( channelName == ImageAlgo::channelNameA )
		{
			return deepResizeData->member<FloatVectorData>( g_AName );
		}

		const std::vector<int> &outputSampleOffsets = deepResizeData->member<IntVectorData>( g_sampleOffsetsName )->readable();
		const std::vector<Box2i> &contributionSupports = deepResizeData->member<Box2iVectorData>( g_contributionSupportsName )->readable();
		const std::vector<int> &contributionCounts = deepResizeData->member<IntVectorData>( g_contributionCountsName )->readable();
		const std::vector<int> &contributionPixelIndices = deepResizeData->member<IntVectorData>( g_contributionPixelIndicesName )->readable();
		const std::vector<int> &contributionSampleIndices = deepResizeData->member<IntVectorData>( g_contributionSampleIndicesName )->readable();
		const std::vector<float> &contributionWeights = deepResizeData->member<FloatVectorData>( g_contributionWeightsName )->readable();

		DeepTileAccessor deepChannelSampler( inPlug(), channelName, ir, boundingMode );

		std::vector<const float *> channelSamples;

		FloatVectorDataPtr resultData = new FloatVectorData;
		std::vector<float> &result = resultData->writable();
		result.reserve( outputSampleOffsets.back() );

		int contributionIndex = 0;
		int prevOffset = 0;
		for( int i = 0; i < ImagePlug::tilePixels(); i++ )
		{
			int offset = outputSampleOffsets[i];

			const Box2i &support = contributionSupports[i];

			channelSamples.reserve( support.size().x * support.size().y );
			channelSamples.resize( 0 );
			for( int iy = support.min.y; iy < support.max.y; ++iy )
			{
				for( int ix = support.min.x; ix < support.max.x; ++ix )
				{
					const float *pixelChannelSamples;
					unsigned int unusedCount;
					deepChannelSampler.sample( ix, iy, pixelChannelSamples, unusedCount );
					channelSamples.push_back( pixelChannelSamples );
				}
			}

			for( int i = prevOffset; i < offset; i++ )
			{
				float combinedChannelValue = 0;
				for( int j = 0; j < contributionCounts[i]; j++ )
				{
					float contributionChannelValue = channelSamples[contributionPixelIndices[contributionIndex]]
						[ contributionSampleIndices[contributionIndex] ];
					combinedChannelValue += contributionWeights[contributionIndex] * contributionChannelValue;
					contributionIndex++;
				}
				result.push_back( combinedChannelValue );
			}

			prevOffset = offset;
		}

		return resultData;
	}

	Sampler sampler(
		passes == Vertical ? horizontalPassPlug() : inPlug(),
		channelName,
		ir,
		boundingMode
	);

	const V2f filterRadius = inputFilterRadius( filter, inputFilterScale );

	FloatVectorDataPtr resultData = new FloatVectorData;
	std::vector<float> &result = resultData->writable();
	result.resize( ImagePlug::tileSize() * ImagePlug::tileSize() );
	std::vector<float>::iterator pIt = result.begin();

	if( !filter )
	{
		std::vector<int> iPx;
		nearestInputPixelX( tileBound.min.x, ratio.x, offset.x, iPx );

		V2i oP; // output pixel position
		int iPy; // input pixel position Y

		for( oP.y = tileBound.min.y; oP.y < tileBound.max.y; ++oP.y )
		{
			iPy = floorf( ( oP.y + 0.5 ) / ratio.y + offset.y );
			std::vector<int>::const_iterator iPxIt = iPx.begin();

			Canceller::check( context->canceller() );
			for( oP.x = tileBound.min.x; oP.x < tileBound.max.x; ++oP.x )
			{
				*pIt = sampler.sample( *iPxIt, iPy );
				++iPxIt;
				++pIt;
			}
		}
	}
	else if( passes == Both )
	{
		// When the filter isn't separable we must perform all the
		// filtering in a single pass. This version also provides
		// a reference implementation against which the two-pass
		// version can be validated - use the SinglePass debug mode
		// to force the use of this code path.

		V2i oP; // output pixel position
		V2f iP; // input pixel position (floating point)

		V2f	filterCoordinateMult = V2f(1.0f) / inputFilterScale;

		for( oP.y = tileBound.min.y; oP.y < tileBound.max.y; ++oP.y )
		{
			iP.y = ( oP.y + 0.5 ) / ratio.y + offset.y;
			int minY = ceilf( iP.y - 0.5f - filterRadius.y );
			int maxY = floorf( iP.y + 0.5f + filterRadius.y );

			for( oP.x = tileBound.min.x; oP.x < tileBound.max.x; ++oP.x )
			{
				Canceller::check( context->canceller() );

				iP.x = ( oP.x + 0.5 ) / ratio.x + offset.x;

				int minX = ceilf( iP.x - 0.5f - filterRadius.x );
				int maxX = floorf( iP.x + 0.5f + filterRadius.x );

				float v = 0.0f;
				float totalW = 0.0f;
				sampler.visitPixels(
					Imath::Box2i( Imath::V2i( minX, minY ), Imath::V2i( maxX, maxY ) ),
					[&filter, &filterCoordinateMult, &iP, &v, &totalW]( float cur, int x, int y )
					{
						const float w = (*filter)(
							filterCoordinateMult.x * ( float(x) + 0.5f - iP.x ),
							filterCoordinateMult.y * ( float(y) + 0.5f - iP.y )
						);

						v += w * cur;
						totalW += w;
					}
				);

				if( totalW != 0.0f )
				{
					*pIt = v / totalW;
				}

				++pIt;
			}
		}
	}
	else if( passes == BothOptimized )
	{
		Box2i support;
		std::vector<float> weights;
		filterWeights2D( filter, inputFilterScale, filterRadius, tileBound.min, V2f( 1 ), offset, support, weights );

		V2i oP; // output pixel position
		V2i supportOffset;
		for( oP.y = tileBound.min.y; oP.y < tileBound.max.y; ++oP.y )
		{
			supportOffset.y = oP.y - tileBound.min.y;

			for( oP.x = tileBound.min.x; oP.x < tileBound.max.x; ++oP.x )
			{
				Canceller::check( context->canceller() );

				supportOffset.x = oP.x - tileBound.min.x;
				std::vector<float>::const_iterator wIt = weights.begin();

				float v = 0.0f;
				float totalW = 0.0f;
				sampler.visitPixels(
					Imath::Box2i( support.min + supportOffset, support.max + supportOffset ),
					[&wIt, &v, &totalW]( float cur, int x, int y )
					{
						const float w = *wIt++;
						v += w * cur;
						totalW += w;
					}
				);

				if( totalW != 0.0f )
				{
					*pIt = v / totalW;
				}

				++pIt;
			}
		}

	}
	else if( passes == Horizontal )
	{
		// When the filter is separable we can perform filtering in two
		// passes, one for the horizontal and one for the vertical. We
		// output the horizontal pass on the horizontalPassPlug() so that
		// it is cached for use in the vertical pass. The HorizontalPass
		// debug mode causes this pass to be output directly for inspection.

		// Pixels in the same column share the same support ranges and filter weights, so
		// we precompute the weights now to avoid repeating work later.
		std::vector<int> supportRanges;
		std::vector<float> weights;
		filterWeights1D( filter, inputFilterScale.x, filterRadius.x, tileBound.min.x, ratio.x, offset.x, Horizontal, supportRanges, weights );

		V2i oP; // output pixel position

		for( oP.y = tileBound.min.y; oP.y < tileBound.max.y; ++oP.y )
		{
			Canceller::check( context->canceller() );

			std::vector<int>::const_iterator supportIt = supportRanges.begin();
			std::vector<float>::const_iterator wIt = weights.begin();
			for( oP.x = tileBound.min.x; oP.x < tileBound.max.x; ++oP.x )
			{
				float v = 0.0f;
				float totalW = 0.0f;

				sampler.visitPixels( Imath::Box2i(
						Imath::V2i( *supportIt, oP.y ),
						Imath::V2i( *( supportIt + 1 ), oP.y + 1 )
					),
					[&wIt, &v, &totalW]( float cur, int x, int y )
					{
						const float w = *wIt++;
						v += w * cur;
						totalW += w;
					}
				);

				supportIt += 2;

				if( totalW != 0.0f )
				{
					*pIt = v / totalW;
				}

				++pIt;
			}
		}
	}
	else if( passes == Vertical )
	{
		V2i oP; // output pixel position

		// Pixels in the same row share the same support ranges and filter weights, so
		// we precompute the weights now to avoid repeating work later.
		std::vector<int> supportRanges;
		std::vector<float> weights;
		filterWeights1D( filter, inputFilterScale.y, filterRadius.y, tileBound.min.y, ratio.y, offset.y, Vertical, supportRanges, weights );

		std::vector<int>::const_iterator supportIt = supportRanges.begin();
		std::vector<float>::const_iterator rowWeightsIt = weights.begin();

		for( oP.y = tileBound.min.y; oP.y < tileBound.max.y; ++oP.y )
		{
			Canceller::check( context->canceller() );

			for( oP.x = tileBound.min.x; oP.x < tileBound.max.x; ++oP.x )
			{
				float v = 0.0f;
				float totalW = 0.0f;

				std::vector<float>::const_iterator wIt = rowWeightsIt;

				sampler.visitPixels( Imath::Box2i(
						Imath::V2i( oP.x, *supportIt ),
						Imath::V2i( oP.x + 1, *(supportIt + 1) )
					),
					[&wIt, &v, &totalW]( float cur, int x, int y )
					{
						const float w = *wIt++;
						v += w * cur;
						totalW += w;
					}
				);


				if( totalW != 0.0f )
				{
					*pIt = v / totalW;
				}

				++pIt;
			}

			rowWeightsIt += (*(supportIt + 1)) - (*supportIt);
			supportIt += 2;
		}
	}

	return resultData;
}

void Resample::hashSampleOffsets( const GafferImage::ImagePlug *parent, const Gaffer::Context *context, IECore::MurmurHash &h ) const
{
	if( !inPlug()->deep() )
	{
		h = ImagePlug::flatTileSampleOffsets()->Object::hash();
	}

	ImageProcessor::hashSampleOffsets( parent, context, h );

	bool nearest = false;
	V2f ratio, offset;
	V2f filterScale;
	Sampler::BoundingMode boundingMode;
	{
		ImagePlug::GlobalScope c( context );
		nearest = filterPlug()->getValue() == g_nearestString;
	
		if( nearest )	
		{
			ratioAndOffset( matrixPlug()->getValue(), ratio, offset );
			boundingMode = (Sampler::BoundingMode)boundingModePlug()->getValue();
		}
	}

	if( !nearest )
	{
		deepResizeDataPlug()->hash( h );
		return;
	}

	h.append( ratio );
	h.append( offset );
	h.append( boundingMode );

	const V2i tileOrigin = context->get<V2i>( ImagePlug::tileOriginContextName );
	Box2i ir = inputRegion( tileOrigin, Both, ratio, offset, nullptr, V2f( 0.0f ) );
	DeepTileAccessor( inPlug(), "", ir, boundingMode ).hash( h );
}

IECore::ConstIntVectorDataPtr Resample::computeSampleOffsets( const Imath::V2i &tileOrigin, const Gaffer::Context *context, const ImagePlug *parent ) const
{
	if( !inPlug()->deep() )
	{
		return ImagePlug::flatTileSampleOffsets();
	}

	bool nearest = false;
	V2f ratio, offset;
	V2f filterScale;
	Sampler::BoundingMode boundingMode;
	{
		ImagePlug::GlobalScope c( context );
		nearest = filterPlug()->getValue() == g_nearestString;
	
		if( nearest )	
		{
			ratioAndOffset( matrixPlug()->getValue(), ratio, offset );
			boundingMode = (Sampler::BoundingMode)boundingModePlug()->getValue();
		}
	}

	if( !nearest )
	{
		ConstCompoundObjectPtr deepResizeData = deepResizeDataPlug()->getValue();
		return deepResizeData->member<IntVectorData>( g_sampleOffsetsName );
	}

	Box2i ir = inputRegion( tileOrigin, Both, ratio, offset, nullptr, V2f( 0.0f ) );
	DeepTileAccessor sampleOffsetsSampler( inPlug(), "", ir, boundingMode );

	IntVectorDataPtr outputSampleOffsetsData = new IntVectorData();
	std::vector<int> &outputSampleOffsets = outputSampleOffsetsData->writable();
	outputSampleOffsets.reserve( ImagePlug::tilePixels() );

	const Box2i tileBound( tileOrigin, tileOrigin + V2i( ImagePlug::tileSize() ) );
	
	std::vector<int> iPx;
	nearestInputPixelX( tileBound.min.x, ratio.x, offset.x, iPx );

	V2i oP; // output pixel position
	int iPy; // input pixel position Y

	int sampleOffset = 0;	
	for( oP.y = tileBound.min.y; oP.y < tileBound.max.y; ++oP.y )
	{
		iPy = floorf( ( oP.y + 0.5 ) / ratio.y + offset.y );
		std::vector<int>::const_iterator iPxIt = iPx.begin();

		Canceller::check( context->canceller() );
		for( oP.x = tileBound.min.x; oP.x < tileBound.max.x; ++oP.x )
		{
			const float *unused;
			unsigned int count;
			sampleOffsetsSampler.sample( *iPxIt, iPy, unused, count );
			sampleOffset += count;
			++iPxIt;
			outputSampleOffsets.push_back( sampleOffset );
		}
	}

	return outputSampleOffsetsData;
}
