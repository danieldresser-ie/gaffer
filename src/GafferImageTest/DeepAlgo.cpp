//////////////////////////////////////////////////////////////////////////
//
//  Copyright (c) 2020, Image Engine Design Inc. All rights reserved.
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

#include "GafferImageTest/DeepAlgo.h"

#include "IECore/VectorTypedData.h"
#include "IECore/SimpleTypedData.h"

using namespace GafferImageTest;

namespace
{

void evaluateDeepPixelInternal( const std::vector<float> &z, const std::vector<float> &zBack, const std::vector<float> &a, const std::vector<const float *> &channels, float depth, std::vector<float> &result )
{
    float accumAlpha = 0;

    result.resize( channels.size() + 1 );
    std::fill( result.begin(), result.end(), 0);

    for( unsigned int i = 0; i < z.size(); i++ )
    {
        if( depth < z[i] )
        {
            break;
        }

        if( depth >= zBack[i] )
        {
            for( unsigned int j = 0; j < channels.size(); j++ )
            {
                result[j + 1] += ( 1 - accumAlpha ) * channels[j][i];
            }
            accumAlpha += a[i] - a[i] * accumAlpha;
        }
        else
        {
			float fraction = ( depth - z[i] ) / ( zBack[i] - z[i] );
			float curAlpha;
			float weight;
			if( a[i] > 1e-8 )
			{
				curAlpha = -expm1( fraction * log1p( -a[i] ) );
				weight = curAlpha / a[i];
			}
			else
			{
				curAlpha = 0;
				weight = fraction;
			}

            for( unsigned int j = 0; j < channels.size(); j++ )
            {
                result[j + 1] += ( 1 - accumAlpha ) * weight * channels[j][i];
            }

            accumAlpha += curAlpha - curAlpha * accumAlpha;

            break;
        }
    }

    result[0] = accumAlpha;
}

} // namespace

IECore::CompoundDataPtr GafferImageTest::evaluateDeepPixel( const IECore::CompoundData* pixelData, float depth )
{
	IECore::CompoundDataPtr resultData = new IECore::CompoundData();

	const IECore::FloatVectorData *z = pixelData->member<IECore::FloatVectorData>( "Z" );
	if( !z )
	{
		return resultData;
	}

	const IECore::FloatVectorData *zBack = pixelData->member<IECore::FloatVectorData>( "ZBack" );
	if( !zBack )
	{
		zBack = z;
	}

	const IECore::FloatVectorData *a = pixelData->member<IECore::FloatVectorData>( "A" );
	if( !a )
	{
		IECore::FloatVectorDataPtr zeroAlpha = new IECore::FloatVectorData();
		zeroAlpha->writable().resize( z->readable().size(), 0.0f );
		a = zeroAlpha.get();
	}

	std::vector<const float *> channels;
	std::vector<std::string> channelNames;
	for( const auto &entry : pixelData->readable() )
	{
		if( entry.first == "A" || entry.first == "Z" || entry.first == "ZBack" )
		{
			continue;
		}
		channelNames.push_back( entry.first );

		const IECore::FloatVectorData *data = IECore::runTimeCast< IECore::FloatVectorData >( entry.second.get() );
		channels.push_back( &( data->readable()[0] ) );
	}

	std::vector<float> resultVector;
	evaluateDeepPixelInternal( z->readable(), zBack->readable(), a->readable(), channels, depth, resultVector );

	resultData->writable()["A"] = new IECore::FloatData( resultVector[0] );
	for( unsigned int i = 0; i < channels.size(); i++ )
	{
		resultData->writable()[ channelNames[i] ] = new IECore::FloatData( resultVector[i + 1] );
	}
	return resultData;
}

void GafferImageTest::assertDeepPixelsEvaluateSame( const IECore::CompoundData* pixelDataA, const IECore::CompoundData* pixelDataB, float depthTolerance, float alphaTolerance, float channelTolerance, const std::string &message )
{
	std::set<std::string> channelNamesA, channelNamesB;
	for( const auto &entry : pixelDataA->readable() )
	{
		if( entry.first != "A" && entry.first != "Z" && entry.first != "ZBack" )
		{
			channelNamesA.insert( entry.first );
		}
	}
	for( const auto &entry : pixelDataA->readable() )
	{
		if( entry.first != "A" && entry.first != "Z" && entry.first != "ZBack" )
		{
			channelNamesB.insert( entry.first );
		}
	}

	if( channelNamesA != channelNamesB )
	{
		for( const std::string &name : channelNamesA )
		{
			if( channelNamesB.find( name ) == channelNamesB.end() )
			{
				throw IECore::Exception( message + "Channel \"" + name + " in pixel A but not B" );
			}
		}
		for( const std::string &name : channelNamesB )
		{
			if( channelNamesA.find( name ) == channelNamesA.end() )
			{
				throw IECore::Exception( message + "Channel \"" + name + " in pixel B but not A" );
			}
		}
	}
	std::vector<std::string> channelNames( channelNamesA.begin(), channelNamesA.end() );

	const IECore::FloatVectorData *zA = pixelDataA->member<IECore::FloatVectorData>( "Z" );
	const IECore::FloatVectorData *zB = pixelDataB->member<IECore::FloatVectorData>( "Z" );
	if( !zA || !zB)
	{
		if( pixelDataA->readable().size() == 0 && pixelDataB->readable().size() == 0 )
		{
			// Pixels with no samples are fine, as long they are both empty
			return;
		}
		throw IECore::Exception( message + "No Z Channel, cannot evaluate pixels." );
	}

	const IECore::FloatVectorData *zBackA = pixelDataA->member<IECore::FloatVectorData>( "ZBack" );
	const IECore::FloatVectorData *zBackB = pixelDataB->member<IECore::FloatVectorData>( "ZBack" );
	if( !zBackA )
	{
		zBackA = zA;
	}
	if( !zBackB )
	{
		zBackB = zB;
	}

	const IECore::FloatVectorData *alphaA = pixelDataA->member<IECore::FloatVectorData>( "A" );
	if( !alphaA )
	{
		IECore::FloatVectorDataPtr zeroAlpha = new IECore::FloatVectorData();
		zeroAlpha->writable().resize( zA->readable().size(), 0.0f );
		alphaA = zeroAlpha.get();
	}
	const IECore::FloatVectorData *alphaB = pixelDataB->member<IECore::FloatVectorData>( "A" );
	if( !alphaB )
	{
		IECore::FloatVectorDataPtr zeroAlpha = new IECore::FloatVectorData();
		zeroAlpha->writable().resize( zB->readable().size(), 0.0f );
		alphaB = zeroAlpha.get();
	}

	std::vector<const float *> channelsA, channelsB;
	for( const std::string &name : channelNames )
	{
		channelsA.push_back( &pixelDataA->member<IECore::FloatVectorData>( name )->readable()[0] );
		channelsB.push_back( &pixelDataB->member<IECore::FloatVectorData>( name )->readable()[0] );
	}

	std::vector<float> resultA;
	std::vector<float> resultBUpper;
	std::vector<float> resultBLower;
	for( const IECore::FloatVectorData *depthData : { zA, zB, zBackA, zBackB } )
	{
		for( float depth : depthData->readable() )
		{
			evaluateDeepPixelInternal( zA->readable(), zBackA->readable(), alphaA->readable(), channelsA, depth, resultA );
			evaluateDeepPixelInternal( zB->readable(), zBackB->readable(), alphaB->readable(), channelsB, depth * ( 1 + depthTolerance ), resultBUpper );
			evaluateDeepPixelInternal( zB->readable(), zBackB->readable(), alphaB->readable(), channelsB, depth * ( 1 - depthTolerance ), resultBLower );

			for( unsigned int j = 0; j < resultA.size(); j++ )
			{
				float tol = j == 0 ? alphaTolerance : channelTolerance;
			
				bool fail = false;
				float compare;

				if( resultA[j] - resultBUpper[j] > tol )
				{
					fail = true;
					compare = resultBUpper[j];
				}
				if( resultA[j] - resultBLower[j] < -tol )
				{
					fail = true;
					compare = resultBLower[j];
				}

				if( fail )
				{
					std::string channelName = j == 0 ? "A" : channelNames[j - 1];
					throw IECore::Exception(
						message + "Mismatch in channel " + channelName + " at depth " + std::to_string( depth ) +
						" : " + std::to_string( resultA[j] ) + " vs " + std::to_string( compare ) +
						" not within " + std::to_string( tol ) + "\n"
					);
				}
			}
		}
	}
}

