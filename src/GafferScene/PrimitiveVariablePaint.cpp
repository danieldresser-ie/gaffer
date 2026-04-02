//////////////////////////////////////////////////////////////////////////
//
//  Copyright (c) 2024, Image Engine Design Inc. All rights reserved.
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

#include "GafferScene/PrimitiveVariablePaint.h"

#include "IECoreScene/Primitive.h"

#include "IECore/DataAlgo.h"

#include "IECore/TypeTraits.h"
#include <unordered_set>

using namespace IECore;
using namespace IECoreScene;
using namespace Gaffer;
using namespace GafferScene;

GAFFER_NODE_DEFINE_TYPE( PrimitiveVariablePaint );

PaintOperation::PaintOperation()
{
}

PaintOperation::~PaintOperation()
{
}

bool PaintOperation::isEqualTo( const IECore::Object *other ) const
{
    if( !Object::isEqualTo( other ) )
    {
        return false;
    }

    const PaintOperation *operation = static_cast<const PaintOperation *>( other );
	if( m_valueData || operation->m_valueData )
	{
		if( !( m_valueData && operation->m_valueData && m_valueData->isEqualTo( operation->m_valueData.get() ) ) )
		{
			return false;
		}
	}

	if( m_opacityData || operation->m_opacityData )
	{
		if( !( m_opacityData && operation->m_opacityData && m_opacityData->isEqualTo( operation->m_opacityData.get() ) ) )
		{
			return false;
		}
	}

	if( m_indicesData || operation->m_indicesData )
	{
		if( !( m_indicesData && operation->m_indicesData && m_indicesData->isEqualTo( operation->m_indicesData.get() ) ) )
		{
			return false;
		}
	}

	return true;
}

void PaintOperation::hash( IECore::MurmurHash &h ) const
{
    Object::hash( h );
	if( m_valueData )
	{
		m_valueData->hash( h );
	}

	if( m_opacityData )
	{
		m_opacityData->hash( h );
	}

	if( m_indicesData )
	{
		m_indicesData->hash( h );
	}
}

void PaintOperation::copyFrom( const IECore::Object *other, IECore::Object::CopyContext *context )
{
    Object::copyFrom( other, context );

    const PaintOperation *operation = static_cast<const PaintOperation *>( other );
    m_valueData = operation->m_valueData;
    m_opacityData = operation->m_opacityData;
    m_indicesData = operation->m_indicesData;
}

void PaintOperation::save( IECore::Object::SaveContext *context ) const
{
    Object::save( context );
    /// \todo Can we implement saving by serialising the
    /// Gaffer script into the IndexedIO file?
    msg( Msg::Warning, "PaintOperation::save", "Not implemented" );
}

void PaintOperation::load( IECore::Object::LoadContextPtr context )
{
    Object::load( context );
    msg( Msg::Warning, "PaintOperation::load", "Not implemented" );
}

void PaintOperation::memoryUsage( IECore::Object::MemoryAccumulator &accumulator ) const
{
    Object::memoryUsage( accumulator );
	if( m_valueData )
	{
		accumulator.accumulate( m_valueData.get() );
	}
	if( m_opacityData )
	{
		accumulator.accumulate( m_opacityData.get() );
	}
	if( m_indicesData )
	{
		accumulator.accumulate( m_indicesData.get() );
	}
}

//IE_CORE_DEFINERUNTIMETYPED( PrimitiveVariablePaint::OperationData );

//IE_CORE_DEFINERUNTIMETYPED( PaintOperation );
//GAFFER_NODE_DEFINE_TYPE( PaintOperation );
IE_CORE_DEFINEOBJECTTYPEDESCRIPTION( PaintOperation );

size_t PrimitiveVariablePaint::g_firstPlugIndex = 0;

PrimitiveVariablePaint::PrimitiveVariablePaint( const std::string &name )
	:	Deformer( name )
{
	storeIndexOfNextChild( g_firstPlugIndex );

	addChild( new CompoundObjectPlug( "paint", Plug::In ) );
}

PrimitiveVariablePaint::~PrimitiveVariablePaint()
{
}

Gaffer::CompoundObjectPlug *PrimitiveVariablePaint::paintPlug()
{
	return getChild<Gaffer::CompoundObjectPlug>( g_firstPlugIndex + 0 );
}

const Gaffer::CompoundObjectPlug *PrimitiveVariablePaint::paintPlug() const
{
	return getChild<Gaffer::CompoundObjectPlug>( g_firstPlugIndex + 0 );
}

bool PrimitiveVariablePaint::affectsProcessedObject( const Gaffer::Plug *input ) const
{
	return
		Deformer::affectsProcessedObject( input ) ||
		input == paintPlug()
	;
}

void PrimitiveVariablePaint::hashProcessedObject( const ScenePath &path, const Gaffer::Context *context, IECore::MurmurHash &h ) const
{
	Deformer::hashProcessedObject( path, context, h );
	// TODO - not really a fan of any approach to this function ... we can either modify the hash for
	// every location in the scene, or we can evaluate the paint plug and do string munging in the hash ...
	// neither seems great.
	//
	// Should we be computing a filter from the stored data?
	/*ConstCompoundDataPtr paint = paintPlug()->getValue();

	std::string pathString = ScenePlug::pathToString( path );

	const CompoundData *locPaint = paint->member<CompoundData>( pathString );

	if( locPaint )
	{
		Deformer::hashProcessedObject( path, context, h );
		locPaint->hash( h );
	}
	else
	{
		h = inPlug()->objectPlug()->hash();
	}*/
	paintPlug()->hash( h );
}

IECore::ConstObjectPtr PrimitiveVariablePaint::computeProcessedObject( const ScenePath &path, const Gaffer::Context *context, const IECore::Object *inputObject ) const
{
	const Primitive *inputPrimitive = runTimeCast<const Primitive>( inputObject );
	if( !inputPrimitive )
	{
		return inputObject;
	}

	if( !inputPrimitive->arePrimitiveVariablesValid() )
	{
		throw IECore::Exception( "Cannot paint primitive with invalid primitive variables" );
	}

	ConstCompoundObjectPtr locPaint = paintPlug()->getValue();

	// TODO - now only needed for errors
	std::string pathString = ScenePlug::pathToString( path );

	PrimitivePtr result = inputPrimitive->copy();

	//static const InternedString interpolationString( "interpolation" );
	for( auto &var : locPaint->members() )
	{
		const PaintOperation *varCompound = IECore::runTimeCast<PaintOperation>( var.second.get() );

		if( !varCompound )
		{
			throw IECore::Exception( fmt::format( "Invalid paint for variable {} with no PaintOperation for key {}", var.first.string(), pathString ) );
		}

		if( !varCompound->m_valueData )
		{
			throw IECore::Exception( fmt::format( "Invalid paint for variable {} with no value at location {}", var.first.string(), pathString ) );
		}


		// TODO - restore support for interpolation
		/*const IntData *interpolationData = varCompound->member<IntData>( interpolationString );

		PrimitiveVariable::Interpolation interp =
			interpolationData ?
			(PrimitiveVariable::Interpolation) interpolationData->readable() :
			PrimitiveVariable::Vertex;
		*/
		PrimitiveVariable::Interpolation interp = PrimitiveVariable::Vertex;

		std::cerr << "TEST TYPE " << varCompound->m_valueData->typeName() << "\n";
		if( IECore::size( varCompound->m_valueData.get() ) != result->variableSize( interp ) )
		{
			// TODO - should we support some sort of reprojection for loading out of date paint? This
			// would require storing a reference P in the paint file
			throw IECore::Exception( fmt::format( "Invalid paint for variable {} at location {} size {} does not match {}", var.first.string(), pathString, IECore::size( varCompound->m_valueData.get() ), result->variableSize( interp ) ) );

		}

		auto existingVar = result->variables.find( var.first );
		if( existingVar != result->variables.end() && existingVar->second.interpolation != interp && varCompound->m_opacityData )
		{
			IECore::msg( IECore::Msg::Warning, "PrimitiveVariablePaint", fmt::format( "Interpolation mismatch for variable {} at location {}, overwriting instead of compositing.", var.first.string(), pathString ) );
			existingVar = result->variables.end();

		}

		// TODO operation
		if( existingVar == result->variables.end() || !varCompound->m_opacityData )
		{
			// TODO - I think this const_cast is safe because the result is treated as const
			result->variables[var.first] = PrimitiveVariable( interp, const_cast<Data*>( varCompound->m_valueData.get() ) );
			continue;
		}

		const std::vector<float> &paintOpacity = varCompound->m_opacityData->readable();
		if( paintOpacity.size() != IECore::size( varCompound->m_valueData.get() ) )
		{
			throw IECore::Exception( "Corrupt paint : Opacity size different from value size." );
		}

		IECore::dispatch( varCompound->m_valueData.get(),
			[&var, &existingVar, &paintOpacity, &result]( auto *typedValueData )
			{
				using SourceType = typename std::remove_const_t< std::remove_pointer_t<decltype( typedValueData )> >;

				// This check should be unnecessary, but IsNumericBasedVectorTypedData fails to compile for
				// non-vector types.
				if constexpr( TypeTraits::IsVectorTypedData< SourceType >::value )
				{
					if constexpr( std::is_same_v< typename SourceType::BaseType, float > )
					//if constexpr( TypeTraits::IsNumericBasedVectorTypedData< SourceType >::value )
					{

                        using ValueType = typename SourceType::ValueType::value_type;

                        // There a few types that are numeric based, but don't support weighting
                        //if constexpr( std::is_same_v< ValueType::BaseType, float > )
                        //if constexpr( !TypeTraits::IsMatrix<ValueType>::value && !TypeTraits::IsQuat<ValueType>::value && !TypeTraits::IsBox<ValueType>::value )
                        //if constexpr( !TypeTraits::IsQuat<ValueType>::value && !TypeTraits::IsBox<ValueType>::value )
                        //if constexpr( !TypeTraits::IsBox<ValueType>::value && !TypeTraits::IsMatrix<ValueType>::value )

                        if constexpr( !TypeTraits::IsBox<ValueType>::value )
						{
							typename SourceType::Ptr resultData = IECore::runTimeCast<SourceType>( existingVar->second.expandedData() );
							auto &resultVec = resultData->writable();
							auto &typedValue = typedValueData->readable();

							for( size_t i = 0; i < resultVec.size(); i++ )
							{
								resultVec[i] = ( 1 - paintOpacity[i] ) * resultVec[i] + typedValue[i];
							}

							result->variables[var.first] = PrimitiveVariable( existingVar->second.interpolation, resultData );
							return;
						}
					}
				}

				throw IECore::Exception( fmt::format(
					"PrimitiveVariablePaint : Cannot apply type \"{}\"", typedValueData->typeName()
				) );
			}
		);

	}

	/*

	*/

	return result;
}

bool PrimitiveVariablePaint::adjustBounds() const
{
	if( !Deformer::adjustBounds() )
	{
		return false;
	}

	// TODO - if we want to support painting P, we would need to query every possible input
	// to see if there is paint on P?
	/*static const InternedString pString( "P" );

	ConstCompoundDataPtr paint = paintPlug()->getValue();
	for( const auto &loc : paint->readable() )
	{
		const CompoundData *locCompound = IECore::runTimeCast<CompoundData>( loc.second.get() );
		if( locCompound->member<Data>( pString ) )
		{
			return true;
		}
	}*/

	return false;
}
