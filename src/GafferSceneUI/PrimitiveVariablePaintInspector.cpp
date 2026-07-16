//////////////////////////////////////////////////////////////////////////
//
//  Copyright (c) 2026, Image Engine Design Inc. All rights reserved.
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

#include "GafferSceneUI/Private/PrimitiveVariablePaintInspector.h"

#include "GafferScene/EditScopeAlgo.h"
#include "GafferScene/PrimitiveVariablePaint.h"

#include "Gaffer/DataStore.h"

#include "IECore/DataAlgo.h"

using namespace IECore;
using namespace IECoreScene;
using namespace Gaffer;
using namespace GafferScene;
using namespace GafferSceneUI::Private;

namespace
{

BoolPlugPtr g_dummyPlug = new BoolPlug();

void edit( DataStore *target, const ScenePlug::ScenePath &scenePath, const IECore::InternedString &primitiveVariable, const IECore::Object *value )
{
	IECore::InternedString pathString( ScenePlug::pathToString( scenePath ) );

	ConstCompoundObjectPtr sourceData = IECore::runTimeCast<const CompoundObject>( target->getEntry( pathString, false ) );

	CompoundObjectPtr locPaintData;
	if( sourceData )
	{
		locPaintData = sourceData->copy();
	}
	else
	{
		locPaintData = new CompoundObject();
	}

	if( !value )
	{
		throw IECore::Exception( "Trying to create with no value" );

	}

	locPaintData->members()[primitiveVariable] = const_cast<IECore::Object*>( value );

	target->setEntry( pathString, locPaintData );
}

bool canEditPaint( const Gaffer::ValuePlug *plug, const IECore::Object *value, std::string &failureReason )
{
	if( !plug )
	{
		failureReason = "Could not find plug";
		return false;
	}

	const DataStore *dataStore = IECore::runTimeCast<const DataStore>( plug->node() );
	if( dataStore && plug == dataStore->enabledPlug() )
	{
		return true;
	}

	if( !plug->parent() )
	{
		// Weird workaround to handle some weirdness from inspector::Result::canEdit - if there is no
		// source, but there is an editScope we can create an edit, this is represented by a temporary
		// dummy that isn't connected to anything
		return true;
	}

	failureReason = fmt::format( "Plug is not on DataStore driving PrimitiveVariablePaint : \"{}\"", plug->fullName() );

    return false;
}

} // namespace

//////////////////////////////////////////////////////////////////////////
// PrimitiveVariablePaintInspector
//////////////////////////////////////////////////////////////////////////

IE_CORE_DEFINERUNTIMETYPED( PrimitiveVariablePaintInspector )

PrimitiveVariablePaintInspector::PrimitiveVariablePaintInspector(
	const GafferScene::ScenePlugPtr &scene,
	const Gaffer::PlugPtr &editScope,
	IECore::InternedString primitiveVariable,
	const std::string &name
)
	:	PrimitiveVariableInspector( scene, editScope, primitiveVariable, Property::Data, name )
{
}

Gaffer::ValuePlugPtr PrimitiveVariablePaintInspector::source( const GafferScene::SceneAlgo::History *history, std::string &editWarning ) const
{
	auto sceneNode = runTimeCast<SceneNode>( history->scene->node() );
    if( !sceneNode || history->scene != sceneNode->outPlug() )
    {
        return nullptr;
    }
	else if( auto primitiveVariablePaintNode = runTimeCast<GafferScene::PrimitiveVariablePaint>( sceneNode ) )
	{
		for( NameValuePlug::Iterator it( primitiveVariablePaintNode->primitiveVariablesPlug() ); !it.done(); ++it )
		{
			NameValuePlug *primVarPlug = it->get();
			bool active = true;
			if( auto enabledPlug = primVarPlug->enabledPlug() )
			{
				active = enabledPlug->getValue();
			}

			if( ( !active ) || primVarPlug->namePlug()->getValue() != m_primitiveVariable.string() )
			{
				continue;
			}

			Plug *dataStoreOutput = primVarPlug->valuePlug()->getInput();
			if( dataStoreOutput )
			{
				DataStore *dataStore = IECore::runTimeCast<DataStore>( dataStoreOutput->node() );
				if( dataStore )
				{
					return dataStore->enabledPlug();
				}
			}
		}
	}

	return PrimitiveVariableInspector::source( history, editWarning );
}

Inspector::AcquireEditFunctionOrFailure PrimitiveVariablePaintInspector::acquireEditFunction( Gaffer::EditScope *editScope, const GafferScene::SceneAlgo::History *history ) const
{
	return [] ( bool createIfNecessary ) {
		return g_dummyPlug.get();
	};
}

Inspector::CanEditFunction PrimitiveVariablePaintInspector::canEditFunction( const GafferScene::SceneAlgo::History *history ) const
{
	return [] ( const Gaffer::ValuePlug *plug, const IECore::Object *value, std::string &failureReason ) { return canEditPaint( plug, value, failureReason ); };
}

Inspector::EditFunction PrimitiveVariablePaintInspector::editFunction( const GafferScene::SceneAlgo::History *history ) const
{
	const ScenePlug::ScenePath &scenePath = history->context->get<ScenePlug::ScenePath>( ScenePlug::scenePathContextName );
    return [scenePath, primitiveVariable = this->m_primitiveVariable] ( Gaffer::ValuePlug *plug, const IECore::Object *value ) {
        ::edit( IECore::runTimeCast<DataStore>( plug->node() ), scenePath, primitiveVariable, value );
    };
}
