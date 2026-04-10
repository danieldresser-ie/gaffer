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

#include "GafferSceneUI/Private/PrimitiveVariableInspector.h"

#include "GafferScene/PrimitiveVariables.h"
#include "GafferScene/PrimitiveVariableTweaks.h"
#include "GafferScene/Camera.h"
#include "GafferScene/EditScopeAlgo.h"
#include "GafferScene/Light.h"
#include "GafferScene/LightFilter.h"
#include "GafferScene/SceneAlgo.h"
#include "GafferScene/SceneNode.h"

#include "Gaffer/Private/IECorePreview/LRUCache.h"
#include "Gaffer/Metadata.h"
#include "Gaffer/MetadataAlgo.h"
#include "Gaffer/NameValuePlug.h"
#include "Gaffer/ScriptNode.h"
#include "Gaffer/ParallelAlgo.h"
#include "Gaffer/ValuePlug.h"

#include "IECore/DataAlgo.h"
#include "IECore/NullObject.h"

#include "fmt/format.h"

using namespace boost::placeholders;
using namespace IECore;
using namespace IECoreScene;
using namespace Gaffer;
using namespace GafferScene;
using namespace GafferSceneUI::Private;

//////////////////////////////////////////////////////////////////////////
// History cache
//////////////////////////////////////////////////////////////////////////

namespace
{

const std::string g_attributePrefix( "attribute:" );
const InternedString g_defaultValue( "defaultValue" );

// This uses the same strategy that ValuePlug uses for the hash cache,
// using `plug->dirtyCount()` to invalidate previous cache entries when
// a plug is dirtied.
struct HistoryCacheKey
{
	HistoryCacheKey() {};
	HistoryCacheKey( const ValuePlug *plug )
		:	plug( plug ), contextHash( Context::current()->hash() ), dirtyCount( plug->dirtyCount() )
	{
	}

	bool operator==( const HistoryCacheKey &rhs ) const
	{
		return
			plug == rhs.plug &&
			contextHash == rhs.contextHash &&
			dirtyCount == rhs.dirtyCount
		;
	}

	const ValuePlug *plug;
	IECore::MurmurHash contextHash;
	uint64_t dirtyCount;
};

size_t hash_value( const HistoryCacheKey &key )
{
	size_t result = 0;
	boost::hash_combine( result, key.plug );
	boost::hash_combine( result, key.contextHash );
	boost::hash_combine( result, key.dirtyCount );
	return result;
}

using HistoryCache = IECorePreview::LRUCache<HistoryCacheKey, SceneAlgo::History::ConstPtr>;

HistoryCache g_historyCache(
	// Getter
	[] ( const HistoryCacheKey &key, size_t &cost, const IECore::Canceller *canceller ) {
		assert( canceller == Context::current()->canceller() );
		cost = 1;
		return SceneAlgo::history(
			key.plug, Context::current()->get<ScenePlug::ScenePath>( ScenePlug::scenePathContextName )
		);
	},
	// Max cost
	1000,
	// Removal callback
	[] ( const HistoryCacheKey &key, const SceneAlgo::History::ConstPtr &history ) {
		// Histories contain PlugPtrs, which could potentially be the sole
		// owners. Destroying plugs can trigger dirty propagation, so as a
		// precaution we destroy the history on the UI thread, where this would
		// be OK.
		ParallelAlgo::callOnUIThread(
			[history] () {}
		);
	}

);

struct PrimitiveVariableHistoryCacheKey : public HistoryCacheKey
{
	PrimitiveVariableHistoryCacheKey() {};
	PrimitiveVariableHistoryCacheKey( const ScenePlug *plug, IECore::InternedString primitiveVariable )
		:	HistoryCacheKey( plug->objectPlug() ), primitiveVariable( primitiveVariable )
	{
	}

	bool operator==( const PrimitiveVariableHistoryCacheKey &rhs ) const
	{
		return HistoryCacheKey::operator==( rhs ) && primitiveVariable == rhs.primitiveVariable;
	}

	IECore::InternedString primitiveVariable;
};

size_t hash_value( const PrimitiveVariableHistoryCacheKey &key )
{
	size_t result = 0;
	boost::hash_combine( result, static_cast<const HistoryCacheKey &>( key ) );
	boost::hash_combine( result, key.primitiveVariable.c_str() );
	return result;
}

using PrimitiveVariableHistoryCache = IECorePreview::LRUCache<PrimitiveVariableHistoryCacheKey, SceneAlgo::History::ConstPtr>;

PrimitiveVariableHistoryCache g_primitiveVariableHistoryCache(
	// Getter
	[] ( const PrimitiveVariableHistoryCacheKey &key, size_t &cost, const IECore::Canceller *canceller ) -> SceneAlgo::History::ConstPtr {
		assert( canceller == Context::current()->canceller() );
		cost = 1;
		SceneAlgo::History::ConstPtr primitiveVariablesHistory = g_historyCache.get( key, canceller );
		if( auto h = SceneAlgo::primitiveVariableHistory( primitiveVariablesHistory.get(), key.primitiveVariable ) )
		{
			return h;
		}
		else
		{
			// The specific primitive variable doesn't exist. But we return the history for the
			// whole CompoundObject so we get a chance to discover nodes that could
			// _create_ the attribute. TODO
			return primitiveVariablesHistory;
		}
	},
	// Max cost
	1000,
	// Removal callback
	[] ( const PrimitiveVariableHistoryCacheKey &key, const SceneAlgo::History::ConstPtr &history ) {
		// See comment in g_historyCache
		ParallelAlgo::callOnUIThread(
			[history] () {}
		);
	}

);

/*Gaffer::ValuePlugPtr TODOprimVarPlug( const Gaffer::CompoundDataPlug *parentPlug, const std::string &primitiveVariableName )
{
	for( const auto &plug : Gaffer::NameValuePlug::Range( *parentPlug ) )
	{
		if(plug->namePlug()->getValue() == primitiveVariableName )
		{
			return plug;
		}
	}
	return nullptr;
}*/

// TODO : Duplicated from src/GafferSceneUIModule/SceneInspectorBinding.cpp
const boost::container::flat_map<IECoreScene::PrimitiveVariable::Interpolation, IECore::ConstStringDataPtr> g_primitiveVariableInterpolations = {
    { PrimitiveVariable::Invalid, new IECore::StringData( "Invalid" ) },
    { PrimitiveVariable::Constant, new IECore::StringData( "Constant" ) },
    { PrimitiveVariable::Uniform, new IECore::StringData( "Uniform" ) },
    { PrimitiveVariable::Vertex, new IECore::StringData( "Vertex" ) },
    { PrimitiveVariable::Varying, new IECore::StringData( "Varying" ) },
    { PrimitiveVariable::FaceVarying, new IECore::StringData( "FaceVarying" ) }
};

const boost::container::flat_map<IECore::GeometricData::Interpretation, IECore::ConstStringDataPtr> g_geometricInterpretations = {
    { GeometricData::None, new IECore::StringData( "None" ) },
    { GeometricData::Point, new IECore::StringData( "Point" ) },
    { GeometricData::Normal, new IECore::StringData( "Normal" ) },
    { GeometricData::Vector, new IECore::StringData( "Vector" ) },
    { GeometricData::Color, new IECore::StringData( "Color" ) },
    { GeometricData::UV, new IECore::StringData( "UV" ) },
    { GeometricData::Rational, new IECore::StringData( "Rational" ) }
};

} // namespace

//////////////////////////////////////////////////////////////////////////
// PrimitiveVariableInspector
//////////////////////////////////////////////////////////////////////////

IE_CORE_DEFINERUNTIMETYPED( PrimitiveVariableInspector )

PrimitiveVariableInspector::PrimitiveVariableInspector(
	const GafferScene::ScenePlugPtr &scene,
	const Gaffer::PlugPtr &editScope,
	IECore::InternedString primitiveVariable,
	Property property,
	const std::string &name,
	const std::string &type
)
	// TODO - do we need globals?
	:	Inspector( { scene->objectPlug(), scene->globalsPlug() }, type, name == "" ? primitiveVariable.string() : name, editScope ),
		m_scene( scene ), m_primitiveVariable( primitiveVariable ), m_property( property )
{
}

GafferScene::SceneAlgo::History::ConstPtr PrimitiveVariableInspector::history() const
{
	if( !m_scene->existsPlug()->getValue() )
	{
		return nullptr;
	}

	return g_primitiveVariableHistoryCache.get( PrimitiveVariableHistoryCacheKey( m_scene.get(), m_primitiveVariable ), Context::current()->canceller() );
}

IECore::ConstObjectPtr PrimitiveVariableInspector::value( const GafferScene::SceneAlgo::History *history ) const
{
	auto primitiveVariableHistory = dynamic_cast<const SceneAlgo::PrimitiveVariableHistory *>( history );
	if( !primitiveVariableHistory )
	{
		// Primitive variable doesn't exist.
		return nullptr;
	}

	if( m_property == Property::Interpolation )
	{
		auto it = g_primitiveVariableInterpolations.find( primitiveVariableHistory->primitiveVariableValue.interpolation );
		return it != g_primitiveVariableInterpolations.end() ? it->second : nullptr;
	}
	else if( m_property == Property::Type )
    {
        return new StringData( primitiveVariableHistory->primitiveVariableValue.data->typeName() );
    }
	else if( m_property == Property::Interpretation )
	{
		auto it = g_geometricInterpretations.find( IECore::getGeometricInterpretation( primitiveVariableHistory->primitiveVariableValue.data.get() ) );
		return it != g_geometricInterpretations.end() ? it->second : nullptr;
	}
	else if( m_property == Property::Data )
	{
		return primitiveVariableHistory->primitiveVariableValue.data;
	}
	else if( m_property == Property::Indices )
	{
		return primitiveVariableHistory->primitiveVariableValue.indices;
	}

	throw IECore::Exception( fmt::format( "Unsupported primitive variable Property {}.", (int)m_property ) );
}

Gaffer::ValuePlugPtr PrimitiveVariableInspector::source( const GafferScene::SceneAlgo::History *history, std::string &editWarning ) const
{
	auto sceneNode = runTimeCast<SceneNode>( history->scene->node() );
	if( !sceneNode || history->scene != sceneNode->outPlug() )
	{
		return nullptr;
	}

	/*else if( auto camera = runTimeCast<GafferScene::Camera>( sceneNode ) )
	{
		return TODOprimVarPlug( camera->visualiserAttributesPlug(), m_attribute );
	}*/

	else if( auto primitiveVariablesNode = runTimeCast<GafferScene::PrimitiveVariables>( sceneNode ) )
	{
		if( !(primitiveVariablesNode->filterPlug()->match( primitiveVariablesNode->inPlug() ) & PathMatcher::ExactMatch ) )
		{
			return nullptr;
		}

		for( const auto &plug : NameValuePlug::Range( *primitiveVariablesNode->primitiveVariablesPlug() ) )
		{
			if(
				plug->namePlug()->getValue() == m_primitiveVariable.string() &&
				( !plug->enabledPlug() || plug->enabledPlug()->getValue() )
			)
			{
				/// \todo This is overly conservative. We should test to see if there is more than
				/// one filter match (but make sure to early-out once two are found, rather than test
				/// the rest of the scene).
				editWarning = fmt::format(
					"Edits to \"{}\" may affect other locations in the scene.",
					m_primitiveVariable.string()
				);
				return plug;
			}
		}
	}

	else if( auto primitiveVariableTweaks = runTimeCast<PrimitiveVariableTweaks>( sceneNode ) )
	{
		if( !( primitiveVariableTweaks->filterPlug()->match( primitiveVariableTweaks->inPlug() ) & PathMatcher::ExactMatch ) )
		{
			return nullptr;
		}

		for( const auto &tweak : TweakPlug::Range( *primitiveVariableTweaks->tweaksPlug() ) )
		{
			if(
				tweak->namePlug()->getValue() == m_primitiveVariable.string() &&
				tweak->enabledPlug()->getValue()
			)
			{
				return tweak;
			}
		}
	}

	return nullptr;
}

Inspector::AcquireEditFunctionOrFailure PrimitiveVariableInspector::acquireEditFunction( Gaffer::EditScope *editScope, const GafferScene::SceneAlgo::History *history ) const
{
	InternedString primitiveVariableName = m_primitiveVariable;
	if( auto primitiveVariableHistory = dynamic_cast<const SceneAlgo::PrimitiveVariableHistory *>( history ) )
	{
		primitiveVariableName = primitiveVariableHistory->primitiveVariableName;
	}

	const GraphComponent *readOnlyReason = EditScopeAlgo::primitiveVariableEditReadOnlyReason(
		editScope,
		history->context->get<ScenePlug::ScenePath>( ScenePlug::scenePathContextName ),
		primitiveVariableName
	);

	if( readOnlyReason )
	{
		// If we don't have an edit and the scope is locked, we error,
		// as we can't add an edit. Other cases where we already _have_
		// an edit will have been found by `source()`.
		return fmt::format( "{} is locked.", readOnlyReason->relativeName( readOnlyReason->ancestor<ScriptNode>() ) );
	}
	else
	{
		return [
			editScope = EditScopePtr( editScope ),
			primitiveVariableName,
			context = history->context
		] ( bool createIfNecessary ) {
			Context::Scope scope( context.get() );
			return EditScopeAlgo::acquirePrimitiveVariableEdit(
				editScope.get(),
				context->get<ScenePlug::ScenePath>( ScenePlug::scenePathContextName ),
				primitiveVariableName,
				createIfNecessary
			);
		};
	}
}

bool PrimitiveVariableInspector::primitiveVariableExists() const
{
	throw IECore::Exception( "Exists gets called" );
	/*if( !m_scene->existsPlug()->getValue() )
	{
		return false;
	}

	ConstCompoundObjectPtr attributes = m_scene->attributesPlug()->getValue();
	const Object *attr = attributes->member<Object>( m_attribute );

	if( !inheritAttributes )
	{
		return attr;
	}

	if( attr )
	{
		return true;
	}

	const Context *c = Context::current();
	ScenePlug::PathScope pathScope( c );

	ScenePlug::ScenePath currentPath( c->get<ScenePlug::ScenePath>( ScenePlug::scenePathContextName ) );

	// No need to check inheritance for immediate children of `/` as we
	// don't allow attributes to be created at the root of the scene.
	if( currentPath.size() > 1 )
	{
		currentPath.pop_back();  // We tested the original path above, start at the parent.
		while( currentPath.size() )
		{
			pathScope.setPath( &currentPath );
			IECore::ConstCompoundObjectPtr a = m_scene->attributesPlug()->getValue();
			if( a->member<Object>( m_attribute ) )
			{
				return true;
			}
			currentPath.pop_back();
		}
	}

	return false;*/
}
