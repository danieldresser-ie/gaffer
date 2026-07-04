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

#include "Gaffer/CachedDataNode.h"

#include "Gaffer/ScriptNode.h"
#include "Gaffer/ValuePlug.h"

#include "IECore/NullObject.h"
#include "IECore/FileIndexedIO.h"

#include "boost/bind/bind.hpp"
#include "fmt/std.h"

#include <regex>

using namespace Gaffer;
using namespace boost::placeholders;

// TODO - where should this live?
namespace Gaffer
{

class RecycleBinManager
{
public:
	RecycleBinManager( const std::filesystem::path &cacheDirectory )
		: m_recycleBinPath( cacheDirectory / ".recycleBin" ), m_acquired( false )
	{
	}

	std::filesystem::path acquire()
	{
		if( !m_acquired )
		{
			if( std::filesystem::exists( m_recycleBinPath ) )
			{
				// TODO - need a way to repair this
				// TODO - put a tag file in the dir so we can identify owning process / handle differently if process still open?
				// TODO - figure out why this is getting is triggered by backups
				throw IECore::Exception( fmt::format( "Cannot acquire recycle bin - something else owns a recycle bin at {}", m_recycleBinPath ) );
			}

			std::filesystem::create_directories( m_recycleBinPath );
			m_acquired = true;
		}

		return m_recycleBinPath;
	}

	std::optional<std::filesystem::path> getIfExists()
	{
		if( m_acquired )
		{
			return m_recycleBinPath;
		}
		else
		{
			return {};
		}
	}



	~RecycleBinManager()
	{
		if( m_acquired )
		{
			// This is a fairly scary looking system call, so this is probably a good place to write
			// down some justification why this seems like it should be safe:
			// In order to be added to m_ownedRecycleBins, a path must be added through acquireRecycleBin,
			// which creates the path, and ensures that:
			// * it ends in ".recycleBin"
			// * it didn't previously exist
			// That should ensure that we're not deleting directories we don't own.
			// In order to be placed in a recycle bin, a file must be found in a "_cacheDir" directory
			// corresponding to a script file that is currently being overwritten, and must look like
			// a Gaffer cache ( ie. matches the regex "[0-9a-f]{32}.io", enforced by
			// cacheFileNameToHash ).

			std::filesystem::remove_all( m_recycleBinPath );
		}
	}

private:

	std::filesystem::path m_recycleBinPath;
	bool m_acquired;
};
}

namespace {

static const IECore::InternedString g_cacheEvaluationKeyName( "__cacheEvaluationKey" );

std::string cacheFileNameFromHash( const IECore::MurmurHash &h )
{
	return h.toString() + ".io";
}

std::optional<IECore::MurmurHash> cacheFileNameToHash( const std::string &fileName )
{
	static const std::regex g_cacheFileNameRegex( R"(([0-9a-f]{32}).io)" );

	std::smatch match;
	if( std::regex_match( fileName, match, g_cacheFileNameRegex ) )
	{
		return IECore::MurmurHash::fromString( match.str( 1 ) );
	}

	return {};
}


std::filesystem::path recycleBinDirectory( const std::filesystem::path &cacheDirectory )
{
	return cacheDirectory / ".recycleBin";
}



std::map< std::filesystem::path, std::weak_ptr< Gaffer::RecycleBinManager > > g_recycleBinManagers;

// TODO - not sure this should ever return null, but I'm still wondering whether a refactor is
// needed for how this is bound to sourceDirectory.
std::shared_ptr<Gaffer::RecycleBinManager> acquireRecycleBinManager( const std::filesystem::path &cacheDirectory )
{
	if( cacheDirectory.empty() )
	{
		return nullptr;
	}

	std::shared_ptr<Gaffer::RecycleBinManager> result;
	auto it = g_recycleBinManagers.find( cacheDirectory );
	if( it != g_recycleBinManagers.end() )
	{
		result = it->second.lock();
	}

	if( !result )
	{
		result = std::make_shared<Gaffer::RecycleBinManager>( cacheDirectory );
		g_recycleBinManagers[ cacheDirectory ] = result;
	}

	return result;
}

std::filesystem::path cacheDirFromScriptPath( const std::filesystem::path &scriptPath )
{
	return scriptPath.parent_path() / ( scriptPath.filename().string() + ".cachedData" );
}

IECore::ConstObjectPtr loadDataFile( const std::filesystem::path &filePath )
{
	IECore::FileIndexedIOPtr file = new IECore::FileIndexedIO(
		filePath.generic_string(), IECore::IndexedIO::rootPath, IECore::IndexedIO::Read
	);

	return IECore::Object::load( file, "object" );
}

} // namespace

class CachedDataNode::SetEntryAction : public Gaffer::Action
{

	public :

		IE_CORE_DECLARERUNTIMETYPEDEXTENSION( Gaffer::CachedDataNode::SetEntryAction, SetEntryActionTypeId, Gaffer::Action );

		SetEntryAction( CachedDataNodePtr node, const IECore::InternedString &key, IECore::ConstObjectPtr value )
			: m_node( node ), m_key( key ), m_doValue( value ? std::make_optional<CacheEntry>( {value->hash(), value} ) : std::nullopt )
		{
			// The source directory isn't changed by a SetEntry - it's changed by serialisation, which
			// happens in between SetEntry's. But undo'ing or redo'ing a SetEntry is what could trigger
			// needing to use a previous or future sourceDirectory, and we can support that by recording
			// this directory which is valid before or after this operation.
			m_sourceDirectory = node->sourceDirectory();

			// If there is a recycle bin created for this directory, we may need access to it,
			// so we acquire a RecycleBinManager
			m_recycleBinManager = acquireRecycleBinManager( m_sourceDirectory );

			auto it = node->m_caches.find( key );
			if( it != node->m_caches.end() )
			{
				m_undoValue = it->second;
			}
		}

		~SetEntryAction()
		{
		}

	protected :

		GraphComponent *subject() const override
		{
			return m_node.get();
		}

		void doAction() override
		{
			Action::doAction();
			m_node->setEntryInternal( m_key, m_doValue );
			m_node->setSourceDirectory( m_sourceDirectory );
		}

		void undoAction() override
		{
			Action::undoAction();
			m_node->setEntryInternal( m_key, m_undoValue );
			m_node->setSourceDirectory( m_sourceDirectory );
		}

		bool canMerge( const Action *other ) const override
		{
			if( !Action::canMerge( other ) )
			{
				return false;
			}
			const SetEntryAction *setEntryAction = IECore::runTimeCast<const SetEntryAction>( other );
			return setEntryAction && setEntryAction->m_node == m_node && setEntryAction->m_key == m_key;
		}

		void merge( const Action *other ) override
		{
			const SetEntryAction *setEntryAction = static_cast<const SetEntryAction *>( other );
			m_doValue = setEntryAction->m_doValue;
		}

	private :

		CachedDataNodePtr m_node;
		IECore::InternedString m_key;

		std::filesystem::path m_sourceDirectory;
		std::shared_ptr<RecycleBinManager> m_recycleBinManager;

		// \todo In a long paint session on heavy geo, it's very plausible that a large amount of memory
		// could be consumed by holding live values in the undo queue. In theory, you could probably
		// even run out of memory and crash. Since we already need the .recycleBin folder for dealing
		// with unused files that haven't been loaded ( but could be needed by undo ), it would be possible
		// to use the .recycleBin folder for dealing with this memory accumulation: when we save, we could
		// write all live value in the undo queue into the recycle bin, and clear them out of the undo
		// queue. Kinda seems like a good idea, but maybe not necessary currently.
		std::optional<CacheEntry> m_doValue;
		std::optional<CacheEntry> m_undoValue;

};

IE_CORE_DEFINERUNTIMETYPED( CachedDataNode::SetEntryAction );

CacheDirectoryManager::CacheDirectoryManager( const std::filesystem::path *scriptPath )
	: m_directory( cacheDirFromScriptPath( *scriptPath ) ), m_created( false )
{
}

CacheDirectoryManager::~CacheDirectoryManager()
{
	// TODO - should we clean if there is no CachedDataNode's?
	if( !m_created )
	{
		// No cleanup needed
		return;

	}

	try
	{
		std::shared_ptr<RecycleBinManager> recycleBinManager = acquireRecycleBinManager( getCacheDirectory() );

		for( auto const& directoryEntry : std::filesystem::directory_iterator( getCacheDirectory() ) )
		{
			auto cacheFileHash = cacheFileNameToHash( directoryEntry.path().filename().generic_string() );
			if( cacheFileHash )
			{
				if( !m_usedCaches.count( *cacheFileHash ) )
				{
					// TODO - check takeOwnership
					// It's a little bit non-obvious whether it's safe to move this file while we have a
					// directory iterator, but the docs say about changing directory contents: "it is unspecified
					// whether the change would be observed through the iterator." Since they don't say
					// anything about the iterator becoming invalid, I guess this is fine.
					const std::filesystem::path recycledPath( recycleBinManager->acquire() / directoryEntry.path().filename() );

					if( std::filesystem::exists( recycledPath ) )
					{
						// We've already stored this in the recycle bin, so it's safe to just delete it
						std::filesystem::remove( directoryEntry.path() );
					}
					else
					{
						std::filesystem::rename( directoryEntry.path(), recycledPath );
					}
				}
			}
			else
			{
				if( directoryEntry.path().filename() != ".recycleBin" )
				{
					IECore::msg(
						IECore::Msg::Warning, "Serialisation",
						fmt::format( "Unexpected file {} in cache directory {}.",
							directoryEntry.path().filename(), getCacheDirectory()
						)
					);
				}
			}
		}
	}
	catch( IECore::Exception &e )
	{
		IECore::msg(
			IECore::Msg::Warning, "Serialisation",
			std::string( "Unable to clean up unused caches : " ) + e.what()
		);
	}

	if( m_warning.size() )
	{
		IECore::msg( IECore::Msg::Warning, "Serialisation", m_warning );
	}
}

std::filesystem::path CacheDirectoryManager::getCacheDirectory()
{
	if( !m_created )
	{
		std::filesystem::create_directories( m_directory );
		m_created = true;
	}

	return m_directory;
}

bool CacheDirectoryManager::addData( const IECore::MurmurHash &hash, const std::filesystem::path &sourceDirectory, const IECore::Object *liveValue )
{
	if( !m_usedCaches.insert( hash ).second )
	{
		// This value was already saved during this serialization
		return true;
	}

	std::string fileName = cacheFileNameFromHash( hash );

	const std::filesystem::path directory = getCacheDirectory();

	std::filesystem::path destPath = directory / fileName;
	if( std::filesystem::exists( destPath ) )
	{
		// This value was already saved during a previous serialization
		return true;
	}

	std::optional<std::filesystem::path> sourcePath;
	if( !sourceDirectory.empty() )
	{
		if( sourceDirectory != directory && std::filesystem::exists( sourceDirectory / fileName ) )
		{
			sourcePath = sourceDirectory / fileName;
		}
		else
		{
			std::filesystem::path recycleBinPath = recycleBinDirectory( sourceDirectory ) / fileName;
			if( std::filesystem::exists( recycleBinPath ) )
			{
				sourcePath = recycleBinPath;
			}
		}
	}

	if( sourcePath )
	{
		// This value already exists on disk, but in a different directory.
		// Try to hardlink to it.
		std::error_code ec;
		std::filesystem::create_hard_link( *sourcePath, destPath, ec );
		if( ec )
		{
			if( !m_warning.size() )
			{
				m_warning = fmt::format( "During saving, could not create hardlink at {} pointing to {}, falling back to copying file.", destPath, *sourcePath );
			}

			// If that failed, just copy.
			std::filesystem::copy_file( *sourcePath, destPath );
		}
	}
	else
	{
		// This value does not yet exist on disk, and we need to write it.
		if( !liveValue )
		{
			return false;
			//throw IECore::Exception( fmt::format( "Unable to save entry \"{}\" on \"{}\" - no live value, but cannot find on disk in directory {}.", cache.first, fullName(), sourceDirectory ) );
		}

		IECore::FileIndexedIOPtr file = new IECore::FileIndexedIO( destPath.generic_string(), IECore::IndexedIO::rootPath, IECore::IndexedIO::Exclusive | IECore::IndexedIO::Write);

		liveValue->save( file, "object" );
	}

	return true;
}

GAFFER_NODE_DEFINE_TYPE( CachedDataNode );

size_t CachedDataNode::g_firstPlugIndex = 0;

CachedDataNode::CachedDataNode(
		const std::string &name,
		const std::string &sourceDirectory, IECore::ConstCompoundDataPtr caches
)
	:	ComputeNode( name )
{
	storeIndexOfNextChild( g_firstPlugIndex );

	addChild( new StringPlug( "selector", Plug::In ) );
	addChild( new ObjectPlug( "out", Plug::Out, new IECore::NullObject() ) );
	addChild( new StringVectorDataPlug( "keys", Plug::Out ) );
	addChild( new IntPlug( "__refreshCount", Plug::In ) );
	addChild( new ObjectPlug( "__evaluate", Plug::Out, new IECore::NullObject() ) );

	// TODO - should we force load as live data if sourceDirectory doesn't match current script
	// ( indicating a paste from a different script? )
	// TODO - rename "sourceDirectory" to "loadFrom"
	if( caches )
	{

		// TODO - check if source matches current file.
		bool needsLoad = !sourceDirectory.empty();
		if( sourceDirectory.empty() )
		{
			m_sourceDirectory = cacheDirFromScriptPath( Context::current()->get<std::string>( ScriptNode::serialisationSourceFileContextName() ) );
		}
		else
		{
			m_sourceDirectory = sourceDirectory;
		}

		m_recycleBinManager = acquireRecycleBinManager( m_sourceDirectory );
		for( const auto &it : caches->readable() )
		{
			IECore::StringData *stringVal = IECore::runTimeCast<IECore::StringData>( it.second.get() );
			if( !stringVal )
			{
				throw IECore::Exception( "Corrupt CachedDataNode serialisation" );
			}

			const IECore::MurmurHash hash( IECore::MurmurHash::fromString( stringVal->readable() ) );

			if( !needsLoad )
			{
				m_caches[ it.first ] = { hash, nullptr };
			}
			else
			{
				try
				{
					m_caches[ it.first ] = { hash, loadDataFile( m_sourceDirectory / cacheFileNameFromHash( hash ) ) };
				}
				catch( const IECore::Exception & )
				{
					throw IECore::Exception( "Cannot paste - source file uses data caches which are not accessible, or have been modified." );
				}
			}
		}
	}
}

CachedDataNode::~CachedDataNode()
{
}

StringPlug *CachedDataNode::selectorPlug()
{
	return getChild<StringPlug>( g_firstPlugIndex + 0 );
}

const StringPlug *CachedDataNode::selectorPlug() const
{
	return getChild<StringPlug>( g_firstPlugIndex + 0 );
}

ObjectPlug *CachedDataNode::outPlug()
{
	return getChild<ObjectPlug>( g_firstPlugIndex + 1 );
}

const ObjectPlug *CachedDataNode::outPlug() const
{
	return getChild<ObjectPlug>( g_firstPlugIndex + 1 );
}

StringVectorDataPlug *CachedDataNode::keysPlug()
{
	return getChild<StringVectorDataPlug>( g_firstPlugIndex + 2 );
}

const StringVectorDataPlug *CachedDataNode::keysPlug() const
{
	return getChild<StringVectorDataPlug>( g_firstPlugIndex + 2 );
}

IntPlug *CachedDataNode::refreshCountPlug()
{
	return getChild<IntPlug>( g_firstPlugIndex + 3 );
}

const IntPlug *CachedDataNode::refreshCountPlug() const
{
	return getChild<IntPlug>( g_firstPlugIndex + 3 );
}

ObjectPlug *CachedDataNode::evaluatePlug()
{
	return getChild<ObjectPlug>( g_firstPlugIndex + 4 );
}

const ObjectPlug *CachedDataNode::evaluatePlug() const
{
	return getChild<ObjectPlug>( g_firstPlugIndex + 4 );
}

void CachedDataNode::save( CacheDirectoryManager *cacheDirectoryManager ) const
{
	// TODO - weird things happen if exceptions occur during serialization

	if( !cacheDirectoryManager )
	{
		// If there is no cache directory set, that means that we're doing a copy,
		// where we serialise to memory instead of a file. We support this only if
		// all the caches are already saved to disk.

		for( auto &cache : m_caches )
		{
			std::string fileName = cacheFileNameFromHash( cache.second.m_hash );

			std::filesystem::path sourcePath = m_sourceDirectory / fileName;
			if( !std::filesystem::exists( sourcePath ) )
			{
				throw IECore::Exception( fmt::format( "Cannot copy, CachedDataNode \"{}\" is not saved yet.", fullName() ) );
			}
		}
	}
	else
	{
		for( auto &cache : m_caches )
		{
			if( !cacheDirectoryManager->addData( cache.second.m_hash, m_sourceDirectory, cache.second.m_liveValue.get() ) )
			{
				throw IECore::Exception( fmt::format( "Unable to save entry \"{}\" on \"{}\" - no live value, but cannot find on disk in directory {}.", cache.first, fullName(), m_sourceDirectory ) );

			}
		}

		// TODO - should this not create the dir?
		m_sourceDirectory = cacheDirectoryManager->getCacheDirectory();
		m_recycleBinManager = acquireRecycleBinManager( m_sourceDirectory );
	}

	// Caches now exist in target directory. Update so that we'll now read the disk caches
	// instead of needing to hold live values.
	for( auto &cache : m_caches )
	{
		cache.second.m_liveValue.reset();
	}

}

std::filesystem::path CachedDataNode::sourceDirectory() const
{
	return m_sourceDirectory;
}

void CachedDataNode::affects( const Plug *input, AffectedPlugsContainer &outputs ) const
{
	ComputeNode::affects( input, outputs );

	if(
		input == refreshCountPlug() ||
		input == selectorPlug()
	)
	{
		outputs.push_back( evaluatePlug() );
	}

	if(
		input == refreshCountPlug()
	)
	{
		outputs.push_back( keysPlug() );
	}

	if(
		input == evaluatePlug()
	)
	{
		outputs.push_back( outPlug() );
	}
}

void CachedDataNode::setEntry( const IECore::InternedString &key, IECore::ConstObjectPtr value )
{
	if( !value )
	{
		throw IECore::Exception( "Null value passed to setValue" );
	}

	// Ignore setEntry calls if it matches the existing value
	auto it = m_caches.find( key );
	if( it != m_caches.end() && it->second.m_hash == value->hash() )
	{
		return;
	}

	Action::enact( new SetEntryAction( this, key, value ) );
}

void CachedDataNode::removeEntry( const IECore::InternedString &key )
{
	// Ignore removeEntry calls if there is no entry
	auto it = m_caches.find( key );
	if( it == m_caches.end() )
	{
		return;
	}

	// We have a removeEntry method in order to present a clearer API,
	// but we internally represent a remove as a SetEntry with a null
	// value in order to avoid duplicating code for SetEntryAction.
	Action::enact( new SetEntryAction( this, key, nullptr ) );
}

void CachedDataNode::setEntryInternal( const IECore::InternedString &key, const std::optional<CacheEntry> &value )
{
	if( value )
	{
		m_caches[key] = *value;
	}
	else
	{
		m_caches.erase( key );
	}

	refreshCountPlug()->setValue( refreshCountPlug()->getValue() + 1 );
}

void CachedDataNode::setSourceDirectory( const std::filesystem::path &sourceDirectory )
{
	m_sourceDirectory = sourceDirectory;
	m_recycleBinManager = acquireRecycleBinManager( m_sourceDirectory );
}

IECore::ConstObjectPtr CachedDataNode::getEntry( const IECore::InternedString &key, bool throwExceptions ) const
{
	try
	{
		// We use an evaluation plug to provide the getEntry() functionality - this means we can
		// use Gaffer's default plug cache to ensure that we don't load a file twice if it is
		// queried both via getEntry and via an output plug.
		Context::EditableScope s( Context::current() );
		s.set( g_cacheEvaluationKeyName, &key );
		return evaluatePlug()->getValue();
	}
	catch( ... )
	{
		if( throwExceptions )
		{
			throw;
		}
		else
		{
			return nullptr;
		}
	}
}

bool CachedDataNode::hasLiveEntries() const
{
	for( auto &cache : m_caches )
	{
		if( cache.second.m_liveValue )
		{
			return true;
		}
	}

	return false;
}

std::map<IECore::InternedString, IECore::MurmurHash> CachedDataNode::entryHashes() const
{
	std::map<IECore::InternedString, IECore::MurmurHash> result;
	for( auto &cache : m_caches )
	{
		result[cache.first] = cache.second.m_hash;
	}

	return result;
}

void CachedDataNode::hash( const ValuePlug *output, const Context *context, IECore::MurmurHash &h ) const
{
	if( output == evaluatePlug() )
	{
		ComputeNode::hash( output, context, h );

		const IECore::InternedString &key = context->get<IECore::InternedString>( g_cacheEvaluationKeyName );

		auto it = m_caches.find( key );
		if( it == m_caches.end() )
		{
			throw IECore::Exception( "Unknown key: " + key.string() );
		}

		h = it->second.m_hash;
		return;
	}
	else if( output == outPlug() )
	{
		Context::EditableScope s( context );
		IECore::InternedString select = selectorPlug()->getValue();
		s.set( g_cacheEvaluationKeyName, &select );

		h = IECore::MurmurHash();
		evaluatePlug()->hash( h );
		return;
	}
	else if( output == keysPlug() )
	{
		ComputeNode::hash( output, context, h );

		for( const auto &i : m_caches )
		{
			h.append( i.first );
		}
	}


	ComputeNode::hash( output, context, h );
}

void CachedDataNode::compute( ValuePlug *output, const Context *context ) const
{
	if( output == evaluatePlug() )
	{
		IECore::ConstObjectPtr result;
		const IECore::InternedString &key = context->get<IECore::InternedString>( g_cacheEvaluationKeyName );

		auto it = m_caches.find( key );
		if( it == m_caches.end() )
		{
			throw IECore::Exception( "Unknown key: " + key.string() );
		}

		if( it->second.m_liveValue )
		{
			result = it->second.m_liveValue;
		}
		else
		{
			auto cacheIt = m_caches.find( key );
			if( cacheIt != m_caches.end() )
			{
				std::string cacheFileName = cacheFileNameFromHash( cacheIt->second.m_hash );

				std::optional<std::filesystem::path> sourcePath;
				if( !m_sourceDirectory.empty() )
				{
					if( IECore::FileIndexedIO::canRead( ( m_sourceDirectory / cacheFileName ).generic_string() ) )
					{
						sourcePath = m_sourceDirectory / cacheFileName;
					}
					else if( const std::optional<std::filesystem::path> recycleBinDir = m_recycleBinManager ? m_recycleBinManager->getIfExists() : std::nullopt )
					{
						std::filesystem::path recycleBinPath = (*recycleBinDir) / cacheFileName;

						if( IECore::FileIndexedIO::canRead( recycleBinPath.generic_string() ) )
						{
							sourcePath = recycleBinPath;
						}
					}
				}

				if( !sourcePath )
				{
					throw IECore::Exception( fmt::format(
						"Could not locate cache file {} in {}.", cacheFileName, m_sourceDirectory
					) );
				}

				result = loadDataFile( *sourcePath );
			}
		}

		if( !result )
		{
			throw IECore::Exception( "Unknown key: " + key.string() );
		}

		static_cast<ObjectPlug *>( output )->setValue( result );
		return;

	}
	else if( output == outPlug() )
	{
		Context::EditableScope s( context );
		IECore::InternedString select = selectorPlug()->getValue();
		s.set( g_cacheEvaluationKeyName, &select );

		static_cast<ObjectPlug *>( output )->setValue( evaluatePlug()->getValue() );
		return;
	}
	else if( output == keysPlug() )
	{
		std::vector<std::string> keys;

		for( const auto &i : m_caches )
		{
			keys.push_back( i.first );
		}

		// TODO - sort unnecessary?
		// TODO - should this be InternedStringVectorData?
		std::sort( keys.begin(), keys.end() );

		static_cast<StringVectorDataPlug *>( output )->setValue(
			new IECore::StringVectorData( std::move( keys ) )
		);
		return;
	}

	ComputeNode::compute( output, context );
}

