//////////////////////////////////////////////////////////////////////////
//
//  Copyright (c) 2011-2012, John Haddon. All rights reserved.
//  Copyright (c) 2013, Image Engine Design Inc. All rights reserved.
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

#ifndef GAFFER_VALUEPLUG_H
#define GAFFER_VALUEPLUG_H

#include "Gaffer/Plug.h"

#include "IECore/Object.h"

namespace Gaffer
{

IE_CORE_FORWARDDECLARE( DependencyNode )

/// The Plug base class defines the concept of a connection
/// point with direction. The ValuePlug class extends this concept
/// to allow the connections to pass values between connection
/// points, and for DependencyNode::compute() to be used to compute output
/// values.
class GAFFER_API ValuePlug : public Plug
{

	public :

		/// Constructs a ValuePlug which can be used as a parent for other ValuePlugs.
		ValuePlug( const std::string &name=defaultName<ValuePlug>(), Direction direction=In, unsigned flags=Default );
		~ValuePlug() override;

		GAFFER_PLUG_DECLARE_TYPE( Gaffer::ValuePlug, ValuePlugTypeId, Plug );

		bool acceptsChild( const GraphComponent *potentialChild ) const override;
		/// Accepts the input only if it is derived from ValuePlug.
		/// Derived classes may accept more types provided they
		/// derive from ValuePlug too, and they can deal with them
		/// in setFrom().
		bool acceptsInput( const Plug *input ) const override;
		/// Reimplemented so that values can be propagated from inputs.
		void setInput( PlugPtr input ) override;

		PlugPtr createCounterpart( const std::string &name, Direction direction ) const override;

		/// Returns true if it is valid to call setFrom(), setToDefault(),
		/// or setValue() on this plug. False will be returned if the plug
		/// has an input connection or the ReadOnly flag is set.
		bool settable() const;

		/// Must be implemented to set the value of this Plug from the other Plug,
		/// performing any necessary conversions on the input value. Should throw
		/// an exception if other is of an unsupported type.
		virtual void setFrom( const ValuePlug *other );

		/// Sets the value to the default for this plug. The default
		/// implementation is sufficient for all subclasses except those
		/// where the number of child plugs varies based on the value.
		virtual void setToDefault();
		/// Returns true if the current value of the plug is the same
		/// as the default value. The default implementation is sufficient
		/// for all subclasses except those where the number of child plugs
		/// varies based on the value.
		/// > Note : If a plug's value is being driven by a ComputeNode,
		/// > we always consider it to be non-default, because it may vary
		/// > by context. `isSetToDefault()` does not trigger computes.
		virtual bool isSetToDefault() const;

		/// Returns a hash to represent the value of this plug
		/// in the current context.
		virtual IECore::MurmurHash hash() const;
		/// Convenience function to append the hash to h.
		void hash( IECore::MurmurHash &h ) const;

		/// Specifies the methodology used to cache the value
		/// and hash for output plugs.
		enum class CachePolicy
		{
			/// No caching is performed. Suitable for
			/// extremely quick processes. Also useful
			/// to avoid double-counting of cache memory when
			/// a compute always returns a sub-object of another
			/// cache entry.
			Uncached,
			/// Suitable for regular processes that don't spawn
			/// TBB tasks. It is essential that any task-spawning
			/// processes use one of the dedicated policies below.
			Standard,
			/// Suitable for processes that spawn TBB tasks.
			/// Threads waiting for the same result will collaborate
			/// to perform tasks together until the work is complete.
			TaskCollaboration,
			/// Suitable for processes that spawn TBB tasks. Threads
			/// waiting for an in-progress compute will block until
			/// it is complete. In theory this is inferior to TaskCollaboration,
			/// but due to TBB overhead it may be preferable for small
			/// but frequent computes.
			TaskIsolation,
			/// Legacy policy, to be removed.
			Legacy
		};

		/// @name Cache management
		/// ValuePlug optimises repeated computation by storing a cache of
		/// recently computed values. These functions allow for management
		/// of the cache.
		////////////////////////////////////////////////////////////////////
		//@{
		/// Returns the maximum amount of memory in bytes to use for the cache.
		static size_t getCacheMemoryLimit();
		/// Sets the maximum amount of memory the cache may use in bytes.
		static void setCacheMemoryLimit( size_t bytes );
		/// Returns the current memory usage of the cache in bytes.
		static size_t cacheMemoryUsage();
		/// Clears the cache.
		static void clearCache();
		//@}

		/// @name Hash cache management
		/// In addition to the cache of recently computed values, we also
		/// keep a per-thread cache of recently computed hashes. These functions
		/// allow for management of that cache.
		////////////////////////////////////////////////////////////////////
		//@{
		static size_t getHashCacheSizeLimit();
		/// > Note : Limits are applied on a per-thread basis as and
		/// > when each thread is used to compute a hash.
		static void setHashCacheSizeLimit( size_t maxEntriesPerThread );
		//@}

	protected :

		/// This constructor must be used by all derived classes which wish
		/// to store their own values - without calling it defaultObjectValue()
		/// and getObjectValue() will return null. The defaultValue will be
		/// referenced directly (not copied) and therefore must not be changed
		/// after passing to the constructor. The defaultValue must be non-null.
		/// When this constructor is used, the ValuePlug does not accept child
		/// plugs - values are always stored on leaf plugs.
		ValuePlug( const std::string &name, Direction direction,
			IECore::ConstObjectPtr defaultValue, unsigned flags );

		/// Returns the default value that was passed to the constructor.
		/// It is imperative that this value is not changed.
		const IECore::Object *defaultObjectValue() const;

		/// Internally all values are stored as instances of classes derived
		/// from IECore::Object, although this isn't necessarily visible to the user.
		/// This function updates the value using node()->compute()
		/// or setFrom( getInput() ) as appropriate and then returns it. Typically
		/// this will be called by a subclass getValue() method which will
		/// extract a value from the object and return it to the user in a more
		/// convenient form. Note that this function will often return different
		/// objects with each query - this allows it to support the calculation
		/// of values in different contexts and on different threads.
		///
		/// The value is returned via a reference counted pointer, as
		/// following return from getObjectValue(), it is possible that nothing
		/// else references the value - the value could have come from the cache
		/// and then have been immediately removed by another thread.
		///
		/// If a precomputed hash is available it may be passed to avoid computing
		/// it again unnecessarily. Passing an incorrect hash has dire consequences, so
		/// use with care.
		IECore::ConstObjectPtr getObjectValue( const IECore::MurmurHash *precomputedHash = nullptr ) const;
		/// Should be called by derived classes when they wish to set the plug
		/// value - the value is referenced directly (not copied) and so must
		/// not be changed following the call.
		void setObjectValue( IECore::ConstObjectPtr value );

		/// Reimplemented to emit `plugSetSignal()` for the parent plugs.
		void parentChanged( Gaffer::GraphComponent *oldParent ) override;
		/// Reimplemented for cache management.
		void dirty() override;

	private :

		class HashProcess;
		class ComputeProcess;
		class SetValueAction;

		void setValueInternal( IECore::ConstObjectPtr value, bool propagateDirtiness );
		void childAddedOrRemoved();
		// Emits the appropriate Node::plugSetSignal() for this plug and all its
		// ancestors, then does the same for its output plugs.
		void emitPlugSet();

		IECore::ConstObjectPtr m_defaultValue;
		// For holding the value of input plugs with no input connections.
		IECore::ConstObjectPtr m_staticValue;

};

IE_CORE_DECLAREPTR( ValuePlug )

typedef FilteredChildIterator<PlugPredicate<Plug::Invalid, ValuePlug> > ValuePlugIterator;
typedef FilteredChildIterator<PlugPredicate<Plug::In, ValuePlug> > InputValuePlugIterator;
typedef FilteredChildIterator<PlugPredicate<Plug::Out, ValuePlug> > OutputValuePlugIterator;

typedef FilteredRecursiveChildIterator<PlugPredicate<Plug::Invalid, ValuePlug>, PlugPredicate<> > RecursiveValuePlugIterator;
typedef FilteredRecursiveChildIterator<PlugPredicate<Plug::In, ValuePlug>, PlugPredicate<> > RecursiveInputValuePlugIterator;
typedef FilteredRecursiveChildIterator<PlugPredicate<Plug::Out, ValuePlug>, PlugPredicate<> > RecursiveOutputValuePlugIterator;

} // namespace Gaffer

#endif // GAFFER_VALUEPLUG_H
