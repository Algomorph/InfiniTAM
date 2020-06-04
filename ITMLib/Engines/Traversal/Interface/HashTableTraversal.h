//  ================================================================
//  Created by Gregory Kramida on 2/5/20.
//  Copyright (c) 2020 Gregory Kramida
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at

//  http://www.apache.org/licenses/LICENSE-2.0

//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
//  ================================================================
#pragma once

#include "../../../../ORUtils/MemoryDeviceType.h"
#include "../../../Objects/Volume/VoxelBlockHash.h"
#include "../Shared/JobCountPolicy.h"
#include "../Shared/TraversalMethod.h"
#include "RawArrayTraversal.h"

namespace ITMLib {
template<MemoryDeviceType TMemoryDeviceType>
class HashTableTraversalEngine;
namespace internal {
template<MemoryDeviceType TMemoryDeviceType, JobCountPolicy TJobCountPolicy, TraversalMethod TTraversalMethod>
class HashTableTraversalEngine_Internal;
template<MemoryDeviceType TMemoryDeviceType, JobCountPolicy TJobCountPolicy>
class HashTableTraversalEngine_Internal<TMemoryDeviceType, TJobCountPolicy, CONTIGUOUS> :
		private internal::RawArrayTraversalEngine_Internal<TMemoryDeviceType, TJobCountPolicy, CONTIGUOUS>{
	friend class HashTableTraversalEngine<TMemoryDeviceType>;
	using internal::RawArrayTraversalEngine_Internal<TMemoryDeviceType, TJobCountPolicy, CONTIGUOUS>::TraverseWithIndex_Generic;
	using internal::RawArrayTraversalEngine_Internal<TMemoryDeviceType, TJobCountPolicy, CONTIGUOUS>::TraverseWithoutIndex_Generic;
};
template<MemoryDeviceType TMemoryDeviceType>
class HashTableTraversalEngine_Internal<TMemoryDeviceType, EXACT, INDEX_SAMPLE> :
private internal::RawArrayTraversalEngine_Internal<TMemoryDeviceType, EXACT, INDEX_SAMPLE>{
	friend class HashTableTraversalEngine<TMemoryDeviceType>;
	template<typename THashEntry, typename TVoxelBlockHash, typename TFunctor, typename TGetIndexCountFunction, typename TGetIndexSubarrayFunction>
	inline static void TraverseWithIndex_Generic(TVoxelBlockHash& index, TFunctor& functor, TGetIndexCountFunction&& get_index_count,
	                                                   TGetIndexSubarrayFunction&& get_index_subarray) {
		internal::RawArrayTraversalEngine_Internal<TMemoryDeviceType, EXACT, INDEX_SAMPLE>::template TraverseWithIndex_Generic(
				index.GetEntries(), functor, get_index_count, get_index_subarray);
	}

	template<typename THashEntry, typename TVoxelBlockHash, typename TFunctor, typename TGetIndexSubarrayFunction, typename TGetSubarrayFunction>
	inline static void TraverseWithoutIndex_Generic(TVoxelBlockHash& index, TFunctor& functor, TGetIndexSubarrayFunction&& get_index_count,
	                                                      TGetSubarrayFunction&& get_index_subarray) {
		internal::RawArrayTraversalEngine_Internal<TMemoryDeviceType, EXACT, INDEX_SAMPLE>::template TraverseWithoutIndex_Generic(
				index.GetEntries(), functor, get_index_count, get_index_subarray);
	}
};
template<MemoryDeviceType TMemoryDeviceType>
class HashTableTraversalEngine_Internal<TMemoryDeviceType, PADDED, INDEX_SAMPLE> :
		private internal::RawArrayTraversalEngine_Internal<TMemoryDeviceType, PADDED, INDEX_SAMPLE>{
	friend class HashTableTraversalEngine<TMemoryDeviceType>;
	template<typename THashEntry, typename TVoxelBlockHash, typename TFunctor, typename TGetIndexCountFunction, typename TGetIndexSubarrayFunction>
	inline static void TraverseWithIndex_Generic(TVoxelBlockHash& index, TFunctor& functor, TGetIndexCountFunction&& get_index_count,
	                                             TGetIndexSubarrayFunction&& get_index_subarray) {
		internal::RawArrayTraversalEngine_Internal<TMemoryDeviceType, PADDED, INDEX_SAMPLE>::template Traverse_Generic(
				index.GetEntries(), functor, get_index_count, get_index_subarray);
	}
};

} // namespace internal
template<MemoryDeviceType TMemoryDeviceType>
class HashTableTraversalEngine {
protected: // static functions
	template<JobCountPolicy TJobCountPolicy = JobCountPolicy::EXACT, typename THashEntry, typename TVoxelBlockHash,  typename TFunctor>
	inline static void TraverseAllWithIndex_Generic(TVoxelBlockHash& index, TFunctor& functor) {
		internal::HashTableTraversalEngine_Internal<TMemoryDeviceType, TJobCountPolicy, CONTIGUOUS>::template TraverseWithIndex_Generic<THashEntry>
				(index.GetEntries(), functor, index.hash_entry_count);
	}
	template<JobCountPolicy TJobCountPolicy = JobCountPolicy::EXACT, typename THashEntry, typename TVoxelBlockHash,  typename TFunctor>
	inline static void TraverseAllWithoutIndex_Generic(TVoxelBlockHash& index, TFunctor& functor) {
		internal::HashTableTraversalEngine_Internal<TMemoryDeviceType, TJobCountPolicy, CONTIGUOUS>::template TraverseWithoutIndex_Generic<THashEntry>
				(index.GetEntries(), functor, index.hash_entry_count);
	}

	template<JobCountPolicy TJobCountPolicy = JobCountPolicy::EXACT,  typename THashEntry, typename TVoxelBlockHash, typename TFunctor>
	inline static void TraverseUtilizedWithIndex_Generic(TVoxelBlockHash& index, TFunctor& functor) {
		internal::HashTableTraversalEngine_Internal<TMemoryDeviceType, TJobCountPolicy, INDEX_SAMPLE>::template TraverseWithIndex_Generic<THashEntry>
				(index, functor,
				 [&index]() { return index.GetUtilizedBlockCount(); },
				 [&index]() { return index.GetUtilizedBlockHashCodes(); });
	}
	template<JobCountPolicy TJobCountPolicy = JobCountPolicy::EXACT,  typename THashEntry, typename TVoxelBlockHash, typename TFunctor>
	inline static void TraverseUtilizedWithoutIndex_Generic(TVoxelBlockHash& index, TFunctor& functor) {
		internal::HashTableTraversalEngine_Internal<TMemoryDeviceType, TJobCountPolicy, INDEX_SAMPLE>::template TraverseWithoutIndex_Generic<THashEntry>
				(index, functor,
				 [&index]() { return index.GetUtilizedBlockCount(); },
				 [&index]() { return index.GetUtilizedBlockHashCodes(); });
	}
	template<JobCountPolicy TJobCountPolicy = JobCountPolicy::EXACT, typename THashEntry, typename TVoxelBlockHash, typename TFunctor>
	inline static void TraverseVisibleWithIndex_Generic(TVoxelBlockHash& index, TFunctor& functor) {
		internal::HashTableTraversalEngine_Internal<TMemoryDeviceType, TJobCountPolicy, INDEX_SAMPLE>::template TraverseWithIndex_Generic<THashEntry>
				(index, functor,
				 [&index]() { return index.GetVisibleBlockCount(); },
				 [&index]() { return index.GetVisibleBlockHashCodes(); });
	}
	template<JobCountPolicy TJobCountPolicy = JobCountPolicy::EXACT, typename THashEntry, typename TVoxelBlockHash, typename TFunctor>
	inline static void TraverseVisibleWithoutIndex_Generic(TVoxelBlockHash& index, TFunctor& functor) {
		internal::HashTableTraversalEngine_Internal<TMemoryDeviceType, TJobCountPolicy, INDEX_SAMPLE>::template TraverseWithoutIndex_Generic<THashEntry>
				(index, functor,
				 [&index]() { return index.GetVisibleBlockCount(); },
				 [&index]() { return index.GetVisibleBlockHashCodes(); });
	}
public: // static functions
	template<typename TFunctor>
	inline static void TraverseAll(VoxelBlockHash& index, TFunctor& functor) {
		TraverseAllWithoutIndex_Generic<EXACT, HashEntry>(index, functor);
	}

	template<typename TFunctor>
	inline static void TraverseAll(const VoxelBlockHash& index, TFunctor& functor) {
		TraverseAllWithoutIndex_Generic<EXACT, const HashEntry>(index, functor);
	}

	template<typename TFunctor>
	inline static void TraverseUtilized(VoxelBlockHash& index, TFunctor& functor) {
		TraverseUtilizedWithoutIndex_Generic<EXACT, HashEntry>(index, functor);
	}

	template<typename TFunctor>
	inline static void TraverseUtilized(const VoxelBlockHash& index, TFunctor& functor) {
		TraverseUtilizedWithoutIndex_Generic<EXACT, const HashEntry>(index, functor);
	}

	template<typename TFunctor>
	inline static void TraverseVisible(VoxelBlockHash& index, TFunctor& functor) {
		TraverseVisibleWithoutIndex_Generic<EXACT, HashEntry>(index, functor);
	}

	template<typename TFunctor>
	inline static void TraverseVisible(const VoxelBlockHash& index, TFunctor& functor) {
		TraverseVisibleWithoutIndex_Generic<EXACT, const HashEntry>(index, functor);
	}

	// *** with index ***

	template<typename TFunctor>
	inline static void TraverseAllWithIndex(VoxelBlockHash& index, TFunctor& functor) {
		TraverseAllWithIndex_Generic<EXACT, HashEntry>(index, functor);
	}

	template<typename TFunctor>
	inline static void TraverseAllWithIndex(const VoxelBlockHash& index, TFunctor& functor) {
		TraverseAllWithIndex_Generic<EXACT, const HashEntry>(index, functor);
	}

	template<typename TFunctor>
	inline static void TraverseUtilizedWithIndex(VoxelBlockHash& index, TFunctor& functor) {
		TraverseUtilizedWithIndex_Generic<EXACT, HashEntry>(index, functor);
	}

	template<typename TFunctor>
	inline static void TraverseUtilizedWithIndex(const VoxelBlockHash& index, TFunctor& functor) {
		TraverseUtilizedWithIndex_Generic<EXACT, const HashEntry>(index, functor);
	}

	template<typename TFunctor>
	inline static void TraverseVisibleWithIndex(VoxelBlockHash& index, TFunctor& functor) {
		TraverseVisibleWithIndex_Generic<EXACT, HashEntry>(index, functor);
	}

	template<typename TFunctor>
	inline static void TraverseVisibleWithIndex(const VoxelBlockHash& index, TFunctor& functor) {
		TraverseVisibleWithIndex_Generic<EXACT, const HashEntry>(index, functor);
	}
	
	// *** padded ***
	template<typename TFunctor>
	inline static void TraverseAll_Padded(VoxelBlockHash& index, TFunctor& functor) {
		TraverseAllWithIndex_Generic<PADDED, HashEntry>(index, functor);
	}

	template<typename TFunctor>
	inline static void TraverseAll_Padded(const VoxelBlockHash& index, TFunctor& functor) {
		TraverseAllWithIndex_Generic<PADDED, const HashEntry>(index, functor);
	}

	template<typename TFunctor>
	inline static void TraverseUtilized_Padded(VoxelBlockHash& index, TFunctor& functor) {
		TraverseUtilizedWithIndex_Generic<PADDED, HashEntry>(index, functor);
	}

	template<typename TFunctor>
	inline static void TraverseUtilized_Padded(const VoxelBlockHash& index, TFunctor& functor) {
		TraverseUtilizedWithIndex_Generic<PADDED, const HashEntry>(index, functor);
	}

	template<typename TFunctor>
	inline static void TraverseVisible_Padded(VoxelBlockHash& index, TFunctor& functor) {
		TraverseVisibleWithIndex_Generic<PADDED, HashEntry>(index, functor);
	}

	template<typename TFunctor>
	inline static void TraverseVisible_Padded(const VoxelBlockHash& index, TFunctor& functor) {
		TraverseVisibleWithIndex_Generic<PADDED, const HashEntry>(index, functor);
	}
};
} // namespace ITMLib
