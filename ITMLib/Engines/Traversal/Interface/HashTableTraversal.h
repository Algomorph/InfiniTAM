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

//	namespace internal {
//	template<MemoryDeviceType TMemoryDeviceType, JobCountPolicy TJobCountPolicy, TraversalMethod TTraversalMethod>
//	class RawArrayTraversalEngine_Internal;
//	} // namespace internal

template<MemoryDeviceType TMemoryDeviceType>
class HashTableTraversalEngine :
		private internal::RawArrayTraversalEngine_Internal<TMemoryDeviceType, EXACT, CONTIGUOUS>,
		private internal::RawArrayTraversalEngine_Internal<TMemoryDeviceType, PADDED, CONTIGUOUS>,
		private internal::RawArrayTraversalEngine_Internal<TMemoryDeviceType, EXACT, INDEX_SAMPLE>,
		private internal::RawArrayTraversalEngine_Internal<TMemoryDeviceType, PADDED, INDEX_SAMPLE> {
protected: // static functions
	//TODO: get rid of THashEntry for all protected static functions & their usages (?)
	template<JobCountPolicy TJobCountPolicy = JobCountPolicy::EXACT, typename THashEntry, typename TVoxelBlockHash, typename TFunctor>
	inline static void TraverseAllWithIndex_Generic(TVoxelBlockHash& index, TFunctor& functor) {
		internal::RawArrayTraversalEngine_Internal<TMemoryDeviceType, TJobCountPolicy, CONTIGUOUS>::template TraverseWithIndex_Generic
				(index.GetEntries(), functor, index.hash_entry_count);
	}

	template<JobCountPolicy TJobCountPolicy = JobCountPolicy::EXACT, typename THashEntry, typename TVoxelBlockHash, typename TFunctor>
	inline static void TraverseAllWithoutIndex_Generic(TVoxelBlockHash& index, TFunctor& functor) {
		internal::RawArrayTraversalEngine_Internal<TMemoryDeviceType, TJobCountPolicy, CONTIGUOUS>::template TraverseWithoutIndex_Generic
				(index.GetEntries(), functor, index.hash_entry_count);
	}

	template<JobCountPolicy TJobCountPolicy = JobCountPolicy::EXACT, typename THashEntry, typename TVoxelBlockHash, typename TFunctor>
	inline static void TraverseUtilizedWithIndex_Generic(TVoxelBlockHash& index, TFunctor& functor) {
		const int block_count = index.GetUtilizedBlockCount();
		const int* block_indices = index.GetUtilizedBlockHashCodes();
		internal::RawArrayTraversalEngine_Internal<TMemoryDeviceType, TJobCountPolicy, INDEX_SAMPLE>::template TraverseWithIndex_Generic
				(block_count, block_indices, index.GetEntries(), functor);
	}

	template<JobCountPolicy TJobCountPolicy = JobCountPolicy::EXACT, typename THashEntry, typename TVoxelBlockHash, typename TFunctor>
	inline static void TraverseUtilizedWithoutIndex_Generic(TVoxelBlockHash& index, TFunctor& functor) {
		const int block_count = index.GetUtilizedBlockCount();
		const int* block_indices = index.GetUtilizedBlockHashCodes();
		internal::RawArrayTraversalEngine_Internal<TMemoryDeviceType, TJobCountPolicy, INDEX_SAMPLE>::template TraverseWithoutIndex_Generic
				(block_count, block_indices, index.GetEntries(), functor);
	}

	template<JobCountPolicy TJobCountPolicy = JobCountPolicy::EXACT, typename THashEntry, typename TVoxelBlockHash, typename TFunctor>
	inline static void TraverseVisibleWithIndex_Generic(TVoxelBlockHash& index, TFunctor& functor) {
		const int block_count =   index.GetVisibleBlockCount();
		const int* block_indices = index.GetVisibleBlockHashCodes();
		internal::RawArrayTraversalEngine_Internal<TMemoryDeviceType, TJobCountPolicy, INDEX_SAMPLE>::template TraverseWithIndex_Generic
				(block_count, block_indices, index.GetEntries(), functor);
	}

	template<JobCountPolicy TJobCountPolicy = JobCountPolicy::EXACT, typename THashEntry, typename TVoxelBlockHash, typename TFunctor>
	inline static void TraverseVisibleWithoutIndex_Generic(TVoxelBlockHash& index, TFunctor& functor) {
		const int block_count =   index.GetVisibleBlockCount();
		const int* block_indices = index.GetVisibleBlockHashCodes();
		internal::RawArrayTraversalEngine_Internal<TMemoryDeviceType, TJobCountPolicy, INDEX_SAMPLE>::template TraverseWithoutIndex_Generic
				(block_count, block_indices, index.GetEntries(), functor);
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
