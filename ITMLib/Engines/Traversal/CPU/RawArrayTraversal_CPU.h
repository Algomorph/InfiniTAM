//  ================================================================
//  Created by Gregory Kramida on 2/4/20.
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
#ifdef WITH_OPENMP

#include <omp.h>

#endif
//local
#include "../Interface/RawArrayTraversal.h"
#include "../../../../ORUtils/MemoryBlock.h"


namespace ITMLib {

namespace internal {

template<>
class RawArrayTraversalEngine_Internal<MEMORYDEVICE_CPU, JobCountPolicy::EXACT, INDEX_SAMPLE>;

template<>
class RawArrayTraversalEngine_Internal<MEMORYDEVICE_CPU, JobCountPolicy::EXACT, CONTIGUOUS> {
	friend class RawArrayTraversalEngine<MEMORYDEVICE_CPU>;
	friend class RawArrayTraversalEngine_Internal<MEMORYDEVICE_CPU, JobCountPolicy::EXACT, INDEX_SAMPLE>;
protected: // static functions
	template<typename TApplyFunction>
	inline static void Traverse_Generic(const unsigned int element_count, TApplyFunction&& apply_function) {
#ifdef WITH_OPENMP
#pragma omp parallel for default(none) shared(apply_function)
#endif
		for (int i_item = 0; i_item < element_count; i_item++) {
			apply_function(i_item);
		}
	}

	template<typename TData, typename TFunctor>
	inline static void TraverseWithIndex_Generic(TData* data, TFunctor& functor, const unsigned int element_count) {
		Traverse_Generic(element_count, [&functor, &data](int i_item) { functor(data[i_item], i_item); });
	}

	template<typename TData, typename TFunctor>
	inline static void TraverseWithoutIndex_Generic(TData* data, TFunctor& functor, const unsigned int element_count) {
		Traverse_Generic(element_count, [&functor, &data](int i_item) { functor(data[i_item]); });
	}
};
template<>
class RawArrayTraversalEngine_Internal<MEMORYDEVICE_CPU, JobCountPolicy::EXACT, INDEX_SAMPLE> {
	friend class RawArrayTraversalEngine<MEMORYDEVICE_CPU>;
protected: // static functions
	template<typename TApplyFunction>
	inline static void Traverse_Generic(const int sample_size, const int* sample_indices, TApplyFunction&& apply_function) {
		RawArrayTraversalEngine_Internal<MEMORYDEVICE_CPU, JobCountPolicy::EXACT, CONTIGUOUS>::
		Traverse_Generic(sample_size, [&sample_indices, &apply_function](const int i_index) {
			const int i_item = sample_indices[i_index];
			apply_function(i_item);
		});
	}

	//TODO: not sure if this is at all useful
	template<typename TApplyFunction, typename TGetSampleSizeFunction, typename TGetSampleIndicesFunction>
	inline static void Traverse_Generic_Functional(TGetSampleSizeFunction&& get_sample_size, TGetSampleIndicesFunction&& get_sample_indices,
	                                               TApplyFunction&& apply_function) {
		const int sample_size = std::forward<TGetSampleSizeFunction>(get_sample_size)();
		const int* sample_indices = std::forward<TGetSampleIndicesFunction>(get_sample_indices)();
		Traverse_Generic(sample_size, sample_indices, apply_function);
	}

	template<typename TData, typename TFunctor>
	inline static void TraverseWithoutIndex_Generic(const int sample_size, const int* sample_indices, TData* data, TFunctor& functor) {
		Traverse_Generic(sample_size, sample_indices, [&functor, &data](int i_item) { functor(data[i_item]); });
	}

	template<typename TData, typename TFunctor>
	inline static void TraverseWithIndex_Generic(const int sample_size, const int* sample_indices, TData* data, TFunctor& functor) {
		Traverse_Generic(sample_size, sample_indices, [&functor, &data](int i_item) { functor(data[i_item], i_item); });
	}
};
template<>
class RawArrayTraversalEngine_Internal<MEMORYDEVICE_CPU, JobCountPolicy::PADDED, CONTIGUOUS> {
	friend class RawArrayTraversalEngine<MEMORYDEVICE_CPU>;
protected: // static functions
	template<typename TData, typename TFunctor>
	inline static void Traverse_Generic(TData* data, TFunctor& functor, const unsigned int element_count) {
#ifdef WITH_OPENMP
		unsigned int thread_count = omp_get_max_threads();
		unsigned int job_count = ceil_of_integer_quotient(element_count, thread_count) * thread_count;
#pragma omp parallel for default(none) shared(data, functor)
		for (int i_item = 0; i_item < job_count; i_item++) {
			functor(data, i_item, i_item >= element_count);
		}
#else
		for (int i_item = 0; i_item < element_count; i_item++) {
			functor(data, i_item, false);
		}
#endif
	}
	template<typename TData, typename TFunctor>
	inline static void TraverseWithIndex_Generic(TData* data, TFunctor& functor, const unsigned int element_count) {
		Traverse_Generic(data, functor, element_count);
	}
	template<typename TData, typename TFunctor>
	inline static void TraverseWithoutIndex_Generic(TData* data, TFunctor& functor, const unsigned int element_count) {
		Traverse_Generic(data, functor, element_count);
	}
};
template<>
class RawArrayTraversalEngine_Internal<MEMORYDEVICE_CPU, JobCountPolicy::PADDED, INDEX_SAMPLE> {
	friend class RawArrayTraversalEngine<MEMORYDEVICE_CPU>;
protected: // static functions
	template<typename TData, typename TFunctor>
	inline static void Traverse_Generic(const int sample_size, const int* sample_indices, TData* data, TFunctor& functor) {
#ifdef WITH_OPENMP
		const int thread_count = omp_get_max_threads();
		const int job_count = ceil_of_integer_quotient(sample_size, thread_count) * thread_count;
#pragma omp parallel for default(none) shared(sample_indices, data, functor)
		for (int i_index = 0; i_index < job_count; i_index++) {
			functor(data, sample_indices[i_index], i_index >= sample_size);
		}
#else
		for (int i_index = 0; i_index < sample_size; i_index++) {
			functor(data, sample_indices[i_index], false);
		}
#endif
	}
	template<typename TData, typename TFunctor, typename TGetSampleSizeFunction, typename TGetSampleIndicesFunction>
	inline static void Traverse_Generic_Functional(TData* data, TFunctor& functor,
	                                    TGetSampleSizeFunction&& get_sample_size, TGetSampleIndicesFunction&& get_sample_indices) {
		const int sample_size = std::forward<TGetSampleSizeFunction>(get_sample_size)();
		const int* sample_indices = std::forward<TGetSampleIndicesFunction>(get_sample_indices)();
		Traverse_Generic(sample_size, sample_indices, data, functor);
	}
	template<typename TData, typename TFunctor>
	inline static void TraverseWithIndex_Generic(const int sample_size, const int* sample_indices, TData* data, TFunctor& functor) {
		Traverse_Generic(sample_size, sample_indices, data, functor);
	}
	template<typename TData, typename TFunctor>
	inline static void TraverseWithoutIndex_Generic(const int sample_size, const int* sample_indices, TData* data, TFunctor& functor) {
		Traverse_Generic(sample_size, sample_indices, data, functor);
	}
};
} // namespace internal
} // namespace ITMLib