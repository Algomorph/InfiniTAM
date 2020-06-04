//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 6/2/20.
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
#include "../Shared/JobCountPolicy.h"
#include "../Shared/TraversalMethod.h"

namespace ITMLib {
namespace internal {
template<MemoryDeviceType TDeviceType, JobCountPolicy TJobCountPolicy, TraversalMethod TTraversalMethod>
class RawArrayTraversalEngine_Internal;
} // namespace internal
template<MemoryDeviceType TDeviceType>
class RawArrayTraversalEngine{
public: // static functions
	template<JobCountPolicy TJobCountPolicy = JobCountPolicy::EXACT, typename T, typename TFunctor>
	inline static void Traverse(T* data, TFunctor& functor, const unsigned int element_count) {
		internal::RawArrayTraversalEngine_Internal<TDeviceType, TJobCountPolicy, CONTIGUOUS>::template TraverseWithoutIndex_Generic<T, TFunctor>(data, functor, element_count);
	}

	template<JobCountPolicy TJobCountPolicy = JobCountPolicy::EXACT, typename T, typename TFunctor>
	inline static void Traverse(const T* data, TFunctor& functor, const unsigned int element_count) {
		internal::RawArrayTraversalEngine_Internal<TDeviceType, TJobCountPolicy, CONTIGUOUS>::template TraverseWithoutIndex_Generic<const T, TFunctor>(data, functor, element_count);
	}

	template<JobCountPolicy TJobCountPolicy = JobCountPolicy::EXACT, typename T, typename TFunctor>
	inline static void TraverseWithIndex(T* data, TFunctor& functor, const unsigned int element_count) {
		internal::RawArrayTraversalEngine_Internal<TDeviceType, TJobCountPolicy, CONTIGUOUS>::template TraverseWithIndex_Generic<T, TFunctor>(data, functor, element_count);
	}

	template<JobCountPolicy TJobCountPolicy = JobCountPolicy::EXACT, typename T, typename TFunctor>
	inline static void TraverseWithIndex(const T* data, TFunctor& functor, const unsigned int element_count) {
		internal::RawArrayTraversalEngine_Internal<TDeviceType, TJobCountPolicy, CONTIGUOUS>::template TraverseWithIndex_Generic<const T, TFunctor>(data, functor, element_count);
	}
};

} // namespace ITMLib
