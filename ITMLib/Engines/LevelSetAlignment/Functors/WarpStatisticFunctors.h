//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 9/19/20.
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
#include "../../../../ORUtils/CrossPlatformMacros.h"
#include "../../../../ORUtils/Vector.h"
#include "../../Reduction/Shared/ReductionResult.h"

namespace ITMLib {

// ==== Functors for retrieval of data from voxels

// just length
template<typename TWarp, MemoryDeviceType TMemoryDeviceType, bool TUseGradient1>
struct RetrieveGradientLengthFunctor;

template<typename TWarp, MemoryDeviceType TMemoryDeviceType>
struct RetrieveGradientLengthFunctor<TWarp, TMemoryDeviceType, true> {
	_CPU_AND_GPU_CODE_
	inline static float retrieve(const TWarp& voxel) {
		return ORUtils::length(voxel.gradient1);
	}
};

template<typename TWarp, MemoryDeviceType TMemoryDeviceType>
struct RetrieveGradientLengthFunctor<TWarp, TMemoryDeviceType, false> {
	_CPU_AND_GPU_CODE_
	inline static float retrieve(const TWarp& voxel) {
		return ORUtils::length(voxel.gradient0);
	}
};

// sum (e.g. of lengths) and count
struct SumAndCount {
	float sum;
	unsigned int count;
};

template<typename TWarp, MemoryDeviceType TMemoryDeviceType, bool TUseGradient1>
struct RetrieveGradientLengthAndCountFunctor;

template<typename TWarp, MemoryDeviceType TMemoryDeviceType>
struct RetrieveGradientLengthAndCountFunctor<TWarp, TMemoryDeviceType, true> {
	_CPU_AND_GPU_CODE_
	inline static SumAndCount retrieve(const TWarp& voxel) {
		float length = ORUtils::length(voxel.gradient1);
		return {length, static_cast<unsigned int>(length != 0.0f)};
	}
};

template<typename TWarp, MemoryDeviceType TMemoryDeviceType>
struct RetrieveGradientLengthAndCountFunctor<TWarp, TMemoryDeviceType, false> {
	_CPU_AND_GPU_CODE_
	inline static SumAndCount retrieve(const TWarp& voxel) {
		float length = ORUtils::length(voxel.gradient0);
		return {length, static_cast<unsigned int>(length != 0.0f)};
	}
};


template<typename TIndex, MemoryDeviceType TMemoryDeviceType>
struct ReduceMaximumFunctor {
public:
	_CPU_AND_GPU_CODE_
	inline static const ReductionResult<float, TIndex>& reduce(
			const ReductionResult<float, TIndex>& item1, const ReductionResult<float, TIndex>& item2) {
		return (item1.value > item2.value) ? item1 : item2;
	}
};


template<typename TIndex, MemoryDeviceType TMemoryDeviceType>
struct ReduceSumAndCountFunctor {
public:
	_CPU_AND_GPU_CODE_
	inline static const ReductionResult<SumAndCount, TIndex> reduce(
			const ReductionResult<SumAndCount, TIndex>& item1, const ReductionResult<SumAndCount, TIndex>& item2) {
		ReductionResult<SumAndCount, TIndex> output;
		output.value.sum = item1.value.sum + item2.value.sum;
		output.value.count = item1.value.count + item2.value.count;
		return output;
	}
};

} // namespace ITMLib