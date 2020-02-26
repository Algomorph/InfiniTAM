//  ================================================================
//  Created by Gregory Kramida on 10/3/19.
//  Copyright (c) 2019 Gregory Kramida
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

#include "../../../../ORUtils/PlatformIndependence.h"
#include "../AlmostEqual.h"

namespace ITMLib{
template<typename TVoxel, typename ToleranceType>
struct VoxelEqualFunctor {
	explicit VoxelEqualFunctor(ToleranceType tolerance) : tolerance(tolerance) {}
	_CPU_AND_GPU_CODE_
	inline
	bool operator()(const TVoxel& a, const TVoxel& b) const {
		return almostEqual(a, b, tolerance);
	}
	ToleranceType tolerance;
};

template<typename TVoxel, typename ToleranceType>
struct VoxelEqualVerboseFunctor {
	explicit VoxelEqualVerboseFunctor(ToleranceType tolerance) : tolerance(tolerance) {}
	_CPU_AND_GPU_CODE_
	inline
	bool operator()(const TVoxel& a, const TVoxel& b, const Vector3i& position) const {
		return almostEqualVerbose_Position(a, b, position, tolerance);
	}
	ToleranceType tolerance;
};
} // namespace ITMLib


