//  ================================================================
//  Created by Gregory Kramida on 1/30/20.
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

//local
#include "../../../ORUtils/PlatformIndependence.h"
#include "../../../ORUtils/MemoryDeviceType.h"

// MemoryDeviceType template parameter needed to disambiguate linker symbols for which PlatformIndependence macros are
// defined differently
template<typename TVoxel, MemoryDeviceType TMemoryDeviceType>
struct TSDFFusionFunctor {
	TSDFFusionFunctor(int maximumWeight) :
			maximumWeight(maximumWeight) {}

	_CPU_AND_GPU_CODE_
	void operator()(TVoxel& liveVoxel, TVoxel& canonicalVoxel) {
		//_DEBUG

		//fusion condition "HARSH" -- yields results almost identical to "COMBINED"
//		if(canonicalVoxel.flags != VOXEL_NONTRUNCATED
//				   && liveVoxel.flag_values[liveSourceFieldIndex] != VOXEL_NONTRUNCATED) return;

		//fusion condition "COMBINED"
		if (liveVoxel.flags == ITMLib::VoxelFlags::VOXEL_UNKNOWN
		    || (canonicalVoxel.flags != ITMLib::VoxelFlags::VOXEL_NONTRUNCATED
		        && liveVoxel.flags != ITMLib::VoxelFlags::VOXEL_NONTRUNCATED))
			return;

		float liveSdf = TVoxel::valueToFloat(liveVoxel.sdf);

		// parameter eta from SobolevFusion, Sec. 3.1, divided by voxel size
		// (voxel size, m) / (narrow-band half-width eta, m) * -("2-3 voxels")
		// we use .3 for the latter value, which means 3 voxels if the max SDF value is 1.0 and values are truncated
		// after 10 voxels in each direction.
		const float threshold = -0.3;

		//fusion condition "THRESHOLD"
		if (liveVoxel.flags == ITMLib::VoxelFlags::VOXEL_UNKNOWN
		    || (canonicalVoxel.flags != ITMLib::VoxelFlags::VOXEL_NONTRUNCATED
		        && liveVoxel.flags != ITMLib::VoxelFlags::VOXEL_NONTRUNCATED) || liveSdf < threshold)
			return;

		//fusion condition "LIVE_UNKNOWN"
//		if(liveVoxel.flags == VOXEL_UNKNOWN) return;

		int oldWDepth = canonicalVoxel.w_depth;
		float oldSdf = TVoxel::valueToFloat(canonicalVoxel.sdf);

		float newSdf = oldWDepth * oldSdf + liveSdf;
		float newWDepth = oldWDepth + 1.0f;
		newSdf /= newWDepth;
		newWDepth = ORUTILS_MIN(newWDepth, maximumWeight);

		canonicalVoxel.sdf = TVoxel::floatToValue(newSdf);
		canonicalVoxel.w_depth = (uchar) newWDepth;
		if (canonicalVoxel.flags != ITMLib::VoxelFlags::VOXEL_NONTRUNCATED) {
			canonicalVoxel.flags = liveVoxel.flags;
		} else if (1.0f - std::abs(newSdf) < 1e-5f) {
			canonicalVoxel.flags = ITMLib::VoxelFlags::VOXEL_TRUNCATED;
		}
	}

private:
	const int maximumWeight;
};


