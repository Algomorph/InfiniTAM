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
	TSDFFusionFunctor(int maximum_weight, unsigned short timestamp) :
			maximum_weight(maximum_weight), timestamp(timestamp) {}

	_CPU_AND_GPU_CODE_
	void operator()(TVoxel& source_voxel, TVoxel& target_voxel) {
		//TODO: remove or utilize dead code

		//fusion condition "HARSH" -- yields results almost identical to "COMBINED"
//		if(canonicalVoxel.flags != VOXEL_NONTRUNCATED
//				   && liveVoxel.flags != VOXEL_NONTRUNCATED) return;

		//fusion condition "COMBINED"
//		if (source_voxel.flags == ITMLib::VoxelFlags::VOXEL_UNKNOWN
//		    || (target_voxel.flags != ITMLib::VoxelFlags::VOXEL_NONTRUNCATED
//		        && source_voxel.flags != ITMLib::VoxelFlags::VOXEL_NONTRUNCATED))
//			return;

		//fusion condition "LIVE_NONTRUNCATED"
		if(source_voxel.flags != ITMLib::VOXEL_NONTRUNCATED) return;

		float live_sdf = TVoxel::valueToFloat(source_voxel.sdf);

		// negated parameter eta from 'SDF-2-SDF Registration for Real-Time 3D Reconstruction from RGB-D Data',
		// Sec. 3.2, and SobolevFusion, Sec. 3.1, as well as 'mu' from original KinectFusion(2011), divided by voxel size
		// (voxel size, m) / (narrow-band half-width eta, m) * -("2-3 voxels")
		// we use .3 for the latter value, which means 3 voxels if the max SDF value is 1.0 and values are truncated
		// after 10 voxels in each direction.
		const float threshold = -0.3f;

		//fusion condition "TRUNCATED_THRESHOLD"
		if (live_sdf < threshold)
			return;

		//fusion condition "LIVE_UNKNOWN"
//		if(liveVoxel.flags == VOXEL_UNKNOWN) return;

		int old_depth_weight = target_voxel.w_depth;
		float old_sdf = TVoxel::valueToFloat(target_voxel.sdf);

		float new_sdf = old_depth_weight * old_sdf + live_sdf;
		float new_depth_weight = old_depth_weight + 1.0f;
		new_sdf /= new_depth_weight;
		new_depth_weight = ORUTILS_MIN(new_depth_weight, maximum_weight);

		target_voxel.sdf = TVoxel::floatToValue(new_sdf);
		target_voxel.w_depth = (uchar) new_depth_weight;

		if (1.0f - std::abs(new_sdf) < 1e-5f) {
			target_voxel.flags = ITMLib::VoxelFlags::VOXEL_TRUNCATED;
		} else {
			target_voxel.flags = ITMLib::VoxelFlags::VOXEL_NONTRUNCATED;
		}
	}

private:
	const int maximum_weight;
	const unsigned short timestamp;
};


