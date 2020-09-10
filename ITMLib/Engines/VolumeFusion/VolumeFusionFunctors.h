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


//#define FUSION_CONDITION_BOTH_NONTRUNCATED
//#define FUSION_CONDITION_COMBINED
#define FUSION_CONDITION_LIVE_NONTRUNCATED
//#define FUSION_CONDITION_LIVE_KNOWN

//#define SET_TRUNCATED_TO_UNKNOWN_DURING_FUSION
#define TRUNCATE_DURING_FUSION

namespace ITMLib {

// MemoryDeviceType template parameter needed to disambiguate linker symbols for which PlatformIndependence macros are
// defined differently
template<typename TVoxel, MemoryDeviceType TMemoryDeviceType, bool TUseSurfaceThicknessCutoff>
struct TSDFFusionFunctor {
	TSDFFusionFunctor(int maximum_weight, unsigned short timestamp, float negative_surface_thickness_sdf_scale) :
			maximum_weight(maximum_weight), timestamp(timestamp),
			negative_surface_thickness_sdf_scale(negative_surface_thickness_sdf_scale){}

	_CPU_AND_GPU_CODE_
	void operator()(TVoxel& source_voxel, TVoxel& target_voxel) {
		//TODO: remove or utilize dead code

		// observation: fusion condition "HARSH" yields results almost identical to "COMBINED"
#if defined(FUSION_CONDITION_BOTH_NONTRUNCATED)
		if(canonicalVoxel.flags != VOXEL_NONTRUNCATED
				   && liveVoxel.flags != VOXEL_NONTRUNCATED) return;
#elif defined (FUSION_CONDITION_COMBINED)
		if (source_voxel.flags == ITMLib::VoxelFlags::VOXEL_UNKNOWN
			|| (target_voxel.flags != ITMLib::VoxelFlags::VOXEL_NONTRUNCATED
				&& source_voxel.flags != ITMLib::VoxelFlags::VOXEL_NONTRUNCATED))
			return;
		// observation: fusion condition "LIVE_NONTRUNCATED" yields results almost identical to "LIVE_KNOWN"
#elif defined (FUSION_CONDITION_LIVE_NONTRUNCATED)
		// fusion condition "LIVE_NONTRUNCATED" is the latest being tested
		if (source_voxel.flags != ITMLib::VOXEL_NONTRUNCATED) return;
#elif defined (FUSION_CONDITION_LIVE_KNOWN)
		if(source_voxel.flags == ITMLib::VOXEL_UNKNOWN) return;
#endif

		float live_sdf = TVoxel::valueToFloat(source_voxel.sdf);

		if(TUseSurfaceThicknessCutoff){
			if (live_sdf < negative_surface_thickness_sdf_scale)
				return;
		}

		int old_depth_weight = target_voxel.w_depth;
		float old_sdf = TVoxel::valueToFloat(target_voxel.sdf);

		float new_sdf = old_depth_weight * old_sdf + live_sdf;
		float new_depth_weight = old_depth_weight + 1.0f;
		new_sdf /= new_depth_weight;
		new_depth_weight = ORUTILS_MIN(new_depth_weight, maximum_weight);

		target_voxel.sdf = TVoxel::floatToValue(new_sdf);
		target_voxel.w_depth = (uchar) new_depth_weight;

		if (1.0f - std::abs(new_sdf) < 1e-5f) {
#if defined(TRUNCATE_DURING_FUSION)
			target_voxel.flags = ITMLib::VoxelFlags::VOXEL_TRUNCATED;
#elif defined(SET_TRUNCATED_TO_UNKNOWN_DURING_FUSION)
			target_voxel.flags = ITMLib::VoxelFlags::VOXEL_UNKNOWN;
			target_voxel.w_depth = (uchar) 0;
#endif
		} else {

			target_voxel.flags = ITMLib::VoxelFlags::VOXEL_NONTRUNCATED;


		}
	}

private:
	const float negative_surface_thickness_sdf_scale;
	const int maximum_weight;
	const unsigned short timestamp;
};


} // namespace ITMLib