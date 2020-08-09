//  ================================================================
//  Created by Gregory Kramida on 10/15/19.
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
#include "../../../ORUtils/PlatformIndependentAtomics.h"

template<MemoryDeviceType TMemoryDeviceType>
struct AdditionalGradientAggregates{
	AdditionalGradientAggregates(){
		INITIALIZE_ATOMIC(float, cumulative_canonical_sdf, 0.f);
		INITIALIZE_ATOMIC(float, cumulative_live_sdf, 0.f);
		INITIALIZE_ATOMIC(float, cumulative_sdf_diff, 0.f);
		INITIALIZE_ATOMIC(float, cumulative_warp_dist, 0.f);

		INITIALIZE_ATOMIC(unsigned int, considered_voxel_count, 0u);
		INITIALIZE_ATOMIC(unsigned int, data_voxel_count, 0u);
		INITIALIZE_ATOMIC(unsigned int, level_set_voxel_count, 0u);
	}
	~AdditionalGradientAggregates(){
		CLEAN_UP_ATOMIC(cumulative_canonical_sdf);
		CLEAN_UP_ATOMIC(cumulative_live_sdf);
		CLEAN_UP_ATOMIC(cumulative_sdf_diff);
		CLEAN_UP_ATOMIC(cumulative_warp_dist);

		CLEAN_UP_ATOMIC(considered_voxel_count);
		CLEAN_UP_ATOMIC(data_voxel_count);
		CLEAN_UP_ATOMIC(level_set_voxel_count);
	}

	DECLARE_ATOMIC_FLOAT(cumulative_canonical_sdf);
	DECLARE_ATOMIC_FLOAT(cumulative_live_sdf);
	DECLARE_ATOMIC_FLOAT(cumulative_sdf_diff);
	DECLARE_ATOMIC_FLOAT(cumulative_warp_dist);

	DECLARE_ATOMIC_UINT(considered_voxel_count);
	DECLARE_ATOMIC_UINT(data_voxel_count);
	DECLARE_ATOMIC_UINT(level_set_voxel_count);
};

template<MemoryDeviceType TMemoryDeviceType>
struct ComponentEnergies{
	ComponentEnergies(){
		INITIALIZE_ATOMIC(float, total_data_energy, 0.f);
		INITIALIZE_ATOMIC(float, total_level_set_energy, 0.f);
		INITIALIZE_ATOMIC(float, total_Tikhonov_energy, 0.f);
		INITIALIZE_ATOMIC(float, total_Killing_energy, 0.f);
	}
	~ComponentEnergies(){
		CLEAN_UP_ATOMIC(total_data_energy);
		CLEAN_UP_ATOMIC(total_level_set_energy);
		CLEAN_UP_ATOMIC(total_Tikhonov_energy);
		CLEAN_UP_ATOMIC(total_Killing_energy);
	}
	DECLARE_ATOMIC_FLOAT(total_data_energy);
	DECLARE_ATOMIC_FLOAT(total_level_set_energy);
	DECLARE_ATOMIC_FLOAT(total_Tikhonov_energy);
	DECLARE_ATOMIC_FLOAT(total_Killing_energy);
};

