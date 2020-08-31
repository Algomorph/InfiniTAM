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
#include "../../../../ORUtils/PlatformIndependentAtomics.h"

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

	float GetAverageCanonicalSdf() const{
		return GET_ATOMIC_VALUE_CPU(cumulative_canonical_sdf) / GET_ATOMIC_VALUE_CPU(considered_voxel_count);
	}
	float GetAverageLiveSdf() const{
		return GET_ATOMIC_VALUE_CPU(cumulative_live_sdf) / GET_ATOMIC_VALUE_CPU(considered_voxel_count);
	}
	float GetAverageSdfDifference() const{
		return GET_ATOMIC_VALUE_CPU(cumulative_sdf_diff) / GET_ATOMIC_VALUE_CPU(considered_voxel_count);
	}
	float GetAverageWarpDistance() const{
		unsigned int data_voxel_count_CPU = GET_ATOMIC_VALUE_CPU(data_voxel_count);
		if(data_voxel_count_CPU > 0){
			return GET_ATOMIC_VALUE_CPU(cumulative_warp_dist) / data_voxel_count_CPU;
		}else{
			return 0.0f;
		}
	}
	unsigned int GetConsideredVoxelCount() const{
		return GET_ATOMIC_VALUE_CPU(considered_voxel_count);
	}

	unsigned int GetDataVoxelCount() const{
		return GET_ATOMIC_VALUE_CPU(data_voxel_count);
	}
	unsigned int GetLevelSetVoxelCount() const{
		return GET_ATOMIC_VALUE_CPU(level_set_voxel_count);
	}

};

template<MemoryDeviceType TMemoryDeviceType>
struct ComponentEnergies{
	ComponentEnergies(){
		INITIALIZE_ATOMIC(float, total_data_energy, 0.f);
		INITIALIZE_ATOMIC(float, total_level_set_energy, 0.f);
		INITIALIZE_ATOMIC(float, total_Tikhonov_energy, 0.f);
		INITIALIZE_ATOMIC(float, total_Killing_energy, 0.f);

		INITIALIZE_ATOMIC(float, combined_data_length, 0.f);
		INITIALIZE_ATOMIC(float, combined_level_set_length, 0.f);
		INITIALIZE_ATOMIC(float, combined_smoothing_length, 0.f);
	}
	~ComponentEnergies(){
		CLEAN_UP_ATOMIC(total_data_energy);
		CLEAN_UP_ATOMIC(total_level_set_energy);
		CLEAN_UP_ATOMIC(total_Tikhonov_energy);
		CLEAN_UP_ATOMIC(total_Killing_energy);

		CLEAN_UP_ATOMIC(combined_data_length);
		CLEAN_UP_ATOMIC(combined_level_set_length);
		CLEAN_UP_ATOMIC(combined_smoothing_length);
	}
	DECLARE_ATOMIC_FLOAT(total_data_energy);
	DECLARE_ATOMIC_FLOAT(total_level_set_energy);
	DECLARE_ATOMIC_FLOAT(total_Tikhonov_energy);
	DECLARE_ATOMIC_FLOAT(total_Killing_energy);

	DECLARE_ATOMIC_FLOAT(combined_data_length);
	DECLARE_ATOMIC_FLOAT(combined_level_set_length);
	DECLARE_ATOMIC_FLOAT(combined_smoothing_length);

	float GetTotalDataEnergy() const{
		return GET_ATOMIC_VALUE_CPU(total_data_energy);
	}
	float GetTotalLevelSetEnergy() const{
		return GET_ATOMIC_VALUE_CPU(total_level_set_energy);
	}
	float GetTotalTikhonovEnergy() const{
		return GET_ATOMIC_VALUE_CPU(total_Tikhonov_energy);
	}
	float GetTotalKillingEnergy() const{
		return GET_ATOMIC_VALUE_CPU(total_Killing_energy);
	}

	float GetCombinedDataLength() const{
		return GET_ATOMIC_VALUE_CPU(combined_data_length);
	}
	float GetCombinedLevelSetLength() const{
		return GET_ATOMIC_VALUE_CPU(combined_level_set_length);
	}
	float GetCombinedSmoothingLength() const{
		return GET_ATOMIC_VALUE_CPU(combined_smoothing_length);
	}
};

