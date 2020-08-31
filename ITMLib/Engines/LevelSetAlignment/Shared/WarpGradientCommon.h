//  ================================================================
//  Created by Gregory Kramida on 6/7/18.
//  Copyright (c) 2018-2000 Gregory Kramida
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


#include <iostream>
#include "../../../Utils/Logging/ConsolePrintColors.h"
#include "WarpGradientAggregates.h"
#include "../../../Utils/Logging/Logging.h"
#include "../../../../ORUtils/MemoryDeviceType.h"

namespace ITMLib {
// region ==================================== STATIC PRINTING / STATISTICS FUNCTIONS ==================================

inline static
void PrintEnergyStatistics(const bool& enable_data_term,
                           const bool& enable_level_set_term,
                           const bool& enable_smoothing_term,
                           const bool& enable_Killing_term,
                           const float& gamma,
                           float total_data_energy,
                           float total_level_set_energy,
                           float total_tikhonov_energy,
                           float total_killing_energy) {
	std::stringstream stringstream;
	stringstream << "[ENERGY]";

	double total_energy = 0.f;
	if (enable_data_term) {
		stringstream << blue << " Data term: " << total_data_energy;
		total_energy += total_data_energy;
	}
	if (enable_level_set_term) {
		stringstream << red << " Level set term: " << total_level_set_energy;
		total_energy += total_level_set_energy;
	}
	if (enable_smoothing_term) {
		if (enable_Killing_term) {
			stringstream << yellow << " Tikhonov sub-term: " << total_tikhonov_energy;
			stringstream << yellow << " Killing sub-term: " << total_killing_energy;
		}
		double total_smoothing_energy = total_tikhonov_energy + total_killing_energy;
		stringstream << cyan << " Smoothing term (total): " << total_smoothing_energy;
		total_energy += total_smoothing_energy;
	}
	stringstream << green << " Total: " << total_energy << reset;
	LOG4CPLUS_PER_ITERATION(logging::get_logger(), stringstream.str());


}
template <MemoryDeviceType TMemoryDeviceType>
inline static
void PrintEnergyStatistics(const bool& enable_data_term,
                           const bool& enable_level_set_term,
                           const bool& enable_smoothing_term,
                           const bool& enable_killing_term,
                           const float& gamma,
                           ComponentEnergies<TMemoryDeviceType>& energies) {
	float total_data_energy = GET_ATOMIC_VALUE_CPU(energies.total_data_energy);
	float total_level_set_energy = GET_ATOMIC_VALUE_CPU(energies.total_level_set_energy);
	float total_tikhonov_energy = GET_ATOMIC_VALUE_CPU(energies.total_Tikhonov_energy);
	float total_rigidity_energy = GET_ATOMIC_VALUE_CPU(energies.total_Killing_energy);
	PrintEnergyStatistics(enable_data_term, enable_level_set_term, enable_smoothing_term, enable_killing_term, gamma,
	                      total_data_energy, total_level_set_energy, total_tikhonov_energy, total_rigidity_energy);
	std::stringstream stringstream;
	stringstream << "[CGL]";
	if(enable_data_term){
		stringstream << blue << " Data term: " << energies.GetCombinedDataLength();
	}
	if(enable_level_set_term){
		stringstream << red << " Level set term: " << energies.GetCombinedLevelSetLength();
	}
	if(enable_smoothing_term){
		stringstream << cyan << " Smoothing term: " << energies.GetCombinedSmoothingLength();
	}
	LOG4CPLUS_PER_ITERATION(logging::get_logger(), stringstream.str());
}


inline static
void CalculateAndPrintAdditionalStatistics(const bool& enable_data_term,
                                           const bool& enable_level_set_term,
                                           double cumulative_canonical_sdf,
                                           double cumulative_live_sdf,
                                           double cumulative_warp_dist,
                                           double cumulative_sdf_diff,
                                           unsigned int considered_voxel_count,
                                           unsigned int data_voxel_count,
                                           unsigned int level_set_voxel_count,
                                           unsigned int used_hash_block_count = 0) {

	double average_canonical_sdf = cumulative_canonical_sdf / considered_voxel_count;
	double average_live_sdf = cumulative_live_sdf / considered_voxel_count;
	double average_warp_distance = cumulative_warp_dist / considered_voxel_count;
	double average_sdf_difference = 0.0;

	if (enable_data_term) {
		average_sdf_difference = cumulative_sdf_diff / data_voxel_count;
	}

	std::cout << " Ave canonical SDF: " << average_canonical_sdf
	          << " Ave live SDF: " <<
	          average_live_sdf;
	if (enable_data_term) {
		std::cout << " Ave SDF diff: " <<
		          average_sdf_difference;
	}
	std::cout << " Used voxel count: " << considered_voxel_count
	          << " Data term v-count: " << data_voxel_count;
	if (enable_level_set_term) {
		std::cout << " LS term v-count: " << level_set_voxel_count;
	}
	std::cout << " Ave warp distance: " << average_warp_distance;


	if (used_hash_block_count > 0) {
		std::cout << " Used hash block count: " <<
		          used_hash_block_count;
	}
	std::cout << std::endl;
}

template<MemoryDeviceType TMemoryDeviceType>
inline static
void CalculateAndPrintAdditionalStatistics(const bool& enable_data_term,
                                           const bool& enable_level_set_term,
                                           AdditionalGradientAggregates<TMemoryDeviceType>& aggregates,
                                           const unsigned int used_hashblock_count = 0) {

	unsigned int considered_voxel_count = GET_ATOMIC_VALUE_CPU(aggregates.considered_voxel_count);
	unsigned int data_voxel_count = GET_ATOMIC_VALUE_CPU(aggregates.data_voxel_count);
	unsigned int level_set_voxel_count = GET_ATOMIC_VALUE_CPU(aggregates.level_set_voxel_count);
	double cumulative_canonical_sdf = GET_ATOMIC_VALUE_CPU(aggregates.cumulative_canonical_sdf);
	double cumulative_live_sdf = GET_ATOMIC_VALUE_CPU(aggregates.cumulative_live_sdf);
	double cumulative_warp_dist = GET_ATOMIC_VALUE_CPU(aggregates.cumulative_warp_dist);
	double cumulative_sdf_diff = GET_ATOMIC_VALUE_CPU(aggregates.cumulative_sdf_diff);

	CalculateAndPrintAdditionalStatistics(enable_data_term, enable_level_set_term, cumulative_canonical_sdf,
	                                      cumulative_live_sdf, cumulative_warp_dist, cumulative_sdf_diff,
	                                      considered_voxel_count, data_voxel_count,
	                                      level_set_voxel_count, used_hashblock_count);
}

// endregion ===========================================================================================================
}//namespace ITMLib