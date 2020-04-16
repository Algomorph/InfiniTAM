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
#include "../../Utils/CPPPrintHelpers.h"
#include "WarpGradientAggregates.h"

namespace ITMLib {
// region ==================================== STATIC PRINTING / STATISTICS FUNCTIONS ==================================

inline static
void PrintEnergyStatistics(const bool& enable_data_term,
                           const bool& enable_level_set_term,
                           const bool& enable_smoothing_term,
                           const bool& enableRigidityTerm,
                           const float& gamma,
                           float totalDataEnergy,
                           float totalLevelSetEnergy,
                           float totalTikhonovEnergy,
                           float totalRigidityEnergy) {
	std::cout << " [ENERGY]";

	double totalEnergy = 0.f;
	if (enable_data_term) {
		std::cout << blue << " Data term: " << totalDataEnergy;
		totalEnergy += totalDataEnergy;
	}
	if (enable_level_set_term) {
		std::cout << red << " Level set term: " << totalLevelSetEnergy;
		totalEnergy += totalLevelSetEnergy;
	}
	if (enable_smoothing_term) {
		if (enableRigidityTerm) {
			std::cout << yellow << " Tikhonov term: " << totalTikhonovEnergy;
			std::cout << yellow << " Killing term: " << totalRigidityEnergy;
		}
		double totalSmoothingEnergy = totalTikhonovEnergy + totalRigidityEnergy;
		std::cout << cyan << " Smoothing term: " << totalSmoothingEnergy;
		totalEnergy += totalSmoothingEnergy;
	}
	std::cout << green << " Total: " << totalEnergy << reset << std::endl;
}
template <MemoryDeviceType TMemoryDeviceType>
inline static
void PrintEnergyStatistics(const bool& enable_data_term,
                           const bool& enable_level_set_term,
                           const bool& enable_smoothing_term,
                           const bool& enableRigidityTerm,
                           const float& gamma,
                           ComponentEnergies<TMemoryDeviceType>& energies) {
	float totalDataEnergy = GET_ATOMIC_VALUE_CPU(energies.totalDataEnergy);
	float totalLevelSetEnergy = GET_ATOMIC_VALUE_CPU(energies.totalLevelSetEnergy);
	float totalTikhonovEnergy = GET_ATOMIC_VALUE_CPU(energies.totalTikhonovEnergy);
	float totalRigidityEnergy = GET_ATOMIC_VALUE_CPU(energies.totalRigidityEnergy);
	PrintEnergyStatistics(enable_data_term, enable_level_set_term, enable_smoothing_term, enableRigidityTerm, gamma,
	                      totalDataEnergy, totalLevelSetEnergy, totalTikhonovEnergy, totalRigidityEnergy);
}


//cumulativeLiveSdf, cumulativeWarpDist, cumulativeSdfDiff, consideredVoxelCount, dataVoxelCount, levelSetVoxelCount;

inline static
void CalculateAndPrintAdditionalStatistics(const bool& enable_data_term,
                                           const bool& enable_level_set_term, double cumulativeCanonicalSdf,
                                           double cumulativeLiveSdf, double cumulativeWarpDist,
                                           double cumulativeSdfDiff,
                                           unsigned int consideredVoxelCount, unsigned int dataVoxelCount,
                                           unsigned int levelSetVoxelCount, unsigned int usedHashblockCount = 0) {

	double averageCanonicalSdf = cumulativeCanonicalSdf / consideredVoxelCount;
	double averageLiveSdf = cumulativeLiveSdf / consideredVoxelCount;
	double averageWarpDist = cumulativeWarpDist / consideredVoxelCount;
	double averageSdfDiff = 0.0;

	if (enable_data_term) {
		averageSdfDiff = cumulativeSdfDiff / dataVoxelCount;
	}

	std::cout << " Ave canonical SDF: " << averageCanonicalSdf
	          << " Ave live SDF: " <<
	          averageLiveSdf;
	if (enable_data_term) {
		std::cout << " Ave SDF diff: " <<
		          averageSdfDiff;
	}
	std::cout << " Used voxel count: " << consideredVoxelCount
	          << " Data term v-count: " << dataVoxelCount;
	if (enable_level_set_term) {
		std::cout << " LS term v-count: " << levelSetVoxelCount;
	}
	std::cout << " Ave warp distance: " <<
	          averageWarpDist;


	if (usedHashblockCount > 0) {
		std::cout << " Used hash block count: " <<
		          usedHashblockCount;
	}
	std::cout <<
	          std::endl;
}

template<MemoryDeviceType TMemoryDeviceType>
inline static
void CalculateAndPrintAdditionalStatistics(const bool& enable_data_term,
                                           const bool& enable_level_set_term,
                                           AdditionalGradientAggregates<TMemoryDeviceType>& aggregates,
                                           const unsigned int usedHashblockCount = 0) {

	unsigned int consideredVoxelCount = GET_ATOMIC_VALUE_CPU(aggregates.consideredVoxelCount);
	unsigned int dataVoxelCount = GET_ATOMIC_VALUE_CPU(aggregates.dataVoxelCount);
	unsigned int levelSetVoxelCount = GET_ATOMIC_VALUE_CPU(aggregates.levelSetVoxelCount);
	double cumulativeCanonicalSdf = GET_ATOMIC_VALUE_CPU(aggregates.cumulativeCanonicalSdf);
	double cumulativeLiveSdf = GET_ATOMIC_VALUE_CPU(aggregates.cumulativeLiveSdf);
	double cumulativeWarpDist = GET_ATOMIC_VALUE_CPU(aggregates.cumulativeWarpDist);
	double cumulativeSdfDiff = GET_ATOMIC_VALUE_CPU(aggregates.cumulativeSdfDiff);

	CalculateAndPrintAdditionalStatistics(enable_data_term, enable_level_set_term, cumulativeCanonicalSdf,
	                                      cumulativeLiveSdf, cumulativeWarpDist, cumulativeSdfDiff,
	                                      consideredVoxelCount, dataVoxelCount,
	                                      levelSetVoxelCount, usedHashblockCount);
}

// endregion ===========================================================================================================
}//namespace ITMLib