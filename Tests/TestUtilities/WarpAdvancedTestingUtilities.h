//  ================================================================
//  Created by Gregory Kramida on 12/17/19.
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

//std
#include <string>
#include <unordered_map>

//ITMLib
#include "../../ITMLib/Engines/LevelSetAlignment/Interface/LevelSetAlignmentEngine.h"

//local
#include "TestUtilities.h"


using namespace ITMLib;

namespace test_utilities {

enum GenericWarpTestMode {
	SAVE_SUCCESSIVE_ITERATIONS,
	SAVE_FINAL_ITERATION_AND_FUSION,
	TEST_SUCCESSIVE_ITERATIONS,
	TEST_FINAL_ITERATION_AND_FUSION
};

template<typename TIndex>
std::string GetWarpsPath(std::string prefix, int iteration);

template<typename TIndex>
std::string GetWarpedLivePath(std::string prefix, int iteration);

template<typename TIndex>
std::string GetFusedPath(std::string prefix, int iteration);

unsigned int SwitchesToIntCode(const LevelSetAlignmentSwitches& switches);
std::string SwitchesToPrefix(const LevelSetAlignmentSwitches& switches);


template<typename TIndex, MemoryDeviceType TMemoryDeviceType>
void
GenerateRawLiveAndCanonicalVolumes(VoxelVolume<TSDFVoxel, TIndex>** canonical_volume,
                                   VoxelVolume<TSDFVoxel, TIndex>** live_volume);

template<typename TIndex, MemoryDeviceType TMemoryDeviceType>
void
GenericWarpConsistencySubtest(const LevelSetAlignmentSwitches& switches,
                              int iteration_limit = 10,
                              GenericWarpTestMode mode = TEST_SUCCESSIVE_ITERATIONS,
                              float absolute_tolerance = 1e-7);

template<MemoryDeviceType TMemoryDeviceType>
void PVA_to_VBH_WarpComparisonSubtest(int iteration, LevelSetAlignmentSwitches trackerSwitches);


template<MemoryDeviceType TMemoryDeviceType>
void
GenericWarpTest(const LevelSetAlignmentSwitches& switches, int iteration_limit = 10,
                GenericWarpTestMode mode = TEST_SUCCESSIVE_ITERATIONS, float absolute_tolerance = 1e-7);

const LevelSetAlignmentTerminationConditions& SingleIterationTerminationConditions();

} // namespace test_utilities