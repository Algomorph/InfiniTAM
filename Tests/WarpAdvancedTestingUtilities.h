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

//local
#include "../ITMLib/SurfaceTrackers/Interface/SurfaceTracker.h"

using namespace ITMLib;

enum GenericWarpTestMode {
	SAVE_SUCCESSIVE_ITERATIONS,
	SAVE_FINAL_ITERATION_AND_FUSION,
	TEST_SUCCESSIVE_ITERATIONS,
	TEST_FINAL_ITERATION_AND_FUSION
};

std::string get_path_warps(std::string prefix, int iteration);
std::string get_path_warped_live(std::string prefix, int iteration);
std::string get_path_fused(std::string prefix, int iteration);
unsigned int switches_to_int_code(const SlavchevaSurfaceTracker::Switches& switches);
std::string switches_to_prefix(const SlavchevaSurfaceTracker::Switches& switches);

template<typename TIndex>
std::string getIndexString();

template<typename TIndex, MemoryDeviceType TMemoryDeviceType>
void
GenericWarpConsistencySubtest(const SlavchevaSurfaceTracker::Switches& switches,
                              int iteration_limit = 10,
                              GenericWarpTestMode mode = TEST_SUCCESSIVE_ITERATIONS,
                              float absolute_tolerance = 1e-7);

template<MemoryDeviceType TMemoryDeviceType>
void Warp_PVA_VBH_simple_subtest(int iteration, SlavchevaSurfaceTracker::Switches trackerSwitches,
		bool expanded_allocation = true);