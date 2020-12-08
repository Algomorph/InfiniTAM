//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 10/8/20.
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

#include "TestCaseOrganizationBySwitches.tpp"
#include "../../../ITMLib/Objects/Volume/PlainVoxelArray.h"
#include "../../../ITMLib/Objects/Volume/VoxelBlockHash.h"

namespace test {

template std::string GetWarpsPath<PlainVoxelArray>(std::string prefix, int iteration);
template std::string GetWarpsPath<VoxelBlockHash>(std::string prefix, int iteration);
template std::string GetWarpedLivePath<PlainVoxelArray>(std::string prefix, int iteration);
template std::string GetWarpedLivePath<VoxelBlockHash>(std::string prefix, int iteration);
template std::string GetFusedPath<PlainVoxelArray>(std::string prefix, int iteration);
template std::string GetFusedPath<VoxelBlockHash>(std::string prefix, int iteration);

std::string SwitchesToPrefix(const LevelSetAlignmentSwitches& switches) {
	static std::unordered_map<unsigned int, std::string> prefix_by_switches_map = {
			{SwitchesToIntCode(LevelSetAlignmentSwitches(true, false, false, false, false)),
					                                              "data"},
			{SwitchesToIntCode(LevelSetAlignmentSwitches(true, false, false, false, true)),
					                                              "data_sobolev"},
			{SwitchesToIntCode(LevelSetAlignmentSwitches(false, false, true, false, false)),
					                                              "tikhonov"},
			{SwitchesToIntCode(LevelSetAlignmentSwitches(true, false, true, false,
			                                             false)), "data_tikhonov"},
			{SwitchesToIntCode(LevelSetAlignmentSwitches(true, false, true, false, true)),
					                                              "data_tikhonov_sobolev"},
			{SwitchesToIntCode(LevelSetAlignmentSwitches(true, false, true, true, false)),
					                                              "data_killing"},
			{SwitchesToIntCode(LevelSetAlignmentSwitches(true, true, false, false, false)),
					                                              "data_level_set"},
			{SwitchesToIntCode(LevelSetAlignmentSwitches(true, true, true, true, false)),
					                                              "data_killing_level_set"}
	};
	return prefix_by_switches_map[SwitchesToIntCode(switches)];
}


unsigned int SwitchesToIntCode(const LevelSetAlignmentSwitches& switches) {
	unsigned int code = 0;
	code |= static_cast<unsigned int>(switches.enable_data_term) << 0u;
	code |= static_cast<unsigned int>(switches.enable_level_set_term) << 1u;
	code |= static_cast<unsigned int>(switches.enable_smoothing_term) << 2u;
	code |= static_cast<unsigned int>(switches.enable_Killing_field) << 3u;
	code |= static_cast<unsigned int>(switches.enable_Sobolev_gradient_smoothing) << 4u;
	return code;
}


} // namespace test_utilities