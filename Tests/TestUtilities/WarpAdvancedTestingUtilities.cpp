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
//local
#include "WarpAdvancedTestingUtilities.tpp"

namespace test_utilities {

template std::string GetWarpsPath<PlainVoxelArray>(std::string prefix, int iteration);
template std::string GetWarpsPath<VoxelBlockHash>(std::string prefix, int iteration);
template std::string GetWarpedLivePath<PlainVoxelArray>(std::string prefix, int iteration);
template std::string GetWarpedLivePath<VoxelBlockHash>(std::string prefix, int iteration);
template std::string GetFusedPath<PlainVoxelArray>(std::string prefix, int iteration);
template std::string GetFusedPath<VoxelBlockHash>(std::string prefix, int iteration);

template
void
GenericWarpConsistencySubtest<PlainVoxelArray, MEMORYDEVICE_CPU>(const LevelSetAlignmentSwitches& switches,
                                                                 int iteration_limit,
                                                                 GenericWarpTestMode mode,
                                                                 float absolute_tolerance);
template
void
GenericWarpConsistencySubtest<VoxelBlockHash, MEMORYDEVICE_CPU>(const LevelSetAlignmentSwitches& switches,
                                                                int iteration_limit,
                                                                GenericWarpTestMode mode,
                                                                float absolute_tolerance);
template
void Warp_PVA_VBH_simple_subtest<MEMORYDEVICE_CPU>(int iteration, LevelSetAlignmentSwitches trackerSwitches);

template
void GenericWarpTest<MEMORYDEVICE_CPU>(const LevelSetAlignmentSwitches& switches, int iteration_limit,
                                       GenericWarpTestMode mode, float absolute_tolerance);

#ifndef COMPILE_WITHOUT_CUDA
template
void
GenericWarpConsistencySubtest<PlainVoxelArray, MEMORYDEVICE_CUDA>(const LevelSetAlignmentSwitches& switches,
                                                                  int iteration_limit,
                                                                  GenericWarpTestMode mode,
                                                                  float absolute_tolerance);

template
void
GenericWarpConsistencySubtest<VoxelBlockHash, MEMORYDEVICE_CUDA>(const LevelSetAlignmentSwitches& switches,
                                                                 int iteration_limit,
                                                                 GenericWarpTestMode mode,
                                                                 float absolute_tolerance);

template
void Warp_PVA_VBH_simple_subtest<MEMORYDEVICE_CUDA>(int iteration, LevelSetAlignmentSwitches trackerSwitches);

template
void GenericWarpTest<MEMORYDEVICE_CUDA>(const LevelSetAlignmentSwitches& switches, int iteration_limit,
                                        GenericWarpTestMode mode, float absolute_tolerance);
#endif


unsigned int SwitchesToIntCode(const LevelSetAlignmentSwitches& switches) {
	unsigned int code = 0;
	code |= static_cast<unsigned int>(switches.enable_data_term) << 0u;
	code |= static_cast<unsigned int>(switches.enable_level_set_term) << 1u;
	code |= static_cast<unsigned int>(switches.enable_smoothing_term) << 2u;
	code |= static_cast<unsigned int>(switches.enable_killing_rigidity_enforcement_term) << 3u;
	code |= static_cast<unsigned int>(switches.enable_sobolev_gradient_smoothing) << 4u;
	return code;
}

std::string SwitchesToPrefix(const LevelSetAlignmentSwitches& switches) {
	static std::unordered_map<unsigned int, std::string> prefix_by_switches_map = {
			{SwitchesToIntCode(LevelSetAlignmentSwitches(true, false, false, false, false)),
					                                                         "data_only"},
			{SwitchesToIntCode(LevelSetAlignmentSwitches(false, false, true, false, false)),
					                                                         "tikhonov_only"},
			{SwitchesToIntCode(LevelSetAlignmentSwitches(true, false, true, false,
			                                                     false)), "data_tikhonov"},
			{SwitchesToIntCode(LevelSetAlignmentSwitches(true, false, true, false, true)),
					                                                         "data_tikhonov_sobolev"}
	};
	return prefix_by_switches_map[SwitchesToIntCode(switches)];
}

template void GenerateRawLiveAndCanonicalVolumes<VoxelBlockHash, MEMORYDEVICE_CPU>(
		VoxelVolume<TSDFVoxel, VoxelBlockHash>** canonical_volume,
		VoxelVolume<TSDFVoxel, VoxelBlockHash>** live_volume);
template void GenerateRawLiveAndCanonicalVolumes<PlainVoxelArray, MEMORYDEVICE_CPU>(
		VoxelVolume<TSDFVoxel, PlainVoxelArray>** canonical_volume,
		VoxelVolume<TSDFVoxel, PlainVoxelArray>** live_volume);
#ifndef COMPILE_WITHOUT_CUDA
template void GenerateRawLiveAndCanonicalVolumes<VoxelBlockHash, MEMORYDEVICE_CUDA>(
		VoxelVolume<TSDFVoxel, VoxelBlockHash>** canonical_volume,
		VoxelVolume<TSDFVoxel, VoxelBlockHash>** live_volume);
template void GenerateRawLiveAndCanonicalVolumes<PlainVoxelArray, MEMORYDEVICE_CUDA>(
		VoxelVolume<TSDFVoxel, PlainVoxelArray>** canonical_volume,
		VoxelVolume<TSDFVoxel, PlainVoxelArray>** live_volume);
#endif

} // namespace test_utilities