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


template
void
GenericWarpConsistencySubtest<PlainVoxelArray, MEMORYDEVICE_CPU>(const SlavchevaSurfaceTracker::Switches& switches,
                                                                    int iteration_limit,
                                                                    GenericWarpTestMode mode,
                                                                    float absolute_tolerance);
template
void
GenericWarpConsistencySubtest<VoxelBlockHash, MEMORYDEVICE_CPU>(const SlavchevaSurfaceTracker::Switches& switches,
                                                                   int iteration_limit,
                                                                   GenericWarpTestMode mode,
                                                                   float absolute_tolerance);
template
void Warp_PVA_VBH_simple_subtest<MEMORYDEVICE_CPU>(int iteration, SlavchevaSurfaceTracker::Switches trackerSwitches);

#ifndef COMPILE_WITHOUT_CUDA
template
void
GenericWarpConsistencySubtest<PlainVoxelArray, MEMORYDEVICE_CUDA>(const SlavchevaSurfaceTracker::Switches& switches,
                                                                     int iteration_limit,
                                                                     GenericWarpTestMode mode,
                                                                     float absolute_tolerance);

template
void
GenericWarpConsistencySubtest<VoxelBlockHash, MEMORYDEVICE_CUDA>(const SlavchevaSurfaceTracker::Switches& switches,
                                                                    int iteration_limit,
                                                                    GenericWarpTestMode mode,
                                                                    float absolute_tolerance);

template
void Warp_PVA_VBH_simple_subtest<MEMORYDEVICE_CUDA>(int iteration, SlavchevaSurfaceTracker::Switches trackerSwitches);
#endif

std::string get_path_warps(std::string prefix, int iteration) {
	return "TestData/snoopy_result_fr16-17_warps/" + prefix + "_iter_" + std::to_string(iteration) + "_";
}

std::string get_path_warped_live(std::string prefix, int iteration) {
	return get_path_warps(prefix, iteration) + "warped_live_";
}

std::string get_path_fused(std::string prefix, int iteration) {
	return get_path_warps(prefix, iteration) + "fused_";
}


unsigned int switches_to_int_code(const SlavchevaSurfaceTracker::Switches& switches) {
	unsigned int code = 0;
	code |= static_cast<unsigned int>(switches.enable_data_term) << 0u;
	code |= static_cast<unsigned int>(switches.enable_level_set_term) << 1u;
	code |= static_cast<unsigned int>(switches.enable_smoothing_term) << 2u;
	code |= static_cast<unsigned int>(switches.enable_killing_rigidity_enforcement_term) << 3u;
	code |= static_cast<unsigned int>(switches.enable_sobolev_gradient_smoothing) << 4u;
	return code;
}

std::string switches_to_prefix(const SlavchevaSurfaceTracker::Switches& switches) {
	static std::unordered_map<unsigned int, std::string> prefix_by_switches_map = {
			{switches_to_int_code(SlavchevaSurfaceTracker::Switches(true, false, false, false, false)),
					                                                         "data_only"},
			{switches_to_int_code(SlavchevaSurfaceTracker::Switches(false, false, true, false, false)),
					                                                         "tikhonov_only"},
			{switches_to_int_code(SlavchevaSurfaceTracker::Switches(true, false, true, false,
			                                                        false)), "data_tikhonov"},
			{switches_to_int_code(SlavchevaSurfaceTracker::Switches(true, false, true, false, true)),
					                                                         "data_tikhonov_sobolev"}
	};
	return prefix_by_switches_map[switches_to_int_code(switches)];
}


template<>
std::string getIndexString<PlainVoxelArray>() { return "PVA"; }

template<>
std::string getIndexString<VoxelBlockHash>() { return "VBH"; }


template void GenerateRawLiveAndCanonicalVolumes<VoxelBlockHash, MEMORYDEVICE_CPU>(VoxelVolume<TSDFVoxel, VoxelBlockHash>** canonical_volume,
                                                                                    VoxelVolume<TSDFVoxel, VoxelBlockHash>** live_volume);
template void GenerateRawLiveAndCanonicalVolumes<PlainVoxelArray, MEMORYDEVICE_CPU>(VoxelVolume<TSDFVoxel, PlainVoxelArray>** canonical_volume,
                                                                                    VoxelVolume<TSDFVoxel, PlainVoxelArray>** live_volume);
#ifndef COMPILE_WITHOUT_CUDA
template void GenerateRawLiveAndCanonicalVolumes<VoxelBlockHash, MEMORYDEVICE_CUDA>(VoxelVolume<TSDFVoxel, VoxelBlockHash>** canonical_volume,
                                                                                    VoxelVolume<TSDFVoxel, VoxelBlockHash>** live_volume);
template void GenerateRawLiveAndCanonicalVolumes<PlainVoxelArray, MEMORYDEVICE_CUDA>(VoxelVolume<TSDFVoxel, PlainVoxelArray>** canonical_volume,
                                                                                    VoxelVolume<TSDFVoxel, PlainVoxelArray>** live_volume);
#endif
