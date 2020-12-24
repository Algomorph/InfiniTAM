//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 12/24/20.
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
// === stdlib ===
#include <array>

// === ITMLib ===
#include "../../ITMLib/Utils/Logging/Logging.h"
#include "../../ITMLib/Engines/Indexing/IndexingEngineFactory.h"
#include "../../ITMLib/Engines/Analytics/AnalyticsEngineFactory.h"

// === test_utilities ===
#include "../TestUtilities/TestUtilities.h"
#include "../TestUtilities/TestDataUtilities.h"
#include "../TestUtilities/LevelSetAlignment/LevelSetAlignmentTestUtilities.h"
#include "../TestUtilities/LevelSetAlignment/SingleIterationTestConditions.h"
#include "../TestUtilities/LevelSetAlignment/TestCaseOrganizationBySwitches.h"

// === local ===
#include "GenerateLevelSetAlignmentDeviceComparisonData.h"


using namespace ITMLib;
using namespace test;

template<typename TIndex, MemoryDeviceType TMemoryDeviceType>
void GenerateWarpGradientTestData() {
	LOG4CPLUS_INFO(log4cplus::Logger::getRoot(),
	               "Generating warp field data from snoopy masked partial volumes 16 & 17 "
			               << IndexString<TIndex>() << "...");
	test::ConstructGeneratedVolumeSubdirectoriesIfMissing();
	std::string volume_output_directory = test::generated_volume_directory.ToString() + IndexString<TIndex>() + "/";
	test::ConstructGeneratedArraysDirectoryIfMissing();

	ORUtils::OStreamWrapper warp_stats_file(
			test::generated_arrays_directory.ToString() + "warp_gradient_stats_" + IndexString<TIndex>() + ".dat", false);

	VoxelVolume<TSDFVoxel, TIndex>* canonical_volume;
	VoxelVolume<TSDFVoxel, TIndex>* live_volume;
	LoadVolume(&live_volume, volume_output_directory + "snoopy_partial_frame_17.dat", TMemoryDeviceType,
	           test::snoopy::InitializationParameters_Fr16andFr17<TIndex>());
	LoadVolume(&canonical_volume, volume_output_directory + "snoopy_partial_frame_16.dat", TMemoryDeviceType,
	           test::snoopy::InitializationParameters_Fr16andFr17<TIndex>());

	VoxelVolume<TSDFVoxel, TIndex>* live_volumes[] = {live_volume, nullptr};

	// *** set up level set switches for both iterations

	std::array<LevelSetAlignmentSwitches, 3> switches_iteration_0 = {
			LevelSetAlignmentSwitches(true, false, false, false, false),
			LevelSetAlignmentSwitches(true, false, false, false, true),
			LevelSetAlignmentSwitches(true, false, true, false, true)
	};

	std::array<LevelSetAlignmentSwitches, 4> switches_iteration_1 = {
			LevelSetAlignmentSwitches(false, false, true, false, false),
			LevelSetAlignmentSwitches(true, false, true, false, false),
			LevelSetAlignmentSwitches(true, false, true, true, false),
			LevelSetAlignmentSwitches(true, true, false, false, false)
	};
	// ========================================================================

	VoxelVolume<WarpVoxel, TIndex> warp_field(TMemoryDeviceType, test::snoopy::InitializationParameters_Fr16andFr17<TIndex>());
	auto reset_warp_field = [&]() {
		warp_field.Reset();
		AllocateUsingOtherVolume(&warp_field, live_volume, MEMORYDEVICE_CPU);
	};

	AllocateUsingOtherVolume(canonical_volume, live_volume, MEMORYDEVICE_CPU);

	std::string warp_field_iteration_0_prefix = "warp_field_0_";
	std::string warp_field_iteration_1_prefix = "warp_field_1_";
	std::string warp_field_file_extension = ".dat";

	// iteration 0
	for (auto& switches : switches_iteration_0) {
		reset_warp_field();

		LevelSetAlignmentEngine<TSDFVoxel, WarpVoxel, TIndex, TMemoryDeviceType, DIAGNOSTIC> data_only_motion_tracker(
				switches, SingleIterationTerminationConditions());

		data_only_motion_tracker.Align(&warp_field, live_volumes, canonical_volume);

		warp_field.SaveToDisk(volume_output_directory + warp_field_iteration_0_prefix + SwitchesToPrefix(switches) + warp_field_file_extension);

		unsigned int altered_warp_update_count =
				AnalyticsEngine<WarpVoxel, TIndex, TMemoryDeviceType>::Instance().CountAlteredWarpUpdates(&warp_field);
		warp_stats_file.OStream().write(reinterpret_cast<const char*>(&altered_warp_update_count), sizeof(unsigned int));
		float average_warp_update_length = AnalyticsEngine<WarpVoxel, TIndex, TMemoryDeviceType>::Instance().ComputeWarpUpdateMean(&warp_field);
		warp_stats_file.OStream().write(reinterpret_cast<const char*>(&average_warp_update_length), sizeof(float));
	}

	reset_warp_field();
	std::string warp_0_data_and_tikhonov_sobolev_smoothed_filename =
			warp_field_iteration_0_prefix + SwitchesToPrefix(switches_iteration_0[switches_iteration_0.size() - 1]) + warp_field_file_extension;

	// iteration 1
	for (auto& switches : switches_iteration_1) {
		warp_field.Reset();
		warp_field.LoadFromDisk(volume_output_directory + warp_0_data_and_tikhonov_sobolev_smoothed_filename);
		LevelSetAlignmentEngine<TSDFVoxel, WarpVoxel, TIndex, TMemoryDeviceType, DIAGNOSTIC> tracker(switches,
		                                                                                             SingleIterationTerminationConditions());
		tracker.Align(&warp_field, live_volumes, canonical_volume);
		warp_field.SaveToDisk(volume_output_directory + warp_field_iteration_1_prefix + SwitchesToPrefix(switches) + warp_field_file_extension);
		unsigned int altered_gradient_count = AnalyticsEngine<WarpVoxel, TIndex, TMemoryDeviceType>::Instance().CountAlteredWarpUpdates(&warp_field);
		warp_stats_file.OStream().write(reinterpret_cast<const char*>(&altered_gradient_count), sizeof(unsigned int));
		float average_warp_update_length = AnalyticsEngine<WarpVoxel, TIndex, TMemoryDeviceType>::Instance().ComputeWarpUpdateMean(&warp_field);
		warp_stats_file.OStream().write(reinterpret_cast<const char*>(&average_warp_update_length), sizeof(float));
	}

	delete canonical_volume;
	delete live_volume;
}

void GenerateLevelSetAlignment_CPU_vs_CUDA_TestData() {
	GenerateWarpGradientTestData<PlainVoxelArray, MEMORYDEVICE_CPU>();
	GenerateWarpGradientTestData<VoxelBlockHash, MEMORYDEVICE_CPU>();
}