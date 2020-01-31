//  ================================================================
//  Created by Gregory Kramida on 10/17/19.
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
#define BOOST_TEST_MODULE VoxelVolumeSlicing
#ifndef WIN32
#define BOOST_TEST_DYN_LINK
#endif

//boost
#include <boost/test/unit_test.hpp>

//local
#include "TestUtils.h"
#include "../ITMLib/Utils/Configuration.h"
#include "../ITMLib/Engines/EditAndCopy/CPU/EditAndCopyEngine_CPU.h"
#include "../ITMLib/Utils/Analytics/VoxelVolumeComparison/ITMVoxelVolumeComparison_CPU.h"
#include "../ITMLib/Utils/Analytics/SceneStatisticsCalculator/CPU/ITMSceneStatisticsCalculator_CPU.h"

using namespace ITMLib;


BOOST_AUTO_TEST_CASE(testPVASceneSlice_CPU) {
	ITMVoxelVolume<ITMVoxel, PlainVoxelArray> canonical_scene_CPU(&configuration::get().general_voxel_volume_parameters,
	                                                                 configuration::get().swapping_mode ==
	                                                                 configuration::SWAPPINGMODE_ENABLED,
	                                                                 MEMORYDEVICE_CPU);
	ManipulationEngine_CPU_PVA_Voxel::Inst().ResetVolume(&canonical_scene_CPU);

	const int nonTruncatedVoxelCount = 41307;
	canonical_scene_CPU.LoadFromDirectory("TestData/snoopy_result_fr16-17_PVA/canonical");

	BOOST_REQUIRE_EQUAL(SceneStatCalc_CPU_PVA_Voxel::Instance().ComputeNonTruncatedVoxelCount(&canonical_scene_CPU),
	                    nonTruncatedVoxelCount);

	BOOST_REQUIRE_CLOSE(SceneStatCalc_CPU_PVA_Voxel::Instance().ComputeNonTruncatedVoxelAbsSdfSum(&canonical_scene_CPU),
	                    17063.5, 0.001);


	ITMVoxelVolume<ITMVoxel, PlainVoxelArray> canonical_scene_slice_same_dimensions_CPU(
			&configuration::get().general_voxel_volume_parameters, configuration::get().swapping_mode == configuration::SWAPPINGMODE_ENABLED,
			MEMORYDEVICE_CPU);
	ManipulationEngine_CPU_PVA_Voxel::Inst().ResetVolume(&canonical_scene_slice_same_dimensions_CPU);


	Vector6i bounds(-64, -24, 168, 16, 72, 312);
	ManipulationEngine_CPU_PVA_Voxel::Inst().CopyVolumeSlice(&canonical_scene_slice_same_dimensions_CPU,
	                                                         &canonical_scene_CPU, bounds);

	BOOST_REQUIRE_EQUAL(SceneStatCalc_CPU_PVA_Voxel::Instance().ComputeNonTruncatedVoxelCount(
			&canonical_scene_slice_same_dimensions_CPU), nonTruncatedVoxelCount);

	float tolerance = 1e-8;
	BOOST_REQUIRE(allocatedContentAlmostEqual_CPU(&canonical_scene_CPU, &canonical_scene_slice_same_dimensions_CPU,
	                                              tolerance));


	Vector3i offsetSlice(bounds.min_x, bounds.min_y, bounds.min_z);
	//64+16=80; -24+72=96; 312-168=300-156=304-160=144
	Vector3i sizeSlice(bounds.max_x - bounds.min_x, bounds.max_y - bounds.min_y, bounds.max_z - bounds.min_z);

	ITMVoxelVolume<ITMVoxel, PlainVoxelArray> canonical_scene_slice_different_dimensions_CPU(
			&configuration::get().general_voxel_volume_parameters, configuration::get().swapping_mode == configuration::SWAPPINGMODE_ENABLED,
			MEMORYDEVICE_CPU, {sizeSlice, offsetSlice});
	ManipulationEngine_CPU_PVA_Voxel::Inst().ResetVolume(&canonical_scene_slice_different_dimensions_CPU);

	ManipulationEngine_CPU_PVA_Voxel::Inst().CopyVolumeSlice(&canonical_scene_slice_different_dimensions_CPU,
	                                                         &canonical_scene_CPU, bounds);
	BOOST_REQUIRE_EQUAL(SceneStatCalc_CPU_PVA_Voxel::Instance().ComputeNonTruncatedVoxelCount(
			&canonical_scene_slice_different_dimensions_CPU), nonTruncatedVoxelCount);
	BOOST_REQUIRE_CLOSE(SceneStatCalc_CPU_PVA_Voxel::Instance().ComputeNonTruncatedVoxelAbsSdfSum(
			&canonical_scene_slice_different_dimensions_CPU), 17063.5, 0.001);
	BOOST_REQUIRE_CLOSE(SceneStatCalc_CPU_PVA_Voxel::Instance().ComputeNonTruncatedVoxelAbsSdfSum(
			&canonical_scene_CPU), 17063.5, 0.001);
	BOOST_REQUIRE(allocatedContentAlmostEqual_CPU_Verbose(&canonical_scene_CPU,
	                                              &canonical_scene_slice_different_dimensions_CPU, tolerance));

	ITMVoxelVolume<ITMVoxel, PlainVoxelArray> canonical_scene_slice_from_disk_CPU(
			&configuration::get().general_voxel_volume_parameters, configuration::get().swapping_mode == configuration::SWAPPINGMODE_ENABLED,
			MEMORYDEVICE_CPU, {sizeSlice, offsetSlice});
	ManipulationEngine_CPU_PVA_Voxel::Inst().ResetVolume(&canonical_scene_slice_from_disk_CPU);

	canonical_scene_slice_from_disk_CPU.LoadFromDirectory("TestData/snoopy_result_fr16-17_partial_PVA/canonical");

	BOOST_REQUIRE(contentAlmostEqual_CPU(&canonical_scene_slice_different_dimensions_CPU,
	                                              &canonical_scene_slice_from_disk_CPU, tolerance));
}