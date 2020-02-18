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
#include "../ITMLib/Utils/Analytics/VoxelVolumeComparison/VoxelVolumeComparison_CPU.h"
#include "../ITMLib/Utils/Analytics/VolumeStatisticsCalculator/CPU/VolumeStatisticsCalculator_CPU.h"


using namespace ITMLib;

//#define GET_SCENE_BOUNDS
#ifdef GET_SCENE_BOUNDS
BOOST_AUTO_TEST_CASE(GetSceneBounds){
	VoxelVolume<TSDFVoxel, VoxelBlockHash> canonical_scene_vbh_CPU(MEMORYDEVICE_CPU);
	canonical_scene_vbh_CPU.Reset();
	canonical_scene_vbh_CPU.LoadFromDirectory("TestData/snoopy_result_fr16-17_full/snoopy_full_frame_16_");
	std::cout << "BOUNDS(16):  " << StatCalc_CPU_VBH_Voxel::Instance().ComputeVoxelBounds(&canonical_scene_vbh_CPU) << std::endl;
	canonical_scene_vbh_CPU.Reset();
	canonical_scene_vbh_CPU.LoadFromDirectory("TestData/snoopy_result_fr16-17_full/snoopy_full_frame_17_");
	std::cout << "BOUNDS(17):  " << StatCalc_CPU_VBH_Voxel::Instance().ComputeVoxelBounds(&canonical_scene_vbh_CPU) << std::endl;
}
#endif

BOOST_AUTO_TEST_CASE(testPVASceneSlice_CPU) {
	VoxelVolume<TSDFVoxel, PlainVoxelArray> canonical_scene_CPU(MEMORYDEVICE_CPU);
	canonical_scene_CPU.Reset();

	const int expected_non_truncated_voxel_count = 58368;
	canonical_scene_CPU.LoadFromDirectory("TestData/snoopy_result_fr16-17_full/snoopy_full_frame_16_");

	BOOST_REQUIRE_EQUAL(StatCalc_CPU_PVA_Voxel::Instance().ComputeNonTruncatedVoxelCount(&canonical_scene_CPU),
	                    expected_non_truncated_voxel_count);

	BOOST_REQUIRE_CLOSE(StatCalc_CPU_PVA_Voxel::Instance().ComputeNonTruncatedVoxelAbsSdfSum(&canonical_scene_CPU),
	                    28887.87700, 0.001);

	VoxelVolume<TSDFVoxel, PlainVoxelArray> canonical_scene_slice_same_dimensions_CPU(
			&configuration::get().general_voxel_volume_parameters,
			configuration::get().swapping_mode == configuration::SWAPPINGMODE_ENABLED,
			MEMORYDEVICE_CPU);
	ManipulationEngine_CPU_PVA_Voxel::Inst().ResetVolume(&canonical_scene_slice_same_dimensions_CPU);


	Vector6i bounds(-85, -30, 159, 25, 85, 320);
	ManipulationEngine_CPU_PVA_Voxel::Inst().CopyVolumeSlice(&canonical_scene_slice_same_dimensions_CPU,
	                                                         &canonical_scene_CPU, bounds);

	BOOST_REQUIRE_EQUAL(StatCalc_CPU_PVA_Voxel::Instance().ComputeNonTruncatedVoxelCount(
			&canonical_scene_slice_same_dimensions_CPU), expected_non_truncated_voxel_count);

	float tolerance = 1e-8;
	BOOST_REQUIRE(allocatedContentAlmostEqual_CPU_Verbose(
			&canonical_scene_CPU, &canonical_scene_slice_same_dimensions_CPU, tolerance));

	bounds = Vector6i(-72, -24, 160, 16, 72, 320);
	Vector3i offsetSlice(bounds.min_x, bounds.min_y, bounds.min_z);
	Vector3i sizeSlice(bounds.max_x - bounds.min_x, bounds.max_y - bounds.min_y, bounds.max_z - bounds.min_z);

	VoxelVolume<TSDFVoxel, PlainVoxelArray> canonical_scene_slice_different_dimensions_CPU(
			&configuration::get().general_voxel_volume_parameters,
			configuration::get().swapping_mode == configuration::SWAPPINGMODE_ENABLED,
			MEMORYDEVICE_CPU, {sizeSlice, offsetSlice});
	ManipulationEngine_CPU_PVA_Voxel::Inst().ResetVolume(&canonical_scene_slice_different_dimensions_CPU);

	ManipulationEngine_CPU_PVA_Voxel::Inst().CopyVolumeSlice(&canonical_scene_slice_different_dimensions_CPU,
	                                                         &canonical_scene_CPU, bounds);
	BOOST_REQUIRE_EQUAL(StatCalc_CPU_PVA_Voxel::Instance().ComputeNonTruncatedVoxelCount(
			&canonical_scene_slice_different_dimensions_CPU), expected_non_truncated_voxel_count);
	BOOST_REQUIRE_CLOSE(StatCalc_CPU_PVA_Voxel::Instance().ComputeNonTruncatedVoxelAbsSdfSum(
			&canonical_scene_slice_different_dimensions_CPU), 17063.5, 0.001);
	BOOST_REQUIRE_CLOSE(StatCalc_CPU_PVA_Voxel::Instance().ComputeNonTruncatedVoxelAbsSdfSum(
			&canonical_scene_CPU), 17063.5, 0.001);
	BOOST_REQUIRE(allocatedContentAlmostEqual_CPU_Verbose(&canonical_scene_CPU,
	                                                      &canonical_scene_slice_different_dimensions_CPU, tolerance));

	VoxelVolume<TSDFVoxel, PlainVoxelArray> canonical_scene_slice_from_disk_CPU(
			&configuration::get().general_voxel_volume_parameters,
			configuration::get().swapping_mode == configuration::SWAPPINGMODE_ENABLED,
			MEMORYDEVICE_CPU, {sizeSlice, offsetSlice});
	ManipulationEngine_CPU_PVA_Voxel::Inst().ResetVolume(&canonical_scene_slice_from_disk_CPU);

	canonical_scene_slice_from_disk_CPU.LoadFromDirectory("TestData/snoopy_result_fr16-17_partial_PVA/canonical");

	BOOST_REQUIRE(contentAlmostEqual_CPU(&canonical_scene_slice_different_dimensions_CPU,
	                                     &canonical_scene_slice_from_disk_CPU, tolerance));
}