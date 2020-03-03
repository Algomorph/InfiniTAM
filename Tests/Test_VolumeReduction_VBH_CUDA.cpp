//  ================================================================
//  Created by Gregory Kramida on 2/24/20.
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

#define BOOST_TEST_MODULE VolumeReduction_VBH_CUDA
#ifndef WIN32
#define BOOST_TEST_DYN_LINK
#endif

//boost
#include <boost/test/unit_test.hpp>


//ITMLib
#include "../ITMLib/Utils/Analytics/VolumeStatisticsCalculator/VolumeStatisticsCalculator.h"
#include "../ITMLib/Engines/EditAndCopy/EditAndCopyEngineFactory.h"

//local
#include "TestUtils.h"
#include "TestUtilsForSnoopyFrames16And17.h"

using namespace ITMLib;

BOOST_FIXTURE_TEST_CASE(Test_VolumeReduction_MaxWarpUpdate_VBH_CUDA, Frame16And17Fixture) {
	const int iteration = 1;

	std::string prefix = "data_tikhonov_sobolev";
	// *** load warps
	std::string path_warps =
			"TestData/snoopy_result_fr16-17_warps/" + prefix + "_iter_" + std::to_string(iteration) + "_";

	VoxelVolume<WarpVoxel, VoxelBlockHash>* warps;
	loadVolume(&warps, path_warps, MEMORYDEVICE_CUDA, InitParams<VoxelBlockHash>());

	float value_gt =
			VolumeStatisticsCalculator<WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA>::Instance().ComputeWarpUpdateMax(
					warps);

	float max_value;
	Vector3i position;
	VolumeStatisticsCalculator<WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA>::Instance().ComputeWarpUpdateMaxAndPosition(
			max_value, position, warps);

	BOOST_REQUIRE_EQUAL(value_gt, max_value);

	WarpVoxel voxel = warps->GetValueAt(position);

	BOOST_REQUIRE_EQUAL(ORUtils::length(voxel.warp_update), max_value);

#ifdef TEST_PERFORMANCE
	TimeIt(
			[&]() {
				VolumeStatisticsCalculator<WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA>::Instance().ComputeWarpUpdateMax(
						warps);
			}, "Warp Update Max (Atomics)", 10
	);

	TimeIt(
			[&]() {
				VolumeStatisticsCalculator<WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA>::Instance().ComputeWarpUpdateMax(
						warps);
			}, "Warp Update Max (Reduciton)", 10
	);
#endif

	delete warps;
}


BOOST_FIXTURE_TEST_CASE(Test_VolumeReduction_CountWeightRange_VBH_CUDA, Frame16And17Fixture) {
	VoxelVolume<TSDFVoxel, VoxelBlockHash> volume(MEMORYDEVICE_CUDA, {65536, 32768});
	volume.Reset();

	//Extent3Di filled_voxel_bounds(0, 0, 0, 256, 256, 256);
	Extent3Di filled_voxel_bounds(0, 0, 0, 48, 48, 48);
	Extent2Di general_range(0, 50);
	GenerateRandomDepthWeightSubVolume<MEMORYDEVICE_CUDA>(&volume, filled_voxel_bounds, general_range);

	const unsigned int voxel_count = filled_voxel_bounds.max_x * filled_voxel_bounds.max_y * filled_voxel_bounds.max_z;

	unsigned int voxels_in_range;
	voxels_in_range = VolumeStatisticsCalculator<TSDFVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA>::Instance()
			.CountVoxelsWithDepthWeightInRange(&volume, general_range);
	BOOST_REQUIRE_EQUAL(voxel_count, voxels_in_range);

	Extent2Di different_range1(50, 56);

	voxels_in_range = VolumeStatisticsCalculator<TSDFVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA>::Instance()
			.CountVoxelsWithDepthWeightInRange(&volume, different_range1);
	BOOST_REQUIRE_EQUAL(0, voxels_in_range);

	Extent2Di different_range2(56, 100);
	voxels_in_range = VolumeStatisticsCalculator<TSDFVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA>::Instance()
			.CountVoxelsWithDepthWeightInRange(&volume, different_range2);
	BOOST_REQUIRE_EQUAL(0, voxels_in_range);

	TSDFVoxel alternative_weight_voxel;
	alternative_weight_voxel.w_depth = 55;
	Vector3i insert_position(12, 12, 12);
	EditAndCopyEngineFactory::Instance<TSDFVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA>()
	        .SetVoxel(&volume, insert_position, alternative_weight_voxel);
	insert_position = {17, 0, 14};
	EditAndCopyEngineFactory::Instance<TSDFVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA>()
	        .SetVoxel(&volume, insert_position, alternative_weight_voxel);
	insert_position = {45, 16, 34};
	EditAndCopyEngineFactory::Instance<TSDFVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA>()
	        .SetVoxel(&volume, insert_position, alternative_weight_voxel);

	voxels_in_range = VolumeStatisticsCalculator<TSDFVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA>::Instance()
			.CountVoxelsWithDepthWeightInRange(&volume, different_range1);

	BOOST_REQUIRE_EQUAL(3, voxels_in_range);

	Extent3Di different_weight_sub_volume_bounds(42, 42, 42, 48, 48, 48);
	Vector3i sub_volume_size = different_weight_sub_volume_bounds.max() - different_weight_sub_volume_bounds.min();
	const unsigned int sub_volume_voxel_count = sub_volume_size.x * sub_volume_size.y * sub_volume_size.z;
	GenerateRandomDepthWeightSubVolume<MEMORYDEVICE_CUDA>(&volume, different_weight_sub_volume_bounds, different_range2);

	volume.GetValueAt(0,0,0).print_self();

	voxels_in_range = VolumeStatisticsCalculator<TSDFVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA>::Instance()
			.CountVoxelsWithDepthWeightInRange(&volume, different_range2);

	BOOST_REQUIRE_EQUAL(sub_volume_voxel_count, voxels_in_range);
}