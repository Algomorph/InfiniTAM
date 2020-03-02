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
			VolumeStatisticsCalculator<WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA>::Instance().ComputeWarpUpdateMax(warps);

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
	VoxelVolume<TSDFVoxel, VoxelBlockHash> volume(MEMORYDEVICE_CUDA, {2048, 2048});
	volume.Reset();

	Extent3Di filled_voxel_bounds(0,0,0,48,48,48);
	Extent2Di general_range(0,50);
	GenerateRandomDepthWeightSubVolume<MEMORYDEVICE_CUDA>(&volume, filled_voxel_bounds, general_range);

//	VolumeStatisticsCalculator<WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA>::Instance().ComputeWarpUpdateMaxAndPosition(
//			max_value, position, volume);

#ifdef TEST_PERFORMANCE
	TimeIt(
			[&]() {
				VolumeStatisticsCalculator<WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA>::Instance().ComputeWarpUpdateMax(
						volume);
			}, "Warp Update Max (Atomics)", 10
	);

	TimeIt(
			[&]() {
				VolumeStatisticsCalculator<WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA>::Instance().ComputeWarpUpdateMax(
						volume);
			}, "Warp Update Max (Reduciton)", 10
	);
#endif

}