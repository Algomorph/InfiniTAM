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

#define BOOST_TEST_MODULE CUDASpatialHashReduction
#ifndef WIN32
#define BOOST_TEST_DYN_LINK
#endif

//boost
#include <boost/test/unit_test.hpp>


//ITMLib
#include "../ITMLib/Utils/Analytics/VolumeReductionStatisticsCalculator/VolumeReductionStatisticsCalculator_CUDA_VBH.h"
#include "../ITMLib/Utils/Analytics/VolumeStatisticsCalculator/VolumeStatisticsCalculator.h"

//local
#include "TestUtils.h"
#include "TestUtilsForSnoopyFrames16And17.h"

using namespace ITMLib;

BOOST_FIXTURE_TEST_CASE(Test_CUDASpatialHashReduction, Frame16And17Fixture){
	const int iteration = 1;

	std::string prefix = "data_tikhonov_sobolev";
	// *** load warps
	std::string path_warps =
			"TestData/snoopy_result_fr16-17_warps/" + prefix + "_iter_" + std::to_string(iteration) + "_";

	VoxelVolume<WarpVoxel, VoxelBlockHash>* warps;
	loadVolume(&warps, path_warps, MEMORYDEVICE_CUDA, InitParams<VoxelBlockHash>());

	float value_gt = VolumeStatisticsCalculator<WarpVoxel,VoxelBlockHash,MEMORYDEVICE_CUDA>::Instance().ComputeWarpUpdateMax(warps);

	float max_value;
	Vector3i position;
	VolumeReductionStatisticsCalculator<WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA>::Instance().ComputeWarpUpdateMax(
			max_value, position, warps);

	BOOST_REQUIRE_EQUAL(value_gt, max_value);

	WarpVoxel voxel = warps->GetValueAt(position);

	BOOST_REQUIRE_EQUAL(ORUtils::length(voxel.warp_update), max_value);

	delete warps;
}