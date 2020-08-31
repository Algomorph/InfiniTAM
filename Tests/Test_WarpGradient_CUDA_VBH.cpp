//  ================================================================
//  Created by Gregory Kramida on 10/15/19.
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
#define BOOST_TEST_MODULE WarpGradient_CUDA_VBH
#ifndef WIN32
#define BOOST_TEST_DYN_LINK
#endif

//stdlib
#include <vector>
#include <atomic>

//boost
#include <boost/test/unit_test.hpp>

//ITMLib
#include "../ITMLib/Utils/Configuration/Configuration.h"
#include "../ITMLib/Engines/EditAndCopy/CUDA/EditAndCopyEngine_CUDA.h"
#include "../ITMLib/Engines/LevelSetAlignment/Interface/LevelSetAlignmentEngineInterface.h"
#include "../ITMLib/Engines/LevelSetAlignment/Interface/LevelSetAlignmentEngine.h"
#include "../ITMLib/Utils/Analytics/VoxelVolumeComparison/VoxelVolumeComparison_CUDA.h"
#include "../ITMLib/Engines/Analytics/AnalyticsEngine.h"

//test_utilities
#include "TestUtilities/TestUtilities.h"
#include "Test_WarpGradient_Common.h"


using namespace ITMLib;


typedef WarpGradientDataFixture<MemoryDeviceType::MEMORYDEVICE_CUDA, VoxelBlockHash> DataFixture;
BOOST_FIXTURE_TEST_CASE(testDataTerm_CUDA_VBH, DataFixture) {

	VoxelVolume<WarpVoxel, VoxelBlockHash> warp_field(MEMORYDEVICE_CUDA, index_parameters);
	ManipulationEngine_CUDA_VBH_Warp::Inst().ResetVolume(&warp_field);

	AllocateUsingOtherVolume(&warp_field, live_volume, MEMORYDEVICE_CUDA);
	AllocateUsingOtherVolume(canonical_volume, live_volume, MEMORYDEVICE_CUDA);


	auto motionTracker_VBH_CUDA = new LevelSetAlignmentEngine<TSDFVoxel, WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA, DIAGNOSTIC>(
			LevelSetAlignmentSwitches(true, false, false, false, false));

	TimeIt([&]() {
		motionTracker_VBH_CUDA->CalculateWarpGradient(&warp_field, canonical_volume, live_volume);
	}, "Calculate Warping Gradient - VBH CUDA data term");

	BOOST_REQUIRE_EQUAL(Analytics_CUDA_VBH_Warp::Instance().CountAllocatedHashBlocks(&warp_field), 633);

	float tolerance = 1e-7;
	BOOST_REQUIRE(contentAlmostEqual_CUDA_Verbose(&warp_field, warp_field_data_term, tolerance));
}

BOOST_FIXTURE_TEST_CASE(testUpdateWarps_CUDA_VBH, DataFixture) {
	auto motion_tracker_VBH_CUDA = new LevelSetAlignmentEngine<TSDFVoxel, WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA, DIAGNOSTIC>(
			LevelSetAlignmentSwitches(false, false, false, false, false));
	VoxelVolume<WarpVoxel, VoxelBlockHash> warp_field_copy(*warp_field_data_term,
	                                                       MemoryDeviceType::MEMORYDEVICE_CUDA);
	AllocateUsingOtherVolume(canonical_volume, live_volume, MEMORYDEVICE_CUDA);

	BOOST_REQUIRE_EQUAL(Analytics_CUDA_VBH_Warp::Instance().CountAllocatedHashBlocks(&warp_field_copy), 633);

	AllocateUsingOtherVolume(canonical_volume, live_volume, MEMORYDEVICE_CUDA);

	float average_warp_update = motion_tracker_VBH_CUDA->UpdateWarps(&warp_field_copy, canonical_volume, live_volume);

	BOOST_REQUIRE_CLOSE(average_warp_update, warp_update_average_length_iter0, 1e-2f);

	float tolerance = 1e-8;
	BOOST_REQUIRE(contentAlmostEqual_CUDA(&warp_field_copy, warp_field_iter0, tolerance));
}


BOOST_FIXTURE_TEST_CASE(testSmoothWarpGradient_CUDA_VBH, DataFixture) {
	VoxelVolume<WarpVoxel, VoxelBlockHash> warp_field(*warp_field_data_term, MEMORYDEVICE_CUDA);
	AllocateUsingOtherVolume(&warp_field, live_volume, MEMORYDEVICE_CUDA);
	AllocateUsingOtherVolume(canonical_volume, live_volume, MEMORYDEVICE_CUDA);
	auto motionTracker_VBH_CUDA = new LevelSetAlignmentEngine<TSDFVoxel, WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA, DIAGNOSTIC>(
			LevelSetAlignmentSwitches(false, false, false, false, true));


	TimeIt([&]() {
		motionTracker_VBH_CUDA->SmoothWarpGradient(&warp_field, canonical_volume, live_volume);
	}, "Smooth Warping Gradient - VBH CUDA");

	float tolerance = 1e-8;
	BOOST_REQUIRE(contentAlmostEqual_CUDA(&warp_field, warp_field_data_term_smoothed, tolerance));
}

BOOST_FIXTURE_TEST_CASE(testTikhonovTerm_CUDA_VBH, DataFixture) {
	VoxelVolume<WarpVoxel, VoxelBlockHash> warp_field(*warp_field_iter0, MEMORYDEVICE_CUDA);

	AllocateUsingOtherVolume(canonical_volume, live_volume, MEMORYDEVICE_CUDA);

	auto motionTracker_VBH_CUDA = new LevelSetAlignmentEngine<TSDFVoxel, WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA, DIAGNOSTIC>(
			LevelSetAlignmentSwitches(false, false, true, false, false));

	Vector3i testPosition(-40, 60, 200);

	TimeIt([&]() {
		motionTracker_VBH_CUDA->CalculateWarpGradient(&warp_field, canonical_volume, live_volume);
	}, "Calculate Warping Gradient - VBH CUDA tikhonov term");
	BOOST_REQUIRE_EQUAL(Analytics_CUDA_VBH_Warp::Instance().CountAllocatedHashBlocks(&warp_field), 633);

	WarpVoxel warp1 = ManipulationEngine_CUDA_VBH_Warp::Inst().ReadVoxel(&warp_field, testPosition);
	WarpVoxel warp2 = ManipulationEngine_CUDA_VBH_Warp::Inst().ReadVoxel(warp_field_tikhonov_term, testPosition);

	float relative_tolerance = 1.0f; //percent
	BOOST_REQUIRE_CLOSE(warp1.gradient0.x, warp2.gradient0.x, relative_tolerance);
	BOOST_REQUIRE_CLOSE(warp1.gradient0.y, warp2.gradient0.y, relative_tolerance);
	BOOST_REQUIRE_CLOSE(warp1.gradient0.z, warp2.gradient0.z, relative_tolerance);

	float absolute_tolerance = 1e-6;
	BOOST_REQUIRE(contentAlmostEqual_CUDA(&warp_field, warp_field_tikhonov_term, absolute_tolerance));
}

BOOST_FIXTURE_TEST_CASE(testDataAndTikhonovTerm_CUDA_VBH, DataFixture) {
	VoxelVolume<WarpVoxel, VoxelBlockHash> warp_field(*warp_field_iter0, MEMORYDEVICE_CUDA);

	AllocateUsingOtherVolume(canonical_volume, live_volume, MEMORYDEVICE_CUDA);

	auto motionTracker_VBH_CUDA = new LevelSetAlignmentEngine<TSDFVoxel, WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA, DIAGNOSTIC>(
			LevelSetAlignmentSwitches(true, false, true, false, false));

	Vector3i testPosition(-40, 60, 200);

	TimeIt([&]() {
		motionTracker_VBH_CUDA->CalculateWarpGradient(&warp_field, canonical_volume, live_volume);
	}, "Calculate Warping Gradient - VBH CUDA data term + tikhonov term");
	BOOST_REQUIRE_EQUAL(Analytics_CUDA_VBH_Warp::Instance().CountAllocatedHashBlocks(&warp_field), 633);

	WarpVoxel warp1 = ManipulationEngine_CUDA_VBH_Warp::Inst().ReadVoxel(&warp_field, testPosition);
	WarpVoxel warp2 = ManipulationEngine_CUDA_VBH_Warp::Inst().ReadVoxel(warp_field_data_and_tikhonov_term, testPosition);
	float tolerance = 1e-6;
	BOOST_REQUIRE_CLOSE(warp1.gradient0.x, warp2.gradient0.x, tolerance);
	BOOST_REQUIRE_CLOSE(warp1.gradient0.y, warp2.gradient0.y, tolerance);
	BOOST_REQUIRE_CLOSE(warp1.gradient0.z, warp2.gradient0.z, tolerance);

	BOOST_REQUIRE(contentAlmostEqual_CUDA_Verbose(&warp_field, warp_field_data_and_tikhonov_term, tolerance));
}


BOOST_FIXTURE_TEST_CASE(testDataAndKillingTerm_CUDA_VBH, DataFixture) {
	VoxelVolume<WarpVoxel, VoxelBlockHash> warp_field(*warp_field_iter0, MEMORYDEVICE_CUDA);

	AllocateUsingOtherVolume(canonical_volume, live_volume, MEMORYDEVICE_CUDA);

	auto motionTracker_VBH_CUDA = new LevelSetAlignmentEngine<TSDFVoxel, WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA, DIAGNOSTIC>(
			LevelSetAlignmentSwitches(true, false, true, true, false));

	TimeIt([&]() {
		motionTracker_VBH_CUDA->CalculateWarpGradient(&warp_field, canonical_volume, live_volume);
	}, "Calculate Warping Gradient - VBH CUDA data term + killing term");

	unsigned int altered_gradient_count = Analytics_CUDA_VBH_Warp::Instance().CountAlteredGradients(&warp_field);
	BOOST_REQUIRE_EQUAL(altered_gradient_count, gradient_count_data_and_killing_term);

	float tolerance = 1e-6;
	BOOST_REQUIRE(contentAlmostEqual_CUDA(&warp_field, warp_field_data_and_killing_term, tolerance));
}


BOOST_FIXTURE_TEST_CASE(testDataAndLevelSetTerm_CUDA_VBH, DataFixture) {
	VoxelVolume<WarpVoxel, VoxelBlockHash> warp_field(*warp_field_iter0, MEMORYDEVICE_CUDA);

	AllocateUsingOtherVolume(canonical_volume, live_volume, MEMORYDEVICE_CUDA);

	auto motion_tracker_VBH_CUDA = new LevelSetAlignmentEngine<TSDFVoxel, WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA, DIAGNOSTIC>(
			LevelSetAlignmentSwitches(true, true, false, false, false)
	);

	TimeIt([&]() {
		motion_tracker_VBH_CUDA->CalculateWarpGradient(&warp_field, canonical_volume, live_volume);
	}, "Calculate Warping Gradient - VBH CUDA data term + level set term");

	unsigned int altered_gradient_count = Analytics_CUDA_VBH_Warp::Instance().CountAlteredGradients(&warp_field);

	// due to CUDA float computations being a tad off, we may get a few more or a few less modified gradients than for CPU here
	BOOST_REQUIRE(std::abs<int>(static_cast<int>(altered_gradient_count) - static_cast<int>(gradient_count_data_and_level_set_term)) < 50);
//	BOOST_REQUIRE_EQUAL(altered_gradient_count, gradient_count_data_and_level_set_term);

	float tolerance = 1e-6;
	BOOST_REQUIRE(contentAlmostEqual_CUDA_Verbose(&warp_field, warp_field_data_and_level_set_term, tolerance));
}
