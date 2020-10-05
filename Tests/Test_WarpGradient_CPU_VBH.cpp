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
#define BOOST_TEST_MODULE WarpGradient_CPU_VBH
#ifndef WIN32
#define BOOST_TEST_DYN_LINK
#endif


//stdlib
#include <random>
#include <vector>
#include <chrono>
#include <atomic>

//boost
#include <boost/test/unit_test.hpp>

//local
#include "TestUtilities/TestUtilities.h"
#include "Test_WarpGradient_Common.h"
#include "../ITMLib/Engines/LevelSetAlignment/Interface/LevelSetAlignmentEngineInterface.h"
#include "../ITMLib/Engines/LevelSetAlignment/Interface/LevelSetAlignmentEngine.h"
#include "../ITMLib/Utils/Analytics/VoxelVolumeComparison/VoxelVolumeComparison_CPU.h"
#include "../ITMLib/Engines/Analytics/AnalyticsEngine.h"

//test_utils
#include "TestUtilities/SnoopyTestUtilities.h"
#include "TestUtilities/TestUtilities.h"

using namespace ITMLib;
using namespace test_utilities;



typedef WarpGradientDataFixture<MemoryDeviceType::MEMORYDEVICE_CPU, VoxelBlockHash> DataFixture;
BOOST_FIXTURE_TEST_CASE(testDataTerm_CPU_VBH, DataFixture) {

	VoxelVolume<WarpVoxel, VoxelBlockHash> warp_field(MEMORYDEVICE_CPU, index_parameters);
	ManipulationEngine_CPU_VBH_Warp::Inst().ResetVolume(&warp_field);

	AllocateUsingOtherVolume(&warp_field, live_volume, MEMORYDEVICE_CPU);
	AllocateUsingOtherVolume(canonical_volume, live_volume, MEMORYDEVICE_CPU);

	BOOST_REQUIRE_EQUAL(Analytics_CPU_VBH_Warp::Instance().CountAllocatedHashBlocks(&warp_field), 633);

	auto motion_tracker_VBH_CPU = new LevelSetAlignmentEngine<TSDFVoxel, WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CPU, DIAGNOSTIC>(
			LevelSetAlignmentSwitches(true, false, false, false, false));

	TimeIt([&]() {
		motion_tracker_VBH_CPU->CalculateEnergyGradient(&warp_field, canonical_volume, live_volume);
	}, "Calculate Warping Gradient - VBH CPU data term");


	unsigned int altered_gradient_count = Analytics_CPU_VBH_Warp::Instance().CountAlteredGradients(&warp_field);
	BOOST_REQUIRE_EQUAL(altered_gradient_count, update_0_count_data);

	float tolerance = 1e-7;
	BOOST_REQUIRE(contentAlmostEqual_CPU(&warp_field, warp_field_0_data, tolerance));
}

BOOST_FIXTURE_TEST_CASE(testUpdateDeformationFieldUsingGradient_CPU_VBH, DataFixture) {

	auto motionTracker_VBH_CPU = new LevelSetAlignmentEngine<TSDFVoxel, WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CPU, DIAGNOSTIC>(
			LevelSetAlignmentSwitches(false, false, false, false, false));
	VoxelVolume<WarpVoxel, VoxelBlockHash> warp_field_copy(*warp_field_0_data,
	                                                       MemoryDeviceType::MEMORYDEVICE_CPU);

	AllocateUsingOtherVolume(canonical_volume, live_volume, MEMORYDEVICE_CPU);

	float max_warp = motionTracker_VBH_CPU->UpdateDeformationFieldUsingGradient(&warp_field_copy, canonical_volume, live_volume);
	BOOST_REQUIRE_CLOSE(max_warp, update_0_average_length, 1e-2f);


	unsigned int altered_warp_update_count = Analytics_CPU_VBH_Warp::Instance().CountAlteredWarpUpdates(&warp_field_copy);
	BOOST_REQUIRE_EQUAL(altered_warp_update_count, warp_update_count_iter0);

	float tolerance = 1e-8;
	BOOST_REQUIRE(contentAlmostEqual_CPU(&warp_field_copy, warp_field_iter0, tolerance));
}


BOOST_FIXTURE_TEST_CASE(testSmoothEnergyGradient_CPU_VBH, DataFixture) {

	VoxelVolume<WarpVoxel, VoxelBlockHash> warp_field(*warp_field_0_data, MEMORYDEVICE_CPU);

	AllocateUsingOtherVolume(canonical_volume, live_volume, MEMORYDEVICE_CPU);


	auto motionTracker_VBH_CPU = new LevelSetAlignmentEngine<TSDFVoxel, WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CPU, DIAGNOSTIC>(
			LevelSetAlignmentSwitches(false, false, false, false, true)
	);

	TimeIt([&]() {
		motionTracker_VBH_CPU->SmoothEnergyGradient(&warp_field, canonical_volume, live_volume);
	}, "Smooth Warping Gradient - VBH CPU");

	float tolerance = 1e-8;
	BOOST_REQUIRE(contentAlmostEqual_CPU(&warp_field, warp_field_0_data_sobolev_smoothed, tolerance));
}

BOOST_FIXTURE_TEST_CASE(testTikhonovTerm_CPU_VBH, DataFixture) {

	VoxelVolume<WarpVoxel, VoxelBlockHash> warp_field(*warp_field_iter0, MEMORYDEVICE_CPU);

	AllocateUsingOtherVolume(canonical_volume, live_volume, MEMORYDEVICE_CPU);


	auto motionTracker_VBH_CPU = new LevelSetAlignmentEngine<TSDFVoxel, WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CPU, DIAGNOSTIC>(
			LevelSetAlignmentSwitches(false, false, true, false, false)
	);
	Vector3i testPosition(-40, 60, 200);

	TimeIt([&]() {
		motionTracker_VBH_CPU->CalculateEnergyGradient(&warp_field, canonical_volume, live_volume);
	}, "Calculate Warping Gradient - VBH CPU data term + tikhonov term");
	BOOST_REQUIRE_EQUAL(Analytics_CPU_VBH_Warp::Instance().CountAllocatedHashBlocks(&warp_field), 633);

	unsigned int altered_gradient_count = Analytics_CPU_VBH_Warp::Instance().CountAlteredGradients(&warp_field);
	BOOST_REQUIRE_EQUAL(altered_gradient_count, update_1_count_tikhonov);


	WarpVoxel warp1 = ManipulationEngine_CPU_VBH_Warp::Inst().ReadVoxel(&warp_field, testPosition);
	WarpVoxel warp2 = ManipulationEngine_CPU_VBH_Warp::Inst().ReadVoxel(warp_field_1_tikhonov, testPosition);
	float tolerance = 1e-8;
	BOOST_REQUIRE_CLOSE(warp1.gradient0.x, warp2.gradient0.x, tolerance);
	BOOST_REQUIRE_CLOSE(warp1.gradient0.y, warp2.gradient0.y, tolerance);
	BOOST_REQUIRE_CLOSE(warp1.gradient0.z, warp2.gradient0.z, tolerance);

	BOOST_REQUIRE(contentAlmostEqual_CPU(&warp_field, warp_field_1_tikhonov, tolerance));
}

BOOST_FIXTURE_TEST_CASE(testDataAndTikhonovTerm_CPU_VBH, DataFixture) {
	VoxelVolume<WarpVoxel, VoxelBlockHash> warp_field(*warp_field_iter0, MEMORYDEVICE_CPU);
	AllocateUsingOtherVolume(canonical_volume, live_volume, MEMORYDEVICE_CPU);

	auto motionTracker_VBH_CPU = new LevelSetAlignmentEngine<TSDFVoxel, WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CPU, DIAGNOSTIC>(
			LevelSetAlignmentSwitches(true, false, true, false, false));

	TimeIt([&]() {
		motionTracker_VBH_CPU->CalculateEnergyGradient(&warp_field, canonical_volume, live_volume);
	}, "Calculate Warping Gradient - VBH CPU data term + tikhonov term");
	BOOST_REQUIRE_EQUAL(Analytics_CPU_VBH_Warp::Instance().CountAllocatedHashBlocks(&warp_field), 633);


	unsigned int altered_gradient_count = Analytics_CPU_VBH_Warp::Instance().CountAlteredGradients(&warp_field);
	BOOST_REQUIRE_EQUAL(altered_gradient_count, update_1_count_data_and_tikhonov);

	Vector3i testPosition(-40, 60, 200);
	WarpVoxel warp1 = ManipulationEngine_CPU_VBH_Warp::Inst().ReadVoxel(&warp_field, testPosition);
	WarpVoxel warp2 = ManipulationEngine_CPU_VBH_Warp::Inst().ReadVoxel(warp_field_1_data_and_tikhonov,
	                                                                    testPosition);
	float tolerance = 1e-8;
	BOOST_REQUIRE_CLOSE(warp1.gradient0.x, warp2.gradient0.x, tolerance);
	BOOST_REQUIRE_CLOSE(warp1.gradient0.y, warp2.gradient0.y, tolerance);
	BOOST_REQUIRE_CLOSE(warp1.gradient0.z, warp2.gradient0.z, tolerance);

	BOOST_REQUIRE(contentAlmostEqual_CPU(&warp_field, warp_field_1_data_and_tikhonov, tolerance));
}


BOOST_FIXTURE_TEST_CASE(testDataAndKillingTerm_CPU_VBH, DataFixture) {

	VoxelVolume<WarpVoxel, VoxelBlockHash> warp_field(*warp_field_iter0, MEMORYDEVICE_CPU);

	AllocateUsingOtherVolume(canonical_volume, live_volume, MEMORYDEVICE_CPU);

	auto motionTracker_VBH_CPU = new LevelSetAlignmentEngine<TSDFVoxel, WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CPU, DIAGNOSTIC>(
			LevelSetAlignmentSwitches(true, false, true, true, false));


	TimeIt([&]() {
		motionTracker_VBH_CPU->CalculateEnergyGradient(&warp_field, canonical_volume, live_volume);
	}, "Calculate Warping Gradient - VBH CPU data term + tikhonov term");

	unsigned int altered_gradient_count = Analytics_CPU_VBH_Warp::Instance().CountAlteredGradients(&warp_field);
	BOOST_REQUIRE_EQUAL(altered_gradient_count, update_1_count_data_and_killing);

	float tolerance = 1e-8;
	BOOST_REQUIRE(contentAlmostEqual_CPU(&warp_field, warp_field_1_data_and_killing, tolerance));
}


BOOST_FIXTURE_TEST_CASE(testDataAndLevelSetTerm_CPU_VBH, DataFixture) {
	VoxelVolume<WarpVoxel, VoxelBlockHash> warp_field(*warp_field_iter0, MEMORYDEVICE_CPU);

	AllocateUsingOtherVolume(canonical_volume, live_volume, MEMORYDEVICE_CPU);

	auto motionTracker_VBH_CPU = new LevelSetAlignmentEngine<TSDFVoxel, WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CPU, DIAGNOSTIC>(
			LevelSetAlignmentSwitches(true, true, false, false, false));

	TimeIt([&]() {
		motionTracker_VBH_CPU->CalculateEnergyGradient(&warp_field, canonical_volume, live_volume);
	}, "Calculate Warping Gradient - VBH CPU data term + level set term");

	unsigned int altered_gradient_count = Analytics_CPU_VBH_Warp::Instance().CountAlteredGradients(&warp_field);
	BOOST_REQUIRE_EQUAL(altered_gradient_count, update_1_count_data_and_level_set);

	float tolerance = 1e-7;
	BOOST_REQUIRE(contentAlmostEqual_CPU(&warp_field, warp_field_1_data_and_level_set, tolerance));
}