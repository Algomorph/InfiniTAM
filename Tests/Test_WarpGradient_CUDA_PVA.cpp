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
#define BOOST_TEST_MODULE WarpGradient_CUDA_PVA
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
#include "../ITMLib/Utils/Analytics/VoxelVolumeComparison/VoxelVolumeComparison_CUDA.h"
#include "../ITMLib/Engines/Analytics/AnalyticsEngine.h"

using namespace ITMLib;

typedef WarpGradientDataFixture<MemoryDeviceType::MEMORYDEVICE_CUDA, PlainVoxelArray> DataFixture;
BOOST_FIXTURE_TEST_CASE(testDataTerm_CUDA_PVA, DataFixture) {

	VoxelVolume<WarpVoxel, PlainVoxelArray> warp_field(MEMORYDEVICE_CUDA, index_parameters);
	ManipulationEngine_CUDA_PVA_Warp::Inst().ResetVolume(&warp_field);


	auto motion_tracker_PVA_CUDA = new LevelSetAlignmentEngine<TSDFVoxel, WarpVoxel, PlainVoxelArray, MEMORYDEVICE_CUDA , DIAGNOSTIC>(
			LevelSetAlignmentSwitches(true, false, false, false, false));

	TimeIt([&]() {
		motion_tracker_PVA_CUDA->CalculateWarpGradient(&warp_field, canonical_volume, live_volume);
	}, "Calculate Warping Gradient - PVA CUDA data term");

	unsigned int altered_gradient_count = Analytics_CUDA_PVA_Warp::Instance().CountAlteredGradients(&warp_field);
	BOOST_REQUIRE_EQUAL(altered_gradient_count, gradient_count_data_term);

	float tolerance = 1e-5;
	BOOST_REQUIRE(contentAlmostEqual_CUDA(&warp_field, warp_field_data_term, tolerance));
}

BOOST_FIXTURE_TEST_CASE(testUpdateWarps_CUDA_PVA, DataFixture) {
	auto motionTracker_PVA_CUDA = new LevelSetAlignmentEngine<TSDFVoxel, WarpVoxel, PlainVoxelArray, MEMORYDEVICE_CUDA , DIAGNOSTIC>(
			LevelSetAlignmentSwitches(false, false, false, false, false));
	VoxelVolume<WarpVoxel, PlainVoxelArray> warp_field_copy(*warp_field_data_term,
	                                                        MemoryDeviceType::MEMORYDEVICE_CUDA);

	float average_warp = motionTracker_PVA_CUDA->UpdateWarps(&warp_field_copy, canonical_volume, live_volume);

	BOOST_REQUIRE_CLOSE(average_warp, warp_update_average_length_iter0, 1e-2);
	unsigned int altered_warp_update_count = Analytics_CUDA_PVA_Warp::Instance().CountAlteredWarpUpdates(&warp_field_copy);
	BOOST_REQUIRE_EQUAL(altered_warp_update_count, warp_update_count_iter0);

	float tolerance = 1e-8;
	BOOST_REQUIRE(contentAlmostEqual_CUDA(&warp_field_copy, warp_field_iter0, tolerance));
}

BOOST_FIXTURE_TEST_CASE(testSmoothWarpGradient_CUDA_PVA, DataFixture) {
	VoxelVolume<WarpVoxel, PlainVoxelArray> warp_field(*warp_field_data_term, MEMORYDEVICE_CUDA);
	auto motionTracker_PVA_CUDA = new LevelSetAlignmentEngine<TSDFVoxel, WarpVoxel, PlainVoxelArray, MEMORYDEVICE_CUDA , DIAGNOSTIC>(
			LevelSetAlignmentSwitches(false, false, false, false, true)
			);

	TimeIt([&]() {
		motionTracker_PVA_CUDA->SmoothWarpGradient(&warp_field, canonical_volume, live_volume);
	}, "Smooth Warping Gradient - PVA CUDA");

	unsigned int altered_gradient_count = Analytics_CUDA_PVA_Warp::Instance().CountAlteredGradients(&warp_field);
	BOOST_REQUIRE_EQUAL(altered_gradient_count, gradient_count_data_smoothed);

	float tolerance = 1e-6;
	BOOST_REQUIRE(contentAlmostEqual_CUDA(&warp_field, warp_field_data_term_smoothed, tolerance));
}

BOOST_FIXTURE_TEST_CASE(testTikhonovTerm_CUDA_PVA, DataFixture) {
	VoxelVolume<WarpVoxel, PlainVoxelArray> warp_field(*warp_field_iter0, MEMORYDEVICE_CUDA);


	auto motionTracker_PVA_CUDA = new LevelSetAlignmentEngine<TSDFVoxel, WarpVoxel, PlainVoxelArray, MEMORYDEVICE_CUDA , DIAGNOSTIC>(
			LevelSetAlignmentSwitches(false, false, true, false, false));


	TimeIt([&]() {
		motionTracker_PVA_CUDA->CalculateWarpGradient(&warp_field, canonical_volume, live_volume);
	}, "Calculate Warping Gradient - PVA CUDA tikhonov term");

	unsigned int altered_gradient_count = Analytics_CUDA_PVA_Warp::Instance().CountAlteredGradients(&warp_field);
	BOOST_REQUIRE_EQUAL(altered_gradient_count, gradient_count_data_and_tikhonov_term);

	Vector3i test_voxel_position(-29, 17, 195);
	WarpVoxel warp1 = warp_field.GetValueAt(test_voxel_position);
	WarpVoxel warp2 = warp_field_tikhonov_term->GetValueAt(test_voxel_position);
	float tolerance = 1e-6;
	BOOST_REQUIRE_CLOSE(warp1.gradient0.x, warp2.gradient0.x, tolerance);
	BOOST_REQUIRE_CLOSE(warp1.gradient0.y, warp2.gradient0.y, tolerance);
	BOOST_REQUIRE_CLOSE(warp1.gradient0.z, warp2.gradient0.z, tolerance);

	BOOST_REQUIRE(contentAlmostEqual_CUDA(&warp_field, warp_field_tikhonov_term, tolerance));
}

BOOST_FIXTURE_TEST_CASE(testDataAndTikhonovTerm_CUDA, DataFixture) {
	VoxelVolume<WarpVoxel, PlainVoxelArray> warp_field(*warp_field_iter0, MEMORYDEVICE_CUDA);

	auto motionTracker_PVA_CUDA = new LevelSetAlignmentEngine<TSDFVoxel, WarpVoxel, PlainVoxelArray, MEMORYDEVICE_CUDA , DIAGNOSTIC>(
			LevelSetAlignmentSwitches(true, false, true, false, false)
			);


	TimeIt([&]() {
		motionTracker_PVA_CUDA->CalculateWarpGradient(&warp_field, canonical_volume, live_volume);
	}, "Calculate Warping Gradient - PVA CUDA data term + Tikhonov term");

	unsigned int altered_gradient_count = Analytics_CUDA_PVA_Warp::Instance().CountAlteredGradients(&warp_field);
	BOOST_REQUIRE_EQUAL(altered_gradient_count, gradient_count_data_and_tikhonov_term);


	float tolerance = 1e-6;
	BOOST_REQUIRE(contentAlmostEqual_CUDA(&warp_field, warp_field_data_and_tikhonov_term, tolerance));
}


BOOST_FIXTURE_TEST_CASE(testDataAndKillingTerm_CUDA, DataFixture) {
	VoxelVolume<WarpVoxel, PlainVoxelArray> warp_field(*warp_field_iter0, MEMORYDEVICE_CUDA);

	auto motionTracker_PVA_CUDA = new LevelSetAlignmentEngine<TSDFVoxel, WarpVoxel, PlainVoxelArray, MEMORYDEVICE_CUDA , DIAGNOSTIC>(
			LevelSetAlignmentSwitches(true, false, true, true, false)
			);

	TimeIt([&]() {
		motionTracker_PVA_CUDA->CalculateWarpGradient(&warp_field, canonical_volume, live_volume);
	}, "Calculate Warping Gradient - PVA CUDA data term + Killing term");

	unsigned int altered_gradient_count = Analytics_CUDA_PVA_Warp::Instance().CountAlteredGradients(&warp_field);
	BOOST_REQUIRE_EQUAL(altered_gradient_count, gradient_count_data_and_killing_term);


	float tolerance = 1e-6;
	BOOST_REQUIRE(contentAlmostEqual_CUDA(&warp_field, warp_field_data_and_killing_term, tolerance));
}


BOOST_FIXTURE_TEST_CASE(testDataAndLevelSetTerm_CUDA, DataFixture) {
	VoxelVolume<WarpVoxel, PlainVoxelArray> warp_field(*warp_field_iter0, MEMORYDEVICE_CUDA);

	auto motionTracker_PVA_CUDA = new LevelSetAlignmentEngine<TSDFVoxel, WarpVoxel, PlainVoxelArray, MEMORYDEVICE_CUDA , DIAGNOSTIC>(
			LevelSetAlignmentSwitches(true, true, false, false, false));


	TimeIt([&]() {
		motionTracker_PVA_CUDA->CalculateWarpGradient(&warp_field, canonical_volume, live_volume);
	}, "Calculate Warping Gradient - PVA CUDA data term + level set term");

	unsigned int altered_gradient_count = Analytics_CUDA_PVA_Warp::Instance().CountAlteredGradients(&warp_field);

	// due to CUDA float computations being a tad off, we may get a few more or a few less modified gradients than for CPU here
	BOOST_REQUIRE(std::abs<int>(altered_gradient_count - gradient_count_data_and_level_set_term) < 50);


	float tolerance = 1e-6;
	BOOST_REQUIRE(contentAlmostEqual_CUDA(&warp_field, warp_field_data_and_level_set_term, tolerance));
}
