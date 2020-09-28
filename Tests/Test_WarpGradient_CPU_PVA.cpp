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
#define BOOST_TEST_MODULE WarpGradient_CPU_PVA
#ifndef WIN32
#define BOOST_TEST_DYN_LINK
#endif

//stdlib
#include <vector>
#include <atomic>

//boost
#include <boost/test/unit_test.hpp>

//local
#include "../ITMLib/Utils/Configuration/Configuration.h"
#include "../ITMLib/Engines/LevelSetAlignment/Interface/LevelSetAlignmentEngine.h"
#include "../ITMLib/Utils/Analytics/VoxelVolumeComparison/VoxelVolumeComparison_CPU.h"
#include "../ITMLib/Engines/Traversal/CPU/VolumeTraversal_CPU_PlainVoxelArray.h"

//test_utils
#include "Test_WarpGradient_Common.h"
#include "../ITMLib/Engines/Analytics/AnalyticsEngineFactory.h"

using namespace ITMLib;
using namespace test_utilities;



typedef WarpGradientDataFixture<MemoryDeviceType::MEMORYDEVICE_CPU, PlainVoxelArray> DataFixture;
BOOST_FIXTURE_TEST_CASE(testDataTerm_CPU_PVA, DataFixture) {

	VoxelVolume<WarpVoxel, PlainVoxelArray> warp_field(MEMORYDEVICE_CPU,
	                                                   index_parameters);
	ManipulationEngine_CPU_PVA_Warp::Inst().ResetVolume(&warp_field);


	auto motion_tracker_PVA_CPU = new LevelSetAlignmentEngine<TSDFVoxel, WarpVoxel, PlainVoxelArray, MEMORYDEVICE_CPU, DIAGNOSTIC>(
			LevelSetAlignmentSwitches(true, false, false, false, false));

	TimeIt([&]() {
		motion_tracker_PVA_CPU->CalculateEnergyGradient(&warp_field, canonical_volume, live_volume);
	}, "Calculate Warping Gradient - PVA CPU data term");

	unsigned int altered_gradient_count = Analytics_CPU_PVA_Warp::Instance().CountAlteredGradients(&warp_field);
	BOOST_REQUIRE_EQUAL(altered_gradient_count, gradient_count_data_term);

	float tolerance = 1e-5;
	BOOST_REQUIRE(contentAlmostEqual_CPU(&warp_field, warp_field_data_term, tolerance));
}

BOOST_FIXTURE_TEST_CASE(testUpdateDeformationFieldUsingGradient_CPU_PVA, DataFixture) {
	auto motion_tracker_PVA_CPU = new LevelSetAlignmentEngine<TSDFVoxel, WarpVoxel, PlainVoxelArray, MEMORYDEVICE_CPU, DIAGNOSTIC>(
			LevelSetAlignmentSwitches(true, false, false, false, false));
	VoxelVolume<WarpVoxel, PlainVoxelArray> warp_field_copy(*warp_field_data_term,
	                                                        MemoryDeviceType::MEMORYDEVICE_CPU);

	float average_warp = motion_tracker_PVA_CPU->UpdateDeformationFieldUsingGradient(&warp_field_copy, canonical_volume, live_volume);
	BOOST_REQUIRE_CLOSE(average_warp, warp_update_average_length_iter0, 1e-2);

	unsigned int altered_warp_update_count = Analytics_CPU_PVA_Warp::Instance().CountAlteredWarpUpdates(&warp_field_copy);
	BOOST_REQUIRE_EQUAL(altered_warp_update_count, warp_update_count_iter0);

	float tolerance = 1e-8;
	BOOST_REQUIRE(contentAlmostEqual_CPU(&warp_field_copy, warp_field_iter0, tolerance));
}

BOOST_FIXTURE_TEST_CASE(testSmoothEnergyGradient_CPU_PVA, DataFixture) {
	VoxelVolume<WarpVoxel, PlainVoxelArray> warp_field_CPU1(*warp_field_data_term, MEMORYDEVICE_CPU);


	auto motion_tracker_PVA_CPU = new LevelSetAlignmentEngine<TSDFVoxel, WarpVoxel, PlainVoxelArray, MEMORYDEVICE_CPU, DIAGNOSTIC>(
			LevelSetAlignmentSwitches(false, false, false, false, true));

	TimeIt([&]() {
		motion_tracker_PVA_CPU->SmoothEnergyGradient(&warp_field_CPU1, canonical_volume, live_volume);
	}, "Smooth Warping Gradient - PVA CPU");

	float tolerance = 1e-8;
	BOOST_REQUIRE(contentAlmostEqual_CPU(&warp_field_CPU1, warp_field_data_term_smoothed, tolerance));
}

BOOST_FIXTURE_TEST_CASE(testDataAndTikhonovTerm_CPU_PVA, DataFixture) {
	VoxelVolume<WarpVoxel, PlainVoxelArray> warp_field(*warp_field_iter0, MEMORYDEVICE_CPU);

	auto motion_tracker_PVA_CPU = new LevelSetAlignmentEngine<TSDFVoxel, WarpVoxel, PlainVoxelArray, MEMORYDEVICE_CPU, DIAGNOSTIC>(
			LevelSetAlignmentSwitches(true, false, true, false, false)
	);

	TimeIt([&]() {
		motion_tracker_PVA_CPU->CalculateEnergyGradient(&warp_field, canonical_volume, live_volume);
	}, "Calculate Warping Gradient - PVA CPU data + tikhonov term");


	unsigned int altered_gradient_count = Analytics_CPU_PVA_Warp::Instance().CountAlteredGradients(&warp_field);
	BOOST_REQUIRE_EQUAL(altered_gradient_count, gradient_count_data_and_tikhonov_term);

	float tolerance = 1e-8;
	BOOST_REQUIRE(contentAlmostEqual_CPU_Verbose(&warp_field, warp_field_data_and_tikhonov_term, tolerance));
}


BOOST_FIXTURE_TEST_CASE(testDataAndKillingTerm_CPU_PVA, DataFixture) {
	VoxelVolume<WarpVoxel, PlainVoxelArray> warp_field(*warp_field_iter0, MEMORYDEVICE_CPU);


	auto motion_tracker_PVA_CPU = new LevelSetAlignmentEngine<TSDFVoxel, WarpVoxel, PlainVoxelArray, MEMORYDEVICE_CPU, DIAGNOSTIC>(
			LevelSetAlignmentSwitches(true, false, true, true, false)
	);


	TimeIt([&]() {
		motion_tracker_PVA_CPU->CalculateEnergyGradient(&warp_field, canonical_volume, live_volume);
	}, "Calculate Warping Gradient - PVA CPU data term + tikhonov term");

	unsigned int altered_gradient_count = Analytics_CPU_PVA_Warp::Instance().CountAlteredGradients(&warp_field);
	BOOST_REQUIRE_EQUAL(altered_gradient_count, gradient_count_data_and_killing_term);

	float tolerance = 1e-8;
	BOOST_REQUIRE(contentAlmostEqual_CPU(&warp_field, warp_field_data_and_killing_term, tolerance));
}


BOOST_FIXTURE_TEST_CASE(testDataAndLevelSetTerm_CPU_PVA, DataFixture) {

	VoxelVolume<WarpVoxel, PlainVoxelArray> warp_field(*warp_field_iter0, MEMORYDEVICE_CPU);


	auto motion_tracker_PVA_CPU = new LevelSetAlignmentEngine<TSDFVoxel, WarpVoxel, PlainVoxelArray, MEMORYDEVICE_CPU, DIAGNOSTIC>(
			LevelSetAlignmentSwitches(true, true, false, false, false)
	);


	TimeIt([&]() {
		motion_tracker_PVA_CPU->CalculateEnergyGradient(&warp_field, canonical_volume, live_volume);
	}, "Calculate Warping Gradient - PVA CPU data term + level set term");

	unsigned int altered_gradient_count = Analytics_CPU_PVA_Warp::Instance().CountAlteredGradients(&warp_field);
	BOOST_REQUIRE_EQUAL(altered_gradient_count, gradient_count_data_and_level_set_term);

	float tolerance = 1e-7;
	BOOST_REQUIRE(contentAlmostEqual_CPU_Verbose(&warp_field, warp_field_data_and_level_set_term, tolerance));
}