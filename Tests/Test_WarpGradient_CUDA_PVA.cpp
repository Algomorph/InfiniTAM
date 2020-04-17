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
#include "TestUtils.h"
#include "Test_WarpGradient_Common.h"
#include "../ITMLib/Utils/Configuration.h"
#include "../ITMLib/Engines/VolumeFileIO/VolumeFileIOEngine.h"
#include "../ITMLib/Engines/EditAndCopy/CUDA/EditAndCopyEngine_CUDA.h"
#include "../ITMLib/SurfaceTrackers/Interface/SurfaceTrackerInterface.h"
#include "../ITMLib/SurfaceTrackers/Interface/SurfaceTracker.h"
#include "../ITMLib/Utils/Analytics/VoxelVolumeComparison/VoxelVolumeComparison_CUDA.h"
#include "../ITMLib/Engines/Analytics/AnalyticsEngine.h"

using namespace ITMLib;

typedef WarpGradientDataFixture<MemoryDeviceType::MEMORYDEVICE_CUDA, PlainVoxelArray> DataFixture;
BOOST_FIXTURE_TEST_CASE(testDataTerm_CUDA_PVA, DataFixture) {

	VoxelVolume<WarpVoxel, PlainVoxelArray> warp_field_CUDA1(MEMORYDEVICE_CUDA, indexParameters);
	ManipulationEngine_CUDA_PVA_Warp::Inst().ResetVolume(&warp_field_CUDA1);


	auto motionTracker_PVA_CUDA = new SurfaceTracker<TSDFVoxel, WarpVoxel, PlainVoxelArray, MEMORYDEVICE_CUDA , TRACKER_SLAVCHEVA_DIAGNOSTIC>(
			SlavchevaSurfaceTracker::Switches(true, false, false, false, false));


	TimeIt([&]() {
		motionTracker_PVA_CUDA->CalculateWarpGradient(&warp_field_CUDA1, canonical_volume, live_volume);
	}, "Calculate Warping Gradient - PVA CPU data term");

	//warp_field_CPU1.SaveToDisk("../../Tests/TestData/snoopy_result_fr16-17_partial_PVA/warp_field_0_data_");

	float tolerance = 1e-5;
	BOOST_REQUIRE(contentAlmostEqual_CUDA(&warp_field_CUDA1, warp_field_data_term, tolerance));
}

BOOST_FIXTURE_TEST_CASE(testUpdateWarps_CUDA_PVA, DataFixture) {
	auto motionTracker_PVA_CUDA = new SurfaceTracker<TSDFVoxel, WarpVoxel, PlainVoxelArray, MEMORYDEVICE_CUDA , TRACKER_SLAVCHEVA_DIAGNOSTIC>(
			SlavchevaSurfaceTracker::Switches(false, false, false, false, false));
	VoxelVolume<WarpVoxel, PlainVoxelArray> warp_field_copy(*warp_field_data_term,
	                                                        MemoryDeviceType::MEMORYDEVICE_CUDA);

	float maxWarp = motionTracker_PVA_CUDA->UpdateWarps(&warp_field_copy, canonical_volume, live_volume);
	//warp_field_copy.SaveToDisk("../../Tests/TestData/snoopy_result_fr16-17_partial_PVA/warp_field_0_data_framewise_warps_");
	BOOST_REQUIRE_CLOSE(maxWarp, 0.121243507f, 1e-7);

	float tolerance = 1e-8;
	BOOST_REQUIRE(contentAlmostEqual_CUDA(&warp_field_copy, warp_field_iter0, tolerance));
}

BOOST_FIXTURE_TEST_CASE(testSmoothWarpGradient_CUDA_PVA, DataFixture) {
	VoxelVolume<WarpVoxel, PlainVoxelArray> warp_field_CUDA1(*warp_field_data_term, MEMORYDEVICE_CUDA);
	auto motionTracker_PVA_CUDA = new SurfaceTracker<TSDFVoxel, WarpVoxel, PlainVoxelArray, MEMORYDEVICE_CUDA , TRACKER_SLAVCHEVA_DIAGNOSTIC>(
			SlavchevaSurfaceTracker::Switches(false, false, false, false, true)
			);

	TimeIt([&]() {
		motionTracker_PVA_CUDA->SmoothWarpGradient(&warp_field_CUDA1, canonical_volume, live_volume);
	}, "Smooth Warping Gradient - PVA CUDA");
//	warp_field_CUDA1.SaveToDisk("../../Tests/TestData/snoopy_result_fr16-17_partial_PVA/warp_field_0_smoothed_");

	float tolerance = 1e-6;
	BOOST_REQUIRE(contentAlmostEqual_CUDA(&warp_field_CUDA1, warp_field_data_term_smoothed, tolerance));
}

BOOST_FIXTURE_TEST_CASE(testTikhonovTerm_CUDA_PVA, DataFixture) {
	VoxelVolume<WarpVoxel, PlainVoxelArray> warp_field_CUDA1(*warp_field_iter0, MEMORYDEVICE_CUDA);


	auto motionTracker_PVA_CUDA = new SurfaceTracker<TSDFVoxel, WarpVoxel, PlainVoxelArray, MEMORYDEVICE_CUDA , TRACKER_SLAVCHEVA_DIAGNOSTIC>(
			SlavchevaSurfaceTracker::Switches(false, false, true, false, false));


	TimeIt([&]() {
		motionTracker_PVA_CUDA->CalculateWarpGradient(&warp_field_CUDA1, canonical_volume, live_volume);
	}, "Calculate Warping Gradient - PVA CUDA tikhonov term");

	BOOST_REQUIRE(true);

	Vector3i test_voxel_position(-29, 17, 195);
	WarpVoxel warp1 = warp_field_CUDA1.GetValueAt(test_voxel_position);
	WarpVoxel warp2 = warp_field_tikhonov_term->GetValueAt(test_voxel_position);
	float tolerance = 1e-6;
	BOOST_REQUIRE_CLOSE(warp1.gradient0.x, warp2.gradient0.x, tolerance);
	BOOST_REQUIRE_CLOSE(warp1.gradient0.y, warp2.gradient0.y, tolerance);
	BOOST_REQUIRE_CLOSE(warp1.gradient0.z, warp2.gradient0.z, tolerance);

	BOOST_REQUIRE(contentAlmostEqual_CUDA(&warp_field_CUDA1, warp_field_tikhonov_term, tolerance));
}

BOOST_FIXTURE_TEST_CASE(testDataAndTikhonovTerm_CUDA, DataFixture) {
	VoxelVolume<WarpVoxel, PlainVoxelArray> warp_field_CUDA1(*warp_field_iter0, MEMORYDEVICE_CUDA);

	auto motionTracker_PVA_CUDA = new SurfaceTracker<TSDFVoxel, WarpVoxel, PlainVoxelArray, MEMORYDEVICE_CUDA , TRACKER_SLAVCHEVA_DIAGNOSTIC>(
			SlavchevaSurfaceTracker::Switches(true, false, true, false, false)
			);


	TimeIt([&]() {
		motionTracker_PVA_CUDA->CalculateWarpGradient(&warp_field_CUDA1, canonical_volume, live_volume);
	}, "Calculate Warping Gradient - PVA CPU data term + Tikhonov term");


	float tolerance = 1e-6;
	BOOST_REQUIRE(contentAlmostEqual_CUDA(&warp_field_CUDA1, warp_field_data_and_tikhonov_term, tolerance));
}


BOOST_FIXTURE_TEST_CASE(testDataAndKillingTerm_CUDA, DataFixture) {
	VoxelVolume<WarpVoxel, PlainVoxelArray> warp_field_CUDA1(*warp_field_iter0, MEMORYDEVICE_CUDA);

	auto motionTracker_PVA_CUDA = new SurfaceTracker<TSDFVoxel, WarpVoxel, PlainVoxelArray, MEMORYDEVICE_CUDA , TRACKER_SLAVCHEVA_DIAGNOSTIC>(
			SlavchevaSurfaceTracker::Switches(true, false, true, true, false)
			);


	TimeIt([&]() {
		motionTracker_PVA_CUDA->CalculateWarpGradient(&warp_field_CUDA1, canonical_volume, live_volume);
	}, "Calculate Warping Gradient - PVA CPU data term + Killing term");


	float tolerance = 1e-6;
	BOOST_REQUIRE(contentAlmostEqual_CUDA(&warp_field_CUDA1, warp_field_data_and_killing_term, tolerance));
}


BOOST_FIXTURE_TEST_CASE(testDataAndLevelSetTerm_CUDA, DataFixture) {
	VoxelVolume<WarpVoxel, PlainVoxelArray> warp_field_CUDA1(*warp_field_iter0, MEMORYDEVICE_CUDA);

	auto motionTracker_PVA_CUDA = new SurfaceTracker<TSDFVoxel, WarpVoxel, PlainVoxelArray, MEMORYDEVICE_CUDA , TRACKER_SLAVCHEVA_DIAGNOSTIC>(
			SlavchevaSurfaceTracker::Switches(true, true, false, false, false));


	TimeIt([&]() {
		motionTracker_PVA_CUDA->CalculateWarpGradient(&warp_field_CUDA1, canonical_volume, live_volume);
	}, "Calculate Warping Gradient - PVA CPU data term + level set term");


	float tolerance = 1e-6;
	BOOST_REQUIRE(contentAlmostEqual_CUDA(&warp_field_CUDA1, warp_field_data_and_level_set_term, tolerance));
}

#ifdef GENERATE_DATA
BOOST_AUTO_TEST_CASE(Test_WarpGradient_CUDA_PVA_GenerateTestData){
	GenerateTestData<PlainVoxelArray,MEMORYDEVICE_CUDA>();
}
#endif