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
#include "TestUtils.h"
#include "Test_WarpGradient_Common.h"
#include "../ITMLib/Utils/Configuration.h"
#include "../ITMLib/SurfaceTrackers/Interface/SurfaceTracker.h"
#include "../ITMLib/Utils/Analytics/VoxelVolumeComparison/ITMVoxelVolumeComparison_CPU.h"
#include "../ITMLib/Engines/Traversal/CPU/VolumeTraversal_CPU_PlainVoxelArray.h"

using namespace ITMLib;

template<typename TVoxel>
struct AlteredGradientCountFunctor {
	AlteredGradientCountFunctor() : count(0) {};

	void operator()(const TVoxel& voxel) {
		if (voxel.gradient0 != Vector3f(0.0f)) {
			count.fetch_add(1u);
		}
	}

	std::atomic<unsigned int> count;
};

template<typename TVoxel>
struct AlteredFramewiseWarpCountFunctor {
	AlteredFramewiseWarpCountFunctor() : count(0) {};

	void operator()(const TVoxel& voxel) {
		if (voxel.framewise_warp != Vector3f(0.0f)) {
			count.fetch_add(1u);
		}
	}

	std::atomic<unsigned int> count;
};

typedef WarpGradientDataFixture<MemoryDeviceType::MEMORYDEVICE_CPU, PlainVoxelArray> DataFixture;
BOOST_FIXTURE_TEST_CASE(testDataTerm_CPU_PVA, DataFixture) {

	ITMVoxelVolume<ITMWarp, PlainVoxelArray> warp_field_CPU1(&configuration::get().general_voxel_volume_parameters,
	                                                            configuration::get().swapping_mode ==
	                                                            configuration::SWAPPINGMODE_ENABLED,
	                                                            MEMORYDEVICE_CPU,
	                                                            indexParameters);
	ManipulationEngine_CPU_PVA_Warp::Inst().ResetScene(&warp_field_CPU1);


	auto motionTracker_PVA_CPU = new SurfaceTracker<ITMVoxel, ITMWarp, PlainVoxelArray, MEMORYDEVICE_CPU, TRACKER_SLAVCHEVA_DIAGNOSTIC>(
			SlavchevaSurfaceTracker::Switches(true, false, false, false, false));


	TimeIt([&]() {
		motionTracker_PVA_CPU->CalculateWarpGradient(canonical_volume, live_volume, &warp_field_CPU1);
	}, "Calculate Warping Gradient - PVA CPU data term");

	//warp_field_CPU1.SaveToDirectory("../../Tests/TestData/snoopy_result_fr16-17_partial_PVA/warp_field_0_data_");

	AlteredGradientCountFunctor<ITMWarp> functor;
	VolumeTraversalEngine<ITMWarp, PlainVoxelArray, MEMORYDEVICE_CPU>::
	VoxelTraversal(&warp_field_CPU1, functor);
	BOOST_REQUIRE_EQUAL(functor.count.load(), 37525);

	float tolerance = 1e-5;
	BOOST_REQUIRE(contentAlmostEqual_CPU(&warp_field_CPU1, warp_field_data_term, tolerance));
}

BOOST_FIXTURE_TEST_CASE(testUpdateWarps_CPU_PVA, DataFixture) {
	auto motionTracker_PVA_CPU = new SurfaceTracker<ITMVoxel, ITMWarp, PlainVoxelArray, MEMORYDEVICE_CPU, TRACKER_SLAVCHEVA_DIAGNOSTIC>(
			SlavchevaSurfaceTracker::Switches(true, false, false, false, false));
	ITMVoxelVolume<ITMWarp, PlainVoxelArray> warp_field_copy(*warp_field_data_term,
	                                                            MemoryDeviceType::MEMORYDEVICE_CPU);

	AlteredGradientCountFunctor<ITMWarp> agcFunctor;
	VolumeTraversalEngine<ITMWarp, PlainVoxelArray, MEMORYDEVICE_CPU>::
	VoxelTraversal(&warp_field_copy, agcFunctor);
	BOOST_REQUIRE_EQUAL(agcFunctor.count.load(), 37525u);


	float maxWarp = motionTracker_PVA_CPU->UpdateWarps(canonical_volume, live_volume, &warp_field_copy);
	//warp_field_copy.SaveToDirectory("../../Tests/TestData/snoopy_result_fr16-17_partial_PVA/warp_field_0_data_framewise_warps_");
	BOOST_REQUIRE_CLOSE(maxWarp, 0.18186526f, 1e-7);

	AlteredFramewiseWarpCountFunctor<ITMWarp> functor;
	VolumeTraversalEngine<ITMWarp, PlainVoxelArray, MEMORYDEVICE_CPU>::
	VoxelTraversal(&warp_field_copy, functor);
	BOOST_REQUIRE_EQUAL(functor.count.load(), 37525u);

	float tolerance = 1e-8;
	BOOST_REQUIRE(contentAlmostEqual_CPU(&warp_field_copy, warp_field_iter0, tolerance));
}

BOOST_FIXTURE_TEST_CASE(testSmoothWarpGradient_CPU_PVA, DataFixture) {
	ITMVoxelVolume<ITMWarp, PlainVoxelArray> warp_field_CPU1(*warp_field_data_term, MEMORYDEVICE_CPU);


	auto motionTracker_PVA_CPU = new SurfaceTracker<ITMVoxel, ITMWarp, PlainVoxelArray, MEMORYDEVICE_CPU, TRACKER_SLAVCHEVA_DIAGNOSTIC>(
			SlavchevaSurfaceTracker::Switches(false, false, false, false, true));

	TimeIt([&]() {
		motionTracker_PVA_CPU->SmoothWarpGradient(canonical_volume, live_volume, &warp_field_CPU1);
	}, "Smooth Warping Gradient - PVA CPU");
//	warp_field_CPU1.SaveToDirectory("../../Tests/TestData/snoopy_result_fr16-17_partial_PVA/warp_field_0_smoothed_");

	float tolerance = 1e-8;
	BOOST_REQUIRE(contentAlmostEqual_CPU(&warp_field_CPU1, warp_field_data_term_smoothed, tolerance));
}

BOOST_FIXTURE_TEST_CASE(testDataAndTikhonovTerm_CPU_PVA, DataFixture) {
	ITMVoxelVolume<ITMWarp, PlainVoxelArray> warp_field_CPU1(*warp_field_iter0, MEMORYDEVICE_CPU);


	auto motionTracker_PVA_CPU = new SurfaceTracker<ITMVoxel, ITMWarp, PlainVoxelArray, MEMORYDEVICE_CPU, TRACKER_SLAVCHEVA_DIAGNOSTIC>(
			SlavchevaSurfaceTracker::Switches(true, false, true, false, false)
	);


	TimeIt([&]() {
		motionTracker_PVA_CPU->CalculateWarpGradient(canonical_volume, live_volume, &warp_field_CPU1);
	}, "Calculate Warping Gradient - PVA CPU data + tikhonov term");
//	warp_field_CPU1.SaveToDirectory("../../Tests/TestData/snoopy_result_fr16-17_partial_PVA/warp_field_1_data_and_tikhonov_");


	AlteredGradientCountFunctor<ITMWarp> functor;
	VolumeTraversalEngine<ITMWarp, PlainVoxelArray, MEMORYDEVICE_CPU>::
	VoxelTraversal(&warp_field_CPU1, functor);
	BOOST_REQUIRE_EQUAL(functor.count.load(), 57416);

	float tolerance = 1e-8;
	BOOST_REQUIRE(contentAlmostEqual_CPU(&warp_field_CPU1, warp_field_data_and_tikhonov_term, tolerance));
}


BOOST_FIXTURE_TEST_CASE(testDataAndKillingTerm_CPU_PVA, DataFixture) {
	ITMVoxelVolume<ITMWarp, PlainVoxelArray> warp_field_CPU1(*warp_field_iter0, MEMORYDEVICE_CPU);


	auto motionTracker_PVA_CPU = new SurfaceTracker<ITMVoxel, ITMWarp, PlainVoxelArray, MEMORYDEVICE_CPU, TRACKER_SLAVCHEVA_DIAGNOSTIC>(
			SlavchevaSurfaceTracker::Switches(true, false, true, true, false)
	);


	TimeIt([&]() {
		motionTracker_PVA_CPU->CalculateWarpGradient(canonical_volume, live_volume, &warp_field_CPU1);
	}, "Calculate Warping Gradient - PVA CPU data term + tikhonov term");
	//warp_field_CPU1.SaveToDirectory("../../Tests/TestData/snoopy_result_fr16-17_partial_PVA/warp_field_1_data_and_killing_");

	AlteredGradientCountFunctor<ITMWarp> functor;
	VolumeTraversalEngine<ITMWarp, PlainVoxelArray, MEMORYDEVICE_CPU>::
	VoxelTraversal(&warp_field_CPU1, functor);
	BOOST_REQUIRE_EQUAL(functor.count.load(), 59093);

	float tolerance = 1e-8;
	BOOST_REQUIRE(contentAlmostEqual_CPU(&warp_field_CPU1, warp_field_data_and_killing_term, tolerance));
}


BOOST_FIXTURE_TEST_CASE(testDataAndLevelSetTerm_CPU_PVA, DataFixture) {

	ITMVoxelVolume<ITMWarp, PlainVoxelArray> warp_field_CPU1(*warp_field_iter0, MEMORYDEVICE_CPU);


	auto motionTracker_PVA_CPU = new SurfaceTracker<ITMVoxel, ITMWarp, PlainVoxelArray, MEMORYDEVICE_CPU, TRACKER_SLAVCHEVA_DIAGNOSTIC>(
			SlavchevaSurfaceTracker::Switches(true, true, false, false, false)
	);


	TimeIt([&]() {
		motionTracker_PVA_CPU->CalculateWarpGradient(canonical_volume, live_volume, &warp_field_CPU1);
	}, "Calculate Warping Gradient - PVA CPU data term + level set term");
//	warp_field_CPU1.SaveToDirectory("../../Tests/TestData/snoopy_result_fr16-17_partial_PVA/warp_field_1_data_and_level_set_");

	AlteredGradientCountFunctor<ITMWarp> functor;
	VolumeTraversalEngine<ITMWarp, PlainVoxelArray, MEMORYDEVICE_CPU>::
	VoxelTraversal(&warp_field_CPU1, functor);
	BOOST_REQUIRE_EQUAL(functor.count.load(), 55369);

	float tolerance = 1e-7;
	BOOST_REQUIRE(contentAlmostEqual_CPU_Verbose(&warp_field_CPU1, warp_field_data_and_level_set_term, tolerance));
}

//#define GENERATE_DATA
#ifdef GENERATE_DATA
BOOST_AUTO_TEST_CASE(Test_WarpGradient_CPU_PVA_GenerateTestData){
	GenerateTestData<PlainVoxelArray,MEMORYDEVICE_CPU>();
}
#endif
