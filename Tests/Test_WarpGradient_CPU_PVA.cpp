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
#include "../ITMLib/Utils/Configuration.h"
#include "../ITMLib/SurfaceTrackers/Interface/SurfaceTracker.h"
#include "../ITMLib/Utils/Analytics/VoxelVolumeComparison/VoxelVolumeComparison_CPU.h"
#include "../ITMLib/Engines/Traversal/CPU/VolumeTraversal_CPU_PlainVoxelArray.h"

//test_utils
#include "Test_WarpGradient_Common.h"

using namespace ITMLib;
using namespace test_utilities;

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
struct AlteredWarpUpdateCountFunctor {
	AlteredWarpUpdateCountFunctor() : count(0) {};

	void operator()(const TVoxel& voxel) {
		if (voxel.warp_update != Vector3f(0.0f)) {
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

	VoxelVolume<WarpVoxel, PlainVoxelArray> warp_field_CPU1(MEMORYDEVICE_CPU,
	                                                        indexParameters);
	ManipulationEngine_CPU_PVA_Warp::Inst().ResetVolume(&warp_field_CPU1);

	auto motionTracker_PVA_CPU = new SurfaceTracker<TSDFVoxel, WarpVoxel, PlainVoxelArray, MEMORYDEVICE_CPU, TRACKER_SLAVCHEVA_DIAGNOSTIC>(
			SlavchevaSurfaceTracker::Switches(true, false, false, false, false));

	TimeIt([&]() {
		motionTracker_PVA_CPU->CalculateWarpGradient(&warp_field_CPU1, canonical_volume, live_volume);
	}, "Calculate Warping Gradient - PVA CPU data term");

	AlteredGradientCountFunctor<WarpVoxel> functor;
	VolumeTraversalEngine<WarpVoxel, PlainVoxelArray, MEMORYDEVICE_CPU>::
	TraverseAll(&warp_field_CPU1, functor);
	BOOST_REQUIRE_EQUAL(functor.count.load(), 37525);

	float tolerance = 1e-5;
	BOOST_REQUIRE(contentAlmostEqual_CPU(&warp_field_CPU1, warp_field_data_term, tolerance));
}

BOOST_FIXTURE_TEST_CASE(testUpdateWarps_CPU_PVA, DataFixture) {
	auto motionTracker_PVA_CPU = new SurfaceTracker<TSDFVoxel, WarpVoxel, PlainVoxelArray, MEMORYDEVICE_CPU, TRACKER_SLAVCHEVA_DIAGNOSTIC>(
			SlavchevaSurfaceTracker::Switches(true, false, false, false, false));
	VoxelVolume<WarpVoxel, PlainVoxelArray> warp_field_copy(*warp_field_data_term,
	                                                        MemoryDeviceType::MEMORYDEVICE_CPU);

	AlteredGradientCountFunctor<WarpVoxel> altered_count_functor;
	VolumeTraversalEngine<WarpVoxel, PlainVoxelArray, MEMORYDEVICE_CPU>::
	TraverseAll(&warp_field_copy, altered_count_functor);
	BOOST_REQUIRE_EQUAL(altered_count_functor.count.load(), 37525u);

	float maxWarp = motionTracker_PVA_CPU->UpdateWarps(&warp_field_copy, canonical_volume, live_volume);
	BOOST_REQUIRE_CLOSE(maxWarp, 0.121243507f, 1e-7);

	AlteredWarpUpdateCountFunctor<WarpVoxel> functor;
	VolumeTraversalEngine<WarpVoxel, PlainVoxelArray, MEMORYDEVICE_CPU>::
	TraverseAll(&warp_field_copy, functor);
	BOOST_REQUIRE_EQUAL(functor.count.load(), 37525u);

	float tolerance = 1e-8;
	BOOST_REQUIRE(contentAlmostEqual_CPU(&warp_field_copy, warp_field_iter0, tolerance));
}

BOOST_FIXTURE_TEST_CASE(testSmoothWarpGradient_CPU_PVA, DataFixture) {
	VoxelVolume<WarpVoxel, PlainVoxelArray> warp_field_CPU1(*warp_field_data_term, MEMORYDEVICE_CPU);


	auto motionTracker_PVA_CPU = new SurfaceTracker<TSDFVoxel, WarpVoxel, PlainVoxelArray, MEMORYDEVICE_CPU, TRACKER_SLAVCHEVA_DIAGNOSTIC>(
			SlavchevaSurfaceTracker::Switches(false, false, false, false, true));

	TimeIt([&]() {
		motionTracker_PVA_CPU->SmoothWarpGradient(&warp_field_CPU1, canonical_volume, live_volume);
	}, "Smooth Warping Gradient - PVA CPU");

	float tolerance = 1e-8;
	BOOST_REQUIRE(contentAlmostEqual_CPU(&warp_field_CPU1, warp_field_data_term_smoothed, tolerance));
}

BOOST_FIXTURE_TEST_CASE(testDataAndTikhonovTerm_CPU_PVA, DataFixture) {
	VoxelVolume<WarpVoxel, PlainVoxelArray> warp_field_CPU1(*warp_field_iter0, MEMORYDEVICE_CPU);


	auto motionTracker_PVA_CPU = new SurfaceTracker<TSDFVoxel, WarpVoxel, PlainVoxelArray, MEMORYDEVICE_CPU, TRACKER_SLAVCHEVA_DIAGNOSTIC>(
			SlavchevaSurfaceTracker::Switches(true, false, true, false, false)
	);


	TimeIt([&]() {
		motionTracker_PVA_CPU->CalculateWarpGradient(&warp_field_CPU1, canonical_volume, live_volume);
	}, "Calculate Warping Gradient - PVA CPU data + tikhonov term");


	AlteredGradientCountFunctor<WarpVoxel> functor;
	VolumeTraversalEngine<WarpVoxel, PlainVoxelArray, MEMORYDEVICE_CPU>::
	TraverseAll(&warp_field_CPU1, functor);
	BOOST_REQUIRE_EQUAL(functor.count.load(), 57416);

	float tolerance = 1e-8;
	BOOST_REQUIRE(contentAlmostEqual_CPU(&warp_field_CPU1, warp_field_data_and_tikhonov_term, tolerance));
}


BOOST_FIXTURE_TEST_CASE(testDataAndKillingTerm_CPU_PVA, DataFixture) {
	VoxelVolume<WarpVoxel, PlainVoxelArray> warp_field_CPU1(*warp_field_iter0, MEMORYDEVICE_CPU);


	auto motionTracker_PVA_CPU = new SurfaceTracker<TSDFVoxel, WarpVoxel, PlainVoxelArray, MEMORYDEVICE_CPU, TRACKER_SLAVCHEVA_DIAGNOSTIC>(
			SlavchevaSurfaceTracker::Switches(true, false, true, true, false)
	);


	TimeIt([&]() {
		motionTracker_PVA_CPU->CalculateWarpGradient(&warp_field_CPU1, canonical_volume, live_volume);
	}, "Calculate Warping Gradient - PVA CPU data term + tikhonov term");

	AlteredGradientCountFunctor<WarpVoxel> functor;
	VolumeTraversalEngine<WarpVoxel, PlainVoxelArray, MEMORYDEVICE_CPU>::
	TraverseAll(&warp_field_CPU1, functor);
	BOOST_REQUIRE_EQUAL(functor.count.load(), 59093);

	float tolerance = 1e-8;
	BOOST_REQUIRE(contentAlmostEqual_CPU(&warp_field_CPU1, warp_field_data_and_killing_term, tolerance));
}


BOOST_FIXTURE_TEST_CASE(testDataAndLevelSetTerm_CPU_PVA, DataFixture) {

	VoxelVolume<WarpVoxel, PlainVoxelArray> warp_field_CPU1(*warp_field_iter0, MEMORYDEVICE_CPU);


	auto motionTracker_PVA_CPU = new SurfaceTracker<TSDFVoxel, WarpVoxel, PlainVoxelArray, MEMORYDEVICE_CPU, TRACKER_SLAVCHEVA_DIAGNOSTIC>(
			SlavchevaSurfaceTracker::Switches(true, true, false, false, false)
	);


	TimeIt([&]() {
		motionTracker_PVA_CPU->CalculateWarpGradient(&warp_field_CPU1, canonical_volume, live_volume);
	}, "Calculate Warping Gradient - PVA CPU data term + level set term");

	AlteredGradientCountFunctor<WarpVoxel> functor;
	VolumeTraversalEngine<WarpVoxel, PlainVoxelArray, MEMORYDEVICE_CPU>::
	TraverseAll(&warp_field_CPU1, functor);
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
