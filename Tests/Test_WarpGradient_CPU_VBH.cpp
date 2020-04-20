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
#include "TestUtilities.h"
#include "Test_WarpGradient_Common.h"
#include "../ITMLib/Utils/Configuration.h"
#include "../ITMLib/Engines/EditAndCopy/CPU/EditAndCopyEngine_CPU.h"
#include "../ITMLib/SurfaceTrackers/Interface/SurfaceTrackerInterface.h"
#include "../ITMLib/SurfaceTrackers/Interface/SurfaceTracker.h"
#include "../ITMLib/Utils/Analytics/VoxelVolumeComparison/VoxelVolumeComparison_CPU.h"
#include "../ITMLib/Engines/Analytics/AnalyticsEngine.h"
#include "../ITMLib/Engines/Traversal/CPU/VolumeTraversal_CPU_VoxelBlockHash.h"
#include "../ITMLib/Engines/Indexing/VBH/CPU/IndexingEngine_CPU_VoxelBlockHash.h"

//test_utils
#include "TestUtilsForSnoopyFrames16And17.h"
#include "TestUtilities.h"

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


typedef WarpGradientDataFixture<MemoryDeviceType::MEMORYDEVICE_CPU, VoxelBlockHash> DataFixture;
BOOST_FIXTURE_TEST_CASE(testDataTerm_CPU_VBH, DataFixture) {

	VoxelVolume<WarpVoxel, VoxelBlockHash> warp_field(MEMORYDEVICE_CPU, indexParameters);
	ManipulationEngine_CPU_VBH_Warp::Inst().ResetVolume(&warp_field);

	indexing_engine.AllocateWarpVolumeFromOtherVolume(&warp_field, live_volume);
	indexing_engine.AllocateUsingOtherVolume(canonical_volume, live_volume);

	BOOST_REQUIRE_EQUAL(Analytics_CPU_VBH_Warp::Instance().CountAllocatedHashBlocks(&warp_field), 633);

	auto motionTracker_VBH_CPU = new SurfaceTracker<TSDFVoxel, WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CPU, TRACKER_SLAVCHEVA_DIAGNOSTIC>(
			SlavchevaSurfaceTracker::Switches(true, false, false, false, false));


	TimeIt([&]() {
		motionTracker_VBH_CPU->CalculateWarpGradient(&warp_field, canonical_volume, live_volume);
	}, "Calculate Warping Gradient - VBH CPU data term");


	AlteredGradientCountFunctor<WarpVoxel> functor;
	VolumeTraversalEngine<WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>::
	TraverseAll(&warp_field, functor);
	BOOST_REQUIRE_EQUAL(functor.count.load(), 37525u);

	float tolerance = 1e-7;
	BOOST_REQUIRE(contentAlmostEqual_CPU(&warp_field, warp_field_data_term, tolerance));
}

BOOST_FIXTURE_TEST_CASE(testUpdateWarps_CPU_VBH, DataFixture) {

	auto motionTracker_VBH_CPU = new SurfaceTracker<TSDFVoxel, WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CPU, TRACKER_SLAVCHEVA_DIAGNOSTIC>(
			SlavchevaSurfaceTracker::Switches(false, false, false, false, false));
	VoxelVolume<WarpVoxel, VoxelBlockHash> warp_field_copy(*warp_field_data_term,
	                                                       MemoryDeviceType::MEMORYDEVICE_CPU);

	AlteredGradientCountFunctor<WarpVoxel> altered_count_functor;
	VolumeTraversalEngine<WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>::
	TraverseAll(&warp_field_copy, altered_count_functor);
	BOOST_REQUIRE_EQUAL(altered_count_functor.count.load(), 37525u);
	BOOST_REQUIRE_EQUAL(Analytics_CPU_VBH_Warp::Instance().CountAllocatedHashBlocks(&warp_field_copy), 633);

	indexing_engine.AllocateUsingOtherVolume(canonical_volume, live_volume);

	float max_warp = motionTracker_VBH_CPU->UpdateWarps(&warp_field_copy, canonical_volume, live_volume);
	BOOST_REQUIRE_CLOSE(max_warp, 0.121243507f, 1e-7f);


	AlteredWarpUpdateCountFunctor<WarpVoxel> functor;
	VolumeTraversalEngine<WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>::
	TraverseAll(&warp_field_copy, functor);
	BOOST_REQUIRE_EQUAL(functor.count.load(), 37525u);

	float tolerance = 1e-8;
	BOOST_REQUIRE(contentAlmostEqual_CPU(&warp_field_copy, warp_field_iter0, tolerance));
}


BOOST_FIXTURE_TEST_CASE(testSmoothWarpGradient_CPU_VBH, DataFixture) {

	VoxelVolume<WarpVoxel, VoxelBlockHash> warp_field(*warp_field_data_term, MEMORYDEVICE_CPU);

	indexing_engine.AllocateUsingOtherVolume(canonical_volume, live_volume);


	auto motionTracker_VBH_CPU = new SurfaceTracker<TSDFVoxel, WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CPU, TRACKER_SLAVCHEVA_DIAGNOSTIC>(
			SlavchevaSurfaceTracker::Switches(false, false, false, false, true)
	);

	TimeIt([&]() {
		motionTracker_VBH_CPU->SmoothWarpGradient(&warp_field, canonical_volume, live_volume);
	}, "Smooth Warping Gradient - VBH CPU");

	float tolerance = 1e-8;
	BOOST_REQUIRE(contentAlmostEqual_CPU(&warp_field, warp_field_data_term_smoothed, tolerance));
}

BOOST_FIXTURE_TEST_CASE(testTikhonovTerm_CPU_VBH, DataFixture) {

	VoxelVolume<WarpVoxel, VoxelBlockHash> warp_field(*warp_field_iter0, MEMORYDEVICE_CPU);

	indexing_engine.AllocateUsingOtherVolume(canonical_volume, live_volume);


	auto motionTracker_VBH_CPU = new SurfaceTracker<TSDFVoxel, WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CPU, TRACKER_SLAVCHEVA_DIAGNOSTIC>(
			SlavchevaSurfaceTracker::Switches(false, false, true, false, false)
	);
	Vector3i testPosition(-40, 60, 200);

	TimeIt([&]() {
		motionTracker_VBH_CPU->CalculateWarpGradient(&warp_field, canonical_volume, live_volume);
	}, "Calculate Warping Gradient - VBH CPU data term + tikhonov term");
	BOOST_REQUIRE_EQUAL(Analytics_CPU_VBH_Warp::Instance().CountAllocatedHashBlocks(&warp_field), 633);

	AlteredGradientCountFunctor<WarpVoxel> functor;
	VolumeTraversalEngine<WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>::
	TraverseAll(&warp_field, functor);
	BOOST_REQUIRE_EQUAL(functor.count.load(), 57416);


	WarpVoxel warp1 = ManipulationEngine_CPU_VBH_Warp::Inst().ReadVoxel(&warp_field, testPosition);
	WarpVoxel warp2 = ManipulationEngine_CPU_VBH_Warp::Inst().ReadVoxel(warp_field_tikhonov_term, testPosition);
	float tolerance = 1e-8;
	BOOST_REQUIRE_CLOSE(warp1.gradient0.x, warp2.gradient0.x, tolerance);
	BOOST_REQUIRE_CLOSE(warp1.gradient0.y, warp2.gradient0.y, tolerance);
	BOOST_REQUIRE_CLOSE(warp1.gradient0.z, warp2.gradient0.z, tolerance);

	BOOST_REQUIRE(contentAlmostEqual_CPU(&warp_field, warp_field_tikhonov_term, tolerance));
}

BOOST_FIXTURE_TEST_CASE(testDataAndTikhonovTerm_CPU_VBH, DataFixture) {
	VoxelVolume<WarpVoxel, VoxelBlockHash> warp_field(*warp_field_iter0, MEMORYDEVICE_CPU);
	indexing_engine.AllocateUsingOtherVolume(canonical_volume, live_volume);

	auto motionTracker_VBH_CPU = new SurfaceTracker<TSDFVoxel, WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CPU, TRACKER_SLAVCHEVA_DIAGNOSTIC>(
			SlavchevaSurfaceTracker::Switches(true, false, true, false, false));

	TimeIt([&]() {
		motionTracker_VBH_CPU->CalculateWarpGradient(&warp_field, canonical_volume, live_volume);
	}, "Calculate Warping Gradient - VBH CPU data term + tikhonov term");
	BOOST_REQUIRE_EQUAL(Analytics_CPU_VBH_Warp::Instance().CountAllocatedHashBlocks(&warp_field), 633);

	AlteredGradientCountFunctor<WarpVoxel> functor;
	VolumeTraversalEngine<WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>::
	TraverseAll(&warp_field, functor);
	BOOST_REQUIRE_EQUAL(functor.count.load(), 57416);

	Vector3i testPosition(-40, 60, 200);
	WarpVoxel warp1 = ManipulationEngine_CPU_VBH_Warp::Inst().ReadVoxel(&warp_field, testPosition);
	WarpVoxel warp2 = ManipulationEngine_CPU_VBH_Warp::Inst().ReadVoxel(warp_field_data_and_tikhonov_term,
	                                                                    testPosition);
	float tolerance = 1e-8;
	BOOST_REQUIRE_CLOSE(warp1.gradient0.x, warp2.gradient0.x, tolerance);
	BOOST_REQUIRE_CLOSE(warp1.gradient0.y, warp2.gradient0.y, tolerance);
	BOOST_REQUIRE_CLOSE(warp1.gradient0.z, warp2.gradient0.z, tolerance);

	BOOST_REQUIRE(contentAlmostEqual_CPU(&warp_field, warp_field_data_and_tikhonov_term, tolerance));
}


BOOST_FIXTURE_TEST_CASE(testDataAndKillingTerm_CPU_VBH, DataFixture) {

	VoxelVolume<WarpVoxel, VoxelBlockHash> warp_field(*warp_field_iter0, MEMORYDEVICE_CPU);

	indexing_engine.AllocateUsingOtherVolume(canonical_volume, live_volume);

	auto motionTracker_VBH_CPU = new SurfaceTracker<TSDFVoxel, WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CPU, TRACKER_SLAVCHEVA_DIAGNOSTIC>(
			SlavchevaSurfaceTracker::Switches(true, false, true, true, false));


	TimeIt([&]() {
		motionTracker_VBH_CPU->CalculateWarpGradient(&warp_field, canonical_volume, live_volume);
	}, "Calculate Warping Gradient - VBH CPU data term + tikhonov term");

	AlteredGradientCountFunctor<WarpVoxel> functor;
	VolumeTraversalEngine<WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>::
	TraverseAll(&warp_field, functor);
	BOOST_REQUIRE_EQUAL(functor.count.load(), 59093);

	float tolerance = 1e-8;
	BOOST_REQUIRE(contentAlmostEqual_CPU(&warp_field, warp_field_data_and_killing_term, tolerance));
}


BOOST_FIXTURE_TEST_CASE(testDataAndLevelSetTerm_CPU_VBH, DataFixture) {
	VoxelVolume<WarpVoxel, VoxelBlockHash> warp_field(*warp_field_iter0, MEMORYDEVICE_CPU);

	indexing_engine.AllocateUsingOtherVolume(canonical_volume, live_volume);

	auto motionTracker_VBH_CPU = new SurfaceTracker<TSDFVoxel, WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CPU, TRACKER_SLAVCHEVA_DIAGNOSTIC>(
			SlavchevaSurfaceTracker::Switches(true, true, false, false, false));

	TimeIt([&]() {
		motionTracker_VBH_CPU->CalculateWarpGradient(&warp_field, canonical_volume, live_volume);
	}, "Calculate Warping Gradient - VBH CPU data term + level set term");

	AlteredGradientCountFunctor<WarpVoxel> functor;
	VolumeTraversalEngine<WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>::
	TraverseAll(&warp_field, functor);
	BOOST_REQUIRE_EQUAL(functor.count.load(), 55369);

	float tolerance = 1e-7;
	BOOST_REQUIRE(contentAlmostEqual_CPU(&warp_field, warp_field_data_and_level_set_term, tolerance));
}

//#define GENERATE_TEST_DATA
#ifdef GENERATE_TEST_DATA
BOOST_AUTO_TEST_CASE(Test_WarpGradient_CUDA_VBH_GenerateTestData){
	GenerateTestData<VoxelBlockHash,MEMORYDEVICE_CPU>();
}
#endif