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
#define BOOST_TEST_MODULE LevelSetAlignmentConsistency
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
#include "../ITMLib/Utils/Analytics/VoxelVolumeComparison/VoxelVolumeComparison.h"
#include "TestUtilities/WarpAdvancedTestingUtilities.h"

using namespace ITMLib;
using namespace test_utilities;


typedef WarpGradientDataFixture<MemoryDeviceType::MEMORYDEVICE_CPU, PlainVoxelArray> DataFixture_CPU_PVA;
typedef WarpGradientDataFixture<MemoryDeviceType::MEMORYDEVICE_CPU, VoxelBlockHash> DataFixture_CPU_VBH;
#ifndef COMPILE_WITH_CUDA
typedef WarpGradientDataFixture<MemoryDeviceType::MEMORYDEVICE_CUDA, PlainVoxelArray> DataFixture_CUDA_PVA;
typedef WarpGradientDataFixture<MemoryDeviceType::MEMORYDEVICE_CUDA, VoxelBlockHash> DataFixture_CUDA_VBH;
#endif

template<typename TIndex, MemoryDeviceType TMemoryDeviceType>
void GenericLevelSetEvolutionConsistencyTest(int iteration, const LevelSetAlignmentSwitches& switches,
                                             WarpGradientDataFixture<TMemoryDeviceType, TIndex>& fixture) {
	if (iteration < 0 || iteration > 1) {
		DIEWITHEXCEPTION_REPORTLOCATION("Iteration number should be 0 or 1!");
	}
	VoxelVolume<WarpVoxel, TIndex>* warp_field;

	if (iteration == 0) {
		warp_field = new VoxelVolume<WarpVoxel, TIndex>(TMemoryDeviceType, fixture.index_parameters);
		warp_field->Reset();
		AllocateUsingOtherVolume(warp_field, fixture.live_volume, TMemoryDeviceType);
	} else {
		warp_field = new VoxelVolume<WarpVoxel, TIndex>(*fixture.GetIteration1StartingWarpField(), TMemoryDeviceType);
	}

	auto motion_tracker = new LevelSetAlignmentEngine<TSDFVoxel, WarpVoxel, TIndex, TMemoryDeviceType, DIAGNOSTIC>(
			switches, SingleIterationTerminationConditions());

	VoxelVolume<TSDFVoxel, TIndex>* live_volumes[2] = {fixture.live_volume, nullptr};
	TimeIt([&]() {
		motion_tracker->Align(warp_field, live_volumes, fixture.canonical_volume);
	}, "Calculate Warping Gradient - data term");

	unsigned int altered_update_count = AnalyticsEngine<WarpVoxel, TIndex, TMemoryDeviceType>::Instance().CountAlteredWarpUpdates(warp_field);
	BOOST_REQUIRE_EQUAL(altered_update_count, fixture.GetUpdateCount(iteration, switches));

	float average_warp = AnalyticsEngine<WarpVoxel, TIndex, TMemoryDeviceType>::Instance().ComputeWarpUpdateMean(warp_field);
	BOOST_REQUIRE_CLOSE(average_warp, fixture.GetAverageUpdateLength(iteration, switches), 1e-4);


	float tolerance = 1e-5;
	BOOST_REQUIRE(ContentAlmostEqual(warp_field, fixture.GetWarpField(iteration, switches), tolerance, TMemoryDeviceType));
	delete warp_field;
};

// Iteration 0: Data term only

BOOST_FIXTURE_TEST_CASE(testDataTerm_CPU_PVA, DataFixture_CPU_PVA) {
	LevelSetAlignmentSwitches switches(true, false, false, false, false);
	GenericLevelSetEvolutionConsistencyTest<PlainVoxelArray, MEMORYDEVICE_CPU>(0, switches, *this);
}

BOOST_FIXTURE_TEST_CASE(testDataTerm_CPU_VBH, DataFixture_CPU_VBH) {
	LevelSetAlignmentSwitches switches(true, false, false, false, false);
	GenericLevelSetEvolutionConsistencyTest<VoxelBlockHash, MEMORYDEVICE_CPU>(0, switches, *this);
}

#ifndef COMPILE_WITHOUT_CUDA
BOOST_FIXTURE_TEST_CASE(testDataTerm_CUDA_PVA, DataFixture_CUDA_PVA) {
	LevelSetAlignmentSwitches switches(true, false, false, false, false);
	GenericLevelSetEvolutionConsistencyTest<PlainVoxelArray, MEMORYDEVICE_CUDA>(0, switches, *this);
}

BOOST_FIXTURE_TEST_CASE(testDataTerm_CUDA_VBH, DataFixture_CUDA_VBH) {
	LevelSetAlignmentSwitches switches(true, false, false, false, false);
	GenericLevelSetEvolutionConsistencyTest<VoxelBlockHash, MEMORYDEVICE_CUDA>(0, switches, *this);
}
#endif

// Iteration 0: Data term, sobolev-smoothed

BOOST_FIXTURE_TEST_CASE(testDataTermSobolevSmoothed_CPU_PVA, DataFixture_CPU_PVA) {
	LevelSetAlignmentSwitches switches(true, false, false, false, true);
	GenericLevelSetEvolutionConsistencyTest<PlainVoxelArray, MEMORYDEVICE_CPU>(0, switches, *this);
}

BOOST_FIXTURE_TEST_CASE(testDataTermSobolevSmoothed_CPU_VBH, DataFixture_CPU_VBH) {
	LevelSetAlignmentSwitches switches(true, false, false, false, true);
	GenericLevelSetEvolutionConsistencyTest<VoxelBlockHash, MEMORYDEVICE_CPU>(0, switches, *this);
}

#ifndef COMPILE_WITHOUT_CUDA
BOOST_FIXTURE_TEST_CASE(testDataTermSobolevSmoothed_CUDA_PVA, DataFixture_CUDA_PVA) {
	LevelSetAlignmentSwitches switches(true, false, false, false, true);
	GenericLevelSetEvolutionConsistencyTest<PlainVoxelArray, MEMORYDEVICE_CUDA>(0, switches, *this);
}

BOOST_FIXTURE_TEST_CASE(testDataTermSobolevSmoothed_CUDA_VBH, DataFixture_CUDA_VBH) {
	LevelSetAlignmentSwitches switches(true, false, false, false, true);
	GenericLevelSetEvolutionConsistencyTest<VoxelBlockHash, MEMORYDEVICE_CUDA>(0, switches, *this);
}
#endif


// Iteration 0: Data term and tikhonov terms, sobolev-smoothed
BOOST_FIXTURE_TEST_CASE(testDataAndTikhonovTermsSobolevSmoothed_CPU_PVA, DataFixture_CPU_PVA) {
	LevelSetAlignmentSwitches switches(true, false, true, false, true);
	GenericLevelSetEvolutionConsistencyTest<PlainVoxelArray, MEMORYDEVICE_CPU>(0, switches, *this);
}

BOOST_FIXTURE_TEST_CASE(testDataAndTikhonovTermsSobolevSmoothed_CPU_VBH, DataFixture_CPU_VBH) {
	LevelSetAlignmentSwitches switches(true, false, true, false, true);
	GenericLevelSetEvolutionConsistencyTest<VoxelBlockHash, MEMORYDEVICE_CPU>(0, switches, *this);
}

#ifndef COMPILE_WITHOUT_CUDA
BOOST_FIXTURE_TEST_CASE(testDataDataAndTikhonovTermsSobolevSmoothed_CUDA_PVA, DataFixture_CUDA_PVA) {
	LevelSetAlignmentSwitches switches(true, false, true, false, true);
	GenericLevelSetEvolutionConsistencyTest<PlainVoxelArray, MEMORYDEVICE_CUDA>(0, switches, *this);
}

BOOST_FIXTURE_TEST_CASE(testDataDataAndTikhonovTermsSobolevSmoothed_CUDA_VBH, DataFixture_CUDA_VBH) {
	LevelSetAlignmentSwitches switches(true, false, true, false, true);
	GenericLevelSetEvolutionConsistencyTest<VoxelBlockHash, MEMORYDEVICE_CUDA>(0, switches, *this);
}
#endif


// Iteration 1: Tikhonov term only
BOOST_FIXTURE_TEST_CASE(testTikhonovTerm_CPU_PVA, DataFixture_CPU_PVA) {
	LevelSetAlignmentSwitches switches(false, false, true, false, false);
	GenericLevelSetEvolutionConsistencyTest<PlainVoxelArray, MEMORYDEVICE_CPU>(1, switches, *this);
}

BOOST_FIXTURE_TEST_CASE(testTikhonovTerm_CPU_VBH, DataFixture_CPU_VBH) {
	LevelSetAlignmentSwitches switches(false, false, true, false, false);
	GenericLevelSetEvolutionConsistencyTest<VoxelBlockHash, MEMORYDEVICE_CPU>(1, switches, *this);
}

#ifndef COMPILE_WITHOUT_CUDA
BOOST_FIXTURE_TEST_CASE(testTikhonovTerm_CUDA_PVA, DataFixture_CUDA_PVA) {
	LevelSetAlignmentSwitches switches(false, false, true, false, false);
	GenericLevelSetEvolutionConsistencyTest<PlainVoxelArray, MEMORYDEVICE_CUDA>(1, switches, *this);
}

BOOST_FIXTURE_TEST_CASE(testTikhonovTerm_CUDA_VBH, DataFixture_CUDA_VBH) {
	LevelSetAlignmentSwitches switches(false, false, true, false, false);
	GenericLevelSetEvolutionConsistencyTest<VoxelBlockHash, MEMORYDEVICE_CUDA>(1, switches, *this);
}
#endif

// Iteration 1: Data & Tikhonov term
BOOST_FIXTURE_TEST_CASE(testDataAndTikhonovTerms_CPU_PVA, DataFixture_CPU_PVA) {
	LevelSetAlignmentSwitches switches(true, false, true, false, false);
	GenericLevelSetEvolutionConsistencyTest<PlainVoxelArray, MEMORYDEVICE_CPU>(1, switches, *this);
}

BOOST_FIXTURE_TEST_CASE(testDataAndTikhonovTerms_CPU_VBH, DataFixture_CPU_VBH) {
	LevelSetAlignmentSwitches switches(true, false, true, false, false);
	GenericLevelSetEvolutionConsistencyTest<VoxelBlockHash, MEMORYDEVICE_CPU>(1, switches, *this);
}

#ifndef COMPILE_WITHOUT_CUDA
BOOST_FIXTURE_TEST_CASE(testDataAndTikhonovTerms_CUDA_PVA, DataFixture_CUDA_PVA) {
	LevelSetAlignmentSwitches switches(true, false, true, false, false);
	GenericLevelSetEvolutionConsistencyTest<PlainVoxelArray, MEMORYDEVICE_CUDA>(1, switches, *this);
}

BOOST_FIXTURE_TEST_CASE(testDataAndTikhonovTerms_CUDA_VBH, DataFixture_CUDA_VBH) {
	LevelSetAlignmentSwitches switches(true, false, true, false, false);
	GenericLevelSetEvolutionConsistencyTest<VoxelBlockHash, MEMORYDEVICE_CUDA>(1, switches, *this);
}
#endif


// Iteration 1: Data & dampened Approximately-Killing-Vector-Field terms
BOOST_FIXTURE_TEST_CASE(testDataAndAKVFTerms_CPU_PVA, DataFixture_CPU_PVA) {
	LevelSetAlignmentSwitches switches(true, false, true, true, false);
	GenericLevelSetEvolutionConsistencyTest<PlainVoxelArray, MEMORYDEVICE_CPU>(1, switches, *this);
}

BOOST_FIXTURE_TEST_CASE(testDataAndAKVFTerms_CPU_VBH, DataFixture_CPU_VBH) {
	LevelSetAlignmentSwitches switches(true, false, true, true, false);
	GenericLevelSetEvolutionConsistencyTest<VoxelBlockHash, MEMORYDEVICE_CPU>(1, switches, *this);
}

#ifndef COMPILE_WITHOUT_CUDA
BOOST_FIXTURE_TEST_CASE(testDataAndAKVFTerms_CUDA_PVA, DataFixture_CUDA_PVA) {
	LevelSetAlignmentSwitches switches(true, false, true, true, false);
	GenericLevelSetEvolutionConsistencyTest<PlainVoxelArray, MEMORYDEVICE_CUDA>(1, switches, *this);
}

BOOST_FIXTURE_TEST_CASE(testDataAndAKVFTerms_CUDA_VBH, DataFixture_CUDA_VBH) {
	LevelSetAlignmentSwitches switches(true, false, true, true, false);
	GenericLevelSetEvolutionConsistencyTest<VoxelBlockHash, MEMORYDEVICE_CUDA>(1, switches, *this);
}
#endif

// Iteration 1: Data & level set terms
BOOST_FIXTURE_TEST_CASE(testDataAndLevelSetTerms_CPU_PVA, DataFixture_CPU_PVA) {
	LevelSetAlignmentSwitches switches(true, true, false, false, false);
	GenericLevelSetEvolutionConsistencyTest<PlainVoxelArray, MEMORYDEVICE_CPU>(1, switches, *this);
}

BOOST_FIXTURE_TEST_CASE(testDataAndLevelSetTerms_CPU_VBH, DataFixture_CPU_VBH) {
	LevelSetAlignmentSwitches switches(true, true, false, false, false);
	GenericLevelSetEvolutionConsistencyTest<VoxelBlockHash, MEMORYDEVICE_CPU>(1, switches, *this);
}

#ifndef COMPILE_WITHOUT_CUDA
BOOST_FIXTURE_TEST_CASE(testDataAndLevelSetTerms_CUDA_PVA, DataFixture_CUDA_PVA) {
	LevelSetAlignmentSwitches switches(true, true, false, false, false);
	GenericLevelSetEvolutionConsistencyTest<PlainVoxelArray, MEMORYDEVICE_CUDA>(1, switches, *this);
}

BOOST_FIXTURE_TEST_CASE(testDataAndLevelSetTerms_CUDA_VBH, DataFixture_CUDA_VBH) {
	LevelSetAlignmentSwitches switches(true, true, false, false, false);
	GenericLevelSetEvolutionConsistencyTest<VoxelBlockHash, MEMORYDEVICE_CUDA>(1, switches, *this);
}
#endif