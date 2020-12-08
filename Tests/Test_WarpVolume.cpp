//  ================================================================
//  Created by Gregory Kramida on 10/31/19.
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

#define BOOST_TEST_MODULE WarpVolume
#ifndef WIN32
#define BOOST_TEST_DYN_LINK
#endif

//boost
#include <boost/test/unit_test.hpp>

//ITMLib
#include "../ITMLib/GlobalTemplateDefines.h"
#include "../ITMLib/Objects/Volume/VoxelVolume.h"
#include "../ITMLib/Engines/DepthFusion/DepthFusionEngine.h"
#include "../ITMLib/Utils/Analytics/VoxelVolumeComparison/VoxelVolumeComparison.h"
#include "../ITMLib/Engines/Warping/WarpingEngine.h"
#include "../ITMLib/Engines/Warping/WarpingEngineFactory.h"
//(CPU)
#include "../ITMLib/Utils/Analytics/VoxelVolumeComparison/VoxelVolumeComparison_CPU.h"
#include "../ITMLib/Engines/Indexing/VBH/CPU/IndexingEngine_VoxelBlockHash_CPU.h"
//(CUDA)
#ifndef COMPILE_WITHOUT_CUDA

#include "../ITMLib/Utils/Analytics/VoxelVolumeComparison/VoxelVolumeComparison_CUDA.h"
#include "../ITMLib/Engines/Indexing/VBH/CUDA/IndexingEngine_VoxelBlockHash_CUDA.h"

#endif


//test_utilities
#include "TestUtilities/TestUtilities.h"
#include "TestUtilities/TestDataUtilities.h"
#include "TestUtilities/LevelSetAlignment/LevelSetAlignmentTestUtilities.h"
#include "TestUtilities/LevelSetAlignment/TestCaseOrganizationBySwitches.h"

using namespace ITMLib;
using namespace test;

/*
 * These tests exists mostly to debug issues with the warping, i.e.
 * if Test_LevelSetAlignment_PVA_vs_VBH succeed, these are also guaranteed to succeed.
 * However, if the former tests fail and the issue is with the warping, these tests provide a window
 * into what happens during the warping over a single iteration.
*/

template<typename TIndex, MemoryDeviceType TMemoryDeviceType>
void GenericWarpVolumeTest() {
	// load warps
	LevelSetAlignmentSwitches data_tikhonov_sobolev_switches(true, false, true, false, true);
	std::string warps_path = GetWarpsPath<TIndex>(SwitchesToPrefix(data_tikhonov_sobolev_switches), 0);
	VoxelVolume<WarpVoxel, TIndex>* warps;
	LoadVolume(&warps, warps_path, TMemoryDeviceType, test::snoopy::InitializationParameters_Fr16andFr17<TIndex>());

	// load original volume
	VoxelVolume<TSDFVoxel, TIndex>* source_volume;
	LoadVolume(&source_volume, test::snoopy::PartialVolume17Path<TIndex>(), TMemoryDeviceType, test::snoopy::InitializationParameters_Fr16andFr17<TIndex>());

	// warp the volume
	auto target_volume = new VoxelVolume<TSDFVoxel, TIndex>(TMemoryDeviceType,
	                                                        test::snoopy::InitializationParameters_Fr16andFr17<TIndex>());
	target_volume->Reset();

	auto warping_engine =
			WarpingEngineFactory::Build<TSDFVoxel, WarpVoxel, TIndex>(TMemoryDeviceType);

	AllocateUsingOtherVolume(target_volume, source_volume, TMemoryDeviceType);
	warping_engine->WarpVolume_WarpUpdates(warps, source_volume, target_volume);

	// load the ground truth warped volume
	VoxelVolume<TSDFVoxel, TIndex>* warped_live_volume_gt;
	std::string warped_live_path = GetWarpedLivePath<TIndex>(SwitchesToPrefix(data_tikhonov_sobolev_switches), 0);
	LoadVolume(&warped_live_volume_gt, warped_live_path, TMemoryDeviceType, test::snoopy::InitializationParameters_Fr16andFr17<TIndex>());

	// compare warped volume with ground truth
	float absolute_tolerance = 1e-6;
	BOOST_REQUIRE(!ContentAlmostEqual(target_volume, source_volume, absolute_tolerance, TMemoryDeviceType));
	BOOST_REQUIRE(ContentAlmostEqual_Verbose(target_volume, warped_live_volume_gt, absolute_tolerance,
	                                         TMemoryDeviceType));

	delete warping_engine;
	delete warps;
	delete source_volume;
	delete target_volume;
}

template<MemoryDeviceType TMemoryDeviceType>
void GenericWarpVolume_VBH_to_PVA_Test(const int iteration = 0) {
	// *** load warps
	LevelSetAlignmentSwitches data_only_switches(true, false, false, false, false);
	std::string path_warps_PVA = GetWarpsPath<PlainVoxelArray>(SwitchesToPrefix(data_only_switches), iteration);
	std::string path_warps_VBH = GetWarpsPath<VoxelBlockHash>(SwitchesToPrefix(data_only_switches), iteration);

	VoxelVolume<WarpVoxel, PlainVoxelArray>* warps_PVA;
	LoadVolume(&warps_PVA, path_warps_PVA, TMemoryDeviceType,
	           test::snoopy::InitializationParameters_Fr16andFr17<PlainVoxelArray>());
	VoxelVolume<WarpVoxel, VoxelBlockHash>* warps_VBH;
	LoadVolume(&warps_VBH, path_warps_VBH, TMemoryDeviceType,
	           test::snoopy::InitializationParameters_Fr16andFr17<VoxelBlockHash>());

	std::string path_frame_17_PVA = test::snoopy::PartialVolume17Path<PlainVoxelArray>();
	std::string path_frame_17_VBH = test::snoopy::PartialVolume17Path<VoxelBlockHash>();

	std::string source_path_PVA;
	std::string source_path_VBH;
	if (iteration == 0) {
		source_path_PVA = path_frame_17_PVA;
		source_path_VBH = path_frame_17_VBH;
	} else {
		source_path_PVA = GetWarpedLivePath<PlainVoxelArray>(SwitchesToPrefix(data_only_switches), iteration - 1);
		source_path_VBH = GetWarpedLivePath<VoxelBlockHash>(SwitchesToPrefix(data_only_switches), iteration - 1);
	}

	// *** load same frame scene as the two different data structures
	VoxelVolume<TSDFVoxel, PlainVoxelArray>* source_volume_PVA;
	LoadVolume(&source_volume_PVA, source_path_PVA, TMemoryDeviceType,
	           test::snoopy::InitializationParameters_Fr16andFr17<PlainVoxelArray>());
	VoxelVolume<TSDFVoxel, VoxelBlockHash>* source_volume_VBH;
	LoadVolume(&source_volume_VBH, source_path_VBH, TMemoryDeviceType,
	           test::snoopy::InitializationParameters_Fr16andFr17<VoxelBlockHash>());

	// *** initialize target scenes
	VoxelVolume<TSDFVoxel, PlainVoxelArray>* target_PVA;
	VoxelVolume<TSDFVoxel, VoxelBlockHash>* target_VBH;
	InitializeVolume(&target_PVA, test::snoopy::InitializationParameters_Fr16andFr17<PlainVoxelArray>(), TMemoryDeviceType);
	InitializeVolume(&target_VBH, test::snoopy::InitializationParameters_Fr16andFr17<VoxelBlockHash>(), TMemoryDeviceType);

	// *** perform the warping
	auto warping_engine_PVA =
			WarpingEngineFactory::Build<TSDFVoxel, WarpVoxel, PlainVoxelArray>(TMemoryDeviceType);
	auto warping_engine_VBH =
			WarpingEngineFactory::Build<TSDFVoxel, WarpVoxel, VoxelBlockHash>(TMemoryDeviceType);

	IndexingEngine<TSDFVoxel, VoxelBlockHash, TMemoryDeviceType>& indexer =
			IndexingEngine<TSDFVoxel, VoxelBlockHash, TMemoryDeviceType>::Instance();
	AllocateUsingOtherVolume(target_VBH, source_volume_VBH, TMemoryDeviceType);
	warping_engine_PVA->WarpVolume_WarpUpdates(warps_PVA, source_volume_PVA, target_PVA);
	warping_engine_VBH->WarpVolume_WarpUpdates(warps_VBH, source_volume_VBH, target_VBH);

	// *** test content
	float absolute_tolerance = 1e-7;
	BOOST_REQUIRE(
			ContentForFlagsAlmostEqual_Verbose(
					target_PVA, target_VBH,
					VOXEL_NONTRUNCATED, absolute_tolerance, TMemoryDeviceType
			)
	);

	delete warps_PVA;
	delete warps_VBH;
	delete source_volume_PVA;
	delete source_volume_VBH;
	delete target_PVA;
	delete target_VBH;
}

BOOST_AUTO_TEST_CASE(Test_WarpVolume_CPU_PVA) {
	GenericWarpVolumeTest<PlainVoxelArray, MEMORYDEVICE_CPU>();
}

BOOST_AUTO_TEST_CASE(Test_WarpVolume_CPU_VBH) {
	GenericWarpVolumeTest<VoxelBlockHash, MEMORYDEVICE_CPU>();
}

BOOST_AUTO_TEST_CASE(Test_WarpVolume_CPU_VBH_to_PVA) {
	GenericWarpVolume_VBH_to_PVA_Test<MEMORYDEVICE_CPU>(0);
}

#ifndef COMPILE_WITHOUT_CUDA
BOOST_AUTO_TEST_CASE(Test_WarpVolume_CUDA_PVA) {
	GenericWarpVolumeTest<PlainVoxelArray, MEMORYDEVICE_CUDA>();
}

BOOST_AUTO_TEST_CASE(Test_WarpVolume_CUDA_VBH) {
	GenericWarpVolumeTest<VoxelBlockHash, MEMORYDEVICE_CUDA>();
}

BOOST_AUTO_TEST_CASE(Test_WarpVolume_CUDA_VBH_to_PVA) {
	GenericWarpVolume_VBH_to_PVA_Test<MEMORYDEVICE_CUDA>(1);
}

#endif