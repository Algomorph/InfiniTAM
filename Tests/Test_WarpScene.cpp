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

#define BOOST_TEST_MODULE WarpScene
#ifndef WIN32
#define BOOST_TEST_DYN_LINK
#endif

//boost
#include <boost/test/unit_test.hpp>

//local
#include "../ITMLib/ITMLibDefines.h"
#include "../ITMLib/Objects/Scene/ITMVoxelVolume.h"
#include "../ITMLib/Engines/DepthFusion/DynamicSceneReconstructionEngine_CPU.h"
#include "../ITMLib/Utils/Analytics/VoxelVolumeComparison/ITMVoxelVolumeComparison_CPU.h"
#include "../ITMLib/Engines/Warping/WarpingEngine.h"

#ifndef COMPILE_WITHOUT_CUDA

#include "../ITMLib/Utils/Analytics/VoxelVolumeComparison/ITMVoxelVolumeComparison_CUDA.h"
#endif

#include "TestUtils.h"
#include "TestUtilsForSnoopyFrames16And17.h"

using namespace ITMLib;

typedef WarpingEngine<ITMVoxel,ITMWarp,PlainVoxelArray,MEMORYDEVICE_CPU> WarpingEngine_CPU_PVA;
typedef WarpingEngine<ITMVoxel,ITMWarp,VoxelBlockHash,MEMORYDEVICE_CPU> WarpingEngine_CPU_VBH;

//#define SAVE_TEST_DATA
BOOST_FIXTURE_TEST_CASE(Test_WarpScene_CPU_PVA, Frame16And17Fixture) {
	const configuration::Configuration& settings = configuration::get();
	ITMVoxelVolume<ITMWarp, PlainVoxelArray>* warps;
	loadVolume(&warps, "TestData/snoopy_result_fr16-17_partial_PVA/warp_field_0_complete_",
	           MEMORYDEVICE_CPU, InitParams<PlainVoxelArray>());
	ITMVoxelVolume<ITMVoxel, PlainVoxelArray>* live_volume;
	loadVolume(&live_volume, "TestData/snoopy_result_fr16-17_partial_PVA/snoopy_partial_frame_17_",
	           MEMORYDEVICE_CPU, InitParams<PlainVoxelArray>());
	auto warped_live_volume = new ITMVoxelVolume<ITMVoxel, PlainVoxelArray>(
			&settings.general_voxel_volume_parameters, false, MEMORYDEVICE_CPU, InitParams<PlainVoxelArray>());
	warped_live_volume->Reset();

	WarpingEngine_CPU_PVA warpingEngine;

	warpingEngine.WarpVolume_WarpUpdates(warps, live_volume, warped_live_volume);
#ifdef SAVE_TEST_DATA
	warped_live_volume->SaveToDirectory("../../Tests/TestData/snoopy_result_fr16-17_partial_PVA/warped_live_");
#endif

	ITMVoxelVolume<ITMVoxel, PlainVoxelArray>* warped_live_volume_gt;
	loadVolume(&warped_live_volume_gt, "TestData/snoopy_result_fr16-17_partial_PVA/warped_live_",
	           MEMORYDEVICE_CPU, InitParams<PlainVoxelArray>());

	float absoluteTolerance = 1e-7;
	BOOST_REQUIRE(!contentAlmostEqual_CPU(warped_live_volume, live_volume, absoluteTolerance));
	BOOST_REQUIRE(contentAlmostEqual_CPU(warped_live_volume, warped_live_volume_gt, absoluteTolerance));

	delete warps;
	delete live_volume;
	delete warped_live_volume;
}

BOOST_FIXTURE_TEST_CASE(Test_WarpScene_CPU_VBH, Frame16And17Fixture) {
	const configuration::Configuration& settings = configuration::get();
	ITMVoxelVolume<ITMWarp, VoxelBlockHash>* warps;
	loadVolume(&warps, "TestData/snoopy_result_fr16-17_partial_VBH/warp_field_0_complete_",
	           MEMORYDEVICE_CPU, InitParams<VoxelBlockHash>());
	ITMVoxelVolume<ITMVoxel, VoxelBlockHash>* live_volume;
	loadVolume(&live_volume, "TestData/snoopy_result_fr16-17_partial_VBH/snoopy_partial_frame_17_",
	           MEMORYDEVICE_CPU, InitParams<VoxelBlockHash>());
	auto warped_live_volume = new ITMVoxelVolume<ITMVoxel, VoxelBlockHash>(
			&settings.general_voxel_volume_parameters, false, MEMORYDEVICE_CPU, InitParams<VoxelBlockHash>());
	warped_live_volume->Reset();

	WarpingEngine_CPU_VBH warpingEngine;

	warpingEngine.WarpVolume_WarpUpdates(warps, live_volume, warped_live_volume);
#ifdef SAVE_TEST_DATA
	warped_live_volume->SaveToDirectory("../../Tests/TestData/snoopy_result_fr16-17_partial_VBH/warped_live_");
#endif

	ITMVoxelVolume<ITMVoxel, VoxelBlockHash>* warped_live_volume_gt;
	loadVolume(&warped_live_volume_gt, "TestData/snoopy_result_fr16-17_partial_VBH/warped_live_",
	           MEMORYDEVICE_CPU, InitParams<VoxelBlockHash>());


	float absoluteTolerance = 1e-6;
	BOOST_REQUIRE(!contentAlmostEqual_CPU(warped_live_volume, live_volume, absoluteTolerance));
	BOOST_REQUIRE(contentAlmostEqual_CPU_Verbose(warped_live_volume, warped_live_volume_gt, absoluteTolerance));

	delete warps;
	delete live_volume;
	delete warped_live_volume;
}

BOOST_FIXTURE_TEST_CASE(Test_WarpScene_CPU_VBH_to_PVA, Frame16And17Fixture) {
	const int iteration = 0;
	// *** load warps
	std::string path_warps = "TestData/snoopy_result_fr16-17_warps/data_only_iter_" + std::to_string(iteration) + "_";
	ITMVoxelVolume<ITMWarp, PlainVoxelArray>* warps_PVA;
	loadVolume(&warps_PVA, path_warps, MEMORYDEVICE_CPU, InitParams<PlainVoxelArray>());
	ITMVoxelVolume<ITMWarp, VoxelBlockHash>* warps_VBH;
	loadVolume(&warps_VBH, path_warps, MEMORYDEVICE_CPU, InitParams<VoxelBlockHash>());

	std::string path_frame_17_PVA = "TestData/snoopy_result_fr16-17_partial_PVA/snoopy_partial_frame_17_";
	std::string path_frame_17_VBH = "TestData/snoopy_result_fr16-17_partial_VBH/snoopy_partial_frame_17_";

	std::string source_path_PVA;
	std::string source_path_VBH;
	if (iteration == 0) {
		source_path_PVA = path_frame_17_PVA;
		source_path_VBH = path_frame_17_VBH;
	} else {
		source_path_PVA =
				"TestData/snoopy_result_fr16-17_warps/data_only_iter_" + std::to_string(iteration - 1) +
				"_warped_live_";
		source_path_VBH =
				"TestData/snoopy_result_fr16-17_warps/data_only_iter_" + std::to_string(iteration - 1) +
				"_warped_live_";
	}

	// *** load same frame scene as the two different data structures
	ITMVoxelVolume<ITMVoxel, PlainVoxelArray>* source_volume_PVA;
	loadVolume(&source_volume_PVA, source_path_PVA, MEMORYDEVICE_CPU, InitParams<PlainVoxelArray>());
	ITMVoxelVolume<ITMVoxel, VoxelBlockHash>* source_volume_VBH;
	loadVolume(&source_volume_VBH, source_path_VBH, MEMORYDEVICE_CPU, InitParams<VoxelBlockHash>());

	// *** initialize target scenes
	ITMVoxelVolume<ITMVoxel, PlainVoxelArray>* target_PVA;
	ITMVoxelVolume<ITMVoxel, VoxelBlockHash>* target_VBH;
	initializeVolume(&target_PVA, InitParams<PlainVoxelArray>(), MEMORYDEVICE_CPU);
	initializeVolume(&target_VBH, InitParams<VoxelBlockHash>(), MEMORYDEVICE_CPU);

	// *** perform the warping
	WarpingEngine_CPU_PVA warpingEngine_PVA;
	WarpingEngine_CPU_VBH warpingEngine_VBH;


	warpingEngine_PVA.WarpVolume_WarpUpdates(warps_PVA, source_volume_PVA, target_PVA);
	warpingEngine_VBH.WarpVolume_WarpUpdates(warps_VBH, source_volume_VBH, target_VBH);

	// *** test content
	float absoluteTolerance = 1e-7;
	BOOST_REQUIRE(
			contentForFlagsAlmostEqual_CPU_Verbose(target_PVA, target_VBH, VOXEL_NONTRUNCATED, absoluteTolerance));


	delete warps_PVA;
	delete warps_VBH;
	delete source_volume_PVA;
	delete source_volume_VBH;
	delete target_PVA;
	delete target_VBH;
}

#ifndef COMPILE_WITHOUT_CUDA
typedef WarpingEngine<ITMVoxel, ITMWarp, PlainVoxelArray, MEMORYDEVICE_CUDA> WarpingEngine_CUDA_PVA;
typedef WarpingEngine<ITMVoxel, ITMWarp, VoxelBlockHash, MEMORYDEVICE_CUDA> WarpingEngine_CUDA_VBH;


BOOST_FIXTURE_TEST_CASE(Test_WarpScene_CUDA_VBH_to_PVA, Frame16And17Fixture) {
	const int iteration = 1;
	//std::string prefix = "data_only";
	std::string prefix = "data_tikhonov";
	// *** load warps
	std::string path_warps =
			"TestData/snoopy_result_fr16-17_warps/" + prefix + "_iter_" + std::to_string(iteration) + "_";
	ITMVoxelVolume<ITMWarp, PlainVoxelArray>* warps_PVA;
	loadVolume(&warps_PVA, path_warps, MEMORYDEVICE_CUDA, InitParams<PlainVoxelArray>());
	ITMVoxelVolume<ITMWarp, VoxelBlockHash>* warps_VBH_CPU;
	loadVolume(&warps_VBH_CPU, path_warps, MEMORYDEVICE_CPU, InitParams<VoxelBlockHash>());
	ITMVoxelVolume<ITMWarp, VoxelBlockHash>* warps_VBH;
	loadVolume(&warps_VBH, path_warps, MEMORYDEVICE_CUDA, InitParams<VoxelBlockHash>());

	std::string path_frame_17_PVA = "TestData/snoopy_result_fr16-17_partial_PVA/snoopy_partial_frame_17_";
	std::string path_frame_17_VBH = "TestData/snoopy_result_fr16-17_partial_VBH/snoopy_partial_frame_17_";

	std::string source_path_PVA;
	std::string source_path_VBH;
	if (iteration == 0) {
		source_path_PVA = path_frame_17_PVA;
		source_path_VBH = path_frame_17_VBH;
	} else {
		source_path_PVA =
				"TestData/snoopy_result_fr16-17_warps/" + prefix + "_iter_" + std::to_string(iteration - 1) +
				"_warped_live_";
		source_path_VBH =
				"TestData/snoopy_result_fr16-17_warps/" + prefix + "_iter_" + std::to_string(iteration - 1) +
				"_warped_live_";
	}

	// *** load same frame scene as the two different data structures
	ITMVoxelVolume<ITMVoxel, PlainVoxelArray>* source_volume_PVA;
	loadVolume(&source_volume_PVA, source_path_PVA, MEMORYDEVICE_CUDA, InitParams<PlainVoxelArray>());
	ITMVoxelVolume<ITMVoxel, VoxelBlockHash>* source_volume_VBH_CPU;
	loadVolume(&source_volume_VBH_CPU, source_path_VBH, MEMORYDEVICE_CPU, InitParams<VoxelBlockHash>());
	ITMVoxelVolume<ITMVoxel, VoxelBlockHash>* source_volume_VBH;
	loadVolume(&source_volume_VBH, source_path_VBH, MEMORYDEVICE_CUDA, InitParams<VoxelBlockHash>());

	// *** initialize target scenes
	ITMVoxelVolume<ITMVoxel, PlainVoxelArray>* target_PVA;
	ITMVoxelVolume<ITMVoxel, VoxelBlockHash>* target_VBH_CPU;
	ITMVoxelVolume<ITMVoxel, VoxelBlockHash>* target_VBH;
	initializeVolume(&target_PVA, InitParams<PlainVoxelArray>(), MEMORYDEVICE_CUDA);
	initializeVolume(&target_VBH_CPU, InitParams<VoxelBlockHash>(), MEMORYDEVICE_CPU);
	initializeVolume(&target_VBH, InitParams<VoxelBlockHash>(), MEMORYDEVICE_CUDA);

	// *** perform the warping
	WarpingEngine_CUDA_PVA warpingEngine_PVA;
	WarpingEngine_CPU_VBH  warpingEngine_VBH_CPU;
	WarpingEngine_CUDA_VBH warpingEngine_VBH;


	warpingEngine_PVA.WarpVolume_WarpUpdates(warps_PVA, source_volume_PVA, target_PVA);
	warpingEngine_VBH_CPU.WarpVolume_WarpUpdates(warps_VBH_CPU, source_volume_VBH_CPU, target_VBH_CPU);
	warpingEngine_VBH.WarpVolume_WarpUpdates(warps_VBH, source_volume_VBH, target_VBH);

	ITMVoxelVolume<ITMVoxel, VoxelBlockHash> target_VBH_copy(*target_VBH, MEMORYDEVICE_CPU);

	// *** test content
	float absoluteTolerance = 1e-6;


	BOOST_REQUIRE(
			contentAlmostEqual_CPU_Verbose(&target_VBH_copy, target_VBH_CPU, absoluteTolerance));
	BOOST_REQUIRE(
			contentForFlagsAlmostEqual_CUDA_Verbose(target_PVA, target_VBH, VOXEL_NONTRUNCATED, absoluteTolerance));


	delete warps_PVA;
	delete warps_VBH_CPU;
	delete warps_VBH;
	delete source_volume_PVA;
	delete source_volume_VBH_CPU;
	delete source_volume_VBH;
	delete target_PVA;
	delete target_VBH_CPU;
	delete target_VBH;
}

BOOST_FIXTURE_TEST_CASE(Test_WarpScene_CUDA_PVA, Frame16And17Fixture) {
	ITMVoxelVolume<ITMWarp, PlainVoxelArray>* warps;
	loadVolume(&warps, "TestData/snoopy_result_fr16-17_partial_PVA/warp_field_0_complete_",
	           MEMORYDEVICE_CUDA, InitParams<PlainVoxelArray>());
	ITMVoxelVolume<ITMVoxel, PlainVoxelArray>* live_volume;
	loadVolume(&live_volume, "TestData/snoopy_result_fr16-17_partial_PVA/snoopy_partial_frame_17_",
	           MEMORYDEVICE_CUDA, InitParams<PlainVoxelArray>());
	auto warped_live_volume = new ITMVoxelVolume<ITMVoxel, PlainVoxelArray>(
			&configuration::get().general_voxel_volume_parameters, false, MEMORYDEVICE_CUDA, InitParams<PlainVoxelArray>());
	warped_live_volume->Reset();

	WarpingEngine_CUDA_PVA warpingEngine;

	warpingEngine.WarpVolume_WarpUpdates(warps, live_volume, warped_live_volume);
	//warped_live_volume->SaveToDirectory("../../Tests/TestData/snoopy_result_fr16-17_partial_PVA/warped_live_");

	ITMVoxelVolume<ITMVoxel, PlainVoxelArray>* warped_live_volume_gt;
	loadVolume(&warped_live_volume_gt, "TestData/snoopy_result_fr16-17_partial_PVA/warped_live_",
	           MEMORYDEVICE_CUDA, InitParams<PlainVoxelArray>());

	float absoluteTolerance = 1e-5;
	BOOST_REQUIRE(contentAlmostEqual_CUDA(warped_live_volume, warped_live_volume_gt, absoluteTolerance));

	delete warps;
	delete live_volume;
	delete warped_live_volume;
}

BOOST_FIXTURE_TEST_CASE(Test_WarpScene_CUDA_VBH, Frame16And17Fixture) {

	ITMVoxelVolume<ITMWarp, VoxelBlockHash>* warps;
	loadVolume(&warps, "TestData/snoopy_result_fr16-17_partial_VBH/warp_field_0_complete_",
	           MEMORYDEVICE_CUDA, InitParams<VoxelBlockHash>());
	ITMVoxelVolume<ITMVoxel, VoxelBlockHash>* live_volume;
	loadVolume(&live_volume, "TestData/snoopy_result_fr16-17_partial_VBH/snoopy_partial_frame_17_",
	           MEMORYDEVICE_CUDA, InitParams<VoxelBlockHash>());
	auto warped_live_volume = new ITMVoxelVolume<ITMVoxel, VoxelBlockHash>(
			&configuration::get().general_voxel_volume_parameters, false, MEMORYDEVICE_CUDA, InitParams<VoxelBlockHash>());
	warped_live_volume->Reset();

	WarpingEngine_CUDA_VBH warpingEngine;

	warpingEngine.WarpVolume_WarpUpdates(warps, live_volume, warped_live_volume);
	//warped_live_volume->SaveToDirectory("../../Tests/TestData/snoopy_result_fr16-17_partial_VBH/warped_live_");

	ITMVoxelVolume<ITMVoxel, VoxelBlockHash>* warped_live_volume_gt;
	loadVolume(&warped_live_volume_gt, "TestData/snoopy_result_fr16-17_partial_VBH/warped_live_",
	           MEMORYDEVICE_CUDA, InitParams<VoxelBlockHash>());

	float absoluteTolerance = 1e-5;
	BOOST_REQUIRE(contentAlmostEqual_CUDA(warped_live_volume, warped_live_volume_gt, absoluteTolerance));

	delete warps;
	delete live_volume;
	delete warped_live_volume;
}


#endif