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
#include "../ITMLib/Engines/Reconstruction/CPU/ITMDynamicSceneReconstructionEngine_CPU.h"
#include "../ITMLib/Utils/Analytics/VoxelVolumeComparison/ITMVoxelVolumeComparison_CPU.h"
#ifndef COMPILE_WITHOUT_CUDA
#include "../ITMLib/Engines/Reconstruction/CUDA/ITMDynamicSceneReconstructionEngine_CUDA.h"
#include "../ITMLib/Utils/Analytics/VoxelVolumeComparison/ITMVoxelVolumeComparison_CUDA.h"
#endif

#include "TestUtils.h"

using namespace ITMLib;

typedef ITMDynamicSceneReconstructionEngine_CPU<ITMVoxel, ITMWarp, ITMPlainVoxelArray> RecoEngine_CPU_PVA;
typedef ITMDynamicSceneReconstructionEngine_CPU<ITMVoxel, ITMWarp, ITMVoxelBlockHash> RecoEngine_CPU_VBH;

BOOST_AUTO_TEST_CASE(Test_WarpScene_CPU_PVA) {
	const Configuration& settings = Configuration::Instance();
	ITMVoxelVolume<ITMWarp, ITMPlainVoxelArray>* warps;
	loadSdfVolume(&warps, "TestData/snoopy_result_fr16-17_partial_PVA/warp_field_0_complete_",
	              MEMORYDEVICE_CPU);
	ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>* live_volume;
	loadSdfVolume(&live_volume, "TestData/snoopy_result_fr16-17_partial_PVA/live",
	              MEMORYDEVICE_CPU);
	auto warped_live_volume = new ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>(
			&settings.sceneParams, false, MEMORYDEVICE_CPU,
			{live_volume->index.GetVolumeSize(), live_volume->index.GetVolumeOffset()});
	ManipulationEngine_CPU_PVA_Voxel::Inst().ResetScene(warped_live_volume);

	RecoEngine_CPU_PVA recoEngine;

	recoEngine.WarpScene_FlowWarps(warps, live_volume, warped_live_volume);
	//warped_live_volume->SaveToDirectory("../../Tests/TestData/snoopy_result_fr16-17_partial_PVA/warped_live_");

	ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>* warped_live_volume_gt;
	loadSdfVolume(&warped_live_volume_gt, "../../Tests/TestData/snoopy_result_fr16-17_partial_PVA/warped_live_",
	              MEMORYDEVICE_CPU);

	float absoluteTolerance = 1e-7;
	BOOST_REQUIRE(!contentAlmostEqual_CPU(warped_live_volume, live_volume, absoluteTolerance));
	BOOST_REQUIRE(contentAlmostEqual_CPU(warped_live_volume, warped_live_volume_gt, absoluteTolerance));

	delete warps;
	delete live_volume;
	delete warped_live_volume;
}

BOOST_AUTO_TEST_CASE(Test_WarpScene_CPU_VBH) {
	const Configuration& settings = Configuration::Instance();
	ITMVoxelVolume<ITMWarp, ITMVoxelBlockHash>* warps;
	loadSdfVolume(&warps, "TestData/snoopy_result_fr16-17_partial_VBH/warp_field_0_complete_",
	              MEMORYDEVICE_CPU);
	ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash>* live_volume;
	loadSdfVolume(&live_volume, "TestData/snoopy_result_fr16-17_partial_VBH/live",
	              MEMORYDEVICE_CPU);
	auto warped_live_volume = new ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash>(
			&settings.sceneParams, false, MEMORYDEVICE_CPU,
			{live_volume->index.GetAllocatedBlockCount(), live_volume->index.GetExcessListSize()});
	ManipulationEngine_CPU_VBH_Voxel::Inst().ResetScene(warped_live_volume);

	RecoEngine_CPU_VBH recoEngine;

	recoEngine.WarpScene_FlowWarps(warps, live_volume, warped_live_volume);
	//warped_live_volume->SaveToDirectory("../../Tests/TestData/snoopy_result_fr16-17_partial_VBH/warped_live_");

	ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash>* warped_live_volume_gt;
	loadSdfVolume(&warped_live_volume_gt, "TestData/snoopy_result_fr16-17_partial_VBH/warped_live_",
	              MEMORYDEVICE_CPU);


	float absoluteTolerance = 1e-7;
	BOOST_REQUIRE(!contentAlmostEqual_CPU(warped_live_volume, live_volume, absoluteTolerance));
	BOOST_REQUIRE(contentAlmostEqual_CPU(warped_live_volume, warped_live_volume_gt, absoluteTolerance));

	delete warps;
	delete live_volume;
	delete warped_live_volume;
}

#ifndef COMPILE_WITHOUT_CUDA
typedef ITMDynamicSceneReconstructionEngine_CUDA<ITMVoxel, ITMWarp, ITMPlainVoxelArray> RecoEngine_CUDA_PVA;
typedef ITMDynamicSceneReconstructionEngine_CUDA<ITMVoxel, ITMWarp, ITMVoxelBlockHash> RecoEngine_CUDA_VBH;

BOOST_AUTO_TEST_CASE(Test_WarpScene_CUDA_PVA) {
	const Configuration& settings = Configuration::Instance();
	ITMVoxelVolume<ITMWarp, ITMPlainVoxelArray>* warps;
	loadSdfVolume(&warps, "TestData/snoopy_result_fr16-17_partial_PVA/warp_field_0_complete_",
	              MEMORYDEVICE_CUDA);
	ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>* live_volume;
	loadSdfVolume(&live_volume, "TestData/snoopy_result_fr16-17_partial_PVA/live",
	              MEMORYDEVICE_CUDA);
	auto warped_live_volume = new ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>(
			&settings.sceneParams, false, MEMORYDEVICE_CUDA,
			{live_volume->index.GetVolumeSize(), live_volume->index.GetVolumeOffset()});
	ManipulationEngine_CUDA_PVA_Voxel::Inst().ResetScene(warped_live_volume);

	RecoEngine_CUDA_PVA recoEngine;

	recoEngine.WarpScene_FlowWarps(warps, live_volume, warped_live_volume);
	//warped_live_volume->SaveToDirectory("../../Tests/TestData/snoopy_result_fr16-17_partial_PVA/warped_live_");

	ITMVoxelVolume<ITMVoxel, ITMPlainVoxelArray>* warped_live_volume_gt;
	loadSdfVolume(&warped_live_volume_gt, "TestData/snoopy_result_fr16-17_partial_PVA/warped_live_",
	              MEMORYDEVICE_CUDA);

	float absoluteTolerance = 1e-7;
	BOOST_REQUIRE(contentAlmostEqual_CUDA(warped_live_volume, warped_live_volume_gt, absoluteTolerance));

	delete warps;
	delete live_volume;
	delete warped_live_volume;
}

BOOST_AUTO_TEST_CASE(Test_WarpScene_CUDA_VBH) {
	const Configuration& settings = Configuration::Instance();
	ITMVoxelVolume<ITMWarp, ITMVoxelBlockHash>* warps;
	loadSdfVolume(&warps, "TestData/snoopy_result_fr16-17_partial_VBH/warp_field_0_complete_",
	              MEMORYDEVICE_CUDA);
	ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash>* live_volume;
	loadSdfVolume(&live_volume, "TestData/snoopy_result_fr16-17_partial_VBH/live",
	              MEMORYDEVICE_CUDA);
	auto warped_live_volume = new ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash>(
			&settings.sceneParams, false, MEMORYDEVICE_CUDA,
			{live_volume->index.GetAllocatedBlockCount(), live_volume->index.GetExcessListSize()});
	ManipulationEngine_CUDA_VBH_Voxel::Inst().ResetScene(warped_live_volume);

	RecoEngine_CUDA_VBH recoEngine;

	recoEngine.WarpScene_FlowWarps(warps, live_volume, warped_live_volume);
	//warped_live_volume->SaveToDirectory("../../Tests/TestData/snoopy_result_fr16-17_partial_VBH/warped_live_");

	ITMVoxelVolume<ITMVoxel, ITMVoxelBlockHash>* warped_live_volume_gt;
	loadSdfVolume(&warped_live_volume_gt, "TestData/snoopy_result_fr16-17_partial_VBH/warped_live_",
	              MEMORYDEVICE_CUDA);

	float absoluteTolerance = 1e-7;
	BOOST_REQUIRE(contentAlmostEqual_CUDA(warped_live_volume, warped_live_volume_gt, absoluteTolerance));

	delete warps;
	delete live_volume;
	delete warped_live_volume;
}
#endif