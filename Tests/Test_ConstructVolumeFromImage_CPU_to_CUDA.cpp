//  ================================================================
//  Created by Gregory Kramida on 12/20/19.
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
#define BOOST_TEST_MODULE depth_to_tsdf_CPU_vs_CUDA
#ifndef WIN32
#define BOOST_TEST_DYN_LINK
#endif

//boost
#include <boost/test/unit_test.hpp>

// *** ITMLib ***
#include "TestUtilsForSnoopyFrames16And17.h"
#include "../ITMLib/ITMLibDefines.h"
#include "../ITMLib/Engines/DepthFusion/DynamicSceneReconstructionEngine.h"
#include "../ITMLib/Engines/DepthFusion/DynamicSceneReconstructionEngineFactory.h"
#include "../ITMLib/Engines/EditAndCopy/EditAndCopyEngineFactory.h"
#include "../ITMLib/Utils/Analytics/ITMAlmostEqual.h"
#include "TestUtils.h"
//(cpu)
#include "../ITMLib/Utils/Analytics/SceneStatisticsCalculator/CPU/ITMSceneStatisticsCalculator_CPU.h"
#include "../ITMLib/Utils/Analytics/SceneStatisticsCalculator/CUDA/ITMSceneStatisticsCalculator_CUDA.h"
//(CUDA)

using namespace ITMLib;

BOOST_FIXTURE_TEST_CASE(Test_SceneConstruct17_VBH_Expnaded_CPU_CUDA, Frame16And17Fixture) {

	ITMView* view_CPU = nullptr;
	updateView(&view_CPU, "TestData/snoopy_depth_000017.png",
	           "TestData/snoopy_color_000017.png", "TestData/snoopy_omask_000017.png",
	           "TestData/snoopy_calib.txt", MEMORYDEVICE_CPU);

	ITMView* view_CUDA = nullptr;
	updateView(&view_CUDA, "TestData/snoopy_depth_000017.png",
	           "TestData/snoopy_color_000017.png", "TestData/snoopy_omask_000017.png",
	           "TestData/snoopy_calib.txt", MEMORYDEVICE_CUDA);


// *** initialize volumes ***
	// CPU
	ITMVoxelVolume<ITMVoxel, VoxelBlockHash> volume_VBH_17_CPU(MEMORYDEVICE_CPU, InitParams<VoxelBlockHash>());
	volume_VBH_17_CPU.Reset();
	ITMVoxelVolume<ITMVoxel, VoxelBlockHash> volume_VBH_17_CPU_depth_allocation(MEMORYDEVICE_CPU,
	                                                                               InitParams<VoxelBlockHash>());
	volume_VBH_17_CPU_depth_allocation.Reset();
	// CUDA
	ITMVoxelVolume<ITMVoxel, VoxelBlockHash> volume_VBH_17_CUDA(MEMORYDEVICE_CUDA, InitParams<VoxelBlockHash>());
	volume_VBH_17_CUDA.Reset();
	ITMVoxelVolume<ITMVoxel, VoxelBlockHash> volume_VBH_17_CUDA_depth_allocation(MEMORYDEVICE_CUDA,
	                                                                               InitParams<VoxelBlockHash>());
	volume_VBH_17_CUDA_depth_allocation.Reset();
	// comparison volume
	ITMVoxelVolume<ITMVoxel, VoxelBlockHash> volume_CUDA_to_CPU(MEMORYDEVICE_CPU, InitParams<VoxelBlockHash>());
	volume_CUDA_to_CPU.Reset();
	
// *** allocate hash blocks ***
	// CPU
	IndexingEngine<ITMVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>& indexer_CPU =
			IndexingEngine<ITMVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>::Instance();
	indexer_CPU.AllocateFromDepth(&volume_VBH_17_CPU_depth_allocation, view_CPU);
	indexer_CPU.AllocateUsingOtherVolumeAndSetVisibilityExpanded(&volume_VBH_17_CPU, &volume_VBH_17_CPU_depth_allocation, view_CPU);
	// CUDA
	IndexingEngine<ITMVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA>& indexer_CUDA =
			IndexingEngine<ITMVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA>::Instance();
	indexer_CUDA.AllocateFromDepth(&volume_VBH_17_CUDA_depth_allocation, view_CUDA);
	indexer_CUDA.AllocateUsingOtherVolumeAndSetVisibilityExpanded(&volume_VBH_17_CUDA, &volume_VBH_17_CUDA_depth_allocation, view_CUDA);


// *** compare before depth integration ***
	volume_CUDA_to_CPU.SetFrom(volume_VBH_17_CUDA);
	float absoluteTolerance = 1e-7;
	BOOST_REQUIRE(contentAlmostEqual_CPU_Verbose(&volume_CUDA_to_CPU, &volume_VBH_17_CPU, absoluteTolerance));

// *** integrate depth ***
	// CPU
	DynamicSceneReconstructionEngine<ITMVoxel, ITMWarp, VoxelBlockHash>* reconstructionEngine_VBH_CPU =
			DynamicSceneReconstructionEngineFactory::MakeSceneReconstructionEngine<ITMVoxel, ITMWarp, VoxelBlockHash>(
					MEMORYDEVICE_CPU);
	reconstructionEngine_VBH_CPU->IntegrateDepthImageIntoTsdfVolume(&volume_VBH_17_CPU, view_CPU);
	reconstructionEngine_VBH_CPU->IntegrateDepthImageIntoTsdfVolume(&volume_VBH_17_CPU_depth_allocation, view_CPU);
	// CUDA
	DynamicSceneReconstructionEngine<ITMVoxel, ITMWarp, VoxelBlockHash>* reconstructionEngine_VBH_CUDA =
			DynamicSceneReconstructionEngineFactory::MakeSceneReconstructionEngine<ITMVoxel, ITMWarp, VoxelBlockHash>(
					MEMORYDEVICE_CUDA);
	reconstructionEngine_VBH_CUDA->IntegrateDepthImageIntoTsdfVolume(&volume_VBH_17_CUDA, view_CUDA);
	reconstructionEngine_VBH_CUDA->IntegrateDepthImageIntoTsdfVolume(&volume_VBH_17_CUDA_depth_allocation, view_CUDA);

// *** compare after depth integration ***
	volume_CUDA_to_CPU.SetFrom(volume_VBH_17_CUDA);
	BOOST_REQUIRE(contentAlmostEqual_CPU_Verbose(&volume_CUDA_to_CPU, &volume_VBH_17_CPU, absoluteTolerance));

	delete reconstructionEngine_VBH_CUDA;
	delete reconstructionEngine_VBH_CPU;
	delete view_CUDA;
	delete view_CPU;
}