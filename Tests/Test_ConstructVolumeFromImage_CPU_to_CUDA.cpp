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
#include "../ITMLib/GlobalTemplateDefines.h"
#include "../ITMLib/Engines/DepthFusion/DepthFusionEngine.h"
#include "../ITMLib/Engines/DepthFusion/DepthFusionEngineFactory.h"
#include "../ITMLib/Engines/EditAndCopy/EditAndCopyEngineFactory.h"
#include "../ITMLib/Engines/Indexing/Interface/IndexingEngine.h"
#include "../ITMLib/Utils/Analytics/AlmostEqual.h"
#include "TestUtils.h"
//(cpu)
#include "../ITMLib/Utils/Analytics/SceneStatisticsCalculator/CPU/SceneStatisticsCalculator_CPU.h"
#include "../ITMLib/Engines/Indexing/VBH/CPU/IndexingEngine_CPU_VoxelBlockHash.h"
//(CUDA)
#ifndef COMPILE_WITHOUT_CUDA
#include "../ITMLib/Utils/Analytics/SceneStatisticsCalculator/CUDA/SceneStatisticsCalculator_CUDA.h"
#include "../ITMLib/Engines/Indexing/VBH/CUDA/IndexingEngine_CUDA_VoxelBlockHash.h"
#endif


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
	VoxelVolume<TSDFVoxel, VoxelBlockHash> volume_VBH_17_CPU(MEMORYDEVICE_CPU, InitParams<VoxelBlockHash>());
	volume_VBH_17_CPU.Reset();
	VoxelVolume<TSDFVoxel, VoxelBlockHash> volume_VBH_17_CPU_depth_allocation(MEMORYDEVICE_CPU,
	                                                                          InitParams<VoxelBlockHash>());
	volume_VBH_17_CPU_depth_allocation.Reset();
	// CUDA
	VoxelVolume<TSDFVoxel, VoxelBlockHash> volume_VBH_17_CUDA(MEMORYDEVICE_CUDA, InitParams<VoxelBlockHash>());
	volume_VBH_17_CUDA.Reset();
	VoxelVolume<TSDFVoxel, VoxelBlockHash> volume_VBH_17_CUDA_depth_allocation(MEMORYDEVICE_CUDA,
	                                                                           InitParams<VoxelBlockHash>());
	volume_VBH_17_CUDA_depth_allocation.Reset();
	// comparison volume
	VoxelVolume<TSDFVoxel, VoxelBlockHash> volume_CUDA_to_CPU(MEMORYDEVICE_CPU, InitParams<VoxelBlockHash>());
	volume_CUDA_to_CPU.Reset();
	
// *** allocate hash blocks ***
	// CPU
	IndexingEngine<TSDFVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>& indexer_CPU =
			IndexingEngine<TSDFVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>::Instance();
	indexer_CPU.AllocateNearSurface(&volume_VBH_17_CPU_depth_allocation, view_CPU);
	indexer_CPU.AllocateUsingOtherVolumeAndSetVisibilityExpanded(&volume_VBH_17_CPU, &volume_VBH_17_CPU_depth_allocation, view_CPU);
	// CUDA
	IndexingEngine<TSDFVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA>& indexer_CUDA =
			IndexingEngine<TSDFVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA>::Instance();
	indexer_CUDA.AllocateNearSurface(&volume_VBH_17_CUDA_depth_allocation, view_CUDA);
	indexer_CUDA.AllocateUsingOtherVolumeAndSetVisibilityExpanded(&volume_VBH_17_CUDA, &volume_VBH_17_CUDA_depth_allocation, view_CUDA);


// *** compare before depth integration ***
	volume_CUDA_to_CPU.SetFrom(volume_VBH_17_CUDA);
	float absoluteTolerance = 1e-7;
	BOOST_REQUIRE(contentAlmostEqual_CPU_Verbose(&volume_CUDA_to_CPU, &volume_VBH_17_CPU, absoluteTolerance));

// *** integrate depth ***
	// CPU
	DepthFusionEngineInterface<TSDFVoxel, WarpVoxel, VoxelBlockHash>* reconstructionEngine_VBH_CPU =
			DepthFusionEngineFactory::Build<TSDFVoxel, WarpVoxel, VoxelBlockHash>(
					MEMORYDEVICE_CPU);
	reconstructionEngine_VBH_CPU->IntegrateDepthImageIntoTsdfVolume(&volume_VBH_17_CPU, view_CPU);
	reconstructionEngine_VBH_CPU->IntegrateDepthImageIntoTsdfVolume(&volume_VBH_17_CPU_depth_allocation, view_CPU);
	// CUDA
	DepthFusionEngineInterface<TSDFVoxel, WarpVoxel, VoxelBlockHash>* reconstructionEngine_VBH_CUDA =
			DepthFusionEngineFactory::Build<TSDFVoxel, WarpVoxel, VoxelBlockHash>(
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