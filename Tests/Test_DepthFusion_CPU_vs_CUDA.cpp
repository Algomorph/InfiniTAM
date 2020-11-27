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
#define BOOST_TEST_MODULE DepthFusion_CPU_to_CUDA
#ifndef WIN32
#define BOOST_TEST_DYN_LINK
#endif

// *** boost ***
#include <boost/test/unit_test.hpp>

// *** ITMLib ***
#include "TestUtilities/TestDataUtilities.h"
#include "../ITMLib/Engines/DepthFusion/DepthFusionEngine.h"
#include "../ITMLib/Engines/DepthFusion/DepthFusionEngineFactory.h"
#include "../ITMLib/Engines/Indexing/Interface/IndexingEngine.h"
#include "../ITMLib/Utils/Analytics/AlmostEqual.h"
//(CPU)
#include "../ITMLib/Engines/Indexing/VBH/CPU/IndexingEngine_VoxelBlockHash_CPU.h"
//(CUDA)
#ifndef COMPILE_WITHOUT_CUDA
#include "../ITMLib/Engines/Rendering/Interface/RenderingEngineInterface.h"
#include "../ITMLib/Engines/Rendering/RenderingEngineFactory.h"
#endif

// *** test_utils ***
#include "TestUtilities/TestUtilities.h"


using namespace ITMLib;
using namespace test;

BOOST_AUTO_TEST_CASE(Test_SceneConstruct17_VBH_CPU_CUDA_NearSurface){
	
		
	View* view_17_CPU = nullptr;
	UpdateView(&view_17_CPU,
	           std::string(test::snoopy::snoopy_frame_17_depth_path),
	           std::string(test::snoopy::snoopy_frame_17_color_path),
	           std::string(test::snoopy::snoopy_frame_17_mask_path),
	           std::string(test::snoopy::snoopy_calibration_path),
	           MEMORYDEVICE_CPU);

	View* view_17_CUDA = nullptr;
	UpdateView(&view_17_CUDA,
	           std::string(test::snoopy::snoopy_frame_17_depth_path),
	           std::string(test::snoopy::snoopy_frame_17_color_path),
	           std::string(test::snoopy::snoopy_frame_17_mask_path),
	           std::string(test::snoopy::snoopy_calibration_path),
	           MEMORYDEVICE_CUDA);


// *** initialize volumes ***
	// CPU
	VoxelVolume<TSDFVoxel, VoxelBlockHash> volume_VBH_17_CPU(MEMORYDEVICE_CPU,
	                                                         test::snoopy::InitializationParameters_Fr16andFr17<VoxelBlockHash>());
	volume_VBH_17_CPU.Reset();
	// CUDA
	VoxelVolume<TSDFVoxel, VoxelBlockHash> volume_VBH_17_CUDA(MEMORYDEVICE_CUDA,
	                                                          test::snoopy::InitializationParameters_Fr16andFr17<VoxelBlockHash>());
	volume_VBH_17_CUDA.Reset();
	// comparison volume
	VoxelVolume<TSDFVoxel, VoxelBlockHash> volume_CUDA_to_CPU(MEMORYDEVICE_CPU,
	                                                          test::snoopy::InitializationParameters_Fr16andFr17<VoxelBlockHash>());
	volume_CUDA_to_CPU.Reset();

	CameraTrackingState tracking_state_CPU(view_17_CPU->depth.dimensions, MEMORYDEVICE_CPU);
	CameraTrackingState tracking_state_CUDA(view_17_CUDA->depth.dimensions, MEMORYDEVICE_CUDA);
// *** allocate hash blocks ***
	// CPU
	IndexingEngine<TSDFVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>& indexer_CPU =
			IndexingEngine<TSDFVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>::Instance();
	indexer_CPU.AllocateNearSurface(&volume_VBH_17_CPU, view_17_CPU);

	// CUDA
	IndexingEngine<TSDFVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA>& indexer_CUDA =
			IndexingEngine<TSDFVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA>::Instance();
	indexer_CUDA.AllocateNearSurface(&volume_VBH_17_CUDA, view_17_CUDA);


// *** compare allocation consistency before depth integration ***
	volume_CUDA_to_CPU.SetFrom(volume_VBH_17_CUDA);
	float absoluteTolerance = 1e-7;
	BOOST_REQUIRE(contentAlmostEqual_CPU_Verbose(&volume_CUDA_to_CPU, &volume_VBH_17_CPU, absoluteTolerance));

// *** integrate depth ***
	// CPU
	DepthFusionEngineInterface<TSDFVoxel, WarpVoxel, VoxelBlockHash>* reconstructionEngine_VBH_CPU =
			DepthFusionEngineFactory::Build<TSDFVoxel, WarpVoxel, VoxelBlockHash>(
					MEMORYDEVICE_CPU);
	reconstructionEngine_VBH_CPU->IntegrateDepthImageIntoTsdfVolume(&volume_VBH_17_CPU, view_17_CPU);
	// CUDA
	DepthFusionEngineInterface<TSDFVoxel, WarpVoxel, VoxelBlockHash>* reconstructionEngine_VBH_CUDA =
			DepthFusionEngineFactory::Build<TSDFVoxel, WarpVoxel, VoxelBlockHash>(
					MEMORYDEVICE_CUDA);
	reconstructionEngine_VBH_CUDA->IntegrateDepthImageIntoTsdfVolume(&volume_VBH_17_CUDA, view_17_CUDA);

// *** compare after depth integration ***
	volume_CUDA_to_CPU.SetFrom(volume_VBH_17_CUDA);
	BOOST_REQUIRE(contentAlmostEqual_CPU_Verbose(&volume_CUDA_to_CPU, &volume_VBH_17_CPU, absoluteTolerance));

	delete reconstructionEngine_VBH_CUDA;
	delete reconstructionEngine_VBH_CPU;
	delete view_17_CUDA;
	delete view_17_CPU;
}


BOOST_AUTO_TEST_CASE(Test_SceneConstruct17_VBH_CPU_CUDA_SurfaceSpan){

	View* view_16_CPU = nullptr;
	UpdateView(&view_16_CPU,
	           std::string(test::snoopy::snoopy_frame_16_depth_path),
	           std::string(test::snoopy::snoopy_frame_16_color_path),
	           std::string(test::snoopy::snoopy_frame_16_mask_path),
	           std::string(test::snoopy::snoopy_calibration_path),
	           MEMORYDEVICE_CPU);

	View* view_16_CUDA = nullptr;
	UpdateView(&view_16_CUDA,
	           std::string(test::snoopy::snoopy_frame_16_depth_path),
	           std::string(test::snoopy::snoopy_frame_16_color_path),
	           std::string(test::snoopy::snoopy_frame_16_mask_path),
	           std::string(test::snoopy::snoopy_calibration_path),
	           MEMORYDEVICE_CUDA);
	
	//*** prep engines and tracking states ***
	// CPU 
	CameraTrackingState tracking_state_CPU(view_16_CPU->depth.dimensions, MEMORYDEVICE_CPU);
	IndexingEngine<TSDFVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>& indexer_CPU =
			IndexingEngine<TSDFVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>::Instance();
	DepthFusionEngineInterface<TSDFVoxel, WarpVoxel, VoxelBlockHash>* depth_fusion_engine_VBH_CPU =
			DepthFusionEngineFactory::Build<TSDFVoxel, WarpVoxel, VoxelBlockHash>(
					MEMORYDEVICE_CPU);
	// CUDA
	CameraTrackingState tracking_state_CUDA(view_16_CUDA->depth.dimensions, MEMORYDEVICE_CUDA);
	IndexingEngine<TSDFVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA>& indexer_CUDA =
			IndexingEngine<TSDFVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA>::Instance();
	DepthFusionEngineInterface<TSDFVoxel, WarpVoxel, VoxelBlockHash>* depth_fusion_engine_VBH_CUDA =
			DepthFusionEngineFactory::Build<TSDFVoxel, WarpVoxel, VoxelBlockHash>(
					MEMORYDEVICE_CUDA);
	// ** generate point cloud for previous frame **
	{
		// ** scenes & render states
		// CPU 
		RenderState render_state_CPU(view_16_CPU->depth.dimensions, configuration::Get().general_voxel_volume_parameters.near_clipping_distance,
		                             configuration::Get().general_voxel_volume_parameters.far_clipping_distance, MEMORYDEVICE_CPU);
		VoxelVolume<TSDFVoxel, VoxelBlockHash> volume_VBH_16_CPU(MEMORYDEVICE_CPU,
		                                                         test::snoopy::InitializationParameters_Fr16andFr17<VoxelBlockHash>());
		volume_VBH_16_CPU.Reset();
		// CUDA
		RenderState render_state_CUDA(view_16_CUDA->depth.dimensions, configuration::Get().general_voxel_volume_parameters.near_clipping_distance,
		                              configuration::Get().general_voxel_volume_parameters.far_clipping_distance, MEMORYDEVICE_CUDA);
		VoxelVolume<TSDFVoxel, VoxelBlockHash> volume_VBH_16_CUDA(MEMORYDEVICE_CUDA,
		                                                          test::snoopy::InitializationParameters_Fr16andFr17<VoxelBlockHash>());
		volume_VBH_16_CUDA.Reset();

		indexer_CPU.AllocateNearSurface(&volume_VBH_16_CPU, view_16_CPU);
		indexer_CUDA.AllocateNearSurface(&volume_VBH_16_CUDA, view_16_CUDA);
		
		depth_fusion_engine_VBH_CPU->IntegrateDepthImageIntoTsdfVolume(&volume_VBH_16_CPU, view_16_CPU);
		depth_fusion_engine_VBH_CUDA->IntegrateDepthImageIntoTsdfVolume(&volume_VBH_16_CUDA, view_16_CUDA);
		
		RenderingEngineBase<TSDFVoxel, VoxelBlockHash>* visualization_engine_CPU =
				RenderingEngineFactory::Build<TSDFVoxel, VoxelBlockHash>(MEMORYDEVICE_CPU);
		RenderingEngineBase<TSDFVoxel, VoxelBlockHash>* visualization_engine_CUDA =
				RenderingEngineFactory::Build<TSDFVoxel, VoxelBlockHash>(MEMORYDEVICE_CUDA);
		
		visualization_engine_CPU->CreateICPMaps(&volume_VBH_16_CPU, view_16_CPU, &tracking_state_CPU, &render_state_CPU);
		visualization_engine_CUDA->CreateICPMaps(&volume_VBH_16_CUDA, view_16_CUDA, &tracking_state_CUDA, &render_state_CUDA);
		
		delete visualization_engine_CPU;
		delete visualization_engine_CUDA;
	}


	View* view_17_CPU = nullptr;
	UpdateView(&view_17_CPU,
	           std::string(test::snoopy::snoopy_frame_17_depth_path),
	           std::string(test::snoopy::snoopy_frame_17_color_path),
	           std::string(test::snoopy::snoopy_frame_17_mask_path),
	           std::string(test::snoopy::snoopy_calibration_path),
	           MEMORYDEVICE_CPU);

	View* view_17_CUDA = nullptr;
	UpdateView(&view_17_CUDA,
	           std::string(test::snoopy::snoopy_frame_17_depth_path),
	           std::string(test::snoopy::snoopy_frame_17_color_path),
	           std::string(test::snoopy::snoopy_frame_17_mask_path),
	           std::string(test::snoopy::snoopy_calibration_path),
	           MEMORYDEVICE_CUDA);

// *** initialize volumes ***
	// CPU
	VoxelVolume<TSDFVoxel, VoxelBlockHash> volume_VBH_17_CPU(MEMORYDEVICE_CPU,
	                                                         test::snoopy::InitializationParameters_Fr16andFr17<VoxelBlockHash>());
	volume_VBH_17_CPU.Reset();
	// CUDA
	VoxelVolume<TSDFVoxel, VoxelBlockHash> volume_VBH_17_CUDA(MEMORYDEVICE_CUDA,
	                                                          test::snoopy::InitializationParameters_Fr16andFr17<VoxelBlockHash>());
	volume_VBH_17_CUDA.Reset();
	// comparison volume
	VoxelVolume<TSDFVoxel, VoxelBlockHash> volume_CUDA_to_CPU(MEMORYDEVICE_CPU,
	                                                          test::snoopy::InitializationParameters_Fr16andFr17<VoxelBlockHash>());
	volume_CUDA_to_CPU.Reset();

// *** allocate hash blocks ***
	indexer_CPU.AllocateNearAndBetweenTwoSurfaces(&volume_VBH_17_CPU, view_17_CPU, &tracking_state_CPU);
	indexer_CUDA.AllocateNearAndBetweenTwoSurfaces(&volume_VBH_17_CUDA, view_17_CUDA, &tracking_state_CUDA);


// *** compare allocation consistency before depth integration ***
	volume_CUDA_to_CPU.SetFrom(volume_VBH_17_CUDA);
	float absolute_tolerance = 1e-7;
	BOOST_REQUIRE(contentAlmostEqual_CPU_Verbose(&volume_CUDA_to_CPU, &volume_VBH_17_CPU, absolute_tolerance));

// *** integrate depth ***
	depth_fusion_engine_VBH_CPU->IntegrateDepthImageIntoTsdfVolume(&volume_VBH_17_CPU, view_17_CPU);
	depth_fusion_engine_VBH_CUDA->IntegrateDepthImageIntoTsdfVolume(&volume_VBH_17_CUDA, view_17_CUDA);

// *** compare after depth integration ***
	volume_CUDA_to_CPU.SetFrom(volume_VBH_17_CUDA);
	BOOST_REQUIRE(contentAlmostEqual_CPU_Verbose(&volume_CUDA_to_CPU, &volume_VBH_17_CPU, absolute_tolerance));

	delete depth_fusion_engine_VBH_CUDA;
	delete depth_fusion_engine_VBH_CPU;
	delete view_17_CUDA;
	delete view_17_CPU;
	delete view_16_CUDA;
	delete view_16_CPU;
}