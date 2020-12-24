//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 12/24/20.
//  Copyright (c) 2020 Gregory Kramida
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
// === local ===
#include "GenerateSnoopyVolumes.h"
// === ORUtils ===
#include "../../ORUtils/FileUtils.h"
#include "../../ORUtils/MathTypePersistence/MathTypePersistence.h"
// === ITMLib ===
#include "../../ITMLib/GlobalTemplateDefines.h"
#include "../../ITMLib/Utils/Logging/Logging.h"
#include "../../ITMLib/Utils/Geometry/GeometryBooleanOperations.h"
#include "../../ITMLib/Objects/Volume/PlainVoxelArray.h"
#include "../../ITMLib/Objects/Volume/VoxelBlockHash.h"
#include "../../ITMLib/Objects/Volume/VoxelVolume.h"
#include "../../ITMLib/Engines/ViewBuilder/ViewBuilderFactory.h"
#include "../../ITMLib/Engines/Analytics/AnalyticsEngine.h"
#include "../../ITMLib/Engines/Indexing/IndexingEngineFactory.h"
#include "../../ITMLib/Engines/DepthFusion/DepthFusionEngineFactory.h"
//(CPU)
#include "../../ITMLib/Engines/Indexing/VBH/CPU/IndexingEngine_VoxelBlockHash_CPU.h"
//(CUDA)
#ifndef COMPILE_WITHOUT_CUDA
#include "../../ITMLib/Engines/Indexing/VBH/CUDA/IndexingEngine_VoxelBlockHash_CUDA.h"
#endif
// === Test Utilities ===
#include "../TestUtilities/TestUtilities.h"
#include "../TestUtilities/TestDataUtilities.h"

using namespace ITMLib;
using namespace test;



void ConstructSnoopyUnmaskedVolumes00() {
	LOG4CPLUS_INFO(log4cplus::Logger::getRoot(), "Constructing snoopy unmasked full volumes at frame 0...");

	VoxelVolume<TSDFVoxel, PlainVoxelArray>* volume_PVA_00;
	VoxelVolume<TSDFVoxel, VoxelBlockHash>* volume_VBH_00;

	BuildSdfVolumeFromImage_NearSurfaceAllocation(&volume_PVA_00,
	                                              test::snoopy::frame_00_depth_path.ToString(),
	                                              test::snoopy::frame_00_color_path.ToString(),
	                                              test::snoopy::calibration_path.ToString(),
	                                              MEMORYDEVICE_CPU,
	                                              test::snoopy::InitializationParameters_Fr00<PlainVoxelArray>());
	BuildSdfVolumeFromImage_NearSurfaceAllocation(&volume_VBH_00,
	                                              test::snoopy::frame_00_depth_path.ToString(),
	                                              test::snoopy::frame_00_color_path.ToString(),
	                                              test::snoopy::calibration_path.ToString(),
	                                              MEMORYDEVICE_CPU,
	                                              test::snoopy::InitializationParameters_Fr00<VoxelBlockHash>());
	test::ConstructGeneratedVolumeSubdirectoriesIfMissing();
	volume_PVA_00->SaveToDisk(test::snoopy::PartialVolume00Path<PlainVoxelArray>());
	volume_VBH_00->SaveToDisk(test::snoopy::PartialVolume00Path<VoxelBlockHash>());

	delete volume_PVA_00;
	delete volume_VBH_00;
}



void ConstructSnoopyMaskedVolumes16and17() {
	test::ConstructGeneratedVolumeSubdirectoriesIfMissing();

	VoxelVolume<TSDFVoxel, PlainVoxelArray>* volume_PVA_16;
	VoxelVolume<TSDFVoxel, PlainVoxelArray>* volume_PVA_17;
	VoxelVolume<TSDFVoxel, VoxelBlockHash>* volume_VBH_16;
	VoxelVolume<TSDFVoxel, VoxelBlockHash>* volume_VBH_17;

	LOG4CPLUS_INFO(log4cplus::Logger::getRoot(), "Constructing snoopy masked full volumes 16 & 17 PVA...");

	BuildSdfVolumeFromImage_SurfaceSpanAllocation(&volume_PVA_16,
	                                              &volume_PVA_17,
	                                              test::snoopy::frame_16_depth_path.ToString(),
	                                              test::snoopy::frame_16_color_path.ToString(),
	                                              test::snoopy::frame_16_mask_path.ToString(),
	                                              test::snoopy::frame_17_depth_path.ToString(),
	                                              test::snoopy::frame_17_color_path.ToString(),
	                                              test::snoopy::frame_17_mask_path.ToString(),
	                                              test::snoopy::calibration_path.ToString(),
	                                              MEMORYDEVICE_CPU);

	LOG4CPLUS_INFO(log4cplus::Logger::getRoot(), "Constructing snoopy masked full volumes 16 & 17 VBH...");

	BuildSdfVolumeFromImage_SurfaceSpanAllocation(&volume_VBH_16,
	                                              &volume_VBH_17,
	                                              test::snoopy::frame_16_depth_path.ToString(),
	                                              test::snoopy::frame_16_color_path.ToString(),
	                                              test::snoopy::frame_16_mask_path.ToString(),
	                                              test::snoopy::frame_17_depth_path.ToString(),
	                                              test::snoopy::frame_17_color_path.ToString(),
	                                              test::snoopy::frame_17_mask_path.ToString(),
	                                              test::snoopy::calibration_path.ToString(),
	                                              MEMORYDEVICE_CPU);

	LOG4CPLUS_INFO(log4cplus::Logger::getRoot(), "Saving snoopy masked full volume 16 PVA...");
	volume_PVA_16->SaveToDisk(test::snoopy::FullVolume16Path<PlainVoxelArray>());
	LOG4CPLUS_INFO(log4cplus::Logger::getRoot(), "Saving snoopy masked full volume 16 VBH...");
	volume_VBH_16->SaveToDisk(test::snoopy::FullVolume16Path<VoxelBlockHash>());
	LOG4CPLUS_INFO(log4cplus::Logger::getRoot(), "Saving snoopy masked full volume 17 PVA...");
	volume_PVA_17->SaveToDisk(test::snoopy::FullVolume17Path<PlainVoxelArray>());
	LOG4CPLUS_INFO(log4cplus::Logger::getRoot(), "Saving snoopy masked full volume 17 VBH...");
	volume_VBH_17->SaveToDisk(test::snoopy::FullVolume17Path<VoxelBlockHash>());

	LOG4CPLUS_INFO(log4cplus::Logger::getRoot(), "Computing voxel bounds for partial volumes...");
	Extent3Di bounds_16 = Analytics_CPU_VBH_Voxel::Instance().ComputeVoxelBounds(volume_VBH_16);
	Extent3Di bounds_17 = Analytics_CPU_VBH_Voxel::Instance().ComputeVoxelBounds(volume_VBH_17);
	Extent3Di union_bounds = UnionExtent(bounds_16, bounds_17);

	test::ConstructGeneratedArraysDirectoryIfMissing();

	ORUtils::OStreamWrapper bounds_file(test::generated_arrays_directory.ToString() + "snoopy_16_and_17_partial_volume_bounds.dat", false);
	bounds_file << union_bounds;
	LOG4CPLUS_INFO(log4cplus::Logger::getRoot(), "Voxel bounds for partial volumes are: " << union_bounds);

	bounds_file.OStream().write(reinterpret_cast<const char*>(&union_bounds.min_x), sizeof(int));
	bounds_file.OStream().write(reinterpret_cast<const char*>(&union_bounds.min_y), sizeof(int));
	bounds_file.OStream().write(reinterpret_cast<const char*>(&union_bounds.min_z), sizeof(int));


	volume_PVA_16->Reset();
	volume_VBH_16->Reset();
	volume_PVA_17->Reset();
	volume_VBH_17->Reset();

	LOG4CPLUS_INFO(log4cplus::Logger::getRoot(), "Constructing snoopy masked partial volumes 16 & 17...");


	BuildSdfVolumeFromImage_SurfaceSpanAllocation(&volume_PVA_16,
	                                              &volume_PVA_17,
	                                              test::snoopy::frame_16_depth_path.ToString(),
	                                              test::snoopy::frame_16_color_path.ToString(),
	                                              test::snoopy::frame_16_mask_path.ToString(),
	                                              test::snoopy::frame_17_depth_path.ToString(),
	                                              test::snoopy::frame_17_color_path.ToString(),
	                                              test::snoopy::frame_17_mask_path.ToString(),
	                                              test::snoopy::calibration_path.ToString(),
	                                              MEMORYDEVICE_CPU,
	                                              test::snoopy::InitializationParameters_Fr16andFr17<PlainVoxelArray>());


	BuildSdfVolumeFromImage_SurfaceSpanAllocation(&volume_VBH_16,
	                                              &volume_VBH_17,
	                                              test::snoopy::frame_16_depth_path.ToString(),
	                                              test::snoopy::frame_16_color_path.ToString(),
	                                              test::snoopy::frame_16_mask_path.ToString(),
	                                              test::snoopy::frame_17_depth_path.ToString(),
	                                              test::snoopy::frame_17_color_path.ToString(),
	                                              test::snoopy::frame_17_mask_path.ToString(),
	                                              test::snoopy::calibration_path.ToString(),
	                                              MEMORYDEVICE_CPU,
	                                              test::snoopy::InitializationParameters_Fr16andFr17<VoxelBlockHash>());

	LOG4CPLUS_INFO(log4cplus::Logger::getRoot(), "Saving snoopy masked partial volumes 16 & 17...");

	test::ConstructGeneratedVolumeSubdirectoriesIfMissing();

	volume_PVA_16->SaveToDisk(test::snoopy::PartialVolume16Path<PlainVoxelArray>());
	volume_VBH_16->SaveToDisk(test::snoopy::PartialVolume16Path<VoxelBlockHash>());
	volume_PVA_17->SaveToDisk(test::snoopy::PartialVolume17Path<PlainVoxelArray>());
	volume_VBH_17->SaveToDisk(test::snoopy::PartialVolume17Path<VoxelBlockHash>());


	delete volume_PVA_16;
	delete volume_VBH_16;
	delete volume_PVA_17;
	delete volume_VBH_17;
}

//TODO: not sure if this is necessary. Might have been started in efforts to revise some older volume generation test
// functions (Test_DepthFusion ?).
// Either continue and finish or remove.
void ConstructStripesTestVolumes() {
	LOG4CPLUS_INFO(log4cplus::Logger::getRoot(), "Constructing stripe test volumes...");
	// region ================================= CONSTRUCT VIEW =========================================================

	RGBD_CalibrationInformation calibration_data;
	readRGBDCalib(test::snoopy::calibration_path.Get(), calibration_data);

	auto view_builder = ViewBuilderFactory::Build(calibration_data, MEMORYDEVICE_CPU);
	Vector2i image_size(640, 480);
	View* view = nullptr;

	UChar4Image rgb(true, false);
	ShortImage depth(true, false);
	ReadImageFromFile(rgb, STATIC_TEST_DATA_PREFIX "TestData/frames/stripes_color.png");
	ReadImageFromFile(depth, STATIC_TEST_DATA_PREFIX "TestData/frames/stripes_depth.png");

	view_builder->UpdateView(&view, &rgb, &depth, false, false, false, true);

	// endregion =======================================================================================================

	VoxelVolume<TSDFVoxel, VoxelBlockHash> volume4(MEMORYDEVICE_CPU, {0x800, 0x20000});
	volume4.Reset();
	IndexingEngineInterface<TSDFVoxel, VoxelBlockHash>& indexer_VBH
			= IndexingEngineFactory::GetDefault<TSDFVoxel, VoxelBlockHash>(MEMORYDEVICE_CPU);
	CameraTrackingState tracking_state(image_size, MEMORYDEVICE_CPU);
	indexer_VBH.AllocateNearAndBetweenTwoSurfaces(&volume4, view, &tracking_state);
	auto depth_fusion_engine_VBH = DepthFusionEngineFactory::Build<TSDFVoxel, VoxelBlockHash>(MEMORYDEVICE_CPU);
	depth_fusion_engine_VBH->IntegrateDepthImageIntoTsdfVolume(&volume4, view, &tracking_state);
	std::string path = STATIC_TEST_DATA_PREFIX "TestData/volumes/VBH/stripes.dat";

	test::ConstructGeneratedVolumeSubdirectoriesIfMissing();
	volume4.SaveToDisk(path);

	delete depth_fusion_engine_VBH;
	delete view;
	delete view_builder;
}