//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 12/30/20.
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
#include "ConstructTeddyVolumes.h"
// === Test Utilities ==
#include "../TestUtilities/TestUtilities.h"
#include "../TestUtilities/TestDataUtilities.h"
// === ITMLib ===
#include "../../ITMLib/Engines/Analytics/AnalyticsEngine.h"
#include "../../ITMLib/Engines/DepthFusion/DepthFusionEngineFactory.h"
#include "../../ITMLib/Engines/Indexing/IndexingEngineFactory.h"
#include "../../ITMLib/Utils/Logging/Logging.h"

// using namespace ITMLib;
using namespace test;

static inline
void ConstructTeddyVolumes115_Generic(const VoxelBlockHash::InitializationParameters& vbh_parameters,
                                      const PlainVoxelArray::InitializationParameters& pva_parameters,
                                      const std::string& vbh_path,
                                      const std::string& pva_path) {
	MemoryDeviceType TMemoryDeviceType = MEMORYDEVICE_CPU;

	View* view = nullptr;

	UpdateView(&view,
	           teddy::frame_115_depth_path.ToString(),
	           teddy::frame_115_color_path.ToString(),
	           teddy::calibration_path.ToString(),
	           TMemoryDeviceType);

	VoxelVolume<TSDFVoxel_f_rgb, VoxelBlockHash> teddy_volume_115_VBH(teddy::DefaultVolumeParameters(), false, TMemoryDeviceType,
	                                                                  vbh_parameters);
	teddy_volume_115_VBH.Reset();
	VoxelVolume<TSDFVoxel_f_rgb, PlainVoxelArray> teddy_volume_115_PVA(teddy::DefaultVolumeParameters(), false, TMemoryDeviceType,
	                                                                   pva_parameters);

	auto* indexing_engine_VBH = IndexingEngineFactory::Build<TSDFVoxel_f_rgb, VoxelBlockHash>(TMemoryDeviceType);
	auto* indexing_engine_PVA = IndexingEngineFactory::Build<TSDFVoxel_f_rgb, PlainVoxelArray>(TMemoryDeviceType);


	indexing_engine_VBH->AllocateNearSurface(&teddy_volume_115_VBH, view);
	indexing_engine_PVA->AllocateNearSurface(&teddy_volume_115_PVA, view);

	DepthFusionEngineInterface<TSDFVoxel_f_rgb, VoxelBlockHash>* depth_fusion_engine_VBH =
			DepthFusionEngineFactory::Build<TSDFVoxel_f_rgb, VoxelBlockHash>(TMemoryDeviceType);
	DepthFusionEngineInterface<TSDFVoxel_f_rgb, PlainVoxelArray>* depth_fusion_engine_PVA =
			DepthFusionEngineFactory::Build<TSDFVoxel_f_rgb, PlainVoxelArray>(TMemoryDeviceType);

	depth_fusion_engine_VBH->IntegrateDepthImageIntoTsdfVolume(&teddy_volume_115_VBH, view);
	depth_fusion_engine_PVA->IntegrateDepthImageIntoTsdfVolume(&teddy_volume_115_PVA, view);

	ConstructGeneratedVolumeSubdirectoriesIfMissing();
	teddy_volume_115_VBH.SaveToDisk(vbh_path);
	teddy_volume_115_PVA.SaveToDisk(pva_path);

	Extent3Di bounds_115 = AnalyticsEngine<TSDFVoxel_f_rgb, VoxelBlockHash, MEMORYDEVICE_CPU>::Instance().ComputeVoxelBounds(&teddy_volume_115_VBH);
	LOG4CPLUS_INFO(log4cplus::Logger::getRoot(), "Voxel bounds for partial teddy 115 volumes are: " << bounds_115);


	delete depth_fusion_engine_VBH;
	delete depth_fusion_engine_PVA;
	delete indexing_engine_VBH;
	delete indexing_engine_PVA;
	delete view;
}

void ConstructTeddyVolumes115_Full() {
	ConstructTeddyVolumes115_Generic(teddy::FullInitializationParameters<VoxelBlockHash>(),
	                                 teddy::FullInitializationParameters<PlainVoxelArray>(),
	                                 teddy::FullVolume115Path<VoxelBlockHash>(),
	                                 teddy::FullVolume115Path<PlainVoxelArray>());
}

void ConstructTeddyVolumes115_Partial() {
	ConstructTeddyVolumes115_Generic(teddy::PartialInitializationParameters<VoxelBlockHash>(),
	                                 teddy::PartialInitializationParameters<PlainVoxelArray>(),
	                                 teddy::PartialVolume115Path<VoxelBlockHash>(),
	                                 teddy::PartialVolume115Path<PlainVoxelArray>());
}