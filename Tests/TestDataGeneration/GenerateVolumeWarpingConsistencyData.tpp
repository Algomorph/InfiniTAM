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
#include "GenerateVolumeWarpingConsistencyData.h"

// === ITMLib ===
#include "../../ITMLib/GlobalTemplateDefines.h"
#include "../../ITMLib/Utils/Logging/Logging.h"
#include "../../ITMLib/Objects/Volume/VoxelVolume.h"
#include "../../ITMLib/Engines/Warping/WarpingEngineFactory.h"
//(CPU)
#include "../../ITMLib/Engines/Indexing/VBH/CPU/IndexingEngine_VoxelBlockHash_CPU.h"
//(CUDA)
#ifndef COMPILE_WITHOUT_CUDA
#include "../../ITMLib/Engines/Indexing/VBH/CUDA/IndexingEngine_VoxelBlockHash_CUDA.h"
#endif

// === Test Utilities ===
#include "../TestUtilities/TestDataUtilities.h"


using namespace ITMLib;
using namespace test;

template<typename TIndex>
void GenerateWarpedVolumeTestData() {
	LOG4CPLUS_INFO(log4cplus::Logger::getRoot(),
	               "Generating warped volume data from snoopy masked partial volume 16 & 17 warps ("
			               << IndexString<TIndex>() << ") ... ");
	VoxelVolume<WarpVoxel, TIndex>* warps;
	LoadVolume(&warps,
	           GENERATED_TEST_DATA_PREFIX "TestData/volumes/" + IndexString<TIndex>() + "/warp_field_0_data_tikhonov_sobolev.dat",
			MEMORYDEVICE_CPU, test::snoopy::InitializationParameters_Fr16andFr17<TIndex>());
	VoxelVolume<TSDFVoxel, TIndex>* live_volume;
	LoadVolume(&live_volume, test::snoopy::PartialVolume17Path<TIndex>(),
	           MEMORYDEVICE_CPU, test::snoopy::InitializationParameters_Fr16andFr17<TIndex>());
	auto warped_live_volume = new VoxelVolume<TSDFVoxel, TIndex>(MEMORYDEVICE_CPU,
	                                                             test::snoopy::InitializationParameters_Fr16andFr17<TIndex>());
	warped_live_volume->Reset();

	auto warping_engine =
			WarpingEngineFactory::Build<TSDFVoxel, WarpVoxel, TIndex>(MEMORYDEVICE_CPU);

	AllocateUsingOtherVolume(warped_live_volume, live_volume, MEMORYDEVICE_CPU);
	warping_engine->WarpVolume_WarpUpdates(warps, live_volume, warped_live_volume);

	test::ConstructGeneratedVolumeSubdirectoriesIfMissing();
	warped_live_volume->SaveToDisk(
			GENERATED_TEST_DATA_PREFIX "TestData/volumes/" + IndexString<TIndex>() + "/warped_live.dat");

	delete warping_engine;
	delete warps;
	delete live_volume;
	delete warped_live_volume;
}