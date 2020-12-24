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
#pragma once
// === local ===
#include "GenerateVolumeFusionConsistencyData.h"

// === ITMLib ===
#include "../../ITMLib/Objects/Volume/VoxelVolume.h"
#include "../../ITMLib/GlobalTemplateDefines.h"
#include "../../ITMLib/Utils/Logging/Logging.h"
#include "../../ITMLib/Engines/LevelSetAlignment/Interface/LevelSetAlignmentParameters.h"
#include "../../ITMLib/Engines/VolumeFusion/VolumeFusionEngineFactory.h"


// === Test Utilities ===
#include "../TestUtilities/LevelSetAlignment/TestCaseOrganizationBySwitches.h"
#include "../TestUtilities/TestUtilities.h"
#include "../TestUtilities/TestDataUtilities.h"
#include "../Test_LevelSetAlignment_WarpGradientDataFixture.h"


using namespace ITMLib;
using namespace test;

template<typename TIndex>
void GenerateFusedVolumeTestData() {
	const int iteration = 4;
	LOG4CPLUS_INFO(log4cplus::Logger::getRoot(),
	               "Generating fused volume data from snoopy masked partial volume 16 & 17 warps ("
			               << IndexString<TIndex>() << ") ... ");
	VoxelVolume<TSDFVoxel, TIndex>* warped_live_volume;
	LevelSetAlignmentSwitches data_tikhonov_sobolev_switches(true, false, true, false, true);
	LoadVolume(&warped_live_volume,
	           GetWarpedLivePath<TIndex>(SwitchesToPrefix(data_tikhonov_sobolev_switches), iteration),
	           MEMORYDEVICE_CPU, test::snoopy::InitializationParameters_Fr16andFr17<TIndex>());
	VoxelVolume<TSDFVoxel, TIndex>* canonical_volume;
	LoadVolume(&canonical_volume, test::snoopy::PartialVolume16Path<TIndex>(),
	           MEMORYDEVICE_CPU, test::snoopy::InitializationParameters_Fr16andFr17<TIndex>());
	AllocateUsingOtherVolume(canonical_volume, warped_live_volume, MEMORYDEVICE_CPU);


	VolumeFusionEngineInterface<TSDFVoxel, TIndex>* volume_fusion_engine =
			VolumeFusionEngineFactory::Build<TSDFVoxel, TIndex>(MEMORYDEVICE_CPU);
	volume_fusion_engine->FuseOneTsdfVolumeIntoAnother(canonical_volume, warped_live_volume, 0);

	test::ConstructGeneratedVolumeSubdirectoriesIfMissing();
	canonical_volume->SaveToDisk(GENERATED_TEST_DATA_PREFIX
	"TestData/volumes/" + IndexString<TIndex>() + "/fused.dat");

	delete volume_fusion_engine;
	delete warped_live_volume;
	delete canonical_volume;
}