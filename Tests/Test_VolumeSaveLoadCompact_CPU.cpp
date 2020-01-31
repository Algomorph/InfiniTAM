//  ================================================================
//  Created by Gregory Kramida on 9/5/19.
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
#define BOOST_TEST_MODULE SceneConstruction
#ifndef WIN32
#define BOOST_TEST_DYN_LINK
#endif

//boost
#include <boost/test/unit_test.hpp>
//ITMLib
#include "../ITMLib/ITMLibDefines.h"
#include "../ITMLib/Objects/Scene/ITMVoxelVolume.h"
#include "../ITMLib/Utils/Configuration.h"
#include "../ITMLib/Engines/EditAndCopy/CPU/EditAndCopyEngine_CPU.h"
#include "TestUtils.h"
#include "../ITMLib/Engines/VolumeFileIO/VolumeFileIOEngine.h"
#include "../ITMLib/Utils/Analytics/SceneStatisticsCalculator/CPU/ITMSceneStatisticsCalculator_CPU.h"
#include "../ITMLib/Utils/Analytics/VoxelVolumeComparison/ITMVoxelVolumeComparison_CPU.h"

using namespace ITMLib;

typedef VolumeFileIOEngine<ITMVoxel, PlainVoxelArray> SceneFileIOEngine_PVA;
typedef VolumeFileIOEngine<ITMVoxel, VoxelBlockHash> SceneFileIOEngine_VBH;

BOOST_AUTO_TEST_CASE(testSaveSceneCompact_CPU) {

	configuration::Configuration* settings = &configuration::get();

	Vector3i volumeSize(40, 68, 20);
	Vector3i volumeOffset(-20, 0, 0);

	ITMVoxelVolume<ITMVoxel, PlainVoxelArray> generated_test_scene_PVA(
			&configuration::get().general_voxel_volume_parameters, configuration::get().swapping_mode == configuration::SWAPPINGMODE_ENABLED,
			MEMORYDEVICE_CPU, {volumeSize, volumeOffset});

	ITMVoxelVolume<ITMVoxel, PlainVoxelArray> loaded_test_scene_PVA(
			&configuration::get().general_voxel_volume_parameters, configuration::get().swapping_mode == configuration::SWAPPINGMODE_ENABLED,
			MEMORYDEVICE_CPU, {volumeSize, volumeOffset});

	GenerateTestScene_CPU(&generated_test_scene_PVA);

	std::string path = "TestData/test_PVA_";
	SceneFileIOEngine_PVA::SaveToDirectoryCompact(&generated_test_scene_PVA, path);
	ManipulationEngine_CPU_PVA_Voxel::Inst().ResetVolume(&loaded_test_scene_PVA);
	SceneFileIOEngine_PVA::LoadFromDirectoryCompact(&loaded_test_scene_PVA, path);

	float tolerance = 1e-8;
	BOOST_REQUIRE_EQUAL(SceneStatCalc_CPU_PVA_Voxel::Instance().ComputeNonTruncatedVoxelCount(&loaded_test_scene_PVA), 19456);
	BOOST_REQUIRE(contentAlmostEqual_CPU(&generated_test_scene_PVA, &loaded_test_scene_PVA, tolerance));

	ITMVoxelVolume<ITMVoxel, VoxelBlockHash> generated_test_scene_VBH(
			&configuration::get().general_voxel_volume_parameters, configuration::get().swapping_mode == configuration::SWAPPINGMODE_ENABLED,
			MEMORYDEVICE_CPU);

	ITMVoxelVolume<ITMVoxel, VoxelBlockHash> loaded_test_scene_VBH(
			&configuration::get().general_voxel_volume_parameters, configuration::get().swapping_mode == configuration::SWAPPINGMODE_ENABLED,
			MEMORYDEVICE_CPU);

	GenerateTestScene_CPU(&generated_test_scene_VBH);
	path = "TestData/test_VBH_";
	SceneFileIOEngine_VBH::SaveToDirectoryCompact(&generated_test_scene_VBH, path);
	ManipulationEngine_CPU_VBH_Voxel::Inst().ResetVolume(&loaded_test_scene_VBH);
	SceneFileIOEngine_VBH::LoadFromDirectoryCompact(&loaded_test_scene_VBH, path);

	BOOST_REQUIRE_EQUAL(SceneStatCalc_CPU_VBH_Voxel::Instance().ComputeNonTruncatedVoxelCount(&loaded_test_scene_VBH), 19456);
	BOOST_REQUIRE(contentAlmostEqual_CPU(&generated_test_scene_VBH, &loaded_test_scene_VBH, tolerance));
	BOOST_REQUIRE(contentAlmostEqual_CPU_Verbose(&generated_test_scene_PVA, &loaded_test_scene_VBH, tolerance));
}