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
#include "../ITMLib/GlobalTemplateDefines.h"
#include "../ITMLib/Objects/Volume/VoxelVolume.h"
#include "../ITMLib/Utils/Configuration.h"
#include "../ITMLib/Engines/EditAndCopy/CPU/EditAndCopyEngine_CPU.h"
#include "TestUtils.h"
#include "../ITMLib/Engines/VolumeFileIO/VolumeFileIOEngine.h"
#include "../ITMLib/Engines/Analytics/AnalyticsEngine.h"
#include "../ITMLib/Utils/Analytics/VoxelVolumeComparison/VoxelVolumeComparison_CPU.h"

using namespace ITMLib;

typedef VolumeFileIOEngine<TSDFVoxel, PlainVoxelArray> SceneFileIOEngine_PVA;
typedef VolumeFileIOEngine<TSDFVoxel, VoxelBlockHash> SceneFileIOEngine_VBH;

BOOST_AUTO_TEST_CASE(testSaveSceneCompact_CPU) {

	configuration::Configuration* settings = &configuration::get();

	Vector3i volumeSize(40, 68, 20);
	Vector3i volumeOffset(-20, 0, 0);

	VoxelVolume<TSDFVoxel, PlainVoxelArray> generated_test_scene_PVA(
			MEMORYDEVICE_CPU, {volumeSize, volumeOffset});

	VoxelVolume<TSDFVoxel, PlainVoxelArray> loaded_test_scene_PVA(
			MEMORYDEVICE_CPU, {volumeSize, volumeOffset});

	GenerateSimpleSurfaceTestVolume<MEMORYDEVICE_CPU>(&generated_test_scene_PVA);

	std::string path = "TestData/testSaveSceneCompact_PVA_CPU.dat";
	generated_test_scene_PVA.SaveToDisk(path);
	ManipulationEngine_CPU_PVA_Voxel::Inst().ResetVolume(&loaded_test_scene_PVA);
	loaded_test_scene_PVA.LoadFromDisk(path);

	float tolerance = 1e-8;
	BOOST_REQUIRE_EQUAL(StatCalc_CPU_PVA_Voxel::Instance().CountNonTruncatedVoxels(&loaded_test_scene_PVA), 19456);
	BOOST_REQUIRE(contentAlmostEqual_CPU(&generated_test_scene_PVA, &loaded_test_scene_PVA, tolerance));

	VoxelVolume<TSDFVoxel, VoxelBlockHash> generated_test_scene_VBH(
			MEMORYDEVICE_CPU);

	VoxelVolume<TSDFVoxel, VoxelBlockHash> loaded_test_scene_VBH(
			MEMORYDEVICE_CPU);

	GenerateSimpleSurfaceTestVolume<MEMORYDEVICE_CPU>(&generated_test_scene_VBH);
	path = "TestData/test_VBH_CPU_generated_test_scene.dat";
	generated_test_scene_VBH.SaveToDisk(path);
	ManipulationEngine_CPU_VBH_Voxel::Inst().ResetVolume(&loaded_test_scene_VBH);
	loaded_test_scene_VBH.LoadFromDisk(path);

	BOOST_REQUIRE_EQUAL(StatCalc_CPU_VBH_Voxel::Instance().CountNonTruncatedVoxels(&loaded_test_scene_VBH), 19456);
	BOOST_REQUIRE(contentAlmostEqual_CPU(&generated_test_scene_VBH, &loaded_test_scene_VBH, tolerance));
	BOOST_REQUIRE(contentAlmostEqual_CPU_Verbose(&generated_test_scene_PVA, &loaded_test_scene_VBH, tolerance));
}