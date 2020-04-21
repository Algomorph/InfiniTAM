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
#include "../ITMLib/Engines/VolumeFileIO/VolumeFileIOEngine.h"
#include "../ITMLib/Engines/Analytics/AnalyticsEngine.h"
#include "../ITMLib/Utils/Analytics/VoxelVolumeComparison/VoxelVolumeComparison_CPU.h"

//test_utilities
#include "TestUtilities/TestUtilities.h"

using namespace ITMLib;
using namespace test_utilities;

typedef VolumeFileIOEngine<TSDFVoxel, PlainVoxelArray> SceneFileIOEngine_PVA;
typedef VolumeFileIOEngine<TSDFVoxel, VoxelBlockHash> SceneFileIOEngine_VBH;

BOOST_AUTO_TEST_CASE(testSaveSceneCompact_CPU) {

	configuration::Configuration* settings = &configuration::get();

	Vector3i volume_size(40, 68, 20);
	Vector3i volume_offset(-20, 0, 0);

	VoxelVolume<TSDFVoxel, PlainVoxelArray> generated_test_volume_PVA(
			MEMORYDEVICE_CPU, {volume_size, volume_offset});

	VoxelVolume<TSDFVoxel, PlainVoxelArray> loaded_test_volume_PVA(
			MEMORYDEVICE_CPU, {volume_size, volume_offset});

	GenerateSimpleSurfaceTestVolume<MEMORYDEVICE_CPU>(&generated_test_volume_PVA);

	std::string path = "TestData/volumes/PVA/generated_test_volume_CPU.dat";
	generated_test_volume_PVA.SaveToDisk(path);
	ManipulationEngine_CPU_PVA_Voxel::Inst().ResetVolume(&loaded_test_volume_PVA);
	loaded_test_volume_PVA.LoadFromDisk(path);

	float tolerance = 1e-8;
	BOOST_REQUIRE_EQUAL(Analytics_CPU_PVA_Voxel::Instance().CountNonTruncatedVoxels(&loaded_test_volume_PVA), 19456);
	BOOST_REQUIRE(contentAlmostEqual_CPU(&generated_test_volume_PVA, &loaded_test_volume_PVA, tolerance));

	VoxelVolume<TSDFVoxel, VoxelBlockHash> generated_test_scene_VBH(
			MEMORYDEVICE_CPU);

	GenerateSimpleSurfaceTestVolume<MEMORYDEVICE_CPU>(&generated_test_scene_VBH);
	path = "TestData/volumes/VBH/generated_test_volume_CPU.dat";
	generated_test_scene_VBH.SaveToDisk(path);

	VoxelVolume<TSDFVoxel, VoxelBlockHash> loaded_test_scene_VBH(
			MEMORYDEVICE_CPU);
	loaded_test_scene_VBH.Reset();
	loaded_test_scene_VBH.LoadFromDisk(path);

	BOOST_REQUIRE_EQUAL(Analytics_CPU_VBH_Voxel::Instance().CountNonTruncatedVoxels(&loaded_test_scene_VBH), 19456);
	BOOST_REQUIRE(contentAlmostEqual_CPU(&generated_test_scene_VBH, &loaded_test_scene_VBH, tolerance));
	BOOST_REQUIRE(contentAlmostEqual_CPU_Verbose(&generated_test_volume_PVA, &loaded_test_scene_VBH, tolerance));
}