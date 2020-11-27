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
#include "../ITMLib/Utils/Configuration/Configuration.h"
#include "../ITMLib/Engines/EditAndCopy/CUDA/EditAndCopyEngine_CUDA.h"
#include "../ITMLib/Engines/VolumeFileIO/VolumeFileIOEngine.h"
#include "../ITMLib/Engines/Analytics/AnalyticsEngine.h"
#include "../ITMLib/Utils/Analytics/VoxelVolumeComparison/VoxelVolumeComparison_CUDA.h"

//test_utilities
#include "TestUtilities/TestUtilities.h"
#include "TestUtilities/TestDataUtilities.h"

using namespace ITMLib;
using namespace test;

typedef VolumeFileIOEngine<TSDFVoxel, PlainVoxelArray> SceneFileIOEngine_PVA;
typedef VolumeFileIOEngine<TSDFVoxel, VoxelBlockHash> SceneFileIOEngine_VBH;

BOOST_AUTO_TEST_CASE(testSaveSceneCompact_CUDA) {


	Vector3i volumeSize(40, 68, 20);
	Vector3i volumeOffset(-20, 0, 0);

	VoxelVolume<TSDFVoxel, PlainVoxelArray> volume_PVA_saved_to_disk(
			MEMORYDEVICE_CUDA, {volumeSize, volumeOffset});

	VoxelVolume<TSDFVoxel, PlainVoxelArray> volume_PVA_loaded_from_disk(
			MEMORYDEVICE_CUDA, {volumeSize, volumeOffset});

	GenerateSimpleSurfaceTestVolume<MEMORYDEVICE_CUDA>(&volume_PVA_saved_to_disk);
	std::string path = GENERATED_TEST_DATA_PREFIX "TestData/volumes/PVA/generated_test_volume_CUDA.dat";
	volume_PVA_saved_to_disk.SaveToDisk(path);
	volume_PVA_loaded_from_disk.Reset();
	volume_PVA_loaded_from_disk.LoadFromDisk(path);

	float tolerance = 1e-8;
	BOOST_REQUIRE_EQUAL(Analytics_CUDA_PVA_Voxel::Instance().CountNonTruncatedVoxels(&volume_PVA_loaded_from_disk), 19456);
	BOOST_REQUIRE(contentAlmostEqual_CUDA(&volume_PVA_saved_to_disk, &volume_PVA_loaded_from_disk, tolerance));

	VoxelVolume<TSDFVoxel, VoxelBlockHash> volume_VBH_saved_to_disk(
			MEMORYDEVICE_CUDA, {0x800, 0x20000});

	VoxelVolume<TSDFVoxel, VoxelBlockHash> volume_VBH_loaded_from_disk(
			MEMORYDEVICE_CUDA, {0x800, 0x20000});

	GenerateSimpleSurfaceTestVolume<MEMORYDEVICE_CUDA>(&volume_VBH_saved_to_disk);
	path = GENERATED_TEST_DATA_PREFIX "TestData/volumes/VBH/generated_test_volume_CUDA.dat";
	volume_VBH_saved_to_disk.SaveToDisk(path);
	volume_VBH_loaded_from_disk.Reset();
	volume_VBH_loaded_from_disk.LoadFromDisk(path);

	BOOST_REQUIRE_EQUAL(Analytics_CUDA_VBH_Voxel::Instance().CountNonTruncatedVoxels(&volume_VBH_loaded_from_disk), 19456);
	BOOST_REQUIRE(contentAlmostEqual_CUDA(&volume_VBH_saved_to_disk, &volume_VBH_loaded_from_disk, tolerance));
	BOOST_REQUIRE(contentAlmostEqual_CUDA(&volume_PVA_saved_to_disk, &volume_VBH_loaded_from_disk, tolerance));
}