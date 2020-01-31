//  ================================================================
//  Created by Gregory Kramida on 10/23/17.
//  Copyright (c) 2017-2025 Gregory Kramida
//  Licensed under the Apache Li
// cense, Version 2.0 (the "License");
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

//stdlib
#include <random>
#include <vector>


//boost
#include <boost/test/unit_test.hpp>

//ITMlib
#include "../ITMLib/ITMLibDefines.h"
#include "../ITMLib/Objects/Volume/VoxelVolume.h"
#include "../ITMLib/Objects/Volume/RepresentationAccess.h"
#include "../ITMLib/Objects/Camera/CalibIO.h"

#include "../ITMLib/Utils/Configuration.h"
#include "../ITMLib/Utils/Analytics/SceneStatisticsCalculator/CPU/ITMSceneStatisticsCalculator_CPU.h"
#include "../ITMLib/Utils/Analytics/VoxelVolumeComparison/ITMVoxelVolumeComparison_CUDA.h"
#include "../ITMLib/Utils/Analytics/ITMAlmostEqual.h"

#include "../ITMLib/Engines/EditAndCopy/CPU/EditAndCopyEngine_CPU.h"
#include "../ITMLib/Engines/EditAndCopy/CUDA/EditAndCopyEngine_CUDA.h"
#include "../ITMLib/Engines/ViewBuilding/ViewBuilderFactory.h"
#include "../ITMLib/Engines/VolumeFileIO/VolumeFileIOEngine.h"
#include "../ITMLib/Engines/DepthFusion/DepthFusionEngine.h"
#include "../ITMLib/Engines/DepthFusion/DepthFusionEngineFactory.h"

#include "../InputSource/ImageSourceEngine.h"
#include "../ORUtils/FileUtils.h"

//local
#include "TestUtils.h"


using namespace ITMLib;


BOOST_AUTO_TEST_CASE(testSetVoxelAndCopy_PlainVoxelArray_CUDA) {
	Vector3i volumeSize(20);
	Vector3i volumeOffset(-10, -10, 0);
	PlainVoxelArray::InitializationParameters indexParameters(volumeSize,volumeOffset);

	VoxelVolume<ITMVoxel, PlainVoxelArray> scene1(&configuration::get().general_voxel_volume_parameters,
	                                                    configuration::get().swapping_mode == configuration::SWAPPINGMODE_ENABLED,
	                                              MEMORYDEVICE_CUDA, indexParameters);


	ManipulationEngine_CUDA_PVA_Voxel::Inst().ResetVolume(&scene1);

	ITMVoxel voxelZero;
	voxelZero.sdf = 0.0f;
	BOOST_REQUIRE(ManipulationEngine_CUDA_PVA_Voxel::Inst().SetVoxel(&scene1, Vector3i(0, 0, 0), voxelZero));

	ITMVoxel out;
	out = ManipulationEngine_CUDA_PVA_Voxel::Inst().ReadVoxel(&scene1, Vector3i(0, 0, 0));
	BOOST_REQUIRE(out.sdf == voxelZero.sdf);
	ITMVoxel voxelHalf;
	voxelHalf.sdf = 0.5f;

	ManipulationEngine_CUDA_PVA_Voxel::Inst().SetVoxel(&scene1, Vector3i(1, 1, 1), voxelHalf);
	out = ManipulationEngine_CUDA_PVA_Voxel::Inst().ReadVoxel(&scene1, Vector3i(1, 1, 1));
	BOOST_REQUIRE(out.sdf == voxelHalf.sdf);
	ManipulationEngine_CUDA_PVA_Voxel::Inst().SetVoxel(&scene1, Vector3i(9, 9, 9), voxelZero);
	ManipulationEngine_CUDA_PVA_Voxel::Inst().SetVoxel(&scene1, Vector3i(9, 9, 9), voxelHalf);
	ManipulationEngine_CUDA_PVA_Voxel::Inst().SetVoxel(&scene1, Vector3i(3, 3, 3), voxelHalf);
	out = ManipulationEngine_CUDA_PVA_Voxel::Inst().ReadVoxel(&scene1, Vector3i(9, 9, 9));
	BOOST_REQUIRE(out.sdf == voxelHalf.sdf);
	Vector3i voxelPos(8, 5, 2);
	ManipulationEngine_CUDA_PVA_Voxel::Inst().SetVoxel(&scene1, voxelPos, voxelZero);
	out = ManipulationEngine_CUDA_PVA_Voxel::Inst().ReadVoxel(&scene1, voxelPos);
	BOOST_REQUIRE(out.sdf == voxelZero.sdf);
	out = ManipulationEngine_CUDA_PVA_Voxel::Inst().ReadVoxel(&scene1, Vector3i(0, 0, 0));
	BOOST_REQUIRE(out.sdf == voxelZero.sdf);

	Vector3i offset(-2, 3, 4);
	VoxelVolume<ITMVoxel, PlainVoxelArray> scene2(&configuration::get().general_voxel_volume_parameters,
	                                                    configuration::get().swapping_mode == configuration::SWAPPINGMODE_ENABLED,
	                                              MEMORYDEVICE_CUDA);
	ManipulationEngine_CUDA_PVA_Voxel::Inst().ResetVolume(&scene2);
	ManipulationEngine_CUDA_PVA_Voxel::Inst().CopyVolume(&scene2, &scene1, offset);

	float tolerance = 1e-5f;
	BOOST_REQUIRE(almostEqual(out.sdf, voxelZero.sdf, tolerance));
	out = ManipulationEngine_CUDA_PVA_Voxel::Inst().ReadVoxel(&scene2, Vector3i(0, 0, 0) + offset);
	BOOST_REQUIRE(out.sdf == voxelZero.sdf);
	out = ManipulationEngine_CUDA_PVA_Voxel::Inst().ReadVoxel(&scene2, Vector3i(3, 3, 3) + offset);
	BOOST_REQUIRE(out.sdf == voxelHalf.sdf);
	out = ManipulationEngine_CUDA_PVA_Voxel::Inst().ReadVoxel(&scene2, Vector3i(1, 1, 1) + offset);
	BOOST_REQUIRE(out.sdf == voxelHalf.sdf);
}

BOOST_AUTO_TEST_CASE(testSetVoxelAndCopy_VoxelBlockHash_CUDA) {
	VoxelVolume<ITMVoxel, VoxelBlockHash> scene1(&configuration::get().general_voxel_volume_parameters,
	                                                   configuration::get().swapping_mode == configuration::SWAPPINGMODE_ENABLED,
	                                             MEMORYDEVICE_CUDA);

	typedef EditAndCopyEngine_CPU<ITMVoxel, VoxelBlockHash> SceneManipulationEngine;

	ManipulationEngine_CUDA_VBH_Voxel::Inst().ResetVolume(&scene1);

	ITMVoxel voxelZero;
	voxelZero.sdf = 0.0f;
	ManipulationEngine_CUDA_VBH_Voxel::Inst().SetVoxel(&scene1, Vector3i(0, 0, 0), voxelZero);

	ITMVoxel out;
	out = ManipulationEngine_CUDA_VBH_Voxel::Inst().ReadVoxel(&scene1, Vector3i(0, 0, 0));
	BOOST_REQUIRE(out.sdf == voxelZero.sdf);
	ITMVoxel voxelHalf;
	voxelHalf.sdf = 0.5f;

	ManipulationEngine_CUDA_VBH_Voxel::Inst().SetVoxel(&scene1, Vector3i(1, 1, 1), voxelHalf);
	out = ManipulationEngine_CUDA_VBH_Voxel::Inst().ReadVoxel(&scene1, Vector3i(1, 1, 1));
	BOOST_REQUIRE(out.sdf == voxelHalf.sdf);
	ManipulationEngine_CUDA_VBH_Voxel::Inst().SetVoxel(&scene1, Vector3i(9, 9, 9), voxelZero);
	ManipulationEngine_CUDA_VBH_Voxel::Inst().SetVoxel(&scene1, Vector3i(9, 9, 9), voxelHalf);
	out = ManipulationEngine_CUDA_VBH_Voxel::Inst().ReadVoxel(&scene1, Vector3i(9, 9, 9));
	BOOST_REQUIRE(out.sdf == voxelHalf.sdf);
	Vector3i voxelPos(232, 125, 62);
	ManipulationEngine_CUDA_VBH_Voxel::Inst().SetVoxel(&scene1, voxelPos, voxelZero);
	out = ManipulationEngine_CUDA_VBH_Voxel::Inst().ReadVoxel(&scene1, voxelPos);
	BOOST_REQUIRE(out.sdf == voxelZero.sdf);
	out = ManipulationEngine_CUDA_VBH_Voxel::Inst().ReadVoxel(&scene1, Vector3i(0, 0, 0));
	BOOST_REQUIRE(out.sdf == voxelZero.sdf);

	Vector3i offset(-34, 6, 9);
	VoxelVolume<ITMVoxel, VoxelBlockHash> scene2(&configuration::get().general_voxel_volume_parameters,
	                                                   configuration::get().swapping_mode == configuration::SWAPPINGMODE_ENABLED,
	                                             MEMORYDEVICE_CUDA);
	ManipulationEngine_CUDA_VBH_Voxel::Inst().ResetVolume(&scene2);
	ManipulationEngine_CUDA_VBH_Voxel::Inst().CopyVolume(&scene2, &scene1, offset);
	out = ManipulationEngine_CUDA_VBH_Voxel::Inst().ReadVoxel(&scene2, voxelPos + offset);
	BOOST_REQUIRE(out.sdf == voxelZero.sdf);
	out = ManipulationEngine_CUDA_VBH_Voxel::Inst().ReadVoxel(&scene2, Vector3i(0, 0, 0) + offset);
	BOOST_REQUIRE(out.sdf == voxelZero.sdf);
	out = ManipulationEngine_CUDA_VBH_Voxel::Inst().ReadVoxel(&scene2, Vector3i(9, 9, 9) + offset);
	BOOST_REQUIRE(out.sdf == voxelHalf.sdf);
	out = ManipulationEngine_CUDA_VBH_Voxel::Inst().ReadVoxel(&scene2, Vector3i(1, 1, 1) + offset);
	BOOST_REQUIRE(out.sdf == voxelHalf.sdf);
}


BOOST_AUTO_TEST_CASE(testCompareVoxelVolumes_CUDA_and_CPU_ITMVoxel) {
	float tolerance = 1e-6;

	Vector3i volumeSize(40);
	Vector3i volumeOffset(-20, -20, 0);
	PlainVoxelArray::InitializationParameters indexParametersPVA(volumeSize, volumeOffset);
	VoxelBlockHash::InitializationParameters indexParametersVBH(0x800, 0x20000);
	Vector3i extentEndVoxel = volumeOffset + volumeSize;


	VoxelVolume<ITMVoxel, PlainVoxelArray> scene_PVA1(&configuration::get().general_voxel_volume_parameters,
	                                                    configuration::get().swapping_mode == configuration::SWAPPINGMODE_ENABLED,
	                                                  MEMORYDEVICE_CUDA,
	                                                  indexParametersPVA);
	ManipulationEngine_CUDA_PVA_Voxel::Inst().ResetVolume(&scene_PVA1);
	VoxelVolume<ITMVoxel, PlainVoxelArray> scene_PVA2(&configuration::get().general_voxel_volume_parameters,
	                                                    configuration::get().swapping_mode == configuration::SWAPPINGMODE_ENABLED,
	                                                  MEMORYDEVICE_CUDA,
	                                                  indexParametersPVA);
	ManipulationEngine_CUDA_PVA_Voxel::Inst().ResetVolume(&scene_PVA2);
	VoxelVolume<ITMVoxel, VoxelBlockHash> scene_VBH1(&configuration::get().general_voxel_volume_parameters,
	                                                   configuration::get().swapping_mode == configuration::SWAPPINGMODE_ENABLED,
	                                                 MEMORYDEVICE_CUDA,
	                                                 indexParametersVBH);
	ManipulationEngine_CUDA_VBH_Voxel::Inst().ResetVolume(&scene_VBH1);
	VoxelVolume<ITMVoxel, VoxelBlockHash> scene_VBH2(&configuration::get().general_voxel_volume_parameters,
	                                                   configuration::get().swapping_mode == configuration::SWAPPINGMODE_ENABLED,
	                                                 MEMORYDEVICE_CUDA,
	                                                 indexParametersVBH);
	ManipulationEngine_CUDA_VBH_Voxel::Inst().ResetVolume(&scene_VBH2);
	VoxelVolume<ITMVoxel, PlainVoxelArray> scene5(&configuration::get().general_voxel_volume_parameters,
	                                                   configuration::get().swapping_mode == configuration::SWAPPINGMODE_ENABLED,
	                                              MEMORYDEVICE_CPU,
	                                              indexParametersPVA);
	ManipulationEngine_CPU_PVA_Voxel::Inst().ResetVolume(&scene5);
	VoxelVolume<ITMVoxel, VoxelBlockHash> scene6(&configuration::get().general_voxel_volume_parameters,
	                                                   configuration::get().swapping_mode == configuration::SWAPPINGMODE_ENABLED,
	                                             MEMORYDEVICE_CPU,
	                                             indexParametersVBH);
	ManipulationEngine_CPU_VBH_Voxel::Inst().ResetVolume(&scene6);


	std::random_device random_device;
	std::mt19937 generator(random_device());

	auto singleVoxelTests = [&]() {
		std::uniform_int_distribution<int> coordinate_distribution2(volumeOffset.x, 0);
		ITMVoxel voxel;
		simulateVoxelAlteration(voxel, -0.1f);

		Vector3i coordinate(coordinate_distribution2(generator), coordinate_distribution2(generator), 0);

		ManipulationEngine_CUDA_PVA_Voxel::Inst().SetVoxel(&scene_PVA2, coordinate, voxel);
		ManipulationEngine_CUDA_VBH_Voxel::Inst().SetVoxel(&scene_VBH2, coordinate, voxel);
		BOOST_REQUIRE(!contentAlmostEqual_CUDA(&scene_PVA1, &scene_PVA2, tolerance));
		BOOST_REQUIRE(!contentAlmostEqual_CUDA(&scene_VBH1, &scene_VBH2, tolerance));
		BOOST_REQUIRE(!contentAlmostEqual_CUDA(&scene_PVA1, &scene_VBH2, tolerance));

		ITMVoxel defaultVoxel;
		ManipulationEngine_CUDA_PVA_Voxel::Inst().SetVoxel(&scene_PVA2, coordinate, defaultVoxel);
		ManipulationEngine_CUDA_VBH_Voxel::Inst().SetVoxel(&scene_VBH2, coordinate, defaultVoxel);
		BOOST_REQUIRE(contentAlmostEqual_CUDA(&scene_PVA1, &scene_PVA2, tolerance));
		BOOST_REQUIRE(contentAlmostEqual_CUDA(&scene_VBH1, &scene_VBH2, tolerance));

		coordinate = volumeOffset + volumeSize - Vector3i(1);
		voxel = ManipulationEngine_CUDA_PVA_Voxel::Inst().ReadVoxel(&scene_PVA2, coordinate);
		simulateVoxelAlteration(voxel, fmod((ITMVoxel::valueToFloat(voxel.sdf) + 0.1f), 1.0f));
		std::cout << coordinate << std::endl;
		ManipulationEngine_CUDA_PVA_Voxel::Inst().SetVoxel(&scene_PVA2, coordinate, voxel);
		ManipulationEngine_CUDA_VBH_Voxel::Inst().SetVoxel(&scene_VBH2, coordinate, voxel);
		BOOST_REQUIRE(!contentAlmostEqual_CUDA(&scene_PVA1, &scene_PVA2, tolerance));
		BOOST_REQUIRE(!contentAlmostEqual_CUDA(&scene_VBH1, &scene_VBH2, tolerance));
		BOOST_REQUIRE(!contentAlmostEqual_CUDA(&scene_PVA1, &scene_VBH2, tolerance));

		ManipulationEngine_CUDA_PVA_Voxel::Inst().SetVoxel(&scene_PVA2, coordinate, defaultVoxel);
		ManipulationEngine_CUDA_VBH_Voxel::Inst().SetVoxel(&scene_VBH2, coordinate, defaultVoxel);
	};

	std::uniform_real_distribution<float> sdf_distribution(-1.0f, 1.0f);
	std::uniform_int_distribution<int> coordinate_distribution(0, extentEndVoxel.x - 1);

	const int modifiedVoxelCount = 120;

	singleVoxelTests();

//	generate only in the positive coordinates' volume, to make sure that the unneeded voxel hash blocks are properly dismissed
	for (int iVoxel = 0; iVoxel < modifiedVoxelCount; iVoxel++) {
		ITMVoxel voxel;

		simulateVoxelAlteration(voxel, sdf_distribution(generator));
		Vector3i coordinate(coordinate_distribution(generator),
		                    coordinate_distribution(generator),
		                    coordinate_distribution(generator));

		ManipulationEngine_CUDA_PVA_Voxel::Inst().SetVoxel(&scene_PVA1, coordinate, voxel);
		ManipulationEngine_CUDA_PVA_Voxel::Inst().SetVoxel(&scene_PVA2, coordinate, voxel);
		ManipulationEngine_CUDA_VBH_Voxel::Inst().SetVoxel(&scene_VBH1, coordinate, voxel);
		ManipulationEngine_CUDA_VBH_Voxel::Inst().SetVoxel(&scene_VBH2, coordinate, voxel);
		ManipulationEngine_CPU_PVA_Voxel::Inst().SetVoxel(&scene5, coordinate, voxel);
		ManipulationEngine_CPU_VBH_Voxel::Inst().SetVoxel(&scene6, coordinate, voxel);
	}

	BOOST_REQUIRE(contentAlmostEqual_CUDA(&scene_PVA1, &scene_PVA2, tolerance));
	BOOST_REQUIRE(contentAlmostEqual_CUDA(&scene_VBH1, &scene_VBH2, tolerance));
	BOOST_REQUIRE(contentAlmostEqual_CUDA_Verbose(&scene_PVA1, &scene_VBH1, tolerance));

	//CUDA-to-CPU comparisons
	VoxelVolume<ITMVoxel, PlainVoxelArray> scene2_cpu_copy(scene_PVA2, MEMORYDEVICE_CPU);
	BOOST_REQUIRE( contentAlmostEqual_CPU(&scene2_cpu_copy, &scene5, tolerance));
	VoxelVolume<ITMVoxel, VoxelBlockHash> scene4_cpu_copy(scene_VBH2, MEMORYDEVICE_CPU);
	BOOST_REQUIRE(contentAlmostEqual_CPU(&scene4_cpu_copy, &scene6, tolerance));

	singleVoxelTests();
}


BOOST_AUTO_TEST_CASE(testCompareVoxelVolumes_CUDA_ITMWarp) {
	float tolerance = 1e-6;

	Vector3i volumeSize(40);
	Vector3i volumeOffset(-20, -20, 0);
	PlainVoxelArray::InitializationParameters indexParametersPVA(volumeSize, volumeOffset);
	VoxelBlockHash::InitializationParameters indexParametersVBH(0x800, 0x20000);
	Vector3i extentEndVoxel = volumeOffset + volumeSize;

	VoxelVolume<ITMWarp, PlainVoxelArray> scene1(&configuration::get().general_voxel_volume_parameters,
	                                                   configuration::get().swapping_mode == configuration::SWAPPINGMODE_ENABLED,
	                                             MEMORYDEVICE_CUDA, indexParametersPVA);
	ManipulationEngine_CUDA_PVA_Warp::Inst().ResetVolume(&scene1);
	VoxelVolume<ITMWarp, PlainVoxelArray> scene2(&configuration::get().general_voxel_volume_parameters,
	                                                   configuration::get().swapping_mode == configuration::SWAPPINGMODE_ENABLED,
	                                             MEMORYDEVICE_CUDA, indexParametersPVA);
	ManipulationEngine_CUDA_PVA_Warp::Inst().ResetVolume(&scene2);
	VoxelVolume<ITMWarp, VoxelBlockHash> scene3(&configuration::get().general_voxel_volume_parameters,
	                                                  configuration::get().swapping_mode == configuration::SWAPPINGMODE_ENABLED,
	                                            MEMORYDEVICE_CUDA, indexParametersVBH);
	ManipulationEngine_CUDA_VBH_Warp::Inst().ResetVolume(&scene3);
	VoxelVolume<ITMWarp, VoxelBlockHash> scene4(&configuration::get().general_voxel_volume_parameters,
	                                                  configuration::get().swapping_mode == configuration::SWAPPINGMODE_ENABLED,
	                                            MEMORYDEVICE_CUDA, indexParametersVBH);
	ManipulationEngine_CUDA_VBH_Warp::Inst().ResetVolume(&scene4);

	std::random_device random_device;
	std::mt19937 generator(random_device());

	auto singleVoxelTests = [&]() {
		std::uniform_int_distribution<int> coordinate_distribution2(volumeOffset.x, 0);
		ITMWarp warp;
		warp.framewise_warp = Vector3f(-0.1);

		Vector3i coordinate(coordinate_distribution2(generator), coordinate_distribution2(generator), 0);

		ManipulationEngine_CUDA_PVA_Warp::Inst().SetVoxel(&scene2, coordinate, warp);
		ManipulationEngine_CUDA_VBH_Warp::Inst().SetVoxel(&scene4, coordinate, warp);
		BOOST_REQUIRE(!contentAlmostEqual_CUDA(&scene1, &scene2, tolerance));
		BOOST_REQUIRE(!contentAlmostEqual_CUDA(&scene3, &scene4, tolerance));
		BOOST_REQUIRE(!contentAlmostEqual_CUDA(&scene1, &scene4, tolerance));

		ITMWarp defaultVoxel;
		ManipulationEngine_CUDA_PVA_Warp::Inst().SetVoxel(&scene2, coordinate, defaultVoxel);
		ManipulationEngine_CUDA_VBH_Warp::Inst().SetVoxel(&scene4, coordinate, defaultVoxel);
		BOOST_REQUIRE(contentAlmostEqual_CUDA(&scene1, &scene2, tolerance));
		BOOST_REQUIRE(contentAlmostEqual_CUDA(&scene3, &scene4, tolerance));

		coordinate = volumeOffset + volumeSize - Vector3i(1);
		warp = ManipulationEngine_CUDA_PVA_Warp::Inst().ReadVoxel(&scene2, coordinate);
		warp.warp_update += Vector3f(0.1);
		ManipulationEngine_CUDA_PVA_Warp::Inst().SetVoxel(&scene2, coordinate, warp);
		ManipulationEngine_CUDA_VBH_Warp::Inst().SetVoxel(&scene4, coordinate, warp);
		BOOST_REQUIRE(!contentAlmostEqual_CUDA(&scene1, &scene2, tolerance));
		BOOST_REQUIRE(!contentAlmostEqual_CUDA(&scene3, &scene4, tolerance));
		BOOST_REQUIRE(!contentAlmostEqual_CUDA(&scene1, &scene4, tolerance));

		ManipulationEngine_CUDA_PVA_Warp::Inst().SetVoxel(&scene2, coordinate, defaultVoxel);
		ManipulationEngine_CUDA_VBH_Warp::Inst().SetVoxel(&scene4, coordinate, defaultVoxel);
	};

	std::uniform_real_distribution<float> warp_distribution(-1.0f, 1.0f);
	std::uniform_int_distribution<int> coordinate_distribution(0, extentEndVoxel.x - 1);

	const int modifiedWarpCount = 120;

	singleVoxelTests();

//	generate only in the positive coordinates' volume, to make sure that the unneeded voxel hash blocks are properly dismissed
	for (int iWarp = 0; iWarp < modifiedWarpCount; iWarp++) {
		ITMWarp warp;
		Vector3f framewise_warp(warp_distribution(generator), warp_distribution(generator), warp_distribution(generator));
		warp.framewise_warp = framewise_warp;

		Vector3i coordinate(coordinate_distribution(generator),
		                    coordinate_distribution(generator),
		                    coordinate_distribution(generator));

		ManipulationEngine_CUDA_PVA_Warp::Inst().SetVoxel(&scene1, coordinate, warp);
		ManipulationEngine_CUDA_PVA_Warp::Inst().SetVoxel(&scene2, coordinate, warp);
		ManipulationEngine_CUDA_VBH_Warp::Inst().SetVoxel(&scene3, coordinate, warp);
		ManipulationEngine_CUDA_VBH_Warp::Inst().SetVoxel(&scene4, coordinate, warp);
	}

	BOOST_REQUIRE(contentAlmostEqual_CUDA(&scene1, &scene2, tolerance));
	BOOST_REQUIRE(contentAlmostEqual_CUDA(&scene3, &scene4, tolerance));
	BOOST_REQUIRE(contentAlmostEqual_CUDA(&scene1, &scene3, tolerance));

	singleVoxelTests();
}

