//  ================================================================
//  Created by Gregory Kramida on 9/4/19.
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
#define BOOST_TEST_MODULE SDFfromImage
#ifndef WIN32
#define BOOST_TEST_DYN_LINK
#endif

//boost
#include <boost/test/unit_test.hpp>

//ITMLib
#include "TestUtilsForSnoopyFrames16And17.h"
#include "../ITMLib/GlobalTemplateDefines.h"
#include "../ITMLib/Engines/DepthFusion/DepthFusionEngine.h"
#include "../ITMLib/Engines/DepthFusion/DepthFusionEngineFactory.h"
#include "../ITMLib/Engines/ViewBuilding/Interface/ViewBuilder.h"
#include "../ITMLib/Engines/ViewBuilding/ViewBuilderFactory.h"
#include "../ITMLib/Engines/EditAndCopy/EditAndCopyEngineFactory.h"
#include "../ITMLib/Engines/Indexing/Interface/IndexingEngine.h"
#include "../ITMLib/Engines/Indexing/VBH/CPU/IndexingEngine_CPU_VoxelBlockHash.h"
#include "../ITMLib/Utils/Configuration.h"
#include "../ITMLib/Utils/Analytics/AlmostEqual.h"
#include "../ITMLib/Utils/Analytics/VoxelVolumeComparison/VoxelVolumeComparison_CPU.h"
#include "../ORUtils/FileUtils.h"
#include "../ITMLib/Engines/VolumeFileIO/VolumeFileIOEngine.h"
#include "../ITMLib/Utils/Analytics/VolumeStatisticsCalculator/CPU/VolumeStatisticsCalculator_CPU.h"
#include "TestUtils.h"
#include "../ITMLib/Engines/DepthFusion/DepthFusionEngineFactory.h"

using namespace ITMLib;

typedef VolumeFileIOEngine<TSDFVoxel, PlainVoxelArray> SceneFileIOEngine_PVA;
typedef VolumeFileIOEngine<TSDFVoxel, VoxelBlockHash> SceneFileIOEngine_VBH;

//#define SAVE_TEST_DATA
BOOST_FIXTURE_TEST_CASE(Test_SceneConstruct16_PVA_VBH_CPU, Frame16And17Fixture) {

	VoxelVolume<TSDFVoxel, PlainVoxelArray>* volume_PVA_16;
	buildSdfVolumeFromImage_NearSurfaceAllocation(&volume_PVA_16, "TestData/snoopy_depth_000016.png",
	                                              "TestData/snoopy_color_000016.png",
	                                              "TestData/snoopy_omask_000016.png",
	                                              "TestData/snoopy_calib.txt", MEMORYDEVICE_CPU,
	                                              InitParams<PlainVoxelArray>());

	VoxelVolume<TSDFVoxel, VoxelBlockHash>* volume_VBH_16;
	buildSdfVolumeFromImage_NearSurfaceAllocation(&volume_VBH_16, "TestData/snoopy_depth_000016.png",
	                                              "TestData/snoopy_color_000016.png",
	                                              "TestData/snoopy_omask_000016.png",
	                                              "TestData/snoopy_calib.txt", MEMORYDEVICE_CPU,
	                                              InitParams<VoxelBlockHash>());
//	Vector3i test_pos = Vector3i(-57, -9, 196);
//	TSDFVoxel voxelPVA = ManipulationEngine_CPU_PVA_Voxel::Inst().ReadVoxel(volume_PVA_16, test_pos);
//	voxelPVA.print_self();
//	TSDFVoxel voxelVBH = ManipulationEngine_CPU_VBH_Voxel::Inst().ReadVoxel(volume_VBH_16, test_pos);
//	voxelVBH.print_self();

#ifdef SAVE_TEST_DATA
	std::string path_PVA = "TestData/snoopy_result_fr16-17_partial_PVA/snoopy_partial_frame_16_";
	volume_PVA_16->SaveToDirectory(std::string("../../Tests/") +path_PVA);
	std::string path_VBH = "TestData/snoopy_result_fr16-17_partial_VBH/snoopy_partial_frame_16_";
	volume_VBH_16->SaveToDirectory(std::string("../../Tests/") +path_VBH);
#endif

	float absoluteTolerance = 1e-7;
	BOOST_REQUIRE(allocatedContentAlmostEqual_CPU(volume_PVA_16, volume_VBH_16, absoluteTolerance));
	BOOST_REQUIRE(contentForFlagsAlmostEqual_CPU(volume_PVA_16, volume_VBH_16, VoxelFlags::VOXEL_NONTRUNCATED,
	                                             absoluteTolerance));

	delete volume_VBH_16;
	delete volume_PVA_16;
}

//#define SAVE_TEST_DATA
BOOST_FIXTURE_TEST_CASE(Test_SceneConstruct17_PVA_VBH_CPU, Frame16And17Fixture) {

	VoxelVolume<TSDFVoxel, PlainVoxelArray>* volume_PVA_17;
	buildSdfVolumeFromImage_NearSurfaceAllocation(&volume_PVA_17, "TestData/snoopy_depth_000017.png",
	                                              "TestData/snoopy_color_000017.png",
	                                              "TestData/snoopy_omask_000017.png",
	                                              "TestData/snoopy_calib.txt", MEMORYDEVICE_CPU,
	                                              InitParams<PlainVoxelArray>());

	VoxelVolume<TSDFVoxel, VoxelBlockHash>* volume_VBH_17;
	buildSdfVolumeFromImage_NearSurfaceAllocation(&volume_VBH_17, "TestData/snoopy_depth_000017.png",
	                                              "TestData/snoopy_color_000017.png",
	                                              "TestData/snoopy_omask_000017.png",
	                                              "TestData/snoopy_calib.txt", MEMORYDEVICE_CPU,
	                                              InitParams<VoxelBlockHash>());

//	Vector3i voxelPosition(-57, -9, 196);
//	TSDFVoxel voxelPVA = ManipulationEngine_CPU_PVA_Voxel::Inst().ReadVoxel(volume_PVA_17, voxelPosition);
//	voxelPVA.print_self();
//	TSDFVoxel voxelVBH = ManipulationEngine_CPU_VBH_Voxel::Inst().ReadVoxel(volume_VBH_17, voxelPosition);
//	voxelVBH.print_self();

#ifdef SAVE_TEST_DATA
	std::string path_PVA = "TestData/snoopy_result_fr16-17_partial_PVA/snoopy_partial_frame_17_";
	volume_PVA_17->SaveToDirectory(std::string("../../Tests/") +path_PVA);
	std::string path_VBH = "TestData/snoopy_result_fr16-17_partial_VBH/snoopy_partial_frame_17_";
	volume_VBH_17->SaveToDirectory(std::string("../../Tests/") +path_VBH);
#endif

	float absoluteTolerance = 1e-7;
	BOOST_REQUIRE(allocatedContentAlmostEqual_CPU(volume_PVA_17, volume_VBH_17, absoluteTolerance));
	BOOST_REQUIRE(contentForFlagsAlmostEqual_CPU(volume_PVA_17, volume_VBH_17, VoxelFlags::VOXEL_NONTRUNCATED,
	                                             absoluteTolerance));

	delete volume_VBH_17;
	delete volume_PVA_17;
}


//#define SAVE_TEST_DATA
BOOST_FIXTURE_TEST_CASE(Test_SceneConstruct17_PVA_VBH_Expnaded_CPU, Frame16And17Fixture) {

	ITMView* view = nullptr;
	updateView(&view, "TestData/snoopy_depth_000017.png",
	           "TestData/snoopy_color_000017.png", "TestData/snoopy_omask_000017.png",
	           "TestData/snoopy_calib.txt", MEMORYDEVICE_CPU);

	// *** construct volumes ***
	VoxelVolume<TSDFVoxel, PlainVoxelArray> volume_PVA_17(MEMORYDEVICE_CPU, InitParams<PlainVoxelArray>());
	EditAndCopyEngineFactory::Instance<TSDFVoxel, PlainVoxelArray, MEMORYDEVICE_CPU>().ResetVolume(&volume_PVA_17);
	DepthFusionEngineInterface<TSDFVoxel, WarpVoxel, PlainVoxelArray>* reconstructionEngine_PVA =
			DepthFusionEngineFactory::Build<TSDFVoxel, WarpVoxel, PlainVoxelArray>(MEMORYDEVICE_CPU);
	reconstructionEngine_PVA->GenerateTsdfVolumeFromView(&volume_PVA_17, view);


	VoxelVolume<TSDFVoxel, VoxelBlockHash> volume_VBH_17(MEMORYDEVICE_CPU, InitParams<VoxelBlockHash>());
	EditAndCopyEngineFactory::Instance<TSDFVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>().ResetVolume(&volume_VBH_17);
	VoxelVolume<TSDFVoxel, VoxelBlockHash> volume_VBH_17_depth_allocation(MEMORYDEVICE_CPU,
	                                                                      InitParams<VoxelBlockHash>());
	EditAndCopyEngineFactory::Instance<TSDFVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>().ResetVolume(
			&volume_VBH_17_depth_allocation);

	IndexingEngine<TSDFVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>& indexer =
			IndexingEngine<TSDFVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>::Instance();
	indexer.AllocateNearSurface(&volume_VBH_17_depth_allocation, view);
	indexer.AllocateUsingOtherVolumeAndSetVisibilityExpanded(&volume_VBH_17, &volume_VBH_17_depth_allocation, view);

	DepthFusionEngineInterface<TSDFVoxel, WarpVoxel, VoxelBlockHash>* reconstructionEngine_VBH =
			DepthFusionEngineFactory::Build<TSDFVoxel, WarpVoxel, VoxelBlockHash>(MEMORYDEVICE_CPU);
	reconstructionEngine_VBH->IntegrateDepthImageIntoTsdfVolume(&volume_VBH_17, view);
	reconstructionEngine_VBH->IntegrateDepthImageIntoTsdfVolume(&volume_VBH_17_depth_allocation, view);


	float absoluteTolerance = 1e-7;
	BOOST_REQUIRE(allocatedContentAlmostEqual_CPU_Verbose(&volume_PVA_17, &volume_VBH_17_depth_allocation,
	                                                      absoluteTolerance));
	BOOST_REQUIRE(contentForFlagsAlmostEqual_CPU_Verbose(&volume_PVA_17, &volume_VBH_17_depth_allocation,
	                                                     VoxelFlags::VOXEL_NONTRUNCATED,
	                                                     absoluteTolerance));

#ifdef SAVE_TEST_DATA
	std::string path_VBH = "TestData/snoopy_result_fr16-17_partial_VBH/snoopy_partial_frame_17_expanded_";
	volume_VBH_17.SaveToDirectory(std::string("../../Tests/") + path_VBH);
#endif

	BOOST_REQUIRE(allocatedContentAlmostEqual_CPU_Verbose(&volume_PVA_17, &volume_VBH_17, absoluteTolerance));
	BOOST_REQUIRE(contentForFlagsAlmostEqual_CPU_Verbose(&volume_PVA_17, &volume_VBH_17, VoxelFlags::VOXEL_NONTRUNCATED,
	                                                     absoluteTolerance));

	delete reconstructionEngine_PVA;
	delete reconstructionEngine_VBH;
	delete view;
}


BOOST_AUTO_TEST_CASE(testConstructVoxelVolumeFromImage_CPU) {
	// region ================================= CONSTRUCT VIEW =========================================================

	RGBDCalib calibrationData;
	readRGBDCalib("TestData/snoopy_calib.txt", calibrationData);

	ViewBuilder* viewBuilder = ViewBuilderFactory::MakeViewBuilder(calibrationData, MEMORYDEVICE_CPU);
	Vector2i imageSize(640, 480);
	ITMView* view = nullptr;

	auto* rgb = new ITMUChar4Image(true, false);
	auto* depth = new ITMShortImage(true, false);
	BOOST_REQUIRE(ReadImageFromFile(rgb, "TestData/stripes_color.png"));
	BOOST_REQUIRE(ReadImageFromFile(depth, "TestData/stripes_depth.png"));

	viewBuilder->UpdateView(&view, rgb, depth, false, false, false, true);

	// endregion =======================================================================================================

	Vector3i volumeSize(1024, 32, 1024), volumeOffset(-volumeSize.x / 2, -volumeSize.y / 2, 0);

	VoxelVolume<TSDFVoxel, PlainVoxelArray> scene1(&configuration::get().general_voxel_volume_parameters,
	                                               configuration::get().swapping_mode ==
	                                               configuration::SWAPPINGMODE_ENABLED,
	                                               MEMORYDEVICE_CPU, {volumeSize, volumeOffset});
	ManipulationEngine_CPU_PVA_Voxel::Inst().ResetVolume(&scene1);
	CameraTrackingState trackingState(imageSize, MEMORYDEVICE_CPU);

	DepthFusionEngineInterface<TSDFVoxel, WarpVoxel, PlainVoxelArray>* reconstructionEngine_PVA =
			DepthFusionEngineFactory
			::Build<TSDFVoxel, WarpVoxel, PlainVoxelArray>(MEMORYDEVICE_CPU);
	reconstructionEngine_PVA->GenerateTsdfVolumeFromView(&scene1, view, &trackingState);

	const int num_stripes = 62;

	//These hardcoded values were precomputed mathematically based on the generated stripe images
	int zero_level_set_voxel_x_coords_mm[] = {-142, -160, -176, -191, -205, -217, -227, -236, -244, -250, -254,
	                                          -256, -258, -257, -255, -252, -247, -240, -232, -222, -211, -198,
	                                          -184, -168, -151, -132, -111, -89, -66, -41, -14, 14, 44,
	                                          75, 108, 143, 179, 216, 255, 296, 338, 382, 427, 474,
	                                          522, 572, 624, 677, 731, 787, 845, 904, 965, 1027, 1091,
	                                          1156, 1223, 1292, 1362, 1433, 1506, 1581};
	int zero_level_set_voxel_z_coords_mm[] = {240, 280, 320, 360, 400, 440, 480, 520, 560, 600, 640,
	                                          680, 720, 760, 800, 840, 880, 920, 960, 1000, 1040, 1080,
	                                          1120, 1160, 1200, 1240, 1280, 1320, 1360, 1400, 1440, 1480, 1520,
	                                          1560, 1600, 1640, 1680, 1720, 1760, 1800, 1840, 1880, 1920, 1960,
	                                          2000, 2040, 2080, 2120, 2160, 2200, 2240, 2280, 2320, 2360, 2400,
	                                          2440, 2480, 2520, 2560, 2600, 2640, 2680};

	std::vector<Vector3i> zeroLevelSetCoords;
	auto getVoxelCoord = [](Vector3f coordinateMeters, float voxelSize) {
		return TO_INT_ROUND3(coordinateMeters / voxelSize);
	};
	for (int iVoxel = 0; iVoxel < num_stripes; iVoxel++) {
		Vector3f coordinateMeters(
				static_cast<float>(zero_level_set_voxel_x_coords_mm[iVoxel]) / 1000.0f,
				0.0f,
				static_cast<float>(zero_level_set_voxel_z_coords_mm[iVoxel]) / 1000.0f
		);
		zeroLevelSetCoords.push_back(
				getVoxelCoord(coordinateMeters, configuration::get().general_voxel_volume_parameters.voxel_size));
	}

	float tolerance = 1e-4;
	int narrowBandHalfwidthVoxels = static_cast<int>(std::round(
			scene1.sceneParams->narrow_band_half_width / scene1.sceneParams->voxel_size));
	float maxSdfStep = 1.0f / narrowBandHalfwidthVoxels;

	// check constructed scene integrity
	for (int iCoord = 0; iCoord < zeroLevelSetCoords.size(); iCoord++) {
		Vector3i coord = zeroLevelSetCoords[iCoord];
		TSDFVoxel voxel = ManipulationEngine_CPU_PVA_Voxel::Inst().ReadVoxel(&scene1, coord);
		float sdf = TSDFVoxel::valueToFloat(voxel.sdf);
		BOOST_REQUIRE(almostEqual(sdf, 0.0f, tolerance));

		// for extremely lateral points close to the camera, the rays are highly skewed,
		// so the value progression won't hold. Skip those.
		if (iCoord > 17) {
			// don't go into low negative sdf values, since those will be overwritten by positive values
			// during sdf construction in certain cases
			for (int iLevelSet = -narrowBandHalfwidthVoxels; iLevelSet < (narrowBandHalfwidthVoxels / 2); iLevelSet++) {
				Vector3i augmentedCoord(coord.x, coord.y, coord.z + iLevelSet);
				float expectedSdf = -static_cast<float>(iLevelSet) * maxSdfStep;
				voxel = ManipulationEngine_CPU_PVA_Voxel::Inst().ReadVoxel(&scene1, augmentedCoord);
				sdf = TSDFVoxel::valueToFloat(voxel.sdf);
				BOOST_REQUIRE(almostEqual(sdf, expectedSdf, tolerance));
			}
		}
	}

	VoxelVolume<TSDFVoxel, VoxelBlockHash> scene2(&configuration::get().general_voxel_volume_parameters,
	                                              configuration::get().swapping_mode ==
	                                              configuration::SWAPPINGMODE_ENABLED,
	                                              MEMORYDEVICE_CPU);
	ManipulationEngine_CPU_VBH_Voxel::Inst().ResetVolume(&scene2);

	DepthFusionEngineInterface<TSDFVoxel, WarpVoxel, VoxelBlockHash>* reconstructionEngine_VBH =
			DepthFusionEngineFactory
			::Build<TSDFVoxel, WarpVoxel, VoxelBlockHash>(MEMORYDEVICE_CPU);

	reconstructionEngine_VBH->GenerateTsdfVolumeFromView(&scene2, view, &trackingState);

	tolerance = 1e-5;
	BOOST_REQUIRE(allocatedContentAlmostEqual_CPU(&scene1, &scene2, tolerance));
	VoxelVolume<TSDFVoxel, PlainVoxelArray> scene3(&configuration::get().general_voxel_volume_parameters,
	                                               configuration::get().swapping_mode ==
	                                               configuration::SWAPPINGMODE_ENABLED,
	                                               MEMORYDEVICE_CPU, {volumeSize, volumeOffset});
	ManipulationEngine_CPU_PVA_Voxel::Inst().ResetVolume(&scene3);
	reconstructionEngine_PVA->GenerateTsdfVolumeFromView(&scene3, view, &trackingState);
	BOOST_REQUIRE(contentAlmostEqual_CPU(&scene1, &scene3, tolerance));
	VoxelVolume<TSDFVoxel, VoxelBlockHash> scene4(&configuration::get().general_voxel_volume_parameters,
	                                              configuration::get().swapping_mode ==
	                                              configuration::SWAPPINGMODE_ENABLED,
	                                              MEMORYDEVICE_CPU, {0x800, 0x20000});
	ManipulationEngine_CPU_VBH_Voxel::Inst().ResetVolume(&scene4);
	reconstructionEngine_VBH->GenerateTsdfVolumeFromView(&scene4, view, &trackingState);
	BOOST_REQUIRE(contentAlmostEqual_CPU(&scene2, &scene4, tolerance));

	Vector3i coordinate = zeroLevelSetCoords[0];
	TSDFVoxel voxel = ManipulationEngine_CPU_PVA_Voxel::Inst().ReadVoxel(&scene3, coordinate);
	voxel.sdf = TSDFVoxel::floatToValue(TSDFVoxel::valueToFloat(voxel.sdf) + 0.05f);
	ManipulationEngine_CPU_PVA_Voxel::Inst().SetVoxel(&scene3, coordinate, voxel);
	ManipulationEngine_CPU_VBH_Voxel::Inst().SetVoxel(&scene2, coordinate, voxel);

	BOOST_REQUIRE(!contentAlmostEqual_CPU(&scene1, &scene3, tolerance));
	BOOST_REQUIRE(!contentAlmostEqual_CPU(&scene2, &scene4, tolerance));
	BOOST_REQUIRE(!allocatedContentAlmostEqual_CPU(&scene1, &scene2, tolerance));

	VoxelVolume<TSDFVoxel, VoxelBlockHash> scene5(
			&configuration::get().general_voxel_volume_parameters,
			configuration::get().swapping_mode == configuration::SWAPPINGMODE_ENABLED,
			MEMORYDEVICE_CPU);
	ManipulationEngine_CPU_VBH_Voxel::Inst().ResetVolume(&scene5);
	std::string path = "TestData/test_VBH_ConstructFromImage_";
	SceneFileIOEngine_VBH::SaveToDirectoryCompact(&scene4, path);
	SceneFileIOEngine_VBH::LoadFromDirectoryCompact(&scene5, path);
	BOOST_REQUIRE(allocatedContentAlmostEqual_CPU(&scene1, &scene5, tolerance));
	BOOST_REQUIRE(contentAlmostEqual_CPU(&scene4, &scene5, tolerance));

	delete view;
	delete reconstructionEngine_PVA;
	delete reconstructionEngine_VBH;
	delete viewBuilder;
	delete rgb;
	delete depth;
}

BOOST_AUTO_TEST_CASE(testConstructVoxelVolumeFromImage2_CPU) {
	// region ================================= CONSTRUCT VIEW =========================================================

	RGBDCalib calibrationData;
	readRGBDCalib("TestData/snoopy_calib.txt", calibrationData);

	ViewBuilder* viewBuilder = ViewBuilderFactory::MakeViewBuilder(calibrationData, MEMORYDEVICE_CPU);
	Vector2i imageSize(640, 480);
	ITMView* view = nullptr;

	auto* rgb = new ITMUChar4Image(true, false);
	auto* depth = new ITMShortImage(true, false);
	BOOST_REQUIRE(ReadImageFromFile(rgb, "TestData/snoopy_color_000000.png"));
	BOOST_REQUIRE(ReadImageFromFile(depth, "TestData/snoopy_depth_000000.png"));

	viewBuilder->UpdateView(&view, rgb, depth, false, false, false, true);

	// endregion =======================================================================================================

	//Vector3i volumeSize(512, 512, 512), volumeOffset(-volumeSize.x / 2, -volumeSize.y / 2, 0);
	Vector3i volumeSize(512, 112, 360), volumeOffset(-512, -24, 152);
	VoxelVolume<TSDFVoxel, PlainVoxelArray> generated_volume(&configuration::get().general_voxel_volume_parameters,
	                                                         configuration::get().swapping_mode ==
	                                                         configuration::SWAPPINGMODE_ENABLED,
	                                                         MEMORYDEVICE_CPU, {volumeSize, volumeOffset});

	ManipulationEngine_CPU_PVA_Voxel::Inst().ResetVolume(&generated_volume);
	CameraTrackingState trackingState(imageSize, MEMORYDEVICE_CPU);

	DepthFusionEngineInterface<TSDFVoxel, WarpVoxel, PlainVoxelArray>* reconstructionEngine_PVA =
			DepthFusionEngineFactory
			::Build<TSDFVoxel, WarpVoxel, PlainVoxelArray>(MEMORYDEVICE_CPU);
	reconstructionEngine_PVA->GenerateTsdfVolumeFromView(&generated_volume, view, &trackingState);
	//generated_volume.SaveToDirectory("../../Tests/TestData/test_PVA_ConstructFromImage2_");

	VoxelVolume<TSDFVoxel, PlainVoxelArray> loaded_volume(&configuration::get().general_voxel_volume_parameters,
	                                                      configuration::get().swapping_mode ==
	                                                      configuration::SWAPPINGMODE_ENABLED,
	                                                      MEMORYDEVICE_CPU,
	                                                      {volumeSize, volumeOffset});

	std::string path = "TestData/test_PVA_ConstructFromImage2_";
	ManipulationEngine_CPU_PVA_Voxel::Inst().ResetVolume(&loaded_volume);
	loaded_volume.LoadFromDirectory(path);

	float tolerance = 1e-5;
	BOOST_REQUIRE(contentAlmostEqual_CPU_Verbose(&generated_volume, &loaded_volume, tolerance));

	VoxelVolume<TSDFVoxel, VoxelBlockHash> scene3(&configuration::get().general_voxel_volume_parameters,
	                                              configuration::get().swapping_mode ==
	                                              configuration::SWAPPINGMODE_ENABLED,
	                                              MEMORYDEVICE_CPU);
	ManipulationEngine_CPU_VBH_Voxel::Inst().ResetVolume(&scene3);

	DepthFusionEngineInterface<TSDFVoxel, WarpVoxel, VoxelBlockHash>* reconstructionEngine_VBH =
			DepthFusionEngineFactory
			::Build<TSDFVoxel, WarpVoxel, VoxelBlockHash>(MEMORYDEVICE_CPU);

	reconstructionEngine_VBH->GenerateTsdfVolumeFromView(&scene3, view, &trackingState);

	BOOST_REQUIRE(allocatedContentAlmostEqual_CPU(&loaded_volume, &scene3, tolerance));
	delete depth;
	delete rgb;
}