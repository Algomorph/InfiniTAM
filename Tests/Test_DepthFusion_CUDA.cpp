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
#define BOOST_TEST_MODULE DepthFusion_CUDA
#ifndef WIN32
#define BOOST_TEST_DYN_LINK
#endif

//boost
#include <boost/test/unit_test.hpp>

//ITMLib
#include "../ITMLib/GlobalTemplateDefines.h"
#include "../ITMLib/Engines/DepthFusion/DepthFusionEngine.h"
#include "../ITMLib/Engines/DepthFusion/DepthFusionEngineFactory.h"
#include "../ITMLib/Engines/ViewBuilder/Interface/ViewBuilder.h"
#include "../ITMLib/Engines/ViewBuilder/ViewBuilderFactory.h"
#include "../ITMLib/Engines/Indexing/Interface/IndexingEngine.h"
#include "../ITMLib/Engines/Indexing/IndexingEngineFactory.h"
#include "../ITMLib/Engines/Indexing/VBH/CUDA/IndexingEngine_VoxelBlockHash_CUDA.h"
#include "../ITMLib/Engines/VolumeFileIO/VolumeFileIOEngine.h"
#include "../ITMLib/Engines/Analytics/AnalyticsEngine.h"
#include "../ITMLib/Engines/EditAndCopy/EditAndCopyEngineFactory.h"
#include "../ITMLib/Engines/Rendering/RenderingEngineFactory.h"
#include "../ITMLib/Utils/Configuration/Configuration.h"
#include "../ITMLib/Utils/Analytics/AlmostEqual.h"
#include "../ITMLib/Utils/Analytics/VoxelVolumeComparison/VoxelVolumeComparison_CPU.h"
#include "../ITMLib/Utils/Analytics/VoxelVolumeComparison/VoxelVolumeComparison_CUDA.h"
#include "../ORUtils/FileUtils.h"

//test_utilities
#include "TestUtilities/TestUtilities.h"
#include "TestUtilities/TestDataUtilities.h"

using namespace ITMLib;
using namespace test;

typedef VolumeFileIOEngine<TSDFVoxel, PlainVoxelArray> SceneFileIOEngine_PVA;
typedef VolumeFileIOEngine<TSDFVoxel, VoxelBlockHash> SceneFileIOEngine_VBH;


BOOST_AUTO_TEST_CASE(Test_SceneConstruct16_PVA_VBH_Near_CUDA) {

	VoxelVolume<TSDFVoxel, PlainVoxelArray>* volume_PVA_16;
	BuildSdfVolumeFromImage_NearSurfaceAllocation(&volume_PVA_16,
	                                              std::string(test::snoopy::frame_16_depth_path),
	                                              std::string(test::snoopy::frame_16_color_path),
	                                              std::string(test::snoopy::frame_16_mask_path),
	                                              std::string(test::snoopy::calibration_path),
	                                              MEMORYDEVICE_CUDA,
	                                              test::snoopy::InitializationParameters_Fr16andFr17<PlainVoxelArray>());

	VoxelVolume<TSDFVoxel, VoxelBlockHash>* volume_VBH_16;
	BuildSdfVolumeFromImage_NearSurfaceAllocation(&volume_VBH_16,
	                                              std::string(test::snoopy::frame_16_depth_path),
	                                              std::string(test::snoopy::frame_16_color_path),
	                                              std::string(test::snoopy::frame_16_mask_path),
	                                              std::string(test::snoopy::calibration_path),
	                                              MEMORYDEVICE_CUDA,
	                                              test::snoopy::InitializationParameters_Fr16andFr17<VoxelBlockHash>());

	float absoluteTolerance = 1e-7;
	BOOST_REQUIRE(allocatedContentAlmostEqual_CUDA(volume_PVA_16, volume_VBH_16, absoluteTolerance));
	BOOST_REQUIRE(contentForFlagsAlmostEqual_CUDA(volume_PVA_16, volume_VBH_16, VoxelFlags::VOXEL_NONTRUNCATED,
	                                              absoluteTolerance));

	delete volume_VBH_16;
	delete volume_PVA_16;
}

BOOST_AUTO_TEST_CASE(Test_SceneConstruct17_PVA_VBH_Near_CUDA) {

	VoxelVolume<TSDFVoxel, PlainVoxelArray>* volume_PVA_17;
	BuildSdfVolumeFromImage_NearSurfaceAllocation(&volume_PVA_17,
	                                              std::string(test::snoopy::frame_17_depth_path),
	                                              std::string(test::snoopy::frame_17_color_path),
	                                              std::string(test::snoopy::frame_17_mask_path),
	                                              std::string(test::snoopy::calibration_path),
	                                              MEMORYDEVICE_CUDA,
	                                              test::snoopy::InitializationParameters_Fr16andFr17<PlainVoxelArray>());

	VoxelVolume<TSDFVoxel, VoxelBlockHash>* volume_VBH_17;
	BuildSdfVolumeFromImage_NearSurfaceAllocation(&volume_VBH_17,
	                                              std::string(test::snoopy::frame_17_depth_path),
	                                              std::string(test::snoopy::frame_17_color_path),
	                                              std::string(test::snoopy::frame_17_mask_path),
	                                              std::string(test::snoopy::calibration_path),
	                                              MEMORYDEVICE_CUDA,
	                                              test::snoopy::InitializationParameters_Fr16andFr17<VoxelBlockHash>());

//	Vector3i voxelPosition(-24, -2, 87);
//	TSDFVoxel voxelPVA = ManipulationEngine_CUDA_PVA_Voxel::Inst().ReadVoxel(volume_PVA_17, voxelPosition);
//	voxelPVA.print_self();
//	TSDFVoxel voxelVBH = ManipulationEngine_CUDA_VBH_Voxel::Inst().ReadVoxel(volume_VBH_17, voxelPosition);
//	voxelVBH.print_self();

	float absoluteTolerance = 1e-7;
	BOOST_REQUIRE(allocatedContentAlmostEqual_CUDA(volume_PVA_17, volume_VBH_17, absoluteTolerance));
	BOOST_REQUIRE(contentForFlagsAlmostEqual_CUDA(volume_PVA_17, volume_VBH_17, VoxelFlags::VOXEL_NONTRUNCATED,
	                                              absoluteTolerance));

	delete volume_VBH_17;
	delete volume_PVA_17;
}


BOOST_AUTO_TEST_CASE(testConstructVoxelVolumeFromImage_CUDA) {
	configuration::Configuration* settings = &configuration::Get();

	// region ================================= CONSTRUCT VIEW =========================================================

	RGBD_CalibrationInformation calibrationData;
	readRGBDCalib(std::string(test::snoopy::calibration_path).c_str(), calibrationData);

	ViewBuilder* viewBuilder = ViewBuilderFactory::Build(calibrationData, MEMORYDEVICE_CUDA);
	Vector2i imageSize(640, 480);
	View* view = nullptr;

	UChar4Image rgb(true, true);
	ShortImage depth(true, true);
	BOOST_REQUIRE(ReadImageFromFile(rgb, STATIC_TEST_DATA_PREFIX "TestData/frames/stripes_color.png"));
	BOOST_REQUIRE(ReadImageFromFile(depth, STATIC_TEST_DATA_PREFIX "TestData/frames/stripes_depth.png"));
	rgb.UpdateDeviceFromHost();
	depth.UpdateDeviceFromHost();

	viewBuilder->UpdateView(&view, &rgb, &depth, false, false, false, true);

	// endregion =======================================================================================================

	Vector3i volumeSize(1024, 32, 1024), volumeOffset(-volumeSize.x / 2, -volumeSize.y / 2, 0);

	VoxelVolume<TSDFVoxel, PlainVoxelArray> volume1(MEMORYDEVICE_CUDA, {volumeSize, volumeOffset});
	ManipulationEngine_CUDA_PVA_Voxel::Inst().ResetVolume(&volume1);
	CameraTrackingState trackingState(imageSize, settings->device_type);

	DepthFusionEngineInterface<TSDFVoxel, PlainVoxelArray>* depth_fusion_engine_PVA =
			DepthFusionEngineFactory::Build<TSDFVoxel, PlainVoxelArray>(MEMORYDEVICE_CUDA);
	IndexingEngineInterface<TSDFVoxel, PlainVoxelArray>& indexer_PVA
			= IndexingEngineFactory::GetDefault<TSDFVoxel, PlainVoxelArray>(MEMORYDEVICE_CUDA);
	indexer_PVA.AllocateNearAndBetweenTwoSurfaces(&volume1, view, &trackingState);
	depth_fusion_engine_PVA->IntegrateDepthImageIntoTsdfVolume(&volume1, view, &trackingState);

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

	std::vector<Vector3i> zero_level_set_coordinates;
	auto getVoxelCoord = [](Vector3f coordinateMeters, float voxelSize) {
		return TO_INT_ROUND3(coordinateMeters / voxelSize);
	};
	for (int iVoxel = 0; iVoxel < num_stripes; iVoxel++) {
		Vector3f coordinateMeters(
				static_cast<float>(zero_level_set_voxel_x_coords_mm[iVoxel]) / 1000.0f,
				0.0f,
				static_cast<float>(zero_level_set_voxel_z_coords_mm[iVoxel]) / 1000.0f
		);
		zero_level_set_coordinates.push_back(
				getVoxelCoord(coordinateMeters, settings->general_voxel_volume_parameters.voxel_size));
	}

	float tolerance = 1e-4;
	int truncation_distance_voxels = static_cast<int>(std::round(
			volume1.GetParameters().truncation_distance / volume1.GetParameters().voxel_size));
	float max_SDF_step = 1.0f / truncation_distance_voxels;

	// check constructed scene integrity
	Vector3i bad_coordinate;
	float bad_sdf, bad_expected_sdf;
	bool unexpected_sdf_at_level_set = false;
	int i_bad_level_set;

	for (int i_coordinate = 0; i_coordinate < zero_level_set_coordinates.size(); i_coordinate++) {
		if (unexpected_sdf_at_level_set) break;
		Vector3i coord = zero_level_set_coordinates[i_coordinate];
		TSDFVoxel voxel = ManipulationEngine_CUDA_PVA_Voxel::Inst().ReadVoxel(&volume1, coord);
		float sdf = TSDFVoxel::valueToFloat(voxel.sdf);
		if (!AlmostEqual(sdf, 0.0f, tolerance)) {
			unexpected_sdf_at_level_set = true;
			i_bad_level_set = 0;
			bad_sdf = sdf;
			bad_expected_sdf = 0.0f;
			bad_coordinate = coord;
			break;
		}

		// for extremely lateral points Close to the camera, the rays are highly skewed,
		// so the value progression won't hold. Skip those.
		if (i_coordinate > 17) {
			// don't go into low negative sdf values, since those will be overwritten by positive values
			// during sdf construction in certain cases
			for (int i_level_set = -truncation_distance_voxels;
			     i_level_set < (truncation_distance_voxels / 2); i_level_set++) {
				Vector3i augmented_coord(coord.x, coord.y, coord.z + i_level_set);
				float expected_sdf = -static_cast<float>(i_level_set) * max_SDF_step;
				voxel = ManipulationEngine_CUDA_PVA_Voxel::Inst().ReadVoxel(&volume1, augmented_coord);
				sdf = TSDFVoxel::valueToFloat(voxel.sdf);
				if (!AlmostEqual(sdf, expected_sdf, tolerance)) {
					unexpected_sdf_at_level_set = true;
					i_bad_level_set = i_level_set;
					bad_coordinate = augmented_coord;
					bad_sdf = sdf;
					bad_expected_sdf = expected_sdf;
					break;
				}
			}
		}
	}
	BOOST_REQUIRE_MESSAGE(!unexpected_sdf_at_level_set, "Expected sdf " << bad_expected_sdf << " for voxel " \
 << bad_coordinate << " at level set " << i_bad_level_set << ", got: " << bad_sdf);

	VoxelVolume<TSDFVoxel, VoxelBlockHash> volume2(MEMORYDEVICE_CUDA, {0x800, 0x20000});
	ManipulationEngine_CUDA_VBH_Voxel::Inst().ResetVolume(&volume2);

	DepthFusionEngineInterface<TSDFVoxel, VoxelBlockHash>* depth_fusion_engine_VBH =
			DepthFusionEngineFactory::Build<TSDFVoxel, VoxelBlockHash>(MEMORYDEVICE_CUDA);

	RenderState renderState(imageSize, configuration::Get().general_voxel_volume_parameters.near_clipping_distance,
	                        configuration::Get().general_voxel_volume_parameters.far_clipping_distance, settings->device_type);
	IndexingEngineInterface<TSDFVoxel, VoxelBlockHash>& indexer_VBH
			= IndexingEngineFactory::GetDefault<TSDFVoxel, VoxelBlockHash>(MEMORYDEVICE_CUDA);
	indexer_VBH.AllocateNearAndBetweenTwoSurfaces(&volume2, view, &trackingState);
	depth_fusion_engine_VBH->IntegrateDepthImageIntoTsdfVolume(&volume2, view, &trackingState);

	tolerance = 1e-5;
	BOOST_REQUIRE(allocatedContentAlmostEqual_CUDA(&volume1, &volume2, tolerance));
	VoxelVolume<TSDFVoxel, PlainVoxelArray> volume3(MEMORYDEVICE_CUDA,
	                                                {volumeSize, volumeOffset});
	ManipulationEngine_CUDA_PVA_Voxel::Inst().ResetVolume(&volume3);
	indexer_PVA.AllocateNearAndBetweenTwoSurfaces(&volume3, view, &trackingState);
	depth_fusion_engine_PVA->IntegrateDepthImageIntoTsdfVolume(&volume3, view, &trackingState);
	BOOST_REQUIRE(contentAlmostEqual_CUDA(&volume1, &volume3, tolerance));
	VoxelVolume<TSDFVoxel, VoxelBlockHash> volume4(settings->device_type, {0x800, 0x20000});
	ManipulationEngine_CUDA_VBH_Voxel::Inst().ResetVolume(&volume4);
	indexer_VBH.AllocateNearAndBetweenTwoSurfaces(&volume4, view, &trackingState);
	depth_fusion_engine_VBH->IntegrateDepthImageIntoTsdfVolume(&volume4, view, &trackingState);
	BOOST_REQUIRE(contentAlmostEqual_CUDA(&volume2, &volume4, tolerance));

	Vector3i coordinate = zero_level_set_coordinates[0];
	TSDFVoxel voxel = ManipulationEngine_CUDA_PVA_Voxel::Inst().ReadVoxel(&volume3, coordinate);
	voxel.sdf = TSDFVoxel::floatToValue(TSDFVoxel::valueToFloat(voxel.sdf) + 0.05f);
	ManipulationEngine_CUDA_PVA_Voxel::Inst().SetVoxel(&volume3, coordinate, voxel);
	ManipulationEngine_CUDA_VBH_Voxel::Inst().SetVoxel(&volume2, coordinate, voxel);

	BOOST_REQUIRE(!contentAlmostEqual_CUDA(&volume1, &volume3, tolerance));
	BOOST_REQUIRE(!contentAlmostEqual_CUDA(&volume2, &volume4, tolerance));
	BOOST_REQUIRE(!allocatedContentAlmostEqual_CUDA(&volume1, &volume2, tolerance));

	VoxelVolume<TSDFVoxel, VoxelBlockHash> volume5(settings->device_type, {0x800, 0x20000});
	ManipulationEngine_CUDA_VBH_Voxel::Inst().ResetVolume(&volume5);
	std::string path = GENERATED_TEST_DATA_PREFIX "TestData/volumes/VBH/stripes_CUDA.dat";
	volume4.SaveToDisk(path);
	volume5.LoadFromDisk(path);
	BOOST_REQUIRE(allocatedContentAlmostEqual_CUDA(&volume1, &volume5, tolerance));
	BOOST_REQUIRE(contentAlmostEqual_CUDA(&volume4, &volume5, tolerance));

	delete view;
	delete depth_fusion_engine_PVA;
	delete depth_fusion_engine_VBH;
	delete viewBuilder;
}

BOOST_AUTO_TEST_CASE(testConstructVoxelVolumeFromImage2_CUDA) {
	configuration::Configuration* settings = &configuration::Get();

	// region ================================= CONSTRUCT VIEW =========================================================

	RGBD_CalibrationInformation calibration_data;
	readRGBDCalib(std::string(test::snoopy::calibration_path).c_str(), calibration_data);

	ViewBuilder* view_builder = ViewBuilderFactory::Build(calibration_data, MEMORYDEVICE_CUDA);
	Vector2i image_size(640, 480);
	View* view = nullptr;

	UChar4Image rgb(true, false);
	ShortImage depth(true, false);
	BOOST_REQUIRE(ReadImageFromFile(rgb, std::string(test::snoopy::frame_00_color_path).c_str()));
	BOOST_REQUIRE(ReadImageFromFile(depth, std::string(test::snoopy::frame_00_depth_path).c_str()));

	view_builder->UpdateView(&view, &rgb, &depth, false, false, false, true);

	// endregion =======================================================================================================

	Vector3i volume_size(512, 112, 360), volume_offset(-512, -24, 152);
	VoxelVolume<TSDFVoxel, PlainVoxelArray> volume2(settings->device_type, {volume_size, volume_offset});
	std::string path = test::snoopy::PartialVolume00Path<PlainVoxelArray>();
	volume2.Reset();
	volume2.LoadFromDisk(path);
	CameraTrackingState trackingState(image_size, settings->device_type);

	float tolerance = 1e-5;
	{
		VoxelVolume<TSDFVoxel, PlainVoxelArray> volume1(settings->device_type, {volume_size, volume_offset});

		ManipulationEngine_CUDA_PVA_Voxel::Inst().ResetVolume(&volume1);


		DepthFusionEngineInterface<TSDFVoxel, PlainVoxelArray>* depth_fusion_engine_PVA =
				DepthFusionEngineFactory::Build<TSDFVoxel, PlainVoxelArray>(MEMORYDEVICE_CUDA);
		IndexingEngineInterface<TSDFVoxel, PlainVoxelArray>& indexer_PVA
				= IndexingEngineFactory::GetDefault<TSDFVoxel, PlainVoxelArray>(MEMORYDEVICE_CUDA);
		indexer_PVA.AllocateNearAndBetweenTwoSurfaces(&volume1, view, &trackingState);
		depth_fusion_engine_PVA->IntegrateDepthImageIntoTsdfVolume(&volume1, view, &trackingState);

		BOOST_REQUIRE(contentAlmostEqual_CUDA(&volume1, &volume2, tolerance));
		delete depth_fusion_engine_PVA;
	}

	VoxelVolume<TSDFVoxel, VoxelBlockHash> volume3(settings->device_type);
	volume3.Reset();

	DepthFusionEngineInterface<TSDFVoxel, VoxelBlockHash>* depth_fusion_engine_VBH =
			DepthFusionEngineFactory::Build<TSDFVoxel, VoxelBlockHash>(MEMORYDEVICE_CUDA);

	IndexingEngineInterface<TSDFVoxel, VoxelBlockHash>& indexer_VBH
			= IndexingEngineFactory::GetDefault<TSDFVoxel, VoxelBlockHash>(MEMORYDEVICE_CUDA);
	indexer_VBH.AllocateNearAndBetweenTwoSurfaces(&volume3, view, &trackingState);
	depth_fusion_engine_VBH->IntegrateDepthImageIntoTsdfVolume(&volume3, view, &trackingState);

	BOOST_REQUIRE(allocatedContentAlmostEqual_CUDA(&volume2, &volume3, tolerance));

	delete depth_fusion_engine_VBH;
	delete view;
	delete view_builder;
}
