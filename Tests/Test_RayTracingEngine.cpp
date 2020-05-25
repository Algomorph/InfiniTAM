//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 5/20/20.
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
#define BOOST_TEST_MODULE RayTracingEngine
#ifndef WIN32
#define BOOST_TEST_DYN_LINK
#endif

//boost
#include <boost/test/unit_test.hpp>

//test_utilities
#include "TestUtilities/TestUtilities.h"
#include "TestUtilities/SnoopyTestUtilities.h"
#include "TestUtilities/CameraPoseAndRayTracingEngineFixture.h"

//ORUtils
#include "../ORUtils/IStreamWrapper.h"

//ITMLib
#include "../ITMLib/Engines/Visualization/VisualizationEngineFactory.h"
#include "../ITMLib/Utils/Analytics/RawMemoryArrayComparison.h"
#include "../ITMLib/Utils/Collections/OperationsOnSTLContainers.h"


using namespace ITMLib;
using namespace test_utilities;
namespace snoopy = snoopy_test_utilities;

template<MemoryDeviceType TMemoryDeviceType>
void GenericFindAndCountVisibleBlocksTest() {
	ORUtils::IStreamWrapper visible_blocks_file("TestData/arrays/visible_blocks.dat", true);

	std::vector<int> block_address_range_bounds = ReadStdVectorFromFile<int>(visible_blocks_file);

	std::vector<int> pose_range_visible_block_counts1;
	std::vector<int> pose_range_visible_block_counts2;
	std::vector<int> pose_range_visible_block_counts3;

	CameraPoseAndRayTracingEngineFixture<TMemoryDeviceType> fixture;

	for (auto& pose : fixture.camera_poses) {
		VoxelVolume<TSDFVoxel, VoxelBlockHash>* volume;
		LoadVolume(&volume, snoopy::PartialVolume17Path<VoxelBlockHash>(), TMemoryDeviceType,
		           snoopy::InitializationParameters_Fr16andFr17<VoxelBlockHash>());

		fixture.visualization_engine->FindVisibleBlocks(volume, &pose, &fixture.calibration_data.intrinsics_d, fixture.render_state);
		int range_visible_block_count1 = fixture.visualization_engine->CountVisibleBlocks(volume, fixture.render_state, block_address_range_bounds[0],
		                                                                                  block_address_range_bounds[1]);
		int range_visible_block_count2 = fixture.visualization_engine->CountVisibleBlocks(volume, fixture.render_state, block_address_range_bounds[2],
		                                                                                  block_address_range_bounds[3]);
		int range_visible_block_count3 = fixture.visualization_engine->CountVisibleBlocks(volume, fixture.render_state, block_address_range_bounds[4],
		                                                                                  block_address_range_bounds[5]);

		pose_range_visible_block_counts1.push_back(range_visible_block_count1);
		pose_range_visible_block_counts2.push_back(range_visible_block_count2);
		pose_range_visible_block_counts3.push_back(range_visible_block_count3);

		const int visible_block_count = volume->index.GetVisibleBlockCount();
		if (visible_block_count == 0) continue; // skip shots without results

		int* visible_codes_device = volume->index.GetVisibleBlockHashCodes();
		ORUtils::MemoryBlock<int> visible_codes_ground_truth(visible_block_count, MEMORYDEVICE_CPU);
		ORUtils::MemoryBlockPersistence::LoadMemoryBlock(visible_blocks_file, visible_codes_ground_truth,
		                                                 MEMORYDEVICE_CPU);
		BOOST_REQUIRE(RawMemoryArraysEqual(visible_codes_ground_truth.GetData(MEMORYDEVICE_CPU), MEMORYDEVICE_CPU,
		                                   visible_codes_device, TMemoryDeviceType, visible_block_count));

		delete volume;
	}

	std::vector<int> pose_range_visible_block_counts1_ground_truth = ReadStdVectorFromFile<int>(visible_blocks_file);
	std::vector<int> pose_range_visible_block_counts2_ground_truth = ReadStdVectorFromFile<int>(visible_blocks_file);
	std::vector<int> pose_range_visible_block_counts3_ground_truth = ReadStdVectorFromFile<int>(visible_blocks_file);

	BOOST_REQUIRE_EQUAL(pose_range_visible_block_counts1, pose_range_visible_block_counts1_ground_truth);
	BOOST_REQUIRE_EQUAL(pose_range_visible_block_counts2, pose_range_visible_block_counts2_ground_truth);
	BOOST_REQUIRE_EQUAL(pose_range_visible_block_counts3, pose_range_visible_block_counts3_ground_truth);
}

template<MemoryDeviceType TMemoryDeviceType>
void GenericCreateExpectedDepthsTest() {
	CameraPoseAndRayTracingEngineFixture<TMemoryDeviceType> fixture;

	ORUtils::IStreamWrapper range_images_file("TestData/arrays/range_images.dat", true);

	for (auto& pose : fixture.camera_poses) {
		VoxelVolume<TSDFVoxel, VoxelBlockHash>* volume;
		LoadVolume(&volume, snoopy::PartialVolume17Path<VoxelBlockHash>(), TMemoryDeviceType,
		           snoopy::InitializationParameters_Fr16andFr17<VoxelBlockHash>());

		fixture.visualization_engine->FindVisibleBlocks(volume, &pose, &fixture.calibration_data.intrinsics_d, fixture.render_state);

		const int visible_block_count = volume->index.GetVisibleBlockCount();
		if (visible_block_count == 0) continue; // skip shots without results
		std::shared_ptr<RenderState> render_state = fixture.MakeRenderState();
		fixture.visualization_engine->CreateExpectedDepths(volume, &pose, &fixture.calibration_data.intrinsics_d, render_state.get());
		ORUtils::Image<Vector2f>& range_image = *render_state->renderingRangeImage;
		ORUtils::Image<Vector2f> range_image_ground_truth = ORUtils::MemoryBlockPersistence::LoadImage<Vector2f>(range_images_file, MEMORYDEVICE_CPU);
		BOOST_REQUIRE(RawMemoryArraysEqual(range_image_ground_truth.GetData(MEMORYDEVICE_CPU), MEMORYDEVICE_CPU,
		                                   range_image.GetData(TMemoryDeviceType), TMemoryDeviceType, range_image.size()));
		delete volume;
	}
}


template<MemoryDeviceType TMemoryDeviceType>
void GenericFindSurfaceTest() {
	CameraPoseAndRayTracingEngineFixture<TMemoryDeviceType> fixture;

	ORUtils::IStreamWrapper raycast_images_file("TestData/arrays/raycast_images.dat", true);

	for (auto& pose : fixture.camera_poses) {
		VoxelVolume<TSDFVoxel, VoxelBlockHash>* volume;
		LoadVolume(&volume, snoopy::PartialVolume17Path<VoxelBlockHash>(), TMemoryDeviceType,
		           snoopy::InitializationParameters_Fr16andFr17<VoxelBlockHash>());

		fixture.visualization_engine->FindVisibleBlocks(volume, &pose, &fixture.calibration_data.intrinsics_d, fixture.render_state);

		const int visible_block_count = volume->index.GetVisibleBlockCount();
		if (visible_block_count == 0) continue; // skip shots without results
		std::shared_ptr<RenderState> render_state = fixture.MakeRenderState();
		fixture.visualization_engine->FindSurface(volume, &pose, &fixture.calibration_data.intrinsics_d, render_state.get());
		ORUtils::Image<Vector4f>& raycast_image = *render_state->raycastResult;
		ORUtils::Image<Vector4f> raycast_image_ground_truth = ORUtils::MemoryBlockPersistence::LoadImage<Vector4f>(raycast_images_file, MEMORYDEVICE_CPU);
		BOOST_REQUIRE(RawMemoryArraysEqual(raycast_image_ground_truth.GetData(MEMORYDEVICE_CPU), MEMORYDEVICE_CPU,
		                                   raycast_image.GetData(TMemoryDeviceType), TMemoryDeviceType, raycast_image.size()));

		delete volume;
	}
}

template<MemoryDeviceType TMemoryDeviceType>
void GenericCreatePointCloudTest() {
	CameraPoseAndRayTracingEngineFixture<TMemoryDeviceType> fixture;

	ORUtils::IStreamWrapper point_cloud_images_file("TestData/arrays/point_cloud_images.dat", true);

	for (auto& pose : fixture.camera_poses) {
		VoxelVolume<TSDFVoxel, VoxelBlockHash>* volume;
		LoadVolume(&volume, snoopy::PartialVolume17Path<VoxelBlockHash>(), TMemoryDeviceType,
		           snoopy::InitializationParameters_Fr16andFr17<VoxelBlockHash>());

		fixture.visualization_engine->FindVisibleBlocks(volume, &pose, &fixture.calibration_data.intrinsics_d, fixture.render_state);

		const int visible_block_count = volume->index.GetVisibleBlockCount();
		if (visible_block_count == 0) continue; // skip shots without results
		std::shared_ptr<RenderState> render_state = fixture.MakeRenderState();
		fixture.visualization_engine->CreatePointCloud(volume, fixture.view_17, fixture.camera_tracking_state, render_state.get(), false);

		ORUtils::Image<Vector4f>& locations = *fixture.camera_tracking_state->pointCloud->locations;
		ORUtils::Image<Vector4f>& colors = *fixture.camera_tracking_state->pointCloud->colours;
		unsigned int total_points_ground_truth;
		point_cloud_images_file.IStream().read(reinterpret_cast<char*>(&total_points_ground_truth), sizeof(unsigned int));
		BOOST_REQUIRE_EQUAL(fixture.camera_tracking_state->pointCloud->noTotalPoints, total_points_ground_truth);
		ORUtils::Image<Vector4f> locations_ground_truth = ORUtils::MemoryBlockPersistence::LoadImage<Vector4f>(point_cloud_images_file, MEMORYDEVICE_CPU);
		ORUtils::Image<Vector4f> colors_ground_truth = ORUtils::MemoryBlockPersistence::LoadImage<Vector4f>(point_cloud_images_file, MEMORYDEVICE_CPU);
		BOOST_REQUIRE(RawMemoryArraysAlmostEqual_Verbose(locations_ground_truth.GetData(MEMORYDEVICE_CPU), MEMORYDEVICE_CPU,
		                                   locations.GetData(TMemoryDeviceType), TMemoryDeviceType, total_points_ground_truth));
		BOOST_REQUIRE(RawMemoryArraysAlmostEqual(colors_ground_truth.GetData(MEMORYDEVICE_CPU), MEMORYDEVICE_CPU,
		                                   colors.GetData(TMemoryDeviceType), TMemoryDeviceType, total_points_ground_truth));

		delete volume;
	}
}

template<MemoryDeviceType TMemoryDeviceType>
void GenericCreateICPMapsTest() {
	CameraPoseAndRayTracingEngineFixture<TMemoryDeviceType> fixture;

	ORUtils::IStreamWrapper ICP_images_file("TestData/arrays/ICP_images.dat", true);

	for (auto& pose : fixture.camera_poses) {
		VoxelVolume<TSDFVoxel, VoxelBlockHash>* volume;
		LoadVolume(&volume, snoopy::PartialVolume17Path<VoxelBlockHash>(), TMemoryDeviceType,
		           snoopy::InitializationParameters_Fr16andFr17<VoxelBlockHash>());

		fixture.visualization_engine->FindVisibleBlocks(volume, &pose, &fixture.calibration_data.intrinsics_d, fixture.render_state);

		const int visible_block_count = volume->index.GetVisibleBlockCount();
		if (visible_block_count == 0) continue; // skip shots without results
		std::shared_ptr<RenderState> render_state = fixture.MakeRenderState();
		fixture.visualization_engine->CreateICPMaps(volume, fixture.view_17, fixture.camera_tracking_state, render_state.get());
		ORUtils::Image<Vector4f>& locations = *fixture.camera_tracking_state->pointCloud->locations;
		ORUtils::Image<Vector4f>& normals = *fixture.camera_tracking_state->pointCloud->colours;
		ORUtils::Image<Vector4f> locations_ground_truth = ORUtils::MemoryBlockPersistence::LoadImage<Vector4f>(ICP_images_file, MEMORYDEVICE_CPU);
		ORUtils::Image<Vector4f> normals_ground_truth = ORUtils::MemoryBlockPersistence::LoadImage<Vector4f>(ICP_images_file, MEMORYDEVICE_CPU);
		BOOST_REQUIRE(RawMemoryArraysEqual(locations_ground_truth.GetData(MEMORYDEVICE_CPU), MEMORYDEVICE_CPU,
		                                   locations.GetData(TMemoryDeviceType), TMemoryDeviceType, locations.size()));
		BOOST_REQUIRE(RawMemoryArraysEqual(normals_ground_truth.GetData(MEMORYDEVICE_CPU), MEMORYDEVICE_CPU,
		                                   normals.GetData(TMemoryDeviceType), TMemoryDeviceType, normals.size()));

		delete volume;
	}
}

BOOST_AUTO_TEST_CASE(Test_FindAndCountVisibleBlocks_CPU) {
	GenericFindAndCountVisibleBlocksTest<MEMORYDEVICE_CPU>();
}

BOOST_AUTO_TEST_CASE(Test_CreateExpectedDepths) {
	GenericCreateExpectedDepthsTest<MEMORYDEVICE_CPU>();
}

BOOST_AUTO_TEST_CASE(Test_RenderImage) {

}

BOOST_AUTO_TEST_CASE(Test_FindSurface) {
	GenericFindSurfaceTest<MEMORYDEVICE_CPU>();
}

BOOST_AUTO_TEST_CASE(Test_CreatePointCloud) {
	GenericCreatePointCloudTest<MEMORYDEVICE_CPU>();
}

BOOST_AUTO_TEST_CASE(Test_CreateICPMaps) {
	GenericCreateICPMapsTest<MEMORYDEVICE_CPU>();
}

BOOST_AUTO_TEST_CASE(Test_ForwardRender) {

}