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
#pragma clang diagnostic push
#pragma ide diagnostic ignored "cert-err58-cpp"
#define BOOST_TEST_MODULE RenderingEngine
#ifndef WIN32
#define BOOST_TEST_DYN_LINK
#endif

//boost
#include <boost/test/unit_test.hpp>

//test_utilities
#include "TestUtilities/TestUtilities.h"
#include "TestUtilities/SnoopyTestUtilities.h"
#include "TestUtilities/CameraPoseAndRenderingEngineFixture.h"

//ORUtils
#include "../ORUtils/IStreamWrapper.h"

//ITMLib
#include "../ITMLib/Engines/Rendering/RenderingEngineFactory.h"
#include "../ITMLib/Utils/Analytics/RawArrayComparison.h"
#include "../ITMLib/Utils/Collections/OperationsOnSTLContainers.h"
#include "../ITMLib/Utils/Collections/MemoryBlock_StdContainer_Convertions.h"


using namespace ITMLib;
using namespace test_utilities;
namespace snoopy = snoopy_test_utilities;

template<MemoryDeviceType TMemoryDeviceType>
void GenericFindAndCountVisibleBlocksTest_Legacy() {
	ORUtils::IStreamWrapper visible_blocks_file("TestData/arrays/visible_blocks.dat", true);

	std::vector<int> block_address_range_bounds = ReadStdVectorFromFile<int>(visible_blocks_file);

	std::vector<int> pose_range_visible_block_counts1;
	std::vector<int> pose_range_visible_block_counts2;
	std::vector<int> pose_range_visible_block_counts3;

	CameraPoseAndRenderingEngineFixture<TMemoryDeviceType> fixture;

	for (auto& tracking_state : fixture.tracking_states) {
		VoxelVolume<TSDFVoxel, VoxelBlockHash>* volume;
		LoadVolume(&volume, snoopy::PartialVolume17Path<VoxelBlockHash>(), TMemoryDeviceType,
		           snoopy::InitializationParameters_Fr16andFr17<VoxelBlockHash>());

		ORUtils::SE3Pose& pose = *tracking_state->pose_d;

		fixture.visualization_engine_legacy->FindVisibleBlocks(volume, &pose, &fixture.calibration_data.intrinsics_d, fixture.render_state);
		int range_visible_block_count1 = fixture.visualization_engine_legacy->CountVisibleBlocks(volume, fixture.render_state,
		                                                                                         block_address_range_bounds[0],
		                                                                                         block_address_range_bounds[1]);
		int range_visible_block_count2 = fixture.visualization_engine_legacy->CountVisibleBlocks(volume, fixture.render_state,
		                                                                                         block_address_range_bounds[2],
		                                                                                         block_address_range_bounds[3]);
		int range_visible_block_count3 = fixture.visualization_engine_legacy->CountVisibleBlocks(volume, fixture.render_state,
		                                                                                         block_address_range_bounds[4],
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

		BOOST_REQUIRE_EQUAL(visible_block_count, visible_codes_ground_truth.size());

		BOOST_REQUIRE(RawArraysEqual_Verbose(visible_codes_ground_truth.GetData(MEMORYDEVICE_CPU), MEMORYDEVICE_CPU,
		                                           visible_codes_device, TMemoryDeviceType, visible_block_count, true));

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
void GenericFindAndCountVisibleBlocksTest() {
	ORUtils::IStreamWrapper visible_blocks_file("TestData/arrays/visible_blocks.dat", true);

	std::vector<int> block_address_range_bounds = ReadStdVectorFromFile<int>(visible_blocks_file);

	std::vector<int> pose_range_visible_block_counts1;
	std::vector<int> pose_range_visible_block_counts2;
	std::vector<int> pose_range_visible_block_counts3;

	CameraPoseAndRenderingEngineFixture<TMemoryDeviceType> fixture;

	for (auto& tracking_state : fixture.tracking_states) {
		VoxelVolume<TSDFVoxel, VoxelBlockHash>* volume;
		LoadVolume(&volume, snoopy::PartialVolume17Path<VoxelBlockHash>(), TMemoryDeviceType,
		           snoopy::InitializationParameters_Fr16andFr17<VoxelBlockHash>());

		ORUtils::SE3Pose& pose = *tracking_state->pose_d;

		fixture.rendering_engine->FindVisibleBlocks(volume, &pose, &fixture.calibration_data.intrinsics_d, fixture.render_state);
		int range_visible_block_count1 = fixture.rendering_engine->CountVisibleBlocks(volume, fixture.render_state,
		                                                                              block_address_range_bounds[0],
		                                                                              block_address_range_bounds[1]);
		int range_visible_block_count2 = fixture.rendering_engine->CountVisibleBlocks(volume, fixture.render_state,
		                                                                              block_address_range_bounds[2],
		                                                                              block_address_range_bounds[3]);
		int range_visible_block_count3 = fixture.rendering_engine->CountVisibleBlocks(volume, fixture.render_state,
		                                                                              block_address_range_bounds[4],
		                                                                              block_address_range_bounds[5]);

		pose_range_visible_block_counts1.push_back(range_visible_block_count1);
		pose_range_visible_block_counts2.push_back(range_visible_block_count2);
		pose_range_visible_block_counts3.push_back(range_visible_block_count3);

		const int visible_block_count = volume->index.GetVisibleBlockCount();
		if (visible_block_count == 0) continue; // skip shots without results

		int* visible_codes_device = volume->index.GetVisibleBlockHashCodes();

		ORUtils::MemoryBlock<int> visible_codes_ground_truth =
				ORUtils::MemoryBlockPersistence::LoadMemoryBlock<int>(visible_blocks_file, MEMORYDEVICE_CPU);

		BOOST_REQUIRE_EQUAL(visible_block_count, visible_codes_ground_truth.size());

		BOOST_REQUIRE(RawArraysEqual_Verbose(visible_codes_ground_truth.GetData(MEMORYDEVICE_CPU), MEMORYDEVICE_CPU,
		                                           visible_codes_device, TMemoryDeviceType, visible_block_count, true));

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
void GenericCreateExpectedDepthsTest_Legacy() {
	CameraPoseAndRenderingEngineFixture<TMemoryDeviceType> fixture;

	ORUtils::IStreamWrapper range_images_file("TestData/arrays/range_images.dat", true);

	for (auto& tracking_state : fixture.tracking_states) {
		VoxelVolume<TSDFVoxel, VoxelBlockHash>* volume;
		LoadVolume(&volume, snoopy::PartialVolume17Path<VoxelBlockHash>(), TMemoryDeviceType,
		           snoopy::InitializationParameters_Fr16andFr17<VoxelBlockHash>());

		ORUtils::SE3Pose& pose = *tracking_state->pose_d;

		fixture.visualization_engine_legacy->FindVisibleBlocks(volume, &pose, &fixture.calibration_data.intrinsics_d, fixture.render_state);

		const int visible_block_count = volume->index.GetVisibleBlockCount();
		if (visible_block_count == 0) continue; // skip shots without results
		std::shared_ptr<RenderState> render_state = fixture.MakeRenderState();
		fixture.visualization_engine_legacy->CreateExpectedDepths(volume, &pose, &fixture.calibration_data.intrinsics_d, render_state.get());
		ORUtils::Image<Vector2f>& range_image = *render_state->renderingRangeImage;
		ORUtils::Image<Vector2f> range_image_ground_truth = ORUtils::MemoryBlockPersistence::LoadImage<Vector2f>(range_images_file, MEMORYDEVICE_CPU);
		BOOST_REQUIRE(RawArraysEqual(range_image_ground_truth.GetData(MEMORYDEVICE_CPU), MEMORYDEVICE_CPU,
		                             range_image.GetData(TMemoryDeviceType), TMemoryDeviceType, range_image.size()));
		delete volume;
	}
}

template<MemoryDeviceType TMemoryDeviceType>
void GenericCreateExpectedDepthsTest() {
	CameraPoseAndRenderingEngineFixture<TMemoryDeviceType> fixture;

	ORUtils::IStreamWrapper range_images_file("TestData/arrays/range_images.dat", true);

	for (auto& tracking_state : fixture.tracking_states) {
		VoxelVolume<TSDFVoxel, VoxelBlockHash>* volume;
		LoadVolume(&volume, snoopy::PartialVolume17Path<VoxelBlockHash>(), TMemoryDeviceType,
		           snoopy::InitializationParameters_Fr16andFr17<VoxelBlockHash>());

		ORUtils::SE3Pose& pose = *tracking_state->pose_d;

		fixture.rendering_engine->FindVisibleBlocks(volume, &pose, &fixture.calibration_data.intrinsics_d, fixture.render_state);

		const int visible_block_count = volume->index.GetVisibleBlockCount();
		if (visible_block_count == 0) continue; // skip shots without results
		std::shared_ptr<RenderState> render_state = fixture.MakeRenderState();

		fixture.rendering_engine->CreateExpectedDepths(volume, &pose, &fixture.calibration_data.intrinsics_d, render_state.get());
		ORUtils::Image<Vector2f>& range_image = *render_state->renderingRangeImage;
		ORUtils::Image<Vector2f> range_image_ground_truth = ORUtils::MemoryBlockPersistence::LoadImage<Vector2f>(range_images_file, MEMORYDEVICE_CPU);
		BOOST_REQUIRE(RawArraysEqual_Verbose(range_image_ground_truth.GetData(MEMORYDEVICE_CPU), MEMORYDEVICE_CPU,
		                                           range_image.GetData(TMemoryDeviceType), TMemoryDeviceType, range_image.size()));
		delete volume;
	}
}


template<MemoryDeviceType TMemoryDeviceType>
void GenericFindSurfaceTest() {
	CameraPoseAndRenderingEngineFixture<TMemoryDeviceType> fixture;

	ORUtils::IStreamWrapper raycast_images_file("TestData/arrays/raycast_images.dat", true);

	for (auto& tracking_state : fixture.tracking_states) {
		VoxelVolume<TSDFVoxel, VoxelBlockHash>* volume;
		LoadVolume(&volume, snoopy::PartialVolume17Path<VoxelBlockHash>(), TMemoryDeviceType,
		           snoopy::InitializationParameters_Fr16andFr17<VoxelBlockHash>());

		ORUtils::SE3Pose& pose = *tracking_state->pose_d;

		fixture.visualization_engine_legacy->FindVisibleBlocks(volume, &pose, &fixture.calibration_data.intrinsics_d, fixture.render_state);

		const int visible_block_count = volume->index.GetVisibleBlockCount();
		if (visible_block_count == 0) continue; // skip shots without results
		std::shared_ptr<RenderState> render_state = fixture.MakeRenderState();
		fixture.visualization_engine_legacy->CreateExpectedDepths(volume, &pose, &fixture.calibration_data.intrinsics_d, render_state.get());
		fixture.visualization_engine_legacy->FindSurface(volume, &pose, &fixture.calibration_data.intrinsics_d, render_state.get());
		ORUtils::Image<Vector4f>& raycast_image = *render_state->raycastResult;
		ORUtils::Image<Vector4f> raycast_image_ground_truth = ORUtils::MemoryBlockPersistence::LoadImage<Vector4f>(raycast_images_file,
		                                                                                                           MEMORYDEVICE_CPU);
		BOOST_REQUIRE(RawArraysAlmostEqual_Verbose(raycast_image_ground_truth.GetData(MEMORYDEVICE_CPU), MEMORYDEVICE_CPU,
		                                                 raycast_image.GetData(TMemoryDeviceType), TMemoryDeviceType, raycast_image.size(), 0.2));

		delete volume;
	}
}

template<MemoryDeviceType TMemoryDeviceType>
void GenericCreatePointCloudTest() {
	CameraPoseAndRenderingEngineFixture<TMemoryDeviceType> fixture;

	ORUtils::IStreamWrapper point_cloud_images_file("TestData/arrays/point_cloud_images.dat", true);

	for (auto& tracking_state : fixture.tracking_states) {
		VoxelVolume<TSDFVoxel, VoxelBlockHash>* volume;
		LoadVolume(&volume, snoopy::PartialVolume17Path<VoxelBlockHash>(), TMemoryDeviceType,
		           snoopy::InitializationParameters_Fr16andFr17<VoxelBlockHash>());

		ORUtils::SE3Pose& pose = *tracking_state->pose_d;

		fixture.visualization_engine_legacy->FindVisibleBlocks(volume, &pose, &fixture.calibration_data.intrinsics_d, fixture.render_state);

		const int visible_block_count = volume->index.GetVisibleBlockCount();
		if (visible_block_count == 0) continue; // skip shots without results
		std::shared_ptr<RenderState> render_state = fixture.MakeRenderState();
		fixture.visualization_engine_legacy->CreateExpectedDepths(volume, &pose, &fixture.calibration_data.intrinsics_d, render_state.get());
		fixture.visualization_engine_legacy->CreatePointCloud(volume, fixture.view_17, tracking_state.get(), render_state.get(), false);

		ORUtils::Image<Vector4f>& locations = *tracking_state->pointCloud->locations;
		ORUtils::Image<Vector4f>& colors = *tracking_state->pointCloud->colours;
		unsigned int total_points_ground_truth;
		point_cloud_images_file.IStream().read(reinterpret_cast<char*>(&total_points_ground_truth), sizeof(unsigned int));
		BOOST_REQUIRE_EQUAL(tracking_state->pointCloud->noTotalPoints, total_points_ground_truth);
		ORUtils::Image<Vector4f> locations_ground_truth = ORUtils::MemoryBlockPersistence::LoadImage<Vector4f>(point_cloud_images_file,
		                                                                                                       MEMORYDEVICE_CPU);
		ORUtils::Image<Vector4f> colors_ground_truth = ORUtils::MemoryBlockPersistence::LoadImage<Vector4f>(point_cloud_images_file,
		                                                                                                    MEMORYDEVICE_CPU);
		BOOST_REQUIRE(RawArraysAlmostEqual_Verbose(locations_ground_truth.GetData(MEMORYDEVICE_CPU), MEMORYDEVICE_CPU,
		                                                 locations.GetData(TMemoryDeviceType), TMemoryDeviceType, total_points_ground_truth, true));
		BOOST_REQUIRE(RawArraysAlmostEqual_Verbose(colors_ground_truth.GetData(MEMORYDEVICE_CPU), MEMORYDEVICE_CPU,
		                                                 colors.GetData(TMemoryDeviceType), TMemoryDeviceType, total_points_ground_truth, true));

		delete volume;
	}
}

template<MemoryDeviceType TMemoryDeviceType>
void GenericCreateICPMapsTest() {
	CameraPoseAndRenderingEngineFixture<TMemoryDeviceType> fixture;

	ORUtils::IStreamWrapper ICP_images_file("TestData/arrays/ICP_images.dat", true);

	for (auto& tracking_state : fixture.tracking_states) {
		VoxelVolume<TSDFVoxel, VoxelBlockHash>* volume;
		LoadVolume(&volume, snoopy::PartialVolume17Path<VoxelBlockHash>(), TMemoryDeviceType,
		           snoopy::InitializationParameters_Fr16andFr17<VoxelBlockHash>());
		ORUtils::SE3Pose& pose = *tracking_state->pose_d;

		fixture.visualization_engine_legacy->FindVisibleBlocks(volume, &pose, &fixture.calibration_data.intrinsics_d, fixture.render_state);

		const int visible_block_count = volume->index.GetVisibleBlockCount();
		if (visible_block_count == 0) continue; // skip shots without results
		std::shared_ptr<RenderState> render_state = fixture.MakeRenderState();
		fixture.visualization_engine_legacy->CreateExpectedDepths(volume, &pose, &fixture.calibration_data.intrinsics_d, render_state.get());
		fixture.visualization_engine_legacy->CreateICPMaps(volume, fixture.view_17, tracking_state.get(), render_state.get());
		ORUtils::Image<Vector4f>& locations = *tracking_state->pointCloud->locations;
		ORUtils::Image<Vector4f>& normals = *tracking_state->pointCloud->colours;
		ORUtils::Image<Vector4f> locations_ground_truth = ORUtils::MemoryBlockPersistence::LoadImage<Vector4f>(ICP_images_file, MEMORYDEVICE_CPU);
		ORUtils::Image<Vector4f> normals_ground_truth = ORUtils::MemoryBlockPersistence::LoadImage<Vector4f>(ICP_images_file, MEMORYDEVICE_CPU);
		BOOST_REQUIRE(RawArraysAlmostEqual_Verbose(locations_ground_truth.GetData(MEMORYDEVICE_CPU), MEMORYDEVICE_CPU,
		                                                 locations.GetData(TMemoryDeviceType), TMemoryDeviceType, locations.size(), 5e-4));
		BOOST_REQUIRE(RawArraysAlmostEqual_Verbose(normals_ground_truth.GetData(MEMORYDEVICE_CPU), MEMORYDEVICE_CPU,
		                                                 normals.GetData(TMemoryDeviceType), TMemoryDeviceType, normals.size(), 0.03));

		delete volume;
	}
}

template<MemoryDeviceType TMemoryDeviceType>
void GenericForwardRenderTest() {
	CameraPoseAndRenderingEngineFixture<TMemoryDeviceType> fixture;

	ORUtils::IStreamWrapper forward_render_images_file("TestData/arrays/forward_render_images.dat", true);

	for (auto& tracking_state : fixture.tracking_states) {
		VoxelVolume<TSDFVoxel, VoxelBlockHash>* volume;
		LoadVolume(&volume, snoopy::PartialVolume17Path<VoxelBlockHash>(), TMemoryDeviceType,
		           snoopy::InitializationParameters_Fr16andFr17<VoxelBlockHash>());

		ORUtils::SE3Pose& pose = *tracking_state->pose_d;

		const int visible_block_count = volume->index.GetVisibleBlockCount();
		if (visible_block_count == 0) continue; // skip shots without results
		std::shared_ptr<RenderState> render_state = fixture.MakeRenderState();
		fixture.visualization_engine_legacy->FindSurface(volume, &pose, &fixture.calibration_data.intrinsics_d, render_state.get());

		// nudge the pose a bit to imitate tracking result
		std::shared_ptr<CameraTrackingState> tracking_state_forward_render = fixture.MakeCameraTrackingState();
		ORUtils::SE3Pose& adjusted_pose = *tracking_state_forward_render->pose_d;
		adjusted_pose.SetFrom(&pose);
		std::vector<float> adjustment_values = ReadStdVectorFromFile<float>(forward_render_images_file);
		ORUtils::SE3Pose adjustment(adjustment_values[0], adjustment_values[1], adjustment_values[2], adjustment_values[3], adjustment_values[4],
		                            adjustment_values[5]);
		adjusted_pose.MultiplyWith(&adjustment);

		fixture.visualization_engine_legacy->CreateExpectedDepths(volume, &adjusted_pose, &fixture.calibration_data.intrinsics_d, render_state.get());
		fixture.visualization_engine_legacy->FindVisibleBlocks(volume, &adjusted_pose, &fixture.calibration_data.intrinsics_d, fixture.render_state);
		fixture.visualization_engine_legacy->ForwardRender(volume, fixture.view_17, tracking_state_forward_render.get(), render_state.get());

		int forward_rendering_missing_point_cout_ground_truth;
		forward_render_images_file.IStream().read(reinterpret_cast<char*>(&forward_rendering_missing_point_cout_ground_truth), sizeof(int));
		BOOST_REQUIRE_EQUAL(render_state->noFwdProjMissingPoints, forward_rendering_missing_point_cout_ground_truth);
		ORUtils::Image<int> missing_points_ground_truth = ORUtils::MemoryBlockPersistence::LoadImage<int>(forward_render_images_file,
		                                                                                                  MEMORYDEVICE_CPU);
		ORUtils::Image<Vector4f> forward_projection_ground_truth = ORUtils::MemoryBlockPersistence::LoadImage<Vector4f>(forward_render_images_file,
		                                                                                                                MEMORYDEVICE_CPU);
		BOOST_REQUIRE(RawArraysEqual(missing_points_ground_truth.GetData(MEMORYDEVICE_CPU), MEMORYDEVICE_CPU,
		                             render_state->fwdProjMissingPoints->GetData(TMemoryDeviceType), TMemoryDeviceType,
		                             forward_rendering_missing_point_cout_ground_truth));
		BOOST_REQUIRE(RawArraysAlmostEqual(forward_projection_ground_truth.GetData(MEMORYDEVICE_CPU), MEMORYDEVICE_CPU,
		                                         render_state->forwardProjection->GetData(TMemoryDeviceType), TMemoryDeviceType,
		                                         forward_projection_ground_truth.size()));

		delete volume;
	}
}

template<MemoryDeviceType TMemoryDeviceType>
void GenericRenderImageTest() {
	CameraPoseAndRenderingEngineFixture<TMemoryDeviceType> fixture;

	ORUtils::IStreamWrapper rendered_images_file("TestData/arrays/rendered_images.dat", true);

	for (auto& tracking_state : fixture.tracking_states) {
		VoxelVolume<TSDFVoxel, VoxelBlockHash>* volume;
		LoadVolume(&volume, snoopy::PartialVolume17Path<VoxelBlockHash>(), TMemoryDeviceType,
		           snoopy::InitializationParameters_Fr16andFr17<VoxelBlockHash>());
		ORUtils::SE3Pose& pose = *tracking_state->pose_d;

		fixture.visualization_engine_legacy->FindVisibleBlocks(volume, &pose, &fixture.calibration_data.intrinsics_d, fixture.render_state);

		const int visible_block_count = volume->index.GetVisibleBlockCount();
		if (visible_block_count == 0) continue; // skip shots without results
		std::shared_ptr<RenderState> render_state = fixture.MakeRenderState();
		fixture.visualization_engine_legacy->CreateExpectedDepths(volume, &pose, &fixture.calibration_data.intrinsics_d, render_state.get());
		fixture.visualization_engine_legacy->FindSurface(volume, &pose, &fixture.calibration_data.intrinsics_d, render_state.get());

		UChar4Image output_image(snoopy::frame_image_size, TMemoryDeviceType);
		fixture.visualization_engine_legacy->RenderImage(volume, &pose, &fixture.calibration_data.intrinsics_d, render_state.get(),
		                                                 &output_image, IRenderingEngine::RenderImageType::RENDER_COLOUR_FROM_VOLUME,
		                                                 IRenderingEngine::RenderRaycastSelection::RENDER_FROM_OLD_RAYCAST);
		UChar4Image output_image_ground_truth = ORUtils::MemoryBlockPersistence::LoadImage<Vector4u>(rendered_images_file, MEMORYDEVICE_CPU);
		BOOST_REQUIRE(RawArraysAlmostEqual_Verbose(output_image_ground_truth.GetData(MEMORYDEVICE_CPU), MEMORYDEVICE_CPU,
		                                                 output_image.GetData(TMemoryDeviceType), TMemoryDeviceType, output_image.size(), 1u));
		// render again re-doing the raycast internally, make sure the results match again
		fixture.visualization_engine_legacy->RenderImage(volume, &pose, &fixture.calibration_data.intrinsics_d, render_state.get(),
		                                                 &output_image, IRenderingEngine::RenderImageType::RENDER_COLOUR_FROM_VOLUME,
		                                                 IRenderingEngine::RenderRaycastSelection::RENDER_FROM_NEW_RAYCAST);
		BOOST_REQUIRE(RawArraysAlmostEqual_Verbose(output_image_ground_truth.GetData(MEMORYDEVICE_CPU), MEMORYDEVICE_CPU,
		                                                 output_image.GetData(TMemoryDeviceType), TMemoryDeviceType, output_image.size(), 1u));
		fixture.visualization_engine_legacy->RenderImage(volume, &pose, &fixture.calibration_data.intrinsics_d, render_state.get(),
		                                                 &output_image, IRenderingEngine::RenderImageType::RENDER_COLOUR_FROM_NORMAL,
		                                                 IRenderingEngine::RenderRaycastSelection::RENDER_FROM_OLD_RAYCAST);
		output_image_ground_truth = ORUtils::MemoryBlockPersistence::LoadImage<Vector4u>(rendered_images_file, MEMORYDEVICE_CPU);
		BOOST_REQUIRE(RawArraysAlmostEqual_Verbose(output_image_ground_truth.GetData(MEMORYDEVICE_CPU), MEMORYDEVICE_CPU,
		                                                 output_image.GetData(TMemoryDeviceType), TMemoryDeviceType, output_image.size(), 1u));
		fixture.visualization_engine_legacy->RenderImage(volume, &pose, &fixture.calibration_data.intrinsics_d, render_state.get(),
		                                                 &output_image, IRenderingEngine::RenderImageType::RENDER_SHADED_GREYSCALE_IMAGENORMALS,
		                                                 IRenderingEngine::RenderRaycastSelection::RENDER_FROM_OLD_RAYCAST);
		output_image_ground_truth = ORUtils::MemoryBlockPersistence::LoadImage<Vector4u>(rendered_images_file, MEMORYDEVICE_CPU);
		BOOST_REQUIRE(RawArraysAlmostEqual_Verbose(output_image_ground_truth.GetData(MEMORYDEVICE_CPU), MEMORYDEVICE_CPU,
		                                                 output_image.GetData(TMemoryDeviceType), TMemoryDeviceType, output_image.size(), 1u));
		fixture.visualization_engine_legacy->RenderImage(volume, &pose, &fixture.calibration_data.intrinsics_d, render_state.get(),
		                                                 &output_image, IRenderingEngine::RenderImageType::RENDER_SHADED_GREEN,
		                                                 IRenderingEngine::RenderRaycastSelection::RENDER_FROM_OLD_RAYCAST);
		output_image_ground_truth = ORUtils::MemoryBlockPersistence::LoadImage<Vector4u>(rendered_images_file, MEMORYDEVICE_CPU);
		BOOST_REQUIRE(RawArraysAlmostEqual_Verbose(output_image_ground_truth.GetData(MEMORYDEVICE_CPU), MEMORYDEVICE_CPU,
		                                                 output_image.GetData(TMemoryDeviceType), TMemoryDeviceType, output_image.size(), 1u));
		fixture.visualization_engine_legacy->RenderImage(volume, &pose, &fixture.calibration_data.intrinsics_d, render_state.get(),
		                                                 &output_image, IRenderingEngine::RenderImageType::RENDER_SHADED_OVERLAY,
		                                                 IRenderingEngine::RenderRaycastSelection::RENDER_FROM_OLD_RAYCAST);
		output_image_ground_truth = ORUtils::MemoryBlockPersistence::LoadImage<Vector4u>(rendered_images_file, MEMORYDEVICE_CPU);
		BOOST_REQUIRE(RawArraysAlmostEqual_Verbose(output_image_ground_truth.GetData(MEMORYDEVICE_CPU), MEMORYDEVICE_CPU,
		                                                 output_image.GetData(TMemoryDeviceType), TMemoryDeviceType, output_image.size(), 1u));

		delete volume;
	}
}

BOOST_AUTO_TEST_CASE(Test_FindAndCountVisibleBlocks_CPU) {
	GenericFindAndCountVisibleBlocksTest<MEMORYDEVICE_CPU>();
}

BOOST_AUTO_TEST_CASE(Test_CreateExpectedDepths_CPU) {
	GenericCreateExpectedDepthsTest<MEMORYDEVICE_CPU>();
}

BOOST_AUTO_TEST_CASE(Test_RenderImage_CPU) {
	GenericRenderImageTest<MEMORYDEVICE_CPU>();
}

BOOST_AUTO_TEST_CASE(Test_FindSurface_CPU) {
	GenericFindSurfaceTest<MEMORYDEVICE_CPU>();
}

BOOST_AUTO_TEST_CASE(Test_CreatePointCloud_CPU) {
	GenericCreatePointCloudTest<MEMORYDEVICE_CPU>();
}

BOOST_AUTO_TEST_CASE(Test_CreateICPMaps_CPU) {
	GenericCreateICPMapsTest<MEMORYDEVICE_CPU>();
}

BOOST_AUTO_TEST_CASE(Test_ForwardRender_CPU) {
	GenericForwardRenderTest<MEMORYDEVICE_CPU>();
}

#ifndef COMPILE_WITHOUT_CUDA

BOOST_AUTO_TEST_CASE(Test_FindAndCountVisibleBlocks_CUDA) {
	GenericFindAndCountVisibleBlocksTest<MEMORYDEVICE_CUDA>();
}

BOOST_AUTO_TEST_CASE(Test_CreateExpectedDepths_CUDA) {
	GenericCreateExpectedDepthsTest<MEMORYDEVICE_CUDA>();
}

BOOST_AUTO_TEST_CASE(Test_RenderImage_CUDA) {
	GenericRenderImageTest<MEMORYDEVICE_CUDA>();
}

BOOST_AUTO_TEST_CASE(Test_FindSurface_CUDA) {
	GenericFindSurfaceTest<MEMORYDEVICE_CUDA>();
}

BOOST_AUTO_TEST_CASE(Test_CreatePointCloud_CUDA) {
	GenericCreatePointCloudTest<MEMORYDEVICE_CUDA>();
}

BOOST_AUTO_TEST_CASE(Test_CreateICPMaps_CUDA) {
	GenericCreateICPMapsTest<MEMORYDEVICE_CUDA>();
}

BOOST_AUTO_TEST_CASE(Test_ForwardRender_CUDA) {
	GenericForwardRenderTest<MEMORYDEVICE_CUDA>();
}

#endif // #ifndef COMPILE_WITHOUT_CUDA
#pragma clang diagnostic pop