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
// === local ===
#include "GenerateRaycastingTestData.h"

// === ORUtils ===
#include "../../ORUtils/MemoryDeviceType.h"

// === ITMLib ===
#include "../../ITMLib/Utils/Logging/Logging.h"

// === Test Utilities ===
#include "../TestUtilities/TestUtilities.h"
#include "../TestUtilities/CameraPoseAndRenderingEngineFixture.h"

using namespace ITMLib;
using namespace test;

template<MemoryDeviceType TMemoryDeviceType>
void GenerateRenderingTestData_VoxelBlockHash() {
	LOG4CPLUS_INFO(log4cplus::Logger::getRoot(),
	               "Generating VBH rendering test data ... ");
	test::ConstructGeneratedArraysDirectoryIfMissing();

	CameraPoseAndRenderingEngineFixture<TMemoryDeviceType> fixture;

	ORUtils::OStreamWrapper visible_blocks_file(GENERATED_TEST_DATA_PREFIX "TestData/arrays/visible_blocks.dat", true);
	ORUtils::OStreamWrapper range_images_file(GENERATED_TEST_DATA_PREFIX "TestData/arrays/range_images.dat", true);
	ORUtils::OStreamWrapper raycast_images_file(GENERATED_TEST_DATA_PREFIX "TestData/arrays/raycast_images.dat", true);
	ORUtils::OStreamWrapper point_cloud_images_file(GENERATED_TEST_DATA_PREFIX "TestData/arrays/point_cloud_images.dat", true);
	ORUtils::OStreamWrapper ICP_images_file(GENERATED_TEST_DATA_PREFIX "TestData/arrays/ICP_images.dat", true);
	ORUtils::OStreamWrapper forward_render_images_file(GENERATED_TEST_DATA_PREFIX "TestData/arrays/forward_render_images.dat", true);
	ORUtils::OStreamWrapper rendered_images_file(GENERATED_TEST_DATA_PREFIX "TestData/arrays/rendered_images.dat", true);

	std::vector<int> pose_range_visible_block_counts1, pose_range_visible_block_counts2, pose_range_visible_block_counts3;

	std::vector<int> block_address_range_bounds =
			{1250, 1500,
			 1450, 1800,
			 1750, 2000};
	size_t range_bound_count = block_address_range_bounds.size();
	SaveRawDataToFile<int>(visible_blocks_file, block_address_range_bounds.data(), range_bound_count, MEMORYDEVICE_CPU);

	int i_pose = 0;
	const int pose_count = static_cast<int>(fixture.tracking_states.size());
	for (auto& tracking_state : fixture.tracking_states) {
		LOG4CPLUS_INFO(log4cplus::Logger::getRoot(), "Generating rendering test data for camera pose " << i_pose + 1 << "/" << pose_count);
		i_pose++;

		VoxelVolume<TSDFVoxel, VoxelBlockHash>* volume;
		LoadVolume(&volume, test::snoopy::PartialVolume17Path<VoxelBlockHash>(), TMemoryDeviceType,
		           test::snoopy::InitializationParameters_Fr16andFr17<VoxelBlockHash>());

		ORUtils::SE3Pose& pose = *tracking_state->pose_d;
		LOG4CPLUS_INFO(log4cplus::Logger::getRoot(), "Generating raycast/TSDF-surface-finding test data ... ");
		// find visible blocks, count visible blocks
		fixture.rendering_engine->FindVisibleBlocks(volume, &pose, &fixture.calibration_data.intrinsics_d, fixture.render_state);
		int range_visible_block_count1 = fixture.rendering_engine->CountVisibleBlocks(volume, fixture.render_state, block_address_range_bounds[0],
		                                                                              block_address_range_bounds[1]);
		int range_visible_block_count2 = fixture.rendering_engine->CountVisibleBlocks(volume, fixture.render_state, block_address_range_bounds[2],
		                                                                              block_address_range_bounds[3]);
		int range_visible_block_count3 = fixture.rendering_engine->CountVisibleBlocks(volume, fixture.render_state, block_address_range_bounds[4],
		                                                                              block_address_range_bounds[5]);

		pose_range_visible_block_counts1.push_back(range_visible_block_count1);
		pose_range_visible_block_counts2.push_back(range_visible_block_count2);
		pose_range_visible_block_counts3.push_back(range_visible_block_count3);

		const int visible_block_count = volume->index.GetVisibleBlockCount();

		if (visible_block_count == 0) continue; // skip rest for shots without results
		int* visible_codes_device = volume->index.GetVisibleBlockHashCodes();
		SaveRawDataToFile<int>(visible_blocks_file, visible_codes_device, visible_block_count, TMemoryDeviceType);

		LOG4CPLUS_INFO(log4cplus::Logger::getRoot(), "Generating expected depth estimation test data ... ");
		// create expected depths (in legacy InfiniTAM, fills "renderingRangeImage" of the "render state")
		{
			std::shared_ptr<RenderState> render_state_expected_depths = fixture.MakeRenderState();
			fixture.rendering_engine->CreateExpectedDepths(volume, &pose, &fixture.calibration_data.intrinsics_d,
			                                               render_state_expected_depths.get());
			ORUtils::MemoryBlockPersistence::SaveImage(range_images_file, *render_state_expected_depths->renderingRangeImage);
		}

		LOG4CPLUS_INFO(log4cplus::Logger::getRoot(), "Generating raycast/TSDF-surface-finding test data ... ");
		// find surface (in legacy InfiniTAM, fills "raycastResult" of the render state)
		{
			std::shared_ptr<RenderState> render_state_find_surface = fixture.MakeRenderState();
			fixture.rendering_engine->CreateExpectedDepths(volume, &pose, &fixture.calibration_data.intrinsics_d,
			                                               render_state_find_surface.get());
			fixture.rendering_engine->FindSurface(volume, &pose, &fixture.calibration_data.intrinsics_d, render_state_find_surface.get());
			ORUtils::MemoryBlockPersistence::SaveImage(raycast_images_file, *render_state_find_surface->raycastResult);
		}
		LOG4CPLUS_INFO(log4cplus::Logger::getRoot(), "Generating raycast-to-point-cloud conversion test data ... ");
		// create point cloud (in legacy InfiniTAM, fills "raycastResult" of "render state", locations & colors of the point cloud in the "tracking state")
		{

			std::shared_ptr<RenderState> render_state_create_point_cloud = fixture.MakeRenderState();
			fixture.rendering_engine->CreateExpectedDepths(volume, &pose, &fixture.calibration_data.intrinsics_d,
			                                               render_state_create_point_cloud.get());
			fixture.rendering_engine->CreatePointCloud(volume, fixture.view_17, tracking_state.get(), render_state_create_point_cloud.get());

			point_cloud_images_file.OStream().write(reinterpret_cast<const char*>(&(tracking_state->point_cloud->point_count)),
			                                        sizeof(unsigned int));
			ORUtils::MemoryBlockPersistence::SaveImage(point_cloud_images_file, tracking_state->point_cloud->locations);
			ORUtils::MemoryBlockPersistence::SaveImage(point_cloud_images_file, tracking_state->point_cloud->colors);
		}

		LOG4CPLUS_INFO(log4cplus::Logger::getRoot(), "Generating raycast-to-ICP-map (projected point cloud) conversion test data ... ");
		// create icp maps (in legacy InfiniTAM, fills "raycastResult" of "render state", locations and "normals" of point_cloud)
		{
			// colors -- interpreted as normals -- honestly, WTF, Oxford? Yeah, I'm blaming you, Oxford, you heard me! -- of the point cloud in the "tracking state")
			std::shared_ptr<RenderState> render_state_create_ICP_maps = fixture.MakeRenderState();
			fixture.rendering_engine->CreateExpectedDepths(volume, &pose, &fixture.calibration_data.intrinsics_d, render_state_create_ICP_maps.get());
			fixture.rendering_engine->CreateICPMaps(volume, fixture.view_17, tracking_state.get(), render_state_create_ICP_maps.get());
			ORUtils::MemoryBlockPersistence::SaveImage(ICP_images_file, tracking_state->point_cloud->locations);
			ORUtils::MemoryBlockPersistence::SaveImage(ICP_images_file, tracking_state->point_cloud->colors);
		}

		LOG4CPLUS_INFO(log4cplus::Logger::getRoot(),
		               "Generating 'forward-rendering' test data ('forward-rendering' is rendering a new point cloud from a different camera vantage point while reusing previous data as much as possible) ... ");
		//forward-render
		{
			std::shared_ptr<RenderState> render_state_forward_render = fixture.MakeRenderState();

			std::shared_ptr<CameraTrackingState> tracking_state_forward_render = fixture.MakeCameraTrackingState();
			ORUtils::SE3Pose& adjusted_pose = *tracking_state_forward_render->pose_d;
			adjusted_pose.SetFrom(&pose);
			// nudge the pose a bit to imitate tracking result
			std::vector<float> adjustment_values = {0.02f, 0.02f, -0.02f, 1.f * PI / 180.f, 3.f * PI / 180.f, 0.0f};
			SaveRawDataToFile<float>(forward_render_images_file, adjustment_values.data(), adjustment_values.size(), MEMORYDEVICE_CPU);
			ORUtils::SE3Pose adjustment(adjustment_values[0], adjustment_values[1], adjustment_values[2], adjustment_values[3], adjustment_values[4],
			                            adjustment_values[5]);
			adjusted_pose.MultiplyWith(&adjustment);

			fixture.rendering_engine->CreateExpectedDepths(volume, &adjusted_pose, &fixture.calibration_data.intrinsics_d,
			                                               render_state_forward_render.get());
			fixture.rendering_engine->FindSurface(volume, &adjusted_pose, &fixture.calibration_data.intrinsics_d,
			                                      render_state_forward_render.get());
			fixture.rendering_engine->ForwardRender(volume, fixture.view_17, tracking_state_forward_render.get(),
			                                        render_state_forward_render.get());
			forward_render_images_file.OStream().write(reinterpret_cast<const char*>(&(render_state_forward_render->noFwdProjMissingPoints)),
			                                           sizeof(int));
			ORUtils::MemoryBlockPersistence::SaveImage(forward_render_images_file, *render_state_forward_render->fwdProjMissingPoints);
			ORUtils::MemoryBlockPersistence::SaveImage(forward_render_images_file, *render_state_forward_render->forwardProjection);
		}

		LOG4CPLUS_INFO(log4cplus::Logger::getRoot(),
		               "Generating test data for render actual images based on raycasting in the TSDF (with various settings)...");
		// render image
		{
			std::shared_ptr<RenderState> render_state_render_image = fixture.MakeRenderState();
			fixture.rendering_engine->CreateExpectedDepths(volume, &pose, &fixture.calibration_data.intrinsics_d,
			                                               render_state_render_image.get());
			fixture.rendering_engine->FindSurface(volume, &pose, &fixture.calibration_data.intrinsics_d, render_state_render_image.get());


			UChar4Image output_image(test::snoopy::frame_image_size, TMemoryDeviceType);
			fixture.rendering_engine->RenderImage(volume, &pose, &fixture.calibration_data.intrinsics_d, render_state_render_image.get(),
			                                      &output_image, IRenderingEngine::RenderImageType::RENDER_COLOUR_FROM_VOLUME,
			                                      IRenderingEngine::RenderRaycastSelection::RENDER_FROM_OLD_RAYCAST);
			ORUtils::MemoryBlockPersistence::SaveImage(rendered_images_file, output_image);
			fixture.rendering_engine->RenderImage(volume, &pose, &fixture.calibration_data.intrinsics_d, render_state_render_image.get(),
			                                      &output_image, IRenderingEngine::RenderImageType::RENDER_COLOUR_FROM_NORMAL,
			                                      IRenderingEngine::RenderRaycastSelection::RENDER_FROM_OLD_RAYCAST);
			ORUtils::MemoryBlockPersistence::SaveImage(rendered_images_file, output_image);
			fixture.rendering_engine->RenderImage(volume, &pose, &fixture.calibration_data.intrinsics_d, render_state_render_image.get(),
			                                      &output_image, IRenderingEngine::RenderImageType::RENDER_SHADED_GREYSCALE_IMAGENORMALS,
			                                      IRenderingEngine::RenderRaycastSelection::RENDER_FROM_OLD_RAYCAST);
			ORUtils::MemoryBlockPersistence::SaveImage(rendered_images_file, output_image);
			fixture.rendering_engine->RenderImage(volume, &pose, &fixture.calibration_data.intrinsics_d, render_state_render_image.get(),
			                                      &output_image, IRenderingEngine::RenderImageType::RENDER_SHADED_GREEN,
			                                      IRenderingEngine::RenderRaycastSelection::RENDER_FROM_OLD_RAYCAST);
			ORUtils::MemoryBlockPersistence::SaveImage(rendered_images_file, output_image);
			fixture.rendering_engine->RenderImage(volume, &pose, &fixture.calibration_data.intrinsics_d, render_state_render_image.get(),
			                                      &output_image, IRenderingEngine::RenderImageType::RENDER_SHADED_OVERLAY,
			                                      IRenderingEngine::RenderRaycastSelection::RENDER_FROM_OLD_RAYCAST);
			ORUtils::MemoryBlockPersistence::SaveImage(rendered_images_file, output_image);
		}

		delete volume;
	}

	SaveRawDataToFile<int>(visible_blocks_file, pose_range_visible_block_counts1.data(), pose_count, MEMORYDEVICE_CPU);
	SaveRawDataToFile<int>(visible_blocks_file, pose_range_visible_block_counts2.data(), pose_count, MEMORYDEVICE_CPU);
	SaveRawDataToFile<int>(visible_blocks_file, pose_range_visible_block_counts3.data(), pose_count, MEMORYDEVICE_CPU);
}