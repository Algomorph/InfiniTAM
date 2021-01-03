//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 1/1/21.
//  Copyright (c) 2021 Gregory Kramida
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
#include "GenerateImageProcessingTestData.h"

// === ORUtils ===
#include "../../ORUtils/FileUtils.h"
#include "../../ORUtils/MemoryBlockPersistence/MemoryBlockPersistenceOperators.h"

// === ITMLib ===
#include "../../ITMLib/Engines/ImageProcessing/ImageProcessingEngineFactory.h"

// === Test Utilities ===
#include "../TestUtilities/TestUtilities.h"
#include "../TestUtilities/TestDataUtilities.h"
#include "../../ITMLib/Objects/RenderStates/RenderState.h"
#include "../../ITMLib/Engines/Raycasting/RaycastingEngineFactory.h"

using namespace test;

void GenerateImageProcessingTestData_Legacy() {

	MemoryDeviceType TMemoryDeviceType = MEMORYDEVICE_CPU;

	auto image_processing_engine = ImageProcessingEngineFactory::BuildLegacy(TMemoryDeviceType);
	test::ConstructGeneratedArraysDirectoryIfMissing();

	UChar4Image frame_115(teddy::frame_image_size, TMemoryDeviceType);
	ReadImageFromFile(frame_115, teddy::frame_115_color_file_name.Get());


	// ConvertColorToIntensity
	FloatImage frame_115_float_intensity(Vector2(0), TMemoryDeviceType);
	image_processing_engine->ConvertColorToIntensity(frame_115_float_intensity, frame_115);
	std::string float_intensity_path = test::generated_arrays_directory.ToString() + "TeddyFrame115_FloatIntensityImage.dat";
	{
		ORUtils::OStreamWrapper file(float_intensity_path, true);
		file << frame_115_float_intensity;
	}

	// FilterIntensity
	FloatImage frame_115_filtered_intensity(Vector2(0), TMemoryDeviceType);
	image_processing_engine->FilterIntensity(frame_115_filtered_intensity, frame_115_float_intensity);
	std::string filtered_intensity_path = test::generated_arrays_directory.ToString() + "TeddyFrame115_FilteredIntensityImage.dat";
	{
		ORUtils::OStreamWrapper file(filtered_intensity_path, true);
		file << frame_115_filtered_intensity;
	}

	// FilterSubsample uchar4 --> uchar4
	UChar4Image frame_115_subsampled(Vector2(0), TMemoryDeviceType);
	image_processing_engine->FilterSubsample(frame_115_subsampled, frame_115);
	std::string subsampled_path = test::generated_arrays_directory.ToString() + "TeddyFrame115_SubsampledImage.dat";
	{
		ORUtils::OStreamWrapper file(subsampled_path, true);
		file << frame_115_subsampled;
	}

	// FilterSubsample float --> float
	FloatImage frame_115_float_subsampled(Vector2(0), TMemoryDeviceType);
	image_processing_engine->FilterSubsample(frame_115_float_subsampled, frame_115_float_intensity);
	std::string float_subsampled_path = test::generated_arrays_directory.ToString() + "TeddyFrame115_FloatSubsampledImage.dat";
	{
		ORUtils::OStreamWrapper file(float_subsampled_path, true);
		file << frame_115_float_subsampled;
	}


	// Get a depth float image for further data generation
	View* view = nullptr;
	UpdateView(&view,
	           teddy::frame_115_depth_path.ToString(),
	           teddy::frame_115_color_path.ToString(),
	           teddy::calibration_path.ToString(),
	           TMemoryDeviceType);
	auto& frame_115_depth = view->depth;
	std::string depth_path = test::generated_arrays_directory.ToString() + "TeddyFrame115_Depth.dat";
	{
		ORUtils::OStreamWrapper file(depth_path, true);
		file << frame_115_depth;
	}

	// FilterSubsampleWithHoles float --> float
	FloatImage frame_115_depth_subsampled_with_holes(Vector2(0), TMemoryDeviceType);
	image_processing_engine->FilterSubsampleWithHoles(frame_115_depth_subsampled_with_holes, frame_115_depth);
	std::string depth_subsampled_with_holes_path = test::generated_arrays_directory.ToString() + "TeddyFrame115_DepthSubsampledWithHolesImage.dat";
	{
		ORUtils::OStreamWrapper file(depth_subsampled_with_holes_path, true);
		file << frame_115_depth_subsampled_with_holes;
	}

	// Assumes teddy volumes are already there
	// Generate ICP maps
	RenderState render_state(teddy::frame_image_size,
	                         teddy::DefaultVolumeParameters().near_clipping_distance,
	                         teddy::DefaultVolumeParameters().far_clipping_distance,
	                         TMemoryDeviceType);
	CameraTrackingState tracking_state(teddy::frame_image_size, TMemoryDeviceType);
	UpdateView(&view,
	           teddy::frame_116_depth_path.ToString(),
	           teddy::frame_116_color_path.ToString(),
	           teddy::calibration_path.ToString(),
	           TMemoryDeviceType);
	RaycastingEngineBase<TSDFVoxel_f_rgb, VoxelBlockHash>* raycasting_engine = ITMLib::RaycastingEngineFactory::Build<TSDFVoxel_f_rgb, VoxelBlockHash>(TMemoryDeviceType);
	{
		VoxelVolume<TSDFVoxel_f_rgb, VoxelBlockHash> volume_teddy_frame115(teddy::DefaultVolumeParameters(), false, TMemoryDeviceType,
		                                                                   teddy::PartialInitializationParameters<VoxelBlockHash>());
		volume_teddy_frame115.Reset();
		volume_teddy_frame115.LoadFromDisk(teddy::PartialVolume115Path<VoxelBlockHash>());
		raycasting_engine->CreateExpectedDepths(&volume_teddy_frame115, tracking_state.pose_d,
		                                        &(view->calibration_information.intrinsics_d), &render_state);
		raycasting_engine->CreateICPMaps(&volume_teddy_frame115, view, &tracking_state, &render_state);
	}
	auto& point_cloud = *tracking_state.point_cloud;
	std::string icp_point_cloud_path = test::generated_arrays_directory.ToString() + "TeddyFrame115_PointCloud.dat";
	{
		ORUtils::OStreamWrapper file(icp_point_cloud_path, true);
		file << point_cloud;
	}

	// FilterSubsampleWithHoles float4 --> float4
	Float4Image frame_115_points_subsampled_with_holes(Vector2(0), TMemoryDeviceType);
	image_processing_engine->FilterSubsampleWithHoles(frame_115_points_subsampled_with_holes, point_cloud.locations);
	std::string subsampled_points_path = test::generated_arrays_directory.ToString() + "TeddyFrame115_SubsampledPoints.dat";
	{
		ORUtils::OStreamWrapper file(subsampled_points_path, true);
		file << frame_115_points_subsampled_with_holes;
	}

	// GradientX
	Short4Image frame_115_gradient_x(Vector2(0), TMemoryDeviceType);
	image_processing_engine->GradientX(frame_115_gradient_x, frame_115);
	std::string gradient_x_path = test::generated_arrays_directory.ToString() + "TeddyFrame115_GradientX.dat";
	{
		ORUtils::OStreamWrapper file(gradient_x_path, true);
		file << frame_115_gradient_x;
	}

	// GradientY
	Short4Image frame_115_gradient_y(Vector2(0), TMemoryDeviceType);
	image_processing_engine->GradientY(frame_115_gradient_y, frame_115);
	std::string gradient_y_path = test::generated_arrays_directory.ToString() + "TeddyFrame115_GradientY.dat";
	{
		ORUtils::OStreamWrapper file(gradient_y_path, true);
		file << frame_115_gradient_y;
	}

	// GradientXY
	Float2Image frame_115_gradient_xy(Vector2(0), TMemoryDeviceType);
	image_processing_engine->GradientXY(frame_115_gradient_xy, frame_115_float_intensity);
	std::string gradient_xy_path = test::generated_arrays_directory.ToString() + "TeddyFrame115_GradientXY.dat";
	{
		ORUtils::OStreamWrapper file(gradient_xy_path, true);
		file << frame_115_gradient_xy;
	}

	// CountValidDepths
	int valid_depths = image_processing_engine->CountValidDepths(frame_115_depth);
	std::string valid_depth_path = test::generated_arrays_directory.ToString() + "TeddyFrame115_ValidDepthCount.txt";
	{
		std::ofstream file;
		file.open(valid_depth_path);
		file << valid_depths;
		file.close();
	}

	delete raycasting_engine;
	delete image_processing_engine;
	delete view;
}