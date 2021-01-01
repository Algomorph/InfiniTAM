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
#include "GenerateRigidAlignmentTestData.h"

// === ORUtils ===
#include "../../ORUtils/MemoryDeviceType.h"
#include "../../ORUtils/MathTypePersistence/MathTypePersistence.h"

// === ITMLib ===
#include "../../ITMLib/Utils/Logging/Logging.h"
#include "../../ITMLib/Objects/Views/View.h"
#include "../../ITMLib/Objects/Tracking/CameraTrackingState.h"
#include "../../ITMLib/Engines/ImageProcessing/ImageProcessingEngineFactory.h"
#include "../../ITMLib/Engines/Indexing/IndexingEngineFactory.h"
#include "../../ITMLib/Engines/DepthFusion/DepthFusionEngineFactory.h"
#include "../../ITMLib/Engines/Raycasting/RaycastingEngineFactory.h"
#include "../../ITMLib/CameraTrackers/CameraTrackerFactory.h"
#include "../../ITMLib/Engines/Indexing/Interface/IndexingEngine.h"

// === Test Utilities ===
#include "../TestUtilities/TestDataUtilities.h"
#include "../TestUtilities/RigidTrackerPresets.h"


using namespace ITMLib;
using namespace test;

template<typename TIndex, MemoryDeviceType TMemoryDeviceType>
void GenerateRigidAlignmentTestData() {
	LOG4CPLUS_INFO(log4cplus::Logger::getRoot(),
	               "Generating rigid alignment test data (" << IndexString<TIndex>() << ", "
	                                                        << DeviceString<TMemoryDeviceType>() << ") ...");
	View* view = nullptr;

	//__DEBUG
	// std::string frame1_depth_path = "/mnt/Data/Reconstruction/real_data/teddy/frames/depth_000062.png";
	// std::string frame1_color_path = "/mnt/Data/Reconstruction/real_data/teddy/frames/color_000062.png";
	// std::string frame2_depth_path = "/mnt/Data/Reconstruction/real_data/teddy/frames/depth_000063.png";
	// std::string frame2_color_path = "/mnt/Data/Reconstruction/real_data/teddy/frames/color_000063.png";

	// std::string frame1_depth_path = "/mnt/Data/Reconstruction/real_data/teddy/frames/depth_000206.png";
	// std::string frame1_color_path = "/mnt/Data/Reconstruction/real_data/teddy/frames/color_000206.png";
	// std::string frame2_depth_path = "/mnt/Data/Reconstruction/real_data/teddy/frames/depth_000207.png";
	// std::string frame2_color_path = "/mnt/Data/Reconstruction/real_data/teddy/frames/color_000207.png";

	// std::string frame1_depth_path = "/mnt/Data/Reconstruction/real_data/teddy/frames/depth_000244.png";
	// std::string frame1_color_path = "/mnt/Data/Reconstruction/real_data/teddy/frames/color_000244.png";
	// std::string frame2_depth_path = "/mnt/Data/Reconstruction/real_data/teddy/frames/depth_000246.png";
	// std::string frame2_color_path = "/mnt/Data/Reconstruction/real_data/teddy/frames/color_000246.png";

	// std::string frame1_depth_path = "/mnt/Data/Reconstruction/real_data/teddy/frames/depth_000310.png";
	// std::string frame1_color_path = "/mnt/Data/Reconstruction/real_data/teddy/frames/color_000310.png";
	// std::string frame2_depth_path = "/mnt/Data/Reconstruction/real_data/teddy/frames/depth_000312.png";
	// std::string frame2_color_path = "/mnt/Data/Reconstruction/real_data/teddy/frames/color_000312.png";

	// std::string frame1_depth_path = "/mnt/Data/Reconstruction/real_data/teddy/frames/depth_000369.png";
	// std::string frame1_color_path = "/mnt/Data/Reconstruction/real_data/teddy/frames/color_000369.png";
	// std::string frame2_depth_path = "/mnt/Data/Reconstruction/real_data/teddy/frames/depth_000370.png";
	// std::string frame2_color_path = "/mnt/Data/Reconstruction/real_data/teddy/frames/color_000370.png";

	//__DEBUG_U (uncomment if commented)
	std::string frame1_depth_path = teddy::frame_115_depth_path.ToString();
	std::string frame1_color_path = teddy::frame_115_color_path.ToString();

	std::string frame2_depth_path = teddy::frame_116_depth_path.ToString();
	std::string frame2_color_path = teddy::frame_116_color_path.ToString();

	// generate teddy_volume_115 for teddy scene frame 115
	UpdateView(&view,
	           frame1_depth_path,
	           frame1_color_path,
	           teddy::calibration_path.ToString(),
	           TMemoryDeviceType);
	// generate rigid tracker outputs for next frame in the sequence
	UpdateView(&view,
	           frame2_depth_path,
	           frame2_color_path,
	           teddy::calibration_path.ToString(),
	           TMemoryDeviceType);

	// load volume (assumed present) for frame 115
	VoxelVolume<TSDFVoxel_f_rgb, TIndex> teddy_volume_115(teddy::DefaultVolumeParameters(), false, TMemoryDeviceType,
	                                                      teddy::PartialInitializationParameters<TIndex>());

	teddy_volume_115.Reset();
	// assumes GenerateTeddyVolumes functions were run already
	teddy_volume_115.LoadFromDisk(teddy::PartialVolume115Path<TIndex>());

	CameraTrackingState camera_tracking_state(teddy::frame_image_size, TMemoryDeviceType);



	IMUCalibrator* imu_calibrator = new ITMIMUCalibrator_iPad();
	ImageProcessingEngineInterface* image_processing_engine = ImageProcessingEngineFactory::BuildLegacy(TMemoryDeviceType);

	ConstructGeneratedMatrixDirectoryIfMissing();

	// the rendering engine will generate the point cloud
	auto raycasting_engine = ITMLib::RaycastingEngineFactory::Build<TSDFVoxel_f_rgb, TIndex>(TMemoryDeviceType);
	// we need a dud rendering state also for the point cloud
	RenderState render_state(teddy::frame_image_size,
	                         teddy::DefaultVolumeParameters().near_clipping_distance, teddy::DefaultVolumeParameters().far_clipping_distance,
	                         TMemoryDeviceType);

	for (auto& pair : test::matrix_file_name_by_preset) {
		// __DEBUG
		// auto it = std::find(test::color_tracker_presets.begin(), test::color_tracker_presets.end(), pair.first);
		// if (it == test::color_tracker_presets.end()) {
		// 	continue;
		// }
		// if (std::string(pair.first) != test::rgb_tracker_preset_rrbb) {
		// 	continue;
		// }
		const std::string& preset = pair.first;
		const std::string& matrix_filename = pair.second;
		CameraTrackingState tracking_state(teddy::frame_image_size, TMemoryDeviceType);
		CameraTracker* tracker = CameraTrackerFactory::Instance().Make(
				TMemoryDeviceType, preset.c_str(), teddy::frame_image_size, teddy::frame_image_size,
				image_processing_engine, imu_calibrator,
				teddy::DefaultVolumeParameters());
		bool requires_color_rendering = tracker->requiresColourRendering();
		if (requires_color_rendering) {
			ORUtils::SE3Pose pose_rgb(view->calibration_information.trafo_rgb_to_depth.calib_inv * tracking_state.pose_d->GetM());
			raycasting_engine->CreateExpectedDepths(&teddy_volume_115, &pose_rgb, &(view->calibration_information.intrinsics_rgb), &render_state);
			raycasting_engine->CreatePointCloud(&teddy_volume_115, view, &tracking_state, &render_state);
			tracking_state.point_cloud_age = 0;
		} else {
			raycasting_engine->CreateExpectedDepths(&teddy_volume_115, tracking_state.pose_d,
										   &(view->calibration_information.intrinsics_d), &render_state);
			raycasting_engine->CreateICPMaps(&teddy_volume_115, view, &tracking_state, &render_state);
			tracking_state.pose_pointCloud->SetFrom(tracking_state.pose_d);
			if (tracking_state.point_cloud_age == -1) tracking_state.point_cloud_age = -2;
			else tracking_state.point_cloud_age = 0;
		}

		//__DEBUG
		// ORUtils::OStreamWrapper debug_writer(test::generated_arrays_directory.ToString() + "/debug.dat");
		// debug_writer << *tracking_state_color_and_depth.point_cloud;
		// debug_writer << *view;


		tracker->TrackCamera(&tracking_state, view);
		std::string matrix_path = test::generated_matrix_directory.ToString() + "/" + matrix_filename;
		ORUtils::OStreamWrapper matrix_writer(matrix_path, false);
		matrix_writer << tracking_state.pose_d->GetM();

		//__DEBUG
		// std::cout << preset << ": " << std::endl << tracking_state.pose_d->GetM() << std::endl;

		tracking_state.pose_d->SetM(Matrix4f::Identity());
		delete tracker;
	}

	delete imu_calibrator;
	delete image_processing_engine;
	delete view;
}