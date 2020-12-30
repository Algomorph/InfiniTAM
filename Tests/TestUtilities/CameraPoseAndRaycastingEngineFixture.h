//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 5/24/20.
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
#pragma once

#include "../../ITMLib/Objects/RenderStates/RenderState.h"
#include "../../ITMLib/Engines/Raycasting/RaycastingEngine.h"
#include "TestDataUtilities.h"

using namespace ITMLib;

namespace test {
template<MemoryDeviceType TMemoryDeviceType>
struct CameraPoseAndRaycastingEngineFixture {
public: // instance variables
	RGBD_CalibrationInformation calibration_data;
	RenderState* render_state;
	const Vector3f target;
	const Vector3f original_viewpoint;
	const int degree_increment;
	std::vector<std::shared_ptr<CameraTrackingState>> tracking_states;
	RaycastingEngineBase<TSDFVoxel, VoxelBlockHash>* rendering_engine;
	View* view_17;

public: // instance functions
	CameraPoseAndRaycastingEngineFixture()
			: render_state(new RenderState(Vector2i(test::snoopy::frame_image_size),
			                               configuration::Get().general_voxel_volume_parameters.near_clipping_distance,
			                               configuration::Get().general_voxel_volume_parameters.far_clipping_distance,
			                               TMemoryDeviceType)),
			  target(-0.09150545, 0.07265271, 0.7908916),
			  original_viewpoint(0.f),
			  degree_increment(30),
			  tracking_states([&]() {
				  std::vector<ORUtils::SE3Pose> camera_poses = GenerateCameraTrajectoryAroundPoint(original_viewpoint, target, degree_increment);
				  std::vector<std::shared_ptr<CameraTrackingState>> camera_tracking_states(camera_poses.size());
				  int i_pose = 0;
				  for(auto& pose: camera_poses){
					  camera_tracking_states[i_pose] = std::make_shared<CameraTrackingState>(test::snoopy::frame_image_size, TMemoryDeviceType);
					  i_pose++;
				  }
				  return camera_tracking_states;
			  }()),
			  rendering_engine(new RaycastingEngine<TSDFVoxel,VoxelBlockHash,TMemoryDeviceType>()),
			  view_17(nullptr) {
		readRGBDCalib(test::snoopy::calibration_path.Get(), calibration_data);
		UpdateView(&view_17,
		           test::snoopy::frame_17_depth_path.Get(),
		           test::snoopy::frame_17_color_path.Get(),
		           test::snoopy::frame_17_mask_path.Get(),
		           test::snoopy::calibration_path.Get(),
		           MEMORYDEVICE_CPU);
	}

	std::shared_ptr<RenderState> MakeRenderState() {
		return std::make_shared<RenderState>(test::snoopy::frame_image_size,
		                                     configuration::Get().general_voxel_volume_parameters.near_clipping_distance,
		                                     configuration::Get().general_voxel_volume_parameters.far_clipping_distance,
		                                     TMemoryDeviceType);
	}

	std::shared_ptr<CameraTrackingState> MakeCameraTrackingState() {
		return std::make_shared<CameraTrackingState>(test::snoopy::frame_image_size, TMemoryDeviceType);
	}

	~CameraPoseAndRaycastingEngineFixture() {
		delete view_17;
		delete render_state;
	}
};
} // namespace test