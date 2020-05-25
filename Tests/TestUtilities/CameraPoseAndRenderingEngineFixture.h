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
#include "../../ITMLib/Engines/Rendering/RenderingEngineFactory.h"
#include "SnoopyTestUtilities.h"

using namespace ITMLib;

namespace test_utilities {
template<MemoryDeviceType TMemoryDeviceType>
struct CameraPoseAndRenderingEngineFixture {
public: // member variables
	RGBDCalib calibration_data;
	RenderState* render_state;
	const Vector3f target;
	const Vector3f original_viewpoint;
	const int degree_increment;
	std::vector<std::shared_ptr<CameraTrackingState>> tracking_states;
	RenderingEngineBase<TSDFVoxel, VoxelBlockHash>* visualization_engine;
	View* view_17;

public: // member functions
	CameraPoseAndRenderingEngineFixture()
			: render_state(new RenderState(Vector2i(snoopy_test_utilities::frame_image_size),
			                               configuration::get().general_voxel_volume_parameters.near_clipping_distance,
			                               configuration::get().general_voxel_volume_parameters.far_clipping_distance,
			                               TMemoryDeviceType)),
			  target(-0.09150545, 0.07265271, 0.7908916),
			  original_viewpoint(0.f),
			  degree_increment(30),
			  tracking_states([&]() {
				  std::vector<ORUtils::SE3Pose> camera_poses = GenerateCameraTrajectoryAroundPoint(original_viewpoint, target, degree_increment);
				  std::vector<std::shared_ptr<CameraTrackingState>> camera_tracking_states(camera_poses.size());
				  int i_pose = 0;
				  for(auto& pose: camera_poses){
					  camera_tracking_states[i_pose] = std::make_shared<CameraTrackingState>(snoopy_test_utilities::frame_image_size, TMemoryDeviceType);
					  i_pose++;
				  }
				  return camera_tracking_states;
			  }()),
			  visualization_engine(RenderingEngineFactory::MakeVisualizationEngine<TSDFVoxel, VoxelBlockHash>(
					  TMemoryDeviceType)),
			  view_17(nullptr) {
		readRGBDCalib(snoopy_test_utilities::SnoopyCalibrationPath().c_str(), calibration_data);
		UpdateView(&view_17,
		           snoopy_test_utilities::Frame17DepthPath(),
		           snoopy_test_utilities::Frame17ColorPath(),
		           snoopy_test_utilities::Frame17MaskPath(),
		           snoopy_test_utilities::SnoopyCalibrationPath(),
		           MEMORYDEVICE_CPU);
	}

	std::shared_ptr<RenderState> MakeRenderState() {
		return std::make_shared<RenderState>(snoopy_test_utilities::frame_image_size,
		                                     configuration::get().general_voxel_volume_parameters.near_clipping_distance,
		                                     configuration::get().general_voxel_volume_parameters.far_clipping_distance,
		                                     TMemoryDeviceType);
	}

	std::shared_ptr<CameraTrackingState> MakeCameraTrackingState() {
		return std::make_shared<CameraTrackingState>(snoopy_test_utilities::frame_image_size, TMemoryDeviceType);
	}

	~CameraPoseAndRenderingEngineFixture() {
		delete view_17;
		delete visualization_engine;
		delete render_state;
	}
};
} // namespace test_utilities