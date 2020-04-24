//  ================================================================
//  Created by Gregory Kramida on 10/18/17.
//  Copyright (c) 2017-2000 Gregory Kramida
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

//stdlib
#include <array>

//local
#include "MainEngine.h"
#include "CameraTrackingController.h"
#include "../LowLevel/Interface/LowLevelEngine.h"
#include "../Meshing/Interface/MeshingEngine.h"
#include "../ViewBuilding/Interface/ViewBuilder.h"
#include "../Visualization/Interface/VisualizationEngine.h"
#include "../Indexing/Interface/IndexingEngine.h"
#include "../DepthFusion/DepthFusionEngine.h"
#include "../VolumeFusion/VolumeFusionEngine.h"
#include "../Swapping/Interface/SwappingEngine.h"
#include "../../Objects/Misc/IMUCalibrator.h"
#include "../../../FernRelocLib/Relocaliser.h"
#include "../../CameraTrackers/Interface/CameraTracker.h"


namespace ITMLib {

template<typename TVoxel, typename TWarp, typename TIndex>
class DynamicSceneVoxelEngine : public MainEngine {
private: // member variables
	static constexpr int live_volume_count = 2;

	bool camera_tracking_enabled, fusion_active, main_processing_active, tracking_initialised;
	int frames_processed, relocalization_count;

	configuration::Configuration config;

	// engines
	LowLevelEngine* low_level_engine;
	VisualizationEngine<TVoxel, TIndex>* visualization_engine;
	MeshingEngine<TVoxel, TIndex>* meshing_engine;
	IndexingEngineInterface<TVoxel, TIndex>* indexing_engine;
	DepthFusionEngineInterface<TVoxel, TWarp, TIndex>* depth_fusion_engine;
	VolumeFusionEngineInterface<TVoxel, TIndex>* volume_fusion_engine;
	SwappingEngine<TVoxel, TIndex>* swapping_engine;

	ViewBuilder* view_builder;
	CameraTrackingController* camera_tracking_controller;

	CameraTracker* camera_tracker;
	IMUCalibrator* imu_calibrator;
	SurfaceTrackerInterface<TVoxel, TWarp, TIndex>* surface_tracker;

	VoxelVolume<TVoxel, TIndex>* canonical_volume;
	VoxelVolume<TVoxel, TIndex>** live_volumes;
	VoxelVolume<TWarp, TIndex>* warp_field;
	RenderState* canonical_render_state;
	RenderState* live_render_state;
	RenderState* freeview_render_state;


	FernRelocLib::Relocaliser<float>* relocalizer;
	ITMUChar4Image* keyframe_raycast;

	/// The current input frame data
	View* view;

	/// Current camera pose and additional tracking information
	CameraTrackingState* tracking_state;
	CameraTrackingState::TrackingResult last_tracking_result;
	ORUtils::SE3Pose previous_frame_pose;


	//TODO: revise TelemetryRecorder and add to that
	ORUtils::OStreamWrapper* canonical_volume_memory_usage_file = nullptr;
public: // member functions

	/** \brief Constructor
		Omitting a separate image size for the depth images
		will assume same resolution as for the RGB images.
	*/
	DynamicSceneVoxelEngine(const RGBDCalib& calibration_info, Vector2i rgb_image_size, Vector2i depth_image_size);
	~DynamicSceneVoxelEngine() override;

	View* GetView() override { return view; }

	CameraTrackingState* GetTrackingState() override { return tracking_state; }

	CameraTrackingState::TrackingResult ProcessFrame(ITMUChar4Image* rgb_image, ITMShortImage* depth_image,
	                                                 IMUMeasurement* imu_measurement = nullptr) override;

	/// Extracts a mesh from the current scene and saves it to the model file specified by the file name
	void SaveSceneToMesh(const char* fileName) override;

	/// save and load the full volume and relocalizer (if any) to/from file
	void SaveToFile() override;
	void SaveToFile(const std::string& path) override;
	void LoadFromFile() override;
	void LoadFromFile(const std::string& path) override;

	Vector2i GetImageSize() const override;

	/// Get a render of the result
	void GetImage(ITMUChar4Image* out, GetImageType type,
	              ORUtils::SE3Pose* pose = nullptr, Intrinsics* intrinsics = nullptr) override;

	void ResetAll() override;

	void TurnOnTracking() override;
	void TurnOffTracking() override;

	// Here "integration" == "fusion", i.e. integration of new data into existing model
	void TurnOnIntegration() override;
	void TurnOffIntegration() override;

	void TurnOnMainProcessing() override;
	void TurnOffMainProcessing() override;

private: // member functions
	void Reset();
	void InitializeScenes();
	//TODO: move to SwappingEngine itself.
	void ProcessSwapping(RenderState* render_state);
	void HandlePotentialCameraTrackingFailure();

};
} // namespace ITMLib
