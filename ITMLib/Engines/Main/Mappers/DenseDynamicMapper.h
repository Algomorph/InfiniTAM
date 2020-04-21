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

// *** local ***
// engines & trackers
#include "../../Indexing/Interface/IndexingEngine.h"
#include "../../DepthFusion/DepthFusionEngine.h"
#include "../../VolumeFusion/VolumeFusionEngine.h"
#include "../../Warping/WarpingEngine.h"
#include "../../Swapping/Interface/SwappingEngine.h"
#include "../../../SurfaceTrackers/Interface/SurfaceTrackerInterface.h"
// utils
#include "../../../Utils/Configuration.h"
#include "../../../Utils/Telemetry/TelemetryRecorder.h"
// misc
#include "../NonRigidTrackingParameters.h"


namespace ITMLib {


//TODO: naming the grouping of these functions as "DynamicMapper" is very arbitrary and even less suitable than for
// static scene fusion, where it's an allusion to "mapping" in SLAM. Here, we also track the surfaces...
// Think about reorganization on a higher level.

template<typename TVoxel, typename TWarp, typename TIndex>
class DenseDynamicMapper {

public:
	// region ============================================ CONSTRUCTORS / DESTRUCTORS ==================================

	/** \brief Constructor
		Omitting a separate image size for the depth images
		will assume same resolution as for the RGB images.
	*/
	explicit DenseDynamicMapper(const TIndex& index);
	~DenseDynamicMapper();
	// endregion
	// region ========================================== MEMBER FUNCTIONS ==============================================

	/**
	* \brief Tracks motions of all points between frames and fuses the new data into the canonical frame
	* 1) Generates new SDF from current view data/point cloud
	* 2) Maps the new SDF to the previous SDF to generate the warp field delta
	* 3) Updates the warp field with the warp field delta
	* 4) Fuses the new data using the updated warp field into the canonical frame
	* 5) Re-projects the (updated) canonical data into the live frame using the updated warp field
	* \tparam TVoxel type of voxel in the voxel grid / implicit function
	* \tparam TIndex type of index used by the voxel grid
	* \param view view with the new (incoming) depth/color data
	* \param trackingState best estimate of the camera tracking from previous to new frame so far
	* \param canonical_volume - canonical/reference 3D scene where all data is fused/aggregated
	* \param liveScene - live/target 3D scene generated from the incoming single frame of the video
	* \param canonical_render_state
	*/
	void ProcessFrame(const View* view, const CameraTrackingState* trackingState,
	                  VoxelVolume <TVoxel, TIndex>* canonical_volume, VoxelVolume <TVoxel, TIndex>** live_volume_pair,
	                  VoxelVolume <TWarp, TIndex>* warp_field);

	void ProcessInitialFrame(const View* view, const CameraTrackingState* tracking_state,
	                         VoxelVolume<TVoxel, TIndex>* canonical_volume, VoxelVolume<TVoxel, TIndex>* live_volume);

	/// Update the visible list (this can be called to update the visible list when fusion is turned off)
	void UpdateVisibleList(const View* view, const CameraTrackingState* trackingState,
	                       VoxelVolume<TVoxel, TIndex>* scene, RenderState* renderState, bool resetVisibleList = false);
	// endregion
private:
	// region ========================================== FUNCTIONS =====================================================
	void ProcessSwapping(
			VoxelVolume <TVoxel, TIndex>* canonicalScene, RenderState* renderState);

	VoxelVolume<TVoxel, TIndex>* TrackFrameMotion(
			VoxelVolume<TVoxel, TIndex>* canonical_volume,
			VoxelVolume<TVoxel, TIndex>** live_volume_pair,
			VoxelVolume<TWarp, TIndex>* warp_field);

	void PerformSingleOptimizationStep(
			VoxelVolume<TVoxel, TIndex>* canonical_volume,
			VoxelVolume<TVoxel, TIndex>* source_live_volume,
			VoxelVolume<TVoxel, TIndex>* target_live_volume,
			VoxelVolume<TWarp, TIndex>* warp_field,
			float& max_update_vector_length,
			int iteration);

	void LogSettings();
	// endregion =======================================================================================================
	// region =========================================== MEMBER VARIABLES =============================================
	// *** engines ***
	IndexingEngineInterface<TVoxel,TIndex>* indexing_engine;
	DepthFusionEngineInterface<TVoxel, TWarp, TIndex>* depth_fusion_engine;
	WarpingEngineInterface<TVoxel, TWarp, TIndex>* warping_engine;
	VolumeFusionEngineInterface<TVoxel, TIndex>* volume_fusion_engine;

	SwappingEngine<TVoxel, TIndex>* swapping_engine;
	SurfaceTrackerInterface<TVoxel, TWarp, TIndex>* surface_tracker;

	// *** parameters ***
	// logging
	const bool log_settings = false;
	bool has_focus_coordinates;
	const Vector3i focus_coordinates;
	//TODO: revise TelemetryRecorder and add to that
	ORUtils::OStreamWrapper* canonical_volume_memory_usage_file = nullptr;

	// algorithm operation
	const configuration::SwappingMode swapping_mode;
	const NonRigidTrackingParameters parameters;
	// needs to be declared after "parameters", derives value from it during initialization
	const float max_vector_update_threshold_in_voxels;
	const configuration::VerbosityLevel verbosity_level;
	// endregion =======================================================================================================

};
}//namespace ITMLib

