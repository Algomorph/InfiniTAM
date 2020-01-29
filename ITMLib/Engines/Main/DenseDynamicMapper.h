//  ================================================================
//  Created by Gregory Kramida on 10/18/17.
//  Copyright (c) 2017-2025 Gregory Kramida
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

#include "../Reconstruction/Interface/DynamicSceneReconstructionEngine.h"
#include "../../Utils/Configuration.h"
#include "../Swapping/Interface/ITMSwappingEngine.h"
#include "../../Utils/FileIO/ITMDynamicFusionLogger.h"
#include "../../SurfaceTrackers/Interface/SurfaceTrackerInterface.h"
#include "NonRigidTrackingParameters.h"


namespace ITMLib {

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
	// region ========================================= INTERNAL DATA STRUCTURES =======================================


	struct AnalysisFlags {
		bool hasFocusCoordinates;
	};

	// endregion =======================================================================================================
	// region ========================================== MEMBER FUNCTIONS ==============================================

	void ResetTSDFVolume(ITMVoxelVolume<TVoxel, TIndex>* volume) const;
	void ResetWarpVolume(ITMVoxelVolume<TWarp, TIndex>* warpVolume) const;

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
	* \param canonicalScene - canonical/reference 3D scene where all data is fused/aggregated
	* \param liveScene - live/target 3D scene generated from the incoming single frame of the video
	* \param renderState
	*/
	void ProcessFrame(const ITMView* view, const ITMTrackingState* trackingState,
	                  ITMVoxelVolume <TVoxel, TIndex>* canonicalScene, ITMVoxelVolume <TVoxel, TIndex>** liveScenePair,
	                  ITMVoxelVolume <TWarp, TIndex>* warpField, ITMRenderState* renderState);

	void ProcessInitialFrame(const ITMView* view, const ITMTrackingState* trackingState,
	                         ITMVoxelVolume<TVoxel, TIndex>* canonicalScene, ITMVoxelVolume<TVoxel, TIndex>* liveScene,
	                         ITMRenderState* renderState);


	void BeginProcessingFrameInStepByStepMode(const ITMView* view, const ITMTrackingState* trackingState,
	                                          ITMVoxelVolume<TWarp, TIndex>* warpField,
	                                          ITMVoxelVolume<TVoxel, TIndex>** liveScenePair);

	bool TakeNextStepInStepByStepMode(ITMVoxelVolume <TVoxel, TIndex>* canonicalScene,
	                                  ITMVoxelVolume <TVoxel, TIndex>** liveScenePair,
	                                  ITMVoxelVolume <TWarp, TIndex>* warpField, ITMRenderState* renderState);

	/// Update the visible list (this can be called to update the visible list when fusion is turned off)
	void UpdateVisibleList(const ITMView* view, const ITMTrackingState* trackingState,
	                       ITMVoxelVolume<TVoxel, TIndex>* scene, ITMRenderState* renderState, bool resetVisibleList = false);
	// endregion
private:
	// region ========================================== FUNCTIONS =====================================================
	void ProcessSwapping(
			ITMVoxelVolume <TVoxel, TIndex>* canonicalScene, ITMRenderState* renderState);

	ITMVoxelVolume<TVoxel, TIndex>* TrackFrameMotion(
			ITMVoxelVolume<TVoxel, TIndex>* canonicalScene,
			ITMVoxelVolume<TVoxel, TIndex>** liveScenePair,
			ITMVoxelVolume<TWarp, TIndex>* warpField);

	bool SceneMotionOptimizationConditionNotReached();
	void InitializeProcessing(const ITMView* view, const ITMTrackingState* trackingState,
	                          ITMVoxelVolume<TWarp, TIndex>* warpField, ITMVoxelVolume<TVoxel, TIndex>** liveScenePair);
	void FinalizeProcessing(ITMVoxelVolume <TVoxel, TIndex>* canonicalScene,
	                        ITMVoxelVolume <TVoxel, TIndex>* liveScene, ITMRenderState* renderState);
	void PerformSingleOptimizationStep(
			ITMVoxelVolume<TVoxel, TIndex>* canonicalScene,
			ITMVoxelVolume<TVoxel, TIndex>* initialLiveScene,
			ITMVoxelVolume<TVoxel, TIndex>* finalLiveScene,
			ITMVoxelVolume<TWarp, TIndex>* warpField,
			int iteration);
	void PrintSettings();
	// endregion =======================================================================================================
	// region =========================================== MEMBER VARIABLES =============================================
	DynamicSceneReconstructionEngine<TVoxel, TWarp, TIndex>* sceneReconstructor;
	ITMSwappingEngine<TVoxel, TIndex>* swappingEngine;
	SurfaceTrackerInterface<TVoxel, TWarp, TIndex>* sceneMotionTracker;

	configuration::SwappingMode swappingMode;

	// state (//TODO: make private, add getters where necessary)
	unsigned int iteration = 0;
	float maxVectorUpdate;

	bool inStepByStepProcessingMode = false;
	bool use_expanded_allocation_during_TSDF_construction = false;
	const NonRigidTrackingParameters parameters;
	// needs to be declared after "parameters", derives value from it
	const float maxVectorUpdateThresholdVoxels;
	const AnalysisFlags analysisFlags;
	const Vector3i focusCoordinates;
	const bool print_volume_statistics = configuration::get().verbosity_level >= configuration::VERBOSITY_PER_FRAME && configuration::get().telemetry_settings.log_volume_statistics;
	const configuration::VerbosityLevel verbosity_level;

	// endregion =======================================================================================================

};
}//namespace ITMLib

