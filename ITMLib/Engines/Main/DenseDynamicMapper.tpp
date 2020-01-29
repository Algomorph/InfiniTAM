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
//stdlib
//#include <limits>
#include <chrono>

//boost
#include <boost/filesystem.hpp>

//local
#include "DenseDynamicMapper.h"
#include "../Reconstruction/DynamicSceneReconstructionEngineFactory.h"
#include "../Warping/WarpingEngineFactory.h"
#include "../Swapping/ITMSwappingEngineFactory.h"
#include "../../SurfaceTrackers/SurfaceTrackerFactory.h"
#include "../../Utils/ITMPrintHelpers.h"
#include "../../Utils/Visualization/ITMSceneSliceVisualizer2D.h"
#include "../../Utils/Analytics/ITMBenchmarkUtils.h"
#include "../../Utils/FileIO/ITMSceneLogger.h"
//** CPU **
#include "../VolumeEditAndCopy/CPU/VolumeEditAndCopyEngine_CPU.h"
#include "../../Utils/Analytics/SceneStatisticsCalculator/CPU/ITMSceneStatisticsCalculator_CPU.h"
//** CUDA **
#ifndef COMPILE_WITHOUT_CUDA
#include "../VolumeEditAndCopy/CUDA/VolumeEditAndCopyEngine_CUDA.h"
#include "../../Utils/Analytics/SceneStatisticsCalculator/CUDA/ITMSceneStatisticsCalculator_CUDA.h"
#endif


using namespace ITMLib;

namespace bench = ITMLib::Bench;

// region ========================================= DEBUG PRINTING =====================================================

template<typename TVoxel, typename TIndex>
inline static void PrintSceneStatistics(
		ITMVoxelVolume<TVoxel, TIndex>* scene,
		std::string description) {
	ITMSceneStatisticsCalculatorInterface<TVoxel, TIndex>* calculator = nullptr;
	switch (scene->index.memoryType){
		case MEMORYDEVICE_CPU:
			calculator = &ITMSceneStatisticsCalculator<TVoxel, TIndex, MEMORYDEVICE_CPU>::Instance();
			break;

		case MEMORYDEVICE_CUDA:
#ifndef COMPILE_WITHOUT_CUDA
			calculator = &ITMSceneStatisticsCalculator<TVoxel, TIndex, MEMORYDEVICE_CUDA>::Instance();
#else
			DIEWITHEXCEPTION_REPORTLOCATION("Built without CUDA support, aborting.");
#endif
			break;
		case MEMORYDEVICE_METAL:
			DIEWITHEXCEPTION_REPORTLOCATION("Metal framework not supported for this.");
	}
			
	std::cout << green << "=== Stats for scene '" << description << "' ===" << reset << std::endl;
	std::cout << "    Total voxel count: " << calculator->ComputeAllocatedVoxelCount(scene) << std::endl;
	std::cout << "    NonTruncated voxel count: " << calculator->ComputeNonTruncatedVoxelCount(scene) << std::endl;
	std::cout << "    +1.0 voxel count: " << calculator->CountVoxelsWithSpecificSdfValue(scene, 1.0f) << std::endl;
	//std::vector<int> allocatedHashes = calculator->GetFilledHashBlockIds(scene);
	std::cout << "    Allocated hash count: " << calculator->ComputeAllocatedHashBlockCount(scene) << std::endl;
	std::cout << "    NonTruncated SDF sum: " << calculator->ComputeNonTruncatedVoxelAbsSdfSum(scene) << std::endl;
	std::cout << "    Truncated SDF sum: " << calculator->ComputeTruncatedVoxelAbsSdfSum(scene) << std::endl;

};

inline static void PrintOperationStatus(const char* status) {
	std::cout << bright_cyan << status << reset << std::endl;
}
// endregion ===========================================================================================================

// region ===================================== CONSTRUCTORS / DESTRUCTORS =============================================

template<typename TVoxel, typename TWarp, typename TIndex>
DenseDynamicMapper<TVoxel, TWarp, TIndex>::DenseDynamicMapper(const TIndex& index) :
		reconstructionEngine(
				DynamicSceneReconstructionEngineFactory::MakeSceneReconstructionEngine<TVoxel, TWarp, TIndex>
						(configuration::get().device_type)),
		WarpingEngine(WarpingEngineFactory::MakeWarpingEngine<TVoxel, TWarp, TIndex>()),
		surfaceTracker(
				SurfaceTrackerFactory::MakeSceneMotionTracker<TVoxel, TWarp, TIndex>()),
		swappingEngine(configuration::get().swapping_mode != configuration::SWAPPINGMODE_DISABLED
		               ? ITMSwappingEngineFactory::MakeSwappingEngine<TVoxel, TIndex>(
						configuration::get().device_type, index)
		               : nullptr),
		swappingMode(configuration::get().swapping_mode),
		parameters(configuration::get().non_rigid_tracking_parameters),
		use_expanded_allocation_during_TSDF_construction(configuration::get().general_voxel_volume_parameters.add_extra_block_ring_during_allocation),
		maxVectorUpdateThresholdVoxels(parameters.max_update_length_threshold /
		                                    configuration::get().general_voxel_volume_parameters.voxel_size),
		analysisFlags{configuration::get().verbosity_level >= configuration::VERBOSITY_FOCUS_SPOTS},
		focusCoordinates(configuration::get().telemetry_settings.focus_coordinates),
		verbosity_level(configuration::get().verbosity_level){ }

template<typename TVoxel, typename TWarp, typename TIndex>
DenseDynamicMapper<TVoxel, TWarp, TIndex>::~DenseDynamicMapper() {
	delete reconstructionEngine;
	delete WarpingEngine;
	delete swappingEngine;
	delete surfaceTracker;
}
//endregion ================================= C/D ======================================================================

template<typename TVoxel, typename TWarp, typename TIndex>
void DenseDynamicMapper<TVoxel, TWarp, TIndex>::ProcessInitialFrame(
		const ITMView* view, const ITMTrackingState* trackingState,
		ITMVoxelVolume<TVoxel, TIndex>* canonicalScene, ITMVoxelVolume<TVoxel, TIndex>* liveScene,
		ITMRenderState* renderState) {
	PrintOperationStatus("Generating raw live frame from view...");
	bench::StartTimer("GenerateRawLiveAndCanonicalVolumes");
	reconstructionEngine->GenerateTsdfVolumeFromView(liveScene, view, trackingState);
	bench::StopTimer("GenerateRawLiveAndCanonicalVolumes");
	//** prepare canonical for new frame
	PrintOperationStatus("Fusing data from live frame into canonical frame...");
	//** fuse the live into canonical directly
	bench::StartTimer("FuseOneTsdfVolumeIntoAnother");
	reconstructionEngine->FuseOneTsdfVolumeIntoAnother(canonicalScene, liveScene);
	bench::StopTimer("FuseOneTsdfVolumeIntoAnother");
	ProcessSwapping(canonicalScene, renderState);
}

template<typename TVoxel, typename TWarp, typename TIndex>
void DenseDynamicMapper<TVoxel, TWarp, TIndex>::InitializeProcessing(const ITMView* view,
                                                                     const ITMTrackingState* trackingState,
                                                                     ITMVoxelVolume<TWarp, TIndex>* warpField,
                                                                     ITMVoxelVolume<TVoxel, TIndex>** liveScenePair) {


	PrintOperationStatus("Generating raw live frame from view...");
	bench::StartTimer("GenerateRawLiveAndCanonicalVolumes");
	if(this->use_expanded_allocation_during_TSDF_construction){
		reconstructionEngine->GenerateTsdfVolumeFromViewExpanded(liveScenePair[0], liveScenePair[1], view, trackingState->pose_d->GetM());
	}else{
		reconstructionEngine->GenerateTsdfVolumeFromView(liveScenePair[0], view, trackingState->pose_d->GetM());
	}
	bench::StopTimer("GenerateRawLiveAndCanonicalVolumes");
	surfaceTracker->ClearOutFramewiseWarp(warpField);
	ITMDynamicFusionLogger<TVoxel, TWarp, TIndex>::Instance().InitializeFrameRecording();
	maxVectorUpdate = std::numeric_limits<float>::infinity();
};

template<typename TVoxel, typename TWarp, typename TIndex>
void DenseDynamicMapper<TVoxel, TWarp, TIndex>::FinalizeProcessing(
		ITMVoxelVolume<TVoxel, TIndex>* canonicalScene, ITMVoxelVolume<TVoxel, TIndex>* liveScene,
		ITMRenderState* renderState) {

	//fuse warped live into canonical
	bench::StartTimer("FuseOneTsdfVolumeIntoAnother");
	reconstructionEngine->FuseOneTsdfVolumeIntoAnother(canonicalScene, liveScene);
	bench::StopTimer("FuseOneTsdfVolumeIntoAnother");

	//bench::StartTimer("AddFramewiseWarpToWarp");
	//surfaceTracker->AddFramewiseWarpToWarp(canonicalScene, true);
	//bench::StopTimer("AddFramewiseWarpToWarp");

	ITMDynamicFusionLogger<TVoxel, TWarp, TIndex>::Instance().FinalizeFrameRecording();

	ProcessSwapping(canonicalScene, renderState);
};

template<typename TVoxel, typename TWarp, typename TIndex>
void
DenseDynamicMapper<TVoxel, TWarp, TIndex>::ProcessFrame(const ITMView* view, const ITMTrackingState* trackingState,
                                                        ITMVoxelVolume<TVoxel, TIndex>* canonicalScene,
                                                        ITMVoxelVolume<TVoxel, TIndex>** liveScenePair,
                                                        ITMVoxelVolume<TWarp, TIndex>* warpField,
                                                        ITMRenderState* renderState) {


	if (inStepByStepProcessingMode) {
		DIEWITHEXCEPTION_REPORTLOCATION("Cannot track motion for full frame when in step-by-step mode");
	}

	InitializeProcessing(view, trackingState, warpField, liveScenePair);
	if(this->print_volume_statistics) {
		PrintSceneStatistics(liveScenePair[0], "[[live frame before tracking]]");
	}
	bench::StartTimer("TrackMotion");
	ITMVoxelVolume<TVoxel, TIndex>* finalWarpedLiveScene = TrackFrameMotion(canonicalScene, liveScenePair, warpField);
	bench::StopTimer("TrackMotion");
	if(this->print_volume_statistics) {
		PrintSceneStatistics(finalWarpedLiveScene, "[[live frame after tracking]]");
	}
	FinalizeProcessing(canonicalScene, finalWarpedLiveScene, renderState);
	if(this->print_volume_statistics) {
		PrintSceneStatistics(canonicalScene, "[[canonical frame after fusion]]");
	}
}

template<typename TVoxel, typename TWarp, typename TIndex>
void DenseDynamicMapper<TVoxel, TWarp, TIndex>::UpdateVisibleList(
		const ITMView* view,
		const ITMTrackingState* trackingState,
		ITMVoxelVolume<TVoxel, TIndex>* scene, ITMRenderState* renderState,
		bool resetVisibleList) {
	reconstructionEngine->UpdateVisibleList(scene, view, trackingState, renderState, resetVisibleList);
}

// region =========================================== MOTION TRACKING ==================================================
/**
 * \brief Tracks motion of voxels from canonical frame to live frame.
 * \details The warp field representing motion of voxels in the canonical frame is updated such that the live frame maps
 * as closely as possible back to the canonical using the warp.
 * \param canonicalScene the canonical voxel grid
 * \param liveScene the live voxel grid (typcially obtained by integrating a single depth image into an empty TSDF grid)
 */
template<typename TVoxel, typename TWarp, typename TIndex>
ITMVoxelVolume<TVoxel, TIndex>* DenseDynamicMapper<TVoxel, TWarp, TIndex>::TrackFrameMotion(
		ITMVoxelVolume<TVoxel, TIndex>* canonicalScene,
		ITMVoxelVolume<TVoxel, TIndex>** liveScenePair,
		ITMVoxelVolume<TWarp, TIndex>* warpField) {

	PrintOperationStatus("*** Optimizing warp based on difference between canonical and live SDF. ***");
	bench::StartTimer("TrackMotion_3_Optimization");
	int sourceLiveSceneIndex = 0;
	int targetLiveSceneIndex = 0;
	for (iteration = 0; SceneMotionOptimizationConditionNotReached(); iteration++) {
		sourceLiveSceneIndex = iteration % 2;
		targetLiveSceneIndex = (iteration + 1) % 2;
		PerformSingleOptimizationStep(canonicalScene, liveScenePair[sourceLiveSceneIndex],
		                              liveScenePair[targetLiveSceneIndex], warpField, iteration);
	}
	bench::StopTimer("TrackMotion_3_Optimization");
	PrintOperationStatus("*** Warping optimization finished for current frame. ***");
	return liveScenePair[targetLiveSceneIndex];
}

template<typename TVoxel, typename TWarp, typename TIndex>
void DenseDynamicMapper<TVoxel, TWarp, TIndex>::PerformSingleOptimizationStep(
		ITMVoxelVolume<TVoxel, TIndex>* canonicalScene,
		ITMVoxelVolume<TVoxel, TIndex>* initialLiveScene,
		ITMVoxelVolume<TVoxel, TIndex>* finalLiveScene,
		ITMVoxelVolume<TWarp, TIndex>* warpField,
		int iteration) {

	ITMDynamicFusionLogger<TVoxel, TWarp, TIndex>::Instance().SaveWarpSlices(iteration);

	if(configuration::get().verbosity_level >= configuration::VERBOSITY_PER_ITERATION){
		std::cout << red << "Iteration: " << iteration << reset << std::endl;
		//** warp update gradient computation
		PrintOperationStatus("Calculating warp energy gradient...");
	}

	bench::StartTimer("TrackMotion_31_CalculateWarpUpdate");
	surfaceTracker->CalculateWarpGradient(canonicalScene, initialLiveScene, warpField);
	bench::StopTimer("TrackMotion_31_CalculateWarpUpdate");

	if(configuration::get().verbosity_level >= configuration::VERBOSITY_PER_ITERATION){
		PrintOperationStatus("Applying Sobolev smoothing to energy gradient...");
	}

	bench::StartTimer("TrackMotion_32_ApplySmoothingToGradient");
	surfaceTracker->SmoothWarpGradient(canonicalScene, initialLiveScene, warpField);
	bench::StopTimer("TrackMotion_32_ApplySmoothingToGradient");

	if(verbosity_level >= configuration::VERBOSITY_PER_ITERATION) {
		PrintOperationStatus("Applying warp update (based on energy gradient) to the cumulative warp...");
	}
	bench::StartTimer("TrackMotion_33_UpdateWarps");
	maxVectorUpdate = surfaceTracker->UpdateWarps(canonicalScene, initialLiveScene, warpField);
	bench::StopTimer("TrackMotion_33_UpdateWarps");

	if(verbosity_level >= configuration::VERBOSITY_PER_ITERATION) {
		PrintOperationStatus(
				"Updating live frame SDF by mapping from raw live SDF to new warped SDF based on latest warp...");
	}
	bench::StartTimer("TrackMotion_35_WarpLiveScene");
	WarpingEngine->WarpVolume_WarpUpdates(warpField, initialLiveScene, finalLiveScene);
	bench::StopTimer("TrackMotion_35_WarpLiveScene");
	ITMDynamicFusionLogger<TVoxel, TWarp, TIndex>::Instance().SaveWarps();
}

template<typename TVoxel, typename TWarp, typename TIndex>
bool DenseDynamicMapper<TVoxel, TWarp, TIndex>::SceneMotionOptimizationConditionNotReached() {
	return maxVectorUpdate > this->maxVectorUpdateThresholdVoxels && iteration < parameters.max_iteration_threshold;
}

//endregion ============================================================================================================

template<typename TVoxel, typename TWarp, typename TIndex>
void DenseDynamicMapper<TVoxel, TWarp, TIndex>::ProcessSwapping(
		ITMVoxelVolume<TVoxel, TIndex>* canonicalScene, ITMRenderState* renderState) {
	if (swappingEngine != nullptr) {
		// swapping: CPU -> CUDA
		if (swappingMode == configuration::SWAPPINGMODE_ENABLED)
			swappingEngine->IntegrateGlobalIntoLocal(canonicalScene, renderState);

		// swapping: CUDA -> CPU
		switch (swappingMode) {
			case configuration::SWAPPINGMODE_ENABLED:
				swappingEngine->SaveToGlobalMemory(canonicalScene, renderState);
				break;
			case configuration::SWAPPINGMODE_DELETE:
				swappingEngine->CleanLocalMemory(canonicalScene, renderState);
				break;
			case configuration::SWAPPINGMODE_DISABLED:
				break;
		}
	}
}
// region ======================================= STEP-BY-STEP MODE ====================================================

template<typename TVoxel, typename TWarp, typename TIndex>
void
DenseDynamicMapper<TVoxel, TWarp, TIndex>::BeginProcessingFrameInStepByStepMode(
		const ITMView* view, const ITMTrackingState* trackingState, ITMVoxelVolume<TWarp, TIndex>* warpField,
		ITMVoxelVolume<TVoxel, TIndex>** liveScenePair) {
	if (inStepByStepProcessingMode) {
		DIEWITHEXCEPTION_REPORTLOCATION("Already in step-by-step tracking mode, cannot restart.");
	}
	inStepByStepProcessingMode = true;
	InitializeProcessing(view, trackingState, warpField, liveScenePair);
	iteration = 0;
}

template<typename TVoxel, typename TWarp, typename TIndex>
bool DenseDynamicMapper<TVoxel, TWarp, TIndex>::TakeNextStepInStepByStepMode(
		ITMVoxelVolume<TVoxel, TIndex>* canonicalScene,
		ITMVoxelVolume<TVoxel, TIndex>** liveScenePair,
		ITMVoxelVolume<TWarp, TIndex>* warpField,
		ITMRenderState* renderState) {
	if (!inStepByStepProcessingMode) {
		DIEWITHEXCEPTION_REPORTLOCATION("Step-by-step mode not initialized.");
	}
	if (SceneMotionOptimizationConditionNotReached()) {
		int sourceLiveSceneIndex = iteration % 2;
		int targetLiveSceneIndex = (iteration + 1) % 2;
		PerformSingleOptimizationStep(canonicalScene,
		                              liveScenePair[sourceLiveSceneIndex],
		                              liveScenePair[targetLiveSceneIndex],
		                              warpField, iteration);
		iteration++;
		return true;
	} else {
		int finalLiveSceneIndex = iteration % 2;
		FinalizeProcessing(canonicalScene, liveScenePair[finalLiveSceneIndex], renderState);
		inStepByStepProcessingMode = false;
		return false;
	}
}

// endregion

template<typename TVoxel, typename TWarp, typename TIndex>
void DenseDynamicMapper<TVoxel, TWarp, TIndex>::PrintSettings() {
	std::cout << bright_cyan << "*** DenseDynamicMapper Settings: ***" << reset << std::endl;
	std::cout << "Max iteration count: " << this->parameters.max_iteration_threshold << std::endl;
	std::cout << "Warping vector update threshold: " << this->parameters.max_update_length_threshold << " m, ";
	std::cout << this->maxVectorUpdateThresholdVoxels << " voxels" << std::endl;
	std::cout << bright_cyan << "*** *********************************** ***" << reset << std::endl;
}
