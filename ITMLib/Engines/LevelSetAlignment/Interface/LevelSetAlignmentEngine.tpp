//  ================================================================
//  Created by Gregory Kramida on 11/18/19.
//  Copyright (c) 2019 Gregory Kramida
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


//local
#include "LevelSetAlignmentEngine.h"
#include "../WarpGradientFunctors/WarpGradientFunctor.h"
#include "../../Indexing/Interface/IndexingEngine.h"
#include "../../Analytics/AnalyticsEngineInterface.h"
#include "../Shared/LevelSetAlignmentSharedFunctors.h"
#include "../../Traversal/Interface/VolumeTraversal.h"
#include "../../Traversal/Interface/ThreeVolumeTraversal.h"
#include "../../../Utils/Analytics/BenchmarkUtilities.h"
#include "../../../Utils/Logging/Logging.h"
#include "../../Warping/WarpingEngineFactory.h"
#include "../../Analytics/AnalyticsEngineFactory.h"

using namespace ITMLib;

namespace bench = ITMLib::benchmarking;


// region ===================================== CONSTRUCTORS / DESTRUCTORS =============================================

template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TMemoryDeviceType, ExecutionMode TExecutionMode>
LevelSetAlignmentEngine<TVoxel, TWarp, TIndex, TMemoryDeviceType, TExecutionMode>::LevelSetAlignmentEngine() :
		LevelSetAlignmentEngineInterface<TVoxel, TWarp, TIndex>(),
		weights(this->parameters.weights),
		switches(this->parameters.switches),
		termination(this->parameters.termination),
		warping_engine(WarpingEngineFactory::Build<TVoxel, TWarp, TIndex>()),
		mean_vector_update_threshold_in_voxels(termination.mean_update_length_threshold /
		                                       configuration::Get().general_voxel_volume_parameters.voxel_size),
		iteration(0),
		execution_mode_specific(*this){}

template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TMemoryDeviceType, ExecutionMode TExecutionMode>
LevelSetAlignmentEngine<TVoxel, TWarp, TIndex, TMemoryDeviceType, TExecutionMode>::LevelSetAlignmentEngine(const LevelSetAlignmentSwitches& switches):
		LevelSetAlignmentEngineInterface<TVoxel, TWarp, TIndex>(
				LevelSetAlignmentParameters(LevelSetAlignmentParameters().execution_mode, LevelSetAlignmentWeights(), switches,
				                            LevelSetAlignmentTerminationConditions(), "")
		),
		weights(this->parameters.weights),
		switches(this->parameters.switches),
		termination(this->parameters.termination),
		warping_engine(WarpingEngineFactory::Build<TVoxel, TWarp, TIndex>()),
		mean_vector_update_threshold_in_voxels(termination.mean_update_length_threshold /
		                                       configuration::Get().general_voxel_volume_parameters.voxel_size),
		iteration(0),
		execution_mode_specific(*this){}

template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TMemoryDeviceType, ExecutionMode TExecutionMode>
LevelSetAlignmentEngine<TVoxel, TWarp, TIndex, TMemoryDeviceType, TExecutionMode>::~LevelSetAlignmentEngine() {
	delete warping_engine;
}

template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TMemoryDeviceType, ExecutionMode TExecutionMode>
void LevelSetAlignmentEngine<TVoxel, TWarp, TIndex, TMemoryDeviceType, TExecutionMode>::LogSettings() {
	if (this->log_settings) {
		LOG4CPLUS_TOP_LEVEL(logging::GetLogger(), bright_cyan << "*** NonRigidTrackingParameters ***" << reset);
		LOG4CPLUS_TOP_LEVEL(logging::GetLogger(), "Max iteration count: " << this->termination.max_iteration_count)
		LOG4CPLUS_TOP_LEVEL(logging::GetLogger(),
		                    "Warping vector update threshold: " << this->termination.mean_update_length_threshold
		                                                        << " m, " << this->mean_vector_update_threshold_in_voxels
		                                                        << " voxels");
		LOG4CPLUS_TOP_LEVEL(logging::GetLogger(),
		                    bright_cyan << "*** *********************************** ***" << reset);
	}
}

// region ===================================== HOUSEKEEPING ===========================================================
template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TMemoryDeviceType, ExecutionMode TExecutionMode>
template<WarpType TWarpType>
void LevelSetAlignmentEngine<TVoxel, TWarp, TIndex, TMemoryDeviceType, TExecutionMode>::ClearOutWarps(
		VoxelVolume<TWarp, TIndex>* warp_field) {
	VolumeTraversalEngine<TWarp, TIndex, TMemoryDeviceType>::
	template TraverseUtilized<ClearOutWarpStaticFunctor<TWarp, TWarpType>>(warp_field);
}

template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TMemoryDeviceType, ExecutionMode TExecutionMode>
void LevelSetAlignmentEngine<TVoxel, TWarp, TIndex, TMemoryDeviceType, TExecutionMode>::ClearOutCumulativeWarps(
		VoxelVolume<TWarp, TIndex>* warp_field) {
	ClearOutWarps<WARP_CUMULATIVE>(warp_field);
};

template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TMemoryDeviceType, ExecutionMode TExecutionMode>
void LevelSetAlignmentEngine<TVoxel, TWarp, TIndex, TMemoryDeviceType, TExecutionMode>::ClearOutFramewiseWarps(
		VoxelVolume<TWarp, TIndex>* warp_field) {
	ClearOutWarps<WARP_FRAMEWISE>(warp_field);
}

template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TMemoryDeviceType, ExecutionMode TExecutionMode>
void LevelSetAlignmentEngine<TVoxel, TWarp, TIndex, TMemoryDeviceType, TExecutionMode>::ClearOutWarpUpdates(
		VoxelVolume<TWarp, TIndex>* warp_field) {
	ClearOutWarps<WARP_UPDATE>(warp_field);
}

// endregion ====================================== LOGGING ============================================================

//TODO: use Logging functions for this instead

template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
inline static void PrintVolumeStatistics(
		VoxelVolume<TVoxel, TIndex>* volume,
		std::string description) {
	AnalyticsEngine<TVoxel, TIndex, TMemoryDeviceType>& calculator =
			AnalyticsEngine<TVoxel, TIndex, TMemoryDeviceType>::Instance();
	std::cout << green << "=== Stats for scene '" << description << "' ===" << reset << std::endl;
	std::cout << "    Total voxel count: " << calculator.CountAllocatedVoxels(volume) << std::endl;
	std::cout << "    NonTruncated voxel count: " << calculator.CountNonTruncatedVoxels(volume) << std::endl;
	std::cout << "    +1.0 voxel count: " << calculator.CountVoxelsWithSpecificSdfValue(volume, 1.0f) << std::endl;
	std::vector<int> allocatedHashes = calculator.GetAllocatedHashCodes(volume);
	std::cout << "    Allocated hash count: " << allocatedHashes.size() << std::endl;
	std::cout << "    NonTruncated SDF sum: " << calculator.SumNonTruncatedVoxelAbsSdf(volume) << std::endl;
	std::cout << "    Truncated SDF sum: " << calculator.SumTruncatedVoxelAbsSdf(volume) << std::endl;
};

// region ==================================== MOTION TRACKING LOOP ====================================================
/**
 * \brief Tracks motion of voxels (around surface) from canonical frame to live frame.
 * \details The warp field representing motion of voxels in the canonical frame is updated such that the live frame maps
 * as closely as possible back to the canonical using the warp.
 * \param canonical_volume the canonical voxel grid
 * \param liveScene the live voxel grid (typically obtained by integrating a single depth image into an empty TSDF grid)
 */
template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TMemoryDeviceType, ExecutionMode TExecutionMode>
VoxelVolume<TVoxel, TIndex>*
LevelSetAlignmentEngine<TVoxel, TWarp, TIndex, TMemoryDeviceType, TExecutionMode>::TrackNonRigidMotion(
		VoxelVolume<TVoxel, TIndex>* canonical_volume,
		VoxelVolume<TVoxel, TIndex>** live_volume_pair,
		VoxelVolume<TWarp, TIndex>* warp_field) {

	float max_vector_update_length_in_voxels = std::numeric_limits<float>::infinity();
	const int min_iteration_count = 10;

	int source_live_volume_index = 0;
	int target_live_volume_index = 1;
	for (iteration = 0; iteration < min_iteration_count || (max_vector_update_length_in_voxels > this->mean_vector_update_threshold_in_voxels
	                                                        && iteration < termination.max_iteration_count); iteration++) {
		PerformSingleOptimizationStep(canonical_volume, live_volume_pair[source_live_volume_index],
		                              live_volume_pair[target_live_volume_index], warp_field, max_vector_update_length_in_voxels);

		std::swap(source_live_volume_index, target_live_volume_index);
	}

	if (TExecutionMode == DIAGNOSTIC) {
		LOG4CPLUS_PER_FRAME(logging::GetLogger(), "Level set evolution iteration count: " << yellow << iteration << reset);
	}


	return live_volume_pair[target_live_volume_index];
}


template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TMemoryDeviceType, ExecutionMode TExecutionMode>
void LevelSetAlignmentEngine<TVoxel, TWarp, TIndex, TMemoryDeviceType, TExecutionMode>::PerformSingleOptimizationStep(
		VoxelVolume<TVoxel, TIndex>* canonical_volume,
		VoxelVolume<TVoxel, TIndex>* source_live_volume,
		VoxelVolume<TVoxel, TIndex>* target_live_volume,
		VoxelVolume<TWarp, TIndex>* warp_field,
		float& average_update_vector_length) {

	execution_mode_specific.PerformSingleOptimizationStep(canonical_volume, source_live_volume, target_live_volume, warp_field,
	                                                      average_update_vector_length, iteration);
}

// region ===================================== CALCULATE WARPS ========================================================


template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TMemoryDeviceType, ExecutionMode TExecutionMode>
void
LevelSetAlignmentEngine<TVoxel, TWarp, TIndex, TMemoryDeviceType, TExecutionMode>::CalculateWarpGradient(
		VoxelVolume<TWarp, TIndex>* warp_field,
		VoxelVolume<TVoxel, TIndex>* canonical_volume,
		VoxelVolume<TVoxel, TIndex>* live_volume) {

	// manage hash
	VolumeTraversalEngine<TWarp, TIndex, TMemoryDeviceType>::template
	TraverseUtilized<ClearOutGradientStaticFunctor<TWarp>>(warp_field);

	WarpGradientFunctor<TVoxel, TWarp, TIndex, TMemoryDeviceType, TExecutionMode>
			calculate_gradient_functor(this->weights, this->switches, warp_field, canonical_volume, live_volume,
			                           canonical_volume->GetParameters().voxel_size,
			                           canonical_volume->GetParameters().truncation_distance, this->iteration);

	ThreeVolumeTraversalEngine<TWarp, TVoxel, TVoxel, TIndex, TMemoryDeviceType>::
	TraverseUtilizedWithPosition(warp_field, canonical_volume, live_volume, calculate_gradient_functor);
	calculate_gradient_functor.PrintStatistics();
	calculate_gradient_functor.SaveStatistics();
}

// endregion ===========================================================================================================
// region ========================================== SOBOLEV GRADIENT SMOOTHING ========================================


template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TMemoryDeviceType, ExecutionMode TExecutionMode>
void LevelSetAlignmentEngine<TVoxel, TWarp, TIndex, TMemoryDeviceType, TExecutionMode>::SmoothWarpGradient(
		VoxelVolume<TWarp, TIndex>* warp_field,
		VoxelVolume<TVoxel, TIndex>* canonical_volume,
		VoxelVolume<TVoxel, TIndex>* live_volume) {

	if (this->switches.enable_sobolev_gradient_smoothing) {
		GradientSmoothingPassFunctor<TVoxel, TWarp, TIndex, X> smoothing_pass_functor_X(warp_field);
		GradientSmoothingPassFunctor<TVoxel, TWarp, TIndex, Y> smoothing_pass_functor_Y(warp_field);
		GradientSmoothingPassFunctor<TVoxel, TWarp, TIndex, Z> smoothing_pass_functor_Z(warp_field);

		ITMLib::ThreeVolumeTraversalEngine<TWarp, TVoxel, TVoxel, TIndex, TMemoryDeviceType>::
		template TraverseUtilizedWithPosition(warp_field, canonical_volume, live_volume, smoothing_pass_functor_X);
		ITMLib::ThreeVolumeTraversalEngine<TWarp, TVoxel, TVoxel, TIndex, TMemoryDeviceType>::
		template TraverseUtilizedWithPosition(warp_field, canonical_volume, live_volume, smoothing_pass_functor_Y);
		ITMLib::ThreeVolumeTraversalEngine<TWarp, TVoxel, TVoxel, TIndex, TMemoryDeviceType>::
		template TraverseUtilizedWithPosition(warp_field, canonical_volume, live_volume, smoothing_pass_functor_Z);
	}
}

// endregion ===========================================================================================================

// region ============================= UPDATE FRAMEWISE & GLOBAL (CUMULATIVE) WARPS ===================================
template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TMemoryDeviceType, ExecutionMode TExecutionMode>
float LevelSetAlignmentEngine<TVoxel, TWarp, TIndex, TMemoryDeviceType, TExecutionMode>::UpdateWarps(
		VoxelVolume<TWarp, TIndex>* warp_field,
		VoxelVolume<TVoxel, TIndex>* canonical_volume,
		VoxelVolume<TVoxel, TIndex>* live_volume) {

	WarpUpdateFunctor<TVoxel, TWarp, TMemoryDeviceType>
			warp_update_functor(this->weights.learning_rate,
			                    this->weights.momentum_weight,
			                    this->switches.enable_sobolev_gradient_smoothing);

	ThreeVolumeTraversalEngine<TWarp, TVoxel, TVoxel, TIndex, TMemoryDeviceType>::
	TraverseUtilizedWithPosition(warp_field, canonical_volume, live_volume, warp_update_functor);

	// return average warp update length
	return warp_update_functor.GetAverageWarpUpdateLength();
}

template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TMemoryDeviceType, ExecutionMode TExecutionMode>
void LevelSetAlignmentEngine<TVoxel, TWarp, TIndex, TMemoryDeviceType, TExecutionMode>::AddFramewiseWarpToWarp(
		VoxelVolume<TWarp, TIndex>* warp_field, bool clear_framewise_warps) {
	if (clear_framewise_warps) {
		VolumeTraversalEngine<TWarp, TIndex, TMemoryDeviceType>::
		template TraverseUtilized<
				AddFramewiseWarpToWarpWithClearStaticFunctor<TWarp, TWarp::hasCumulativeWarp>>(warp_field);
	} else {
		VolumeTraversalEngine<TWarp, TIndex, TMemoryDeviceType>::
		template TraverseUtilized<AddFramewiseWarpToWarpStaticFunctor<TWarp, TWarp::hasCumulativeWarp>>(warp_field);

	}
}


//endregion ============================================================================================================


namespace ITMLib {
namespace internal {

template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TMemoryDeviceType, typename TSurfaceTracker>
void SurfaceTracker_ExecutionModeSpecific<TVoxel, TWarp, TIndex, TMemoryDeviceType, OPTIMIZED, TSurfaceTracker>::PerformSingleOptimizationStep(
		VoxelVolume<TVoxel, TIndex>* canonical_volume, VoxelVolume<TVoxel, TIndex>* source_live_volume,
		VoxelVolume<TVoxel, TIndex>* target_live_volume, VoxelVolume<TWarp, TIndex>* warp_field, float& average_update_vector_length, int iteration) {

	parent.CalculateWarpGradient(warp_field, canonical_volume, source_live_volume);
	parent.SmoothWarpGradient(warp_field, canonical_volume, source_live_volume);
	average_update_vector_length = parent.UpdateWarps(warp_field, canonical_volume, source_live_volume);
	parent.warping_engine->WarpVolume_WarpUpdates(warp_field, source_live_volume, target_live_volume);
}


template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TMemoryDeviceType, typename TSurfaceTracker>
SurfaceTracker_ExecutionModeSpecific<TVoxel, TWarp, TIndex, TMemoryDeviceType, DIAGNOSTIC, TSurfaceTracker>::SurfaceTracker_ExecutionModeSpecific(
		TSurfaceTracker& parent) :
		parent(parent) {}


template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TMemoryDeviceType, typename TSurfaceTracker>
void SurfaceTracker_ExecutionModeSpecific<TVoxel, TWarp, TIndex, TMemoryDeviceType, DIAGNOSTIC, TSurfaceTracker>::PerformSingleOptimizationStep(
		VoxelVolume<TVoxel, TIndex>* canonical_volume, VoxelVolume<TVoxel, TIndex>* source_live_volume,
		VoxelVolume<TVoxel, TIndex>* target_live_volume, VoxelVolume<TWarp, TIndex>* warp_field, float& average_update_vector_length, int iteration) {
	auto& config = configuration::Get();

	if (config.logging_settings.log_iteration_number) {
		LOG4CPLUS_PER_ITERATION(logging::GetLogger(), red << "Iteration: " << iteration << reset);
	}


	if (config.logging_settings.log_surface_tracking_procedure_names) {
		LOG4CPLUS_PER_ITERATION(logging::GetLogger(), bright_cyan << "Calculating warp energy gradient..." << reset);
	}

	bench::start_timer("TrackMotion_1_CalculateWarpUpdate");
	parent.CalculateWarpGradient(warp_field, canonical_volume, source_live_volume);
	bench::stop_timer("TrackMotion_1_CalculateWarpUpdate");

	if (parent.parameters.switches.enable_sobolev_gradient_smoothing && config.logging_settings.log_surface_tracking_procedure_names) {
		LOG4CPLUS_PER_ITERATION(logging::GetLogger(),
		                        bright_cyan << "Applying Sobolev smoothing to energy gradient..." << reset);
	}

	bench::start_timer("TrackMotion_2_ApplySmoothingToGradient");
	parent.SmoothWarpGradient(warp_field, canonical_volume, source_live_volume);
	bench::stop_timer("TrackMotion_2_ApplySmoothingToGradient");

	if (config.logging_settings.log_surface_tracking_procedure_names) {
		LOG4CPLUS_PER_ITERATION(logging::GetLogger(),
		                        bright_cyan << "Applying warp update (based on energy gradient) to the cumulative warp..."
		                                    << reset);
	}

	bench::start_timer("TrackMotion_3_UpdateWarps");
	average_update_vector_length = parent.UpdateWarps(warp_field, canonical_volume, source_live_volume);
	bench::stop_timer("TrackMotion_3_UpdateWarps");

	TelemetryRecorder<TVoxel, TWarp, TIndex, TMemoryDeviceType>::GetDefaultInstance()
			.RecordSurfaceTrackingMeanUpdate(average_update_vector_length);

	if (config.logging_settings.log_average_warp_update) {
		LOG4CPLUS_PER_ITERATION(logging::GetLogger(), "Average update vector length:" << yellow << average_update_vector_length << reset);
	}

	if (config.logging_settings.log_surface_tracking_procedure_names) {
		LOG4CPLUS_PER_ITERATION(logging::GetLogger(),
		                        bright_cyan << "Updating live frame SDF by mapping from raw live SDF "
		                                       "to new warped SDF based on latest warp..." << reset);
	}
	bench::start_timer("TrackMotion_4_WarpLiveScene");
	parent.warping_engine->WarpVolume_WarpUpdates(warp_field, source_live_volume, target_live_volume);
	bench::stop_timer("TrackMotion_4_WarpLiveScene");

	auto& telemetry_recorder = TelemetryRecorder<TVoxel,TWarp, TIndex,TMemoryDeviceType>::GetDefaultInstance();
	telemetry_recorder.RecordAndLogWarpUpdateLengthHistogram(*warp_field, iteration);
}
} // namespace internal
} // namespace ITMLib