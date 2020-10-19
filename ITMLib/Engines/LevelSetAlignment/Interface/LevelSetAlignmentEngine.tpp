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
#include "../Functors/WarpGradientFunctor.h"
#include "../Functors/WarpStatisticFunctors.h"
#include "../../Indexing/Interface/IndexingEngine.h"
#include "../../Analytics/AnalyticsEngineInterface.h"
#include "../Shared/LevelSetAlignmentSharedFunctors.h"
#include "../../Traversal/Interface/VolumeTraversal.h"
#include "../../Reduction/Interface/VolumeReduction.h"
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
		warping_engine(WarpingEngineFactory::Build<TVoxel, TWarp, TIndex>(TMemoryDeviceType)),
		vector_update_threshold_in_voxels(termination.update_length_threshold /
		                                  configuration::Get().general_voxel_volume_parameters.voxel_size),
		iteration(0) {}

template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TMemoryDeviceType, ExecutionMode TExecutionMode>
LevelSetAlignmentEngine<TVoxel, TWarp, TIndex, TMemoryDeviceType, TExecutionMode>::LevelSetAlignmentEngine(const LevelSetAlignmentSwitches& switches):
		LevelSetAlignmentEngineInterface<TVoxel, TWarp, TIndex>(
				LevelSetAlignmentParameters(LevelSetAlignmentParameters().execution_mode, LevelSetAlignmentWeights(), switches,
				                            LevelSetAlignmentTerminationConditions(), "")
		),
		weights(this->parameters.weights),
		switches(this->parameters.switches),
		termination(this->parameters.termination),
		warping_engine(WarpingEngineFactory::Build<TVoxel, TWarp, TIndex>(TMemoryDeviceType)),
		vector_update_threshold_in_voxels(termination.update_length_threshold /
		                                  configuration::Get().general_voxel_volume_parameters.voxel_size),
		iteration(0) {}


template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TMemoryDeviceType, ExecutionMode TExecutionMode>
LevelSetAlignmentEngine<TVoxel, TWarp, TIndex, TMemoryDeviceType, TExecutionMode>::LevelSetAlignmentEngine(
		const LevelSetAlignmentSwitches& switches, const LevelSetAlignmentTerminationConditions& termination_conditions) :
		LevelSetAlignmentEngineInterface<TVoxel, TWarp, TIndex>(
				LevelSetAlignmentParameters(LevelSetAlignmentParameters().execution_mode, LevelSetAlignmentWeights(), switches,
				                            termination_conditions, "")
		),
		weights(this->parameters.weights),
		switches(this->parameters.switches),
		termination(this->parameters.termination),
		warping_engine(WarpingEngineFactory::Build<TVoxel, TWarp, TIndex>(TMemoryDeviceType)),
		vector_update_threshold_in_voxels(termination.update_length_threshold /
		                                  configuration::Get().general_voxel_volume_parameters.voxel_size),
		iteration(0) {}

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
		                    "Warping vector update threshold: " << this->termination.update_length_threshold
		                                                        << " meters, " << this->vector_update_threshold_in_voxels
		                                                        << " voxels");
		LOG4CPLUS_TOP_LEVEL(logging::GetLogger(),
		                    bright_cyan << "*** *********************************** ***" << reset);
	}
}

// region ===================================== HOUSEKEEPING ===========================================================
template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TMemoryDeviceType, ExecutionMode TExecutionMode>
template<WarpType TWarpType>
void LevelSetAlignmentEngine<TVoxel, TWarp, TIndex, TMemoryDeviceType, TExecutionMode>::ClearOutWarps(
		VoxelVolume<TWarp, TIndex>* warp_field) const {
	VolumeTraversalEngine<TWarp, TIndex, TMemoryDeviceType>::
	template TraverseUtilized<ClearOutWarpStaticFunctor<TWarp, TWarpType>>(warp_field);
}

template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TMemoryDeviceType, ExecutionMode TExecutionMode>
void LevelSetAlignmentEngine<TVoxel, TWarp, TIndex, TMemoryDeviceType, TExecutionMode>::ClearOutCumulativeWarps(
		VoxelVolume<TWarp, TIndex>* warp_field) const {
	ClearOutWarps<WARP_CUMULATIVE>(warp_field);
};

template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TMemoryDeviceType, ExecutionMode TExecutionMode>
void LevelSetAlignmentEngine<TVoxel, TWarp, TIndex, TMemoryDeviceType, TExecutionMode>::ClearOutFramewiseWarps(
		VoxelVolume<TWarp, TIndex>* warp_field) const {
	ClearOutWarps<WARP_FRAMEWISE>(warp_field);
}

template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TMemoryDeviceType, ExecutionMode TExecutionMode>
void LevelSetAlignmentEngine<TVoxel, TWarp, TIndex, TMemoryDeviceType, TExecutionMode>::ClearOutWarpUpdates(
		VoxelVolume<TWarp, TIndex>* warp_field) const {
	ClearOutWarps<WARP_UPDATE>(warp_field);
}

// endregion ====================================== LOGGING ============================================================

// region ==================================== MOTION TRACKING LOOP ====================================================
template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TMemoryDeviceType, ExecutionMode TExecutionMode>
VoxelVolume<TVoxel, TIndex>*
LevelSetAlignmentEngine<TVoxel, TWarp, TIndex, TMemoryDeviceType, TExecutionMode>::Align(
		VoxelVolume<TWarp, TIndex>* warp_field,
		VoxelVolume<TVoxel, TIndex>** live_volume_pair,
		VoxelVolume<TVoxel, TIndex>* canonical_volume) {

	bool optimizationConverged = true;
	return this->Align(warp_field, live_volume_pair, canonical_volume, optimizationConverged);
}


template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TMemoryDeviceType, ExecutionMode TExecutionMode>
VoxelVolume<TVoxel, TIndex>*
LevelSetAlignmentEngine<TVoxel, TWarp, TIndex, TMemoryDeviceType, TExecutionMode>::Align(VoxelVolume<TWarp, TIndex>* warp_field,
                                                                                         VoxelVolume<TVoxel, TIndex>** live_volume_pair,
                                                                                         VoxelVolume<TVoxel, TIndex>* canonical_volume,
                                                                                         bool& optimizationConverged) {
	const float voxel_size = configuration::Get().general_voxel_volume_parameters.voxel_size;

	float gradient_length_statistic_in_voxels = std::numeric_limits<float>::infinity();

	int source_live_volume_index = 0;
	int target_live_volume_index = 1;
	for (iteration = 0;
	     iteration < termination.min_iteration_count ||
	     (this->weights.learning_rate * gradient_length_statistic_in_voxels > this->vector_update_threshold_in_voxels &&
	      iteration < termination.max_iteration_count);
	     iteration++) {
		PerformSingleOptimizationStep(canonical_volume, live_volume_pair[source_live_volume_index],
		                              live_volume_pair[target_live_volume_index], warp_field, gradient_length_statistic_in_voxels);

		std::swap(source_live_volume_index, target_live_volume_index);
	}

	optimizationConverged = iteration < termination.max_iteration_count;

	configuration::Configuration& config = configuration::Get();
	if (config.logging_settings.log_gradient_length_statistic) {
		LOG4CPLUS_PER_FRAME(logging::GetLogger(), "Frame-final level set evolution gradient length statistic (meters): "
				<< yellow << gradient_length_statistic_in_voxels * voxel_size << reset);
	}

	if (config.logging_settings.log_iteration_number) {
		LOG4CPLUS_PER_FRAME(logging::GetLogger(), "Frame-final level set evolution iteration count: " << yellow << iteration << reset);
	}


	return live_volume_pair[target_live_volume_index];
}


template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TMemoryDeviceType, ExecutionMode TExecutionMode>
void LevelSetAlignmentEngine<TVoxel, TWarp, TIndex, TMemoryDeviceType, TExecutionMode>::PerformSingleOptimizationStep(
		VoxelVolume<TVoxel, TIndex>* canonical_volume,
		VoxelVolume<TVoxel, TIndex>* source_live_volume,
		VoxelVolume<TVoxel, TIndex>* target_live_volume,
		VoxelVolume<TWarp, TIndex>* warp_field,
		float& gradient_length_statistic_in_voxels) {

	switch (TExecutionMode) {
		case DIAGNOSTIC:
			PerformSingleOptimizationStep_Diagnostic(canonical_volume, source_live_volume, target_live_volume, warp_field,
			                                         gradient_length_statistic_in_voxels);
			break;
		case OPTIMIZED:
			PerformSingleOptimizationStep_Optimized(canonical_volume, source_live_volume, target_live_volume, warp_field,
			                                        gradient_length_statistic_in_voxels);
			break;
	}
}

// region ===================================== CALCULATE WARPS ========================================================


template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TMemoryDeviceType, ExecutionMode TExecutionMode>
void
LevelSetAlignmentEngine<TVoxel, TWarp, TIndex, TMemoryDeviceType, TExecutionMode>::CalculateEnergyGradient(
		VoxelVolume<TWarp, TIndex>* warp_field,
		VoxelVolume<TVoxel, TIndex>* canonical_volume,
		VoxelVolume<TVoxel, TIndex>* live_volume) {

	// clear out gradient
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
void LevelSetAlignmentEngine<TVoxel, TWarp, TIndex, TMemoryDeviceType, TExecutionMode>::SmoothEnergyGradient(
		VoxelVolume<TWarp, TIndex>* warp_field,
		VoxelVolume<TVoxel, TIndex>* canonical_volume,
		VoxelVolume<TVoxel, TIndex>* live_volume) {

	if (this->switches.enable_Sobolev_gradient_smoothing) {
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
void LevelSetAlignmentEngine<TVoxel, TWarp, TIndex, TMemoryDeviceType, TExecutionMode>::UpdateDeformationFieldUsingGradient(
		VoxelVolume<TWarp, TIndex>* warp_field,
		VoxelVolume<TVoxel, TIndex>* canonical_volume,
		VoxelVolume<TVoxel, TIndex>* live_volume) {
	if (this->switches.enable_Sobolev_gradient_smoothing) {
		// if Sobolev smoothing has been enabled, .gradient1 field of each TWarp voxel has to be used,
		// since that's where the final result ends up after ping-ponging three times between .gradient0 and .gradient1
		WarpUpdateFunctor<TVoxel, TWarp, TMemoryDeviceType, true> warp_update_functor(this->weights.learning_rate);
		ThreeVolumeTraversalEngine<TWarp, TVoxel, TVoxel, TIndex, TMemoryDeviceType>::
		TraverseUtilizedWithPosition(warp_field, canonical_volume, live_volume, warp_update_functor);
	} else {
		WarpUpdateFunctor<TVoxel, TWarp, TMemoryDeviceType, false> warp_update_functor(this->weights.learning_rate);
		ThreeVolumeTraversalEngine<TWarp, TVoxel, TVoxel, TIndex, TMemoryDeviceType>::
		TraverseUtilizedWithPosition(warp_field, canonical_volume, live_volume, warp_update_functor);
	}
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

template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TMemoryDeviceType, ExecutionMode TExecutionMode>
void LevelSetAlignmentEngine<TVoxel, TWarp, TIndex, TMemoryDeviceType, TExecutionMode>::PerformSingleOptimizationStep_Diagnostic(
		VoxelVolume<TVoxel, TIndex>* canonical_volume,
		VoxelVolume<TVoxel, TIndex>* source_live_volume,
		VoxelVolume<TVoxel, TIndex>* target_live_volume,
		VoxelVolume<TWarp, TIndex>* warp_field,
		float& gradient_length_statistic_in_voxels) {
	auto& config = configuration::Get();

	if (config.logging_settings.log_iteration_number) {
		LOG4CPLUS_PER_ITERATION(logging::GetLogger(), red << "Iteration: " << iteration << reset);
	}

	if (config.logging_settings.log_surface_tracking_procedure_names) {
		LOG4CPLUS_PER_ITERATION(logging::GetLogger(), bright_cyan << "Calculating warp energy gradient..." << reset);
	}

	bench::start_timer("TrackMotion_1_CalculateEnergyGradient");
	CalculateEnergyGradient(warp_field, canonical_volume, source_live_volume);
	bench::stop_timer("TrackMotion_1_CalculateEnergyGradient");

	if (this->parameters.switches.enable_Sobolev_gradient_smoothing && config.logging_settings.log_surface_tracking_procedure_names) {
		LOG4CPLUS_PER_ITERATION(logging::GetLogger(),
		                        bright_cyan << "Applying Sobolev smoothing to energy gradient..." << reset);
	}
	bench::start_timer("TrackMotion_2_SmoothEnergyGradient");
	SmoothEnergyGradient(warp_field, canonical_volume, source_live_volume);
	bench::stop_timer("TrackMotion_2_SmoothEnergyGradient");

	if (config.logging_settings.log_surface_tracking_procedure_names) {
		LOG4CPLUS_PER_ITERATION(logging::GetLogger(),
		                        bright_cyan << "Applying warp update (based on energy gradient) to the cumulative warp..." << reset);
	}

	bench::start_timer("TrackMotion_3_UpdateGradientLengthStatistic");
	UpdateGradientLengthStatistic(gradient_length_statistic_in_voxels, warp_field);
	bench::stop_timer("TrackMotion_3_UpdateGradientLengthStatistic");

	bench::start_timer("TrackMotion_4_UpdateDeformationFieldUsingGradient");
	UpdateDeformationFieldUsingGradient(warp_field, canonical_volume, source_live_volume);
	bench::stop_timer("TrackMotion_4_UpdateDeformationFieldUsingGradient");

	TelemetryRecorder<TVoxel, TWarp, TIndex, TMemoryDeviceType>::GetDefaultInstance()
			.RecordSurfaceTrackingMeanUpdate(gradient_length_statistic_in_voxels);

	if (config.logging_settings.log_gradient_length_statistic) {
		LOG4CPLUS_PER_ITERATION(logging::GetLogger(), "[Gradient length statistic (meters)] * learning_rate vs. threshold: ");
		LOG4CPLUS_PER_ITERATION(logging::GetLogger(), "   "
				<< yellow << gradient_length_statistic_in_voxels * configuration::Get().general_voxel_volume_parameters.voxel_size
				             * this->weights.learning_rate << " vs. " << this->termination.update_length_threshold << reset);
	}

	if (config.logging_settings.log_surface_tracking_procedure_names) {
		LOG4CPLUS_PER_ITERATION(logging::GetLogger(),
		                        bright_cyan << "Updating live frame SDF by mapping from raw live SDF "
		                                       "to new warped SDF based on latest warp..." << reset);
	}
	bench::start_timer("TrackMotion_4_WarpLiveScene");
	// special case for testing
	if (target_live_volume != nullptr) {
		warping_engine->WarpVolume_WarpUpdates(warp_field, source_live_volume, target_live_volume);
	}
	bench::stop_timer("TrackMotion_4_WarpLiveScene");

	auto& telemetry_recorder = TelemetryRecorder<TVoxel, TWarp, TIndex, TMemoryDeviceType>::GetDefaultInstance();
	telemetry_recorder.RecordAndLogWarpUpdateLengthHistogram(*warp_field, iteration);
}

template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TMemoryDeviceType, ExecutionMode TExecutionMode>
void LevelSetAlignmentEngine<TVoxel, TWarp, TIndex, TMemoryDeviceType, TExecutionMode>::PerformSingleOptimizationStep_Optimized(
		VoxelVolume<TVoxel, TIndex>* canonical_volume,
		VoxelVolume<TVoxel, TIndex>* source_live_volume,
		VoxelVolume<TVoxel, TIndex>* target_live_volume,
		VoxelVolume<TWarp, TIndex>* warp_field,
		float& gradient_length_statistic_in_voxels) {

	CalculateEnergyGradient(warp_field, canonical_volume, source_live_volume);
	SmoothEnergyGradient(warp_field, canonical_volume, source_live_volume);
	UpdateGradientLengthStatistic(gradient_length_statistic_in_voxels, warp_field);
	UpdateDeformationFieldUsingGradient(warp_field, canonical_volume, source_live_volume);
	warping_engine->WarpVolume_WarpUpdates(warp_field, source_live_volume, target_live_volume);
}

template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TMemoryDeviceType, ExecutionMode TExecutionMode>
void LevelSetAlignmentEngine<TVoxel, TWarp, TIndex, TMemoryDeviceType, TExecutionMode>::UpdateGradientLengthStatistic(
		float& gradient_length_statistic_in_voxels, VoxelVolume<TWarp, TIndex>* warp_field) {
	Vector3i maximum_warp_length_position;
	switch (this->termination.warp_length_termination_threshold_type) {
		case AVERAGE:
			this->FindAverageGradientLength(gradient_length_statistic_in_voxels, warp_field);
			break;
		case MAXIMUM:
			this->FindMaximumGradientLength(gradient_length_statistic_in_voxels, maximum_warp_length_position, warp_field);
			break;
		default:
			DIEWITHEXCEPTION_REPORTLOCATION("Unsupported warp termination threshold type, aborting.");
	}
}

template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TMemoryDeviceType, ExecutionMode TExecutionMode>
void LevelSetAlignmentEngine<TVoxel, TWarp, TIndex, TMemoryDeviceType, TExecutionMode>::FindMaximumGradientLength(
		float& maximum_warp_length, Vector3i& position,
		VoxelVolume<TWarp, TIndex>* warp_field) {
	ReductionResult<float, TIndex> ignored_value;
	typedef ReduceMaximumFunctor<TIndex, TMemoryDeviceType> ReduceFunctorType;
	ignored_value.value = FLT_MIN;
	if (this->switches.enable_Sobolev_gradient_smoothing) {
		typedef RetrieveGradientLengthFunctor<TWarp, TMemoryDeviceType, true> RetrieveFunctorType;
		maximum_warp_length = VolumeReductionEngine<TWarp, TIndex, TMemoryDeviceType>::
		template ReduceUtilized<ReduceFunctorType, RetrieveFunctorType>(position, warp_field, ignored_value);
	} else {
		typedef RetrieveGradientLengthFunctor<TWarp, TMemoryDeviceType, false> RetrieveFunctorType;
		maximum_warp_length = VolumeReductionEngine<TWarp, TIndex, TMemoryDeviceType>::
		template ReduceUtilized<ReduceFunctorType, RetrieveFunctorType>(position, warp_field, ignored_value);
	}
}

template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TMemoryDeviceType, ExecutionMode TExecutionMode>
void LevelSetAlignmentEngine<TVoxel, TWarp, TIndex, TMemoryDeviceType, TExecutionMode>::FindAverageGradientLength(
		float& average_warp_length,
		VoxelVolume<TWarp, TIndex>* warp_field) {
	ReductionResult<SumAndCount, TIndex> ignored_value;
	ignored_value.value.sum = 0.0f;
	ignored_value.value.count = 0u;
	typedef ReduceSumAndCountFunctor<TIndex, TMemoryDeviceType> ReduceFunctorType;

	Vector3i position;
	SumAndCount final_sum_and_count;
	if (this->switches.enable_Sobolev_gradient_smoothing) {
		typedef RetrieveGradientLengthAndCountFunctor<TWarp, TMemoryDeviceType, true> RetrieveFunctorType;
		final_sum_and_count = VolumeReductionEngine<TWarp, TIndex, TMemoryDeviceType>::
		template ReduceUtilized<ReduceFunctorType, RetrieveFunctorType>(position, warp_field, ignored_value);
	} else {
		typedef RetrieveGradientLengthAndCountFunctor<TWarp, TMemoryDeviceType, false> RetrieveFunctorType;
		final_sum_and_count = VolumeReductionEngine<TWarp, TIndex, TMemoryDeviceType>::
		template ReduceUtilized<ReduceFunctorType, RetrieveFunctorType>(position, warp_field, ignored_value);
	}
	average_warp_length = final_sum_and_count.sum / final_sum_and_count.count;
}






//endregion ============================================================================================================
