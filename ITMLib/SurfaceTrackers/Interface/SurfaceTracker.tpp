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
#include "SurfaceTracker.h"
#include "../WarpGradientFunctors/WarpGradientFunctor.h"
#include "../../Engines/Indexing/Interface/IndexingEngine.h"
#include "../../Utils/Analytics/VolumeStatisticsCalculator/Interface/VolumeStatisticsCalculatorInterface.h"
#include "../Shared/SurfaceTrackerSharedFunctors.h"
#include "../../Engines/Traversal/Interface/VolumeTraversal.h"
#include "../../Engines/Traversal/Interface/ThreeVolumeTraversal.h"

using namespace ITMLib;

// region ===================================== HOUSEKEEPING ===========================================================


template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TMemoryDeviceType, GradientFunctorType TGradientFunctorType>
void SurfaceTracker<TVoxel, TWarp, TIndex, TMemoryDeviceType, TGradientFunctorType>::ResetWarps(
		VoxelVolume<TWarp, TIndex>* warp_field) {
	VolumeTraversalEngine<TWarp, TIndex, TMemoryDeviceType>::
#ifdef TRAVERSE_ALL_HASH_BLOCKS
	template TraverseAll<WarpClearFunctor<TWarp, TWarp::hasCumulativeWarp>>(warp_field);
#else
	template TraverseUtilized<WarpClearFunctor<TWarp, TWarp::hasCumulativeWarp>>(warp_field);
#endif
};


template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TMemoryDeviceType, GradientFunctorType TGradientFunctorType>
void SurfaceTracker<TVoxel, TWarp, TIndex, TMemoryDeviceType, TGradientFunctorType>::ClearOutFramewiseWarp(
		VoxelVolume<TWarp, TIndex>* warp_field) {

	VolumeTraversalEngine<TWarp, TIndex, TMemoryDeviceType>::
#ifdef TRAVERSE_ALL_HASH_BLOCKS
	template TraverseAll<ClearOutFramewiseWarpStaticFunctor<TWarp>>(warp_field);
#else
	template TraverseUtilized<ClearOutFramewiseWarpStaticFunctor<TWarp>>(warp_field);
#endif
}


// endregion ===========================================================================================================

template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
inline static void PrintVolumeStatistics(
		VoxelVolume<TVoxel, TIndex>* volume,
		std::string description) {
	VolumeStatisticsCalculator<TVoxel, TIndex, TMemoryDeviceType>& calculator =
			VolumeStatisticsCalculator<TVoxel, TIndex, TMemoryDeviceType>::Instance();
	std::cout << green << "=== Stats for scene '" << description << "' ===" << reset << std::endl;
	std::cout << "    Total voxel count: " << calculator.ComputeAllocatedVoxelCount(volume) << std::endl;
	std::cout << "    NonTruncated voxel count: " << calculator.ComputeNonTruncatedVoxelCount(volume) << std::endl;
	std::cout << "    +1.0 voxel count: " << calculator.CountVoxelsWithSpecificSdfValue(volume, 1.0f) << std::endl;
	std::vector<int> allocatedHashes = calculator.GetAllocatedHashCodes(volume);
	std::cout << "    Allocated hash count: " << allocatedHashes.size() << std::endl;
	std::cout << "    NonTruncated SDF sum: " << calculator.ComputeNonTruncatedVoxelAbsSdfSum(volume) << std::endl;
	std::cout << "    Truncated SDF sum: " << calculator.ComputeTruncatedVoxelAbsSdfSum(volume) << std::endl;
};

// region ===================================== CALCULATE WARPS ========================================================


template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TMemoryDeviceType, GradientFunctorType TGradientFunctorType>
void
SurfaceTracker<TVoxel, TWarp, TIndex, TMemoryDeviceType, TGradientFunctorType>::CalculateWarpGradient(
		VoxelVolume<TWarp, TIndex>* warp_field,
		VoxelVolume<TVoxel, TIndex>* canonical_volume,
		VoxelVolume<TVoxel, TIndex>* live_volume) {

	// manage hash
	VolumeTraversalEngine<TWarp, TIndex, TMemoryDeviceType>::template
#ifdef TRAVERSE_ALL_HASH_BLOCKS
	TraverseAll<ClearOutGradientStaticFunctor<TWarp>>(warp_field);
#else
	TraverseUtilized<ClearOutGradientStaticFunctor<TWarp>>(warp_field);
#endif

	WarpGradientFunctor<TVoxel, TWarp, TIndex, TMemoryDeviceType, TGradientFunctorType>
			calculate_gradient_functor(this->parameters, this->switches, warp_field, canonical_volume, live_volume,
			                           canonical_volume->sceneParams->voxel_size,
			                           canonical_volume->sceneParams->narrow_band_half_width);

	ThreeVolumeTraversalEngine<TWarp, TVoxel, TVoxel, TIndex, TMemoryDeviceType>::
#ifdef TRAVERSE_ALL_HASH_BLOCKS
	TraverseAllWithPosition(warp_field, live_volume, canonical_volume, calculate_gradient_functor);
#else
	TraverseUtilizedWithPosition(warp_field, live_volume, canonical_volume, calculate_gradient_functor);
#endif
	calculate_gradient_functor.PrintStatistics();
}

// endregion ===========================================================================================================
// region ========================================== SOBOLEV GRADIENT SMOOTHING ========================================


template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TMemoryDeviceType, GradientFunctorType TGradientFunctorType>
void SurfaceTracker<TVoxel, TWarp, TIndex, TMemoryDeviceType, TGradientFunctorType>::SmoothWarpGradient(
		VoxelVolume<TWarp, TIndex>* warp_field,
		VoxelVolume<TVoxel, TIndex>* canonical_volume,
		VoxelVolume<TVoxel, TIndex>* live_volume) {

	if (this->switches.enable_sobolev_gradient_smoothing) {
		GradientSmoothingPassFunctor<TVoxel, TWarp, TIndex, X> smoothing_pass_functor_X(warp_field);
		GradientSmoothingPassFunctor<TVoxel, TWarp, TIndex, Y> smoothing_pass_functor_Y(warp_field);
		GradientSmoothingPassFunctor<TVoxel, TWarp, TIndex, Z> smoothing_pass_functor_Z(warp_field);

		ITMLib::ThreeVolumeTraversalEngine<TWarp, TVoxel, TVoxel, TIndex, TMemoryDeviceType>::
#ifdef TRAVERSE_ALL_HASH_BLOCKS
		template TraverseAllWithPosition(warp_field, canonical_volume, live_volume, smoothing_pass_functor_X);
#else
		template TraverseUtilizedWithPosition(warp_field, canonical_volume, live_volume, smoothing_pass_functor_X);
#endif
		ITMLib::ThreeVolumeTraversalEngine<TWarp, TVoxel, TVoxel, TIndex, TMemoryDeviceType>::
#ifdef TRAVERSE_ALL_HASH_BLOCKS
		template TraverseAllWithPosition(warp_field, canonical_volume, live_volume, smoothing_pass_functor_Y);
#else
		template TraverseUtilizedWithPosition(warp_field, canonical_volume, live_volume, smoothing_pass_functor_Y);
#endif
		ITMLib::ThreeVolumeTraversalEngine<TWarp, TVoxel, TVoxel, TIndex, TMemoryDeviceType>::
#ifdef TRAVERSE_ALL_HASH_BLOCKS
		template TraverseAllWithPosition(warp_field, canonical_volume, live_volume, smoothing_pass_functor_Z);
#else
		template TraverseUtilizedWithPosition(warp_field, canonical_volume, live_volume, smoothing_pass_functor_Z);
#endif
	}
}

// endregion ===========================================================================================================

// region ============================= UPDATE FRAMEWISE & GLOBAL (CUMULATIVE) WARPS ===================================
template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TMemoryDeviceType, GradientFunctorType TGradientFunctorType>
float SurfaceTracker<TVoxel, TWarp, TIndex, TMemoryDeviceType, TGradientFunctorType>::UpdateWarps(
		VoxelVolume<TWarp, TIndex>* warp_field,
		VoxelVolume<TVoxel, TIndex>* canonical_volume,
		VoxelVolume<TVoxel, TIndex>* live_volume) {

	WarpUpdateFunctor<TVoxel, TWarp, TMemoryDeviceType>
			warp_update_functor(this->parameters.learning_rate,
			                    ITMLib::configuration::get().non_rigid_tracking_parameters.momentum_weight,
			                    this->switches.enable_sobolev_gradient_smoothing);

	ThreeVolumeTraversalEngine<TWarp, TVoxel, TVoxel, TIndex, TMemoryDeviceType>::
#ifdef TRAVERSE_ALL_HASH_BLOCKS
	TraverseAllWithPosition(warp_field, canonical_volume, live_volume, warp_update_functor);
#else
	TraverseUtilizedWithPosition(warp_field, canonical_volume, live_volume, warp_update_functor);
#endif

	//TODO: move histogram printing / logging to a separate function
	//don't compute histogram in CUDA version
#ifndef __CUDACC__
	if (histograms_enabled) {
		WarpHistogramFunctor<TVoxel, TWarp>
				warp_histogram_functor(warp_update_functor.GetMaxFramewiseWarpLength(),
				                       warp_update_functor.GetMaxWarpUpdateLength());
		ThreeVolumeTraversalEngine<TWarp, TVoxel, TVoxel, TIndex, TMemoryDeviceType>::
#ifdef TRAVERSE_ALL_HASH_BLOCKS
		TraverseAll(warp_field, canonical_volume, live_volume, warp_histogram_functor);
#else
		TraverseUtilized(warp_field, canonical_volume, live_volume, warp_histogram_functor);
#endif
		warp_histogram_functor.PrintHistogram();
		warp_update_functor.PrintWarp();
	}
#endif
	//return warp_update_functor.max_warp_update_length;
	return warp_update_functor.GetMaxWarpUpdateLength();
}

template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TMemoryDeviceType, GradientFunctorType TGradientFunctorType>
void SurfaceTracker<TVoxel, TWarp, TIndex, TMemoryDeviceType, TGradientFunctorType>::AddFramewiseWarpToWarp(
		VoxelVolume<TWarp, TIndex>* warp_field, bool clear_framewise_warps) {
	if (clear_framewise_warps) {
		VolumeTraversalEngine<TWarp, TIndex, TMemoryDeviceType>::
#ifdef TRAVERSE_ALL_HASH_BLOCKS
		template TraverseAll<
				AddFramewiseWarpToWarpWithClearStaticFunctor<TWarp, TWarp::hasCumulativeWarp>>(warp_field);
#else
		template TraverseUtilized<
				AddFramewiseWarpToWarpWithClearStaticFunctor<TWarp, TWarp::hasCumulativeWarp>>(warp_field);
#endif
	} else {
		VolumeTraversalEngine<TWarp, TIndex, TMemoryDeviceType>::
#ifdef TRAVERSE_ALL_HASH_BLOCKS
		template TraverseAll<
				AddFramewiseWarpToWarpStaticFunctor<TWarp, TWarp::hasCumulativeWarp>>(warp_field);
#else
		template TraverseUtilized<
				AddFramewiseWarpToWarpStaticFunctor<TWarp, TWarp::hasCumulativeWarp>>(warp_field);
#endif
	}
}

//endregion ============================================================================================================