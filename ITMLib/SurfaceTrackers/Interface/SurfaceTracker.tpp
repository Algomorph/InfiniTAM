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
#include "../../Engines/Analytics/AnalyticsEngineInterface.h"
#include "../Shared/SurfaceTrackerSharedFunctors.h"
#include "../../Engines/Traversal/Interface/VolumeTraversal.h"
#include "../../Engines/Traversal/Interface/ThreeVolumeTraversal.h"

using namespace ITMLib;

// region ===================================== HOUSEKEEPING ===========================================================
template<typename T_TSDF_Voxel, typename TWarpVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType, GradientFunctorType TGradientFunctorType>
template<WarpType TWarpType>
void SurfaceTracker<T_TSDF_Voxel, TWarpVoxel, TIndex, TMemoryDeviceType, TGradientFunctorType>::ClearOutWarps(
		VoxelVolume<TWarpVoxel, TIndex>* warp_field) {
	VolumeTraversalEngine<TWarpVoxel, TIndex, TMemoryDeviceType>::
#ifdef TRAVERSE_ALL_HASH_BLOCKS
	template TraverseAll<ClearOutFramewiseWarpStaticFunctor<TWarpVoxel, TWarpVoxel::hasFramewiseWarp>>(warp_field);
#else
	template TraverseUtilized<ClearOutWarpStaticFunctor<TWarpVoxel, TWarpType>>(warp_field);
#endif
}

//#define TRAVERSE_ALL_HASH_BLOCKS
template<typename T_TSDF_Voxel, typename TWarpVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType, GradientFunctorType TGradientFunctorType>
void SurfaceTracker<T_TSDF_Voxel, TWarpVoxel, TIndex, TMemoryDeviceType, TGradientFunctorType>::ClearOutCumulativeWarps(
		VoxelVolume<TWarpVoxel, TIndex>* warp_field) {
	ClearOutWarps<WARP_CUMULATIVE>(warp_field);
};

template<typename T_TSDF_Voxel, typename TWarpVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType, GradientFunctorType TGradientFunctorType>
void SurfaceTracker<T_TSDF_Voxel, TWarpVoxel, TIndex, TMemoryDeviceType, TGradientFunctorType>::ClearOutFramewiseWarps(
		VoxelVolume<TWarpVoxel, TIndex>* warp_field) {
	ClearOutWarps<WARP_FRAMEWISE>(warp_field);
}

template<typename T_TSDF_Voxel, typename TWarpVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType, GradientFunctorType TGradientFunctorType>
void SurfaceTracker<T_TSDF_Voxel, TWarpVoxel, TIndex, TMemoryDeviceType, TGradientFunctorType>::ClearOutWarpUpdates(
		VoxelVolume<TWarpVoxel, TIndex>* warp_field) {
	ClearOutWarps<WARP_UPDATE>(warp_field);
}

// endregion ===========================================================================================================

template<typename T_TSDF_Voxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
inline static void PrintVolumeStatistics(
		VoxelVolume<T_TSDF_Voxel, TIndex>* volume,
		std::string description) {
	AnalyticsEngine<T_TSDF_Voxel, TIndex, TMemoryDeviceType>& calculator =
			AnalyticsEngine<T_TSDF_Voxel, TIndex, TMemoryDeviceType>::Instance();
	std::cout << green << "=== Stats for scene '" << description << "' ===" << reset << std::endl;
	std::cout << "    Total voxel count: " << calculator.CountAllocatedVoxels(volume) << std::endl;
	std::cout << "    NonTruncated voxel count: " << calculator.CountNonTruncatedVoxels(volume) << std::endl;
	std::cout << "    +1.0 voxel count: " << calculator.CountVoxelsWithSpecificSdfValue(volume, 1.0f) << std::endl;
	std::vector<int> allocatedHashes = calculator.GetAllocatedHashCodes(volume);
	std::cout << "    Allocated hash count: " << allocatedHashes.size() << std::endl;
	std::cout << "    NonTruncated SDF sum: " << calculator.SumNonTruncatedVoxelAbsSdf(volume) << std::endl;
	std::cout << "    Truncated SDF sum: " << calculator.SumTruncatedVoxelAbsSdf(volume) << std::endl;
};

// region ===================================== CALCULATE WARPS ========================================================


template<typename T_TSDF_Voxel, typename TWarpVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType, GradientFunctorType TGradientFunctorType>
void
SurfaceTracker<T_TSDF_Voxel, TWarpVoxel, TIndex, TMemoryDeviceType, TGradientFunctorType>::CalculateWarpGradient(
		VoxelVolume<TWarpVoxel, TIndex>* warp_field,
		VoxelVolume<T_TSDF_Voxel, TIndex>* canonical_volume,
		VoxelVolume<T_TSDF_Voxel, TIndex>* live_volume) {

	// manage hash
	VolumeTraversalEngine<TWarpVoxel, TIndex, TMemoryDeviceType>::template
#ifdef TRAVERSE_ALL_HASH_BLOCKS
	TraverseAll<ClearOutGradientStaticFunctor<TWarpVoxel>>(warp_field);
#else
	TraverseUtilized<ClearOutGradientStaticFunctor<TWarpVoxel>>(warp_field);
#endif

	WarpGradientFunctor<T_TSDF_Voxel, TWarpVoxel, TIndex, TMemoryDeviceType, TGradientFunctorType>
			calculate_gradient_functor(this->parameters, this->switches, warp_field, canonical_volume, live_volume,
			                           canonical_volume->GetParameters().voxel_size,
			                           canonical_volume->GetParameters().narrow_band_half_width);

	ThreeVolumeTraversalEngine<TWarpVoxel, T_TSDF_Voxel, T_TSDF_Voxel, TIndex, TMemoryDeviceType>::
#ifdef TRAVERSE_ALL_HASH_BLOCKS
	TraverseAllWithPosition(warp_field, canonical_volume, live_volume, calculate_gradient_functor);
#else
	TraverseUtilizedWithPosition(warp_field,  canonical_volume, live_volume, calculate_gradient_functor);
#endif
	calculate_gradient_functor.PrintStatistics();
}

// endregion ===========================================================================================================
// region ========================================== SOBOLEV GRADIENT SMOOTHING ========================================


template<typename T_TSDF_Voxel, typename TWarpVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType, GradientFunctorType TGradientFunctorType>
void SurfaceTracker<T_TSDF_Voxel, TWarpVoxel, TIndex, TMemoryDeviceType, TGradientFunctorType>::SmoothWarpGradient(
		VoxelVolume<TWarpVoxel, TIndex>* warp_field,
		VoxelVolume<T_TSDF_Voxel, TIndex>* canonical_volume,
		VoxelVolume<T_TSDF_Voxel, TIndex>* live_volume) {

	if (this->switches.enable_sobolev_gradient_smoothing) {
		GradientSmoothingPassFunctor<T_TSDF_Voxel, TWarpVoxel, TIndex, X> smoothing_pass_functor_X(warp_field);
		GradientSmoothingPassFunctor<T_TSDF_Voxel, TWarpVoxel, TIndex, Y> smoothing_pass_functor_Y(warp_field);
		GradientSmoothingPassFunctor<T_TSDF_Voxel, TWarpVoxel, TIndex, Z> smoothing_pass_functor_Z(warp_field);

		ITMLib::ThreeVolumeTraversalEngine<TWarpVoxel, T_TSDF_Voxel, T_TSDF_Voxel, TIndex, TMemoryDeviceType>::
#ifdef TRAVERSE_ALL_HASH_BLOCKS
		template TraverseAllWithPosition(warp_field, canonical_volume, live_volume, smoothing_pass_functor_X);
#else
		template TraverseUtilizedWithPosition(warp_field, canonical_volume, live_volume, smoothing_pass_functor_X);
#endif
		ITMLib::ThreeVolumeTraversalEngine<TWarpVoxel, T_TSDF_Voxel, T_TSDF_Voxel, TIndex, TMemoryDeviceType>::
#ifdef TRAVERSE_ALL_HASH_BLOCKS
		template TraverseAllWithPosition(warp_field, canonical_volume, live_volume, smoothing_pass_functor_Y);
#else
		template TraverseUtilizedWithPosition(warp_field, canonical_volume, live_volume, smoothing_pass_functor_Y);
#endif
		ITMLib::ThreeVolumeTraversalEngine<TWarpVoxel, T_TSDF_Voxel, T_TSDF_Voxel, TIndex, TMemoryDeviceType>::
#ifdef TRAVERSE_ALL_HASH_BLOCKS
		template TraverseAllWithPosition(warp_field, canonical_volume, live_volume, smoothing_pass_functor_Z);
#else
		template TraverseUtilizedWithPosition(warp_field, canonical_volume, live_volume, smoothing_pass_functor_Z);
#endif
	}
}

// endregion ===========================================================================================================

// region ============================= UPDATE FRAMEWISE & GLOBAL (CUMULATIVE) WARPS ===================================
template<typename T_TSDF_Voxel, typename TWarpVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType, GradientFunctorType TGradientFunctorType>
float SurfaceTracker<T_TSDF_Voxel, TWarpVoxel, TIndex, TMemoryDeviceType, TGradientFunctorType>::UpdateWarps(
		VoxelVolume<TWarpVoxel, TIndex>* warp_field,
		VoxelVolume<T_TSDF_Voxel, TIndex>* canonical_volume,
		VoxelVolume<T_TSDF_Voxel, TIndex>* live_volume) {

	WarpUpdateFunctor<T_TSDF_Voxel, TWarpVoxel, TMemoryDeviceType>
			warp_update_functor(this->parameters.learning_rate,
			                    ITMLib::configuration::get().non_rigid_tracking_parameters.momentum_weight,
			                    this->switches.enable_sobolev_gradient_smoothing);

	ThreeVolumeTraversalEngine<TWarpVoxel, T_TSDF_Voxel, T_TSDF_Voxel, TIndex, TMemoryDeviceType>::
#ifdef TRAVERSE_ALL_HASH_BLOCKS
	TraverseAllWithPosition(warp_field, canonical_volume, live_volume, warp_update_functor);
#else
	TraverseUtilizedWithPosition(warp_field, canonical_volume, live_volume, warp_update_functor);
#endif

	//TODO: move histogram printing / logging to a separate function
	//don't compute_allocated histogram in CUDA version
#ifndef __CUDACC__
	if (histograms_enabled) {
		WarpHistogramFunctor<T_TSDF_Voxel, TWarpVoxel>
				warp_histogram_functor(/*warp_update_functor.GetMaxFramewiseWarpLength(),*/
				                       warp_update_functor.GetMaxWarpUpdateLength());
		ThreeVolumeTraversalEngine<TWarpVoxel, T_TSDF_Voxel, T_TSDF_Voxel, TIndex, TMemoryDeviceType>::
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

template<typename T_TSDF_Voxel, typename TWarpVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType, GradientFunctorType TGradientFunctorType>
void SurfaceTracker<T_TSDF_Voxel, TWarpVoxel, TIndex, TMemoryDeviceType, TGradientFunctorType>::AddFramewiseWarpToWarp(
		VoxelVolume<TWarpVoxel, TIndex>* warp_field, bool clear_framewise_warps) {
	if (clear_framewise_warps) {
		VolumeTraversalEngine<TWarpVoxel, TIndex, TMemoryDeviceType>::
#ifdef TRAVERSE_ALL_HASH_BLOCKS
		template TraverseAll<
				AddFramewiseWarpToWarpWithClearStaticFunctor<TWarpVoxel, TWarpVoxel::hasCumulativeWarp>>(warp_field);
#else
		template TraverseUtilized<
				AddFramewiseWarpToWarpWithClearStaticFunctor<TWarpVoxel, TWarpVoxel::hasCumulativeWarp>>(warp_field);
#endif
	} else {
		VolumeTraversalEngine<TWarpVoxel, TIndex, TMemoryDeviceType>::
#ifdef TRAVERSE_ALL_HASH_BLOCKS
		template TraverseAll<
				AddFramewiseWarpToWarpStaticFunctor<TWarpVoxel, TWarpVoxel::hasCumulativeWarp>>(warp_field);
#else
		template TraverseUtilized<
				AddFramewiseWarpToWarpStaticFunctor<TWarpVoxel, TWarpVoxel::hasCumulativeWarp>>(warp_field);
#endif
	}
}



//endregion ============================================================================================================