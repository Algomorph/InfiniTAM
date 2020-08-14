//  ================================================================
//  Created by Gregory Kramida on 7/12/18.
//  Copyright (c) 2018-2000 Gregory Kramida
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

#include "../../Utils/Math.h"
#include "../../Objects/Volume/VoxelVolume.h"
#include "../../Utils/Configuration/Configuration.h"
#include "SurfaceTrackerSharedRoutines.h"
#include "../../../ORUtils/PlatformIndependentAtomics.h"
#include "../../../ORUtils/CrossPlatformMacros.h"
#include "../../Engines/Traversal/Interface/VolumeTraversal.h"
#include "../../Engines/Traversal/Interface/ThreeVolumeTraversal.h"
#include "../../Engines/Common/WarpAccessFunctors.h"

#ifdef __CUDACC__
#include "../../Utils/CUDAUtils.h"
#endif


template<typename TWarpVoxel, WarpType TWarpType>
struct ClearOutWarpStaticFunctor {
	_CPU_AND_GPU_CODE_
	static inline void run(TWarpVoxel& voxel) {
		WarpAccessStaticFunctor<TWarpVoxel, TWarpType>::SetWarp(voxel, Vector3f(0.0f));
	}
};

template<typename TWarpVoxel>
struct ClearOutGradientStaticFunctor {
	_CPU_AND_GPU_CODE_
	static inline void run(TWarpVoxel& voxel) {
		voxel.gradient0 = Vector3f(0.0f);
		voxel.gradient1 = Vector3f(0.0f);
	}
};


template<typename TTSDFVoxel, typename TWarpVoxel, MemoryDeviceType TMemoryDeviceType>
struct WarpUpdateFunctor {
	WarpUpdateFunctor(float learning_rate, float momentum_weight, bool gradient_smoothing_enabled) :
			gradient_weight(learning_rate * (1.0f - momentum_weight)), momentum_weight(momentum_weight),
			gradient_smoothing_enabled(gradient_smoothing_enabled) {
		INITIALIZE_ATOMIC(float, aggregate_warp_update_length, 0.0f);
		INITIALIZE_ATOMIC(unsigned int, affected_voxel_count, 0u);
	}

	~WarpUpdateFunctor() {
		CLEAN_UP_ATOMIC(aggregate_warp_update_length);
		CLEAN_UP_ATOMIC(affected_voxel_count);
	}

	_DEVICE_WHEN_AVAILABLE_
	void
	operator()(TWarpVoxel& warp_voxel, TTSDFVoxel& canonical_voxel, TTSDFVoxel& live_voxel, const Vector3i& position) {
		if (!VoxelIsConsideredForTracking(canonical_voxel, live_voxel)) return;

		Vector3f warp_update = -gradient_weight * (gradient_smoothing_enabled ?
		                                           warp_voxel.gradient1 : warp_voxel.gradient0);

		warp_voxel.warp_update = warp_update + momentum_weight * warp_voxel.warp_update;

		// update stats
		float warp_update_length = ORUtils::length(warp_update);

		ATOMIC_ADD(aggregate_warp_update_length, warp_update_length);
		ATOMIC_ADD(affected_voxel_count, 1u);
	}

	float GetAverageWarpUpdateLength() {
		return GET_ATOMIC_VALUE_CPU(aggregate_warp_update_length) / GET_ATOMIC_VALUE_CPU(affected_voxel_count);
	}


private:
	DECLARE_ATOMIC_FLOAT(aggregate_warp_update_length);
	DECLARE_ATOMIC(unsigned int, affected_voxel_count);
	const float gradient_weight;
	const float momentum_weight;
	const bool gradient_smoothing_enabled;
};



template<typename TWarpVoxel, MemoryDeviceType TMemoryDeviceType>
struct WarpHistogramFunctor {
	WarpHistogramFunctor(float max_warp_update_length) :
	 max_warp_update_length(max_warp_update_length) {
	}

	static const int histogram_bin_count = 10;

	_DEVICE_WHEN_AVAILABLE_
	void operator()(TWarpVoxel& warp) {
		float warp_update_length = ORUtils::length(warp.warp_update);
		int bin_index = 0;

		if (max_warp_update_length > 0) {
			bin_index = ORUTILS_MIN(histogram_bin_count - 1,
			                        (int) (warp_update_length * histogram_bin_count / max_warp_update_length));
		}
		warp_update_bins[bin_index]++;
	}

	void PrintHistogram() {
		std::cout << "Update length histogram: ";
		for (int iBin = 0; iBin < histogram_bin_count; iBin++) {
			std::cout << std::setfill(' ') << std::setw(7) << warp_update_bins[iBin] << "  ";
		}
		std::cout << std::endl;
	}


private:
	const float max_warp_update_length;

	// <20%, 40%, 60%, 80%, 100%
	int warp_update_bins[histogram_bin_count] = {0};
};

enum TraversalDirection : int {
	X = 0, Y = 1, Z = 2
};

template<typename TTSDFVoxel, typename TWarpVoxel, typename TIndex, TraversalDirection TDirection>
struct GradientSmoothingPassFunctor {
	GradientSmoothingPassFunctor(ITMLib::VoxelVolume<TWarpVoxel, TIndex>* warp_field) :
			warp_field(warp_field),
			warp_voxels(warp_field->GetVoxels()),
			warp_index_data(warp_field->index.GetIndexData()),
			warp_field_cache() {}

	_CPU_AND_GPU_CODE_
	void
	operator()(TWarpVoxel& warp_voxel, TTSDFVoxel& canonical_voxel, TTSDFVoxel& live_voxel, Vector3i voxel_position) {
		const int sobolev_filter_size = 7;
		const float sobolev_filter1D[sobolev_filter_size] = {
				2.995861099047703036e-04f,
				4.410932423926419363e-03f,
				6.571314272194948847e-02f,
				9.956527876693953560e-01f,
				6.571314272194946071e-02f,
				4.410932423926422832e-03f,
				2.995861099045313996e-04f
		};

		int vmIndex = 0;
		if (!VoxelIsConsideredForTracking(canonical_voxel, live_voxel)) return;

		const auto directionIndex = (int) TDirection;

		Vector3i receptive_voxel_position = voxel_position;
		receptive_voxel_position[directionIndex] -= (sobolev_filter_size / 2);
		Vector3f smoothed_gradient(0.0f);

		for (int iVoxel = 0; iVoxel < sobolev_filter_size; iVoxel++, receptive_voxel_position[directionIndex]++) {
#if !defined(__CUDACC__) && !defined(WITH_OPENMP)
			const TWarp& receptiveVoxel = readVoxel(warp_voxels, warp_index_data,
													receptive_voxel_position, vmIndex, warp_field_cache);
#else
			const TWarpVoxel& receptiveVoxel = readVoxel(warp_voxels, warp_index_data,
			                                             receptive_voxel_position, vmIndex);
#endif
			smoothed_gradient += sobolev_filter1D[iVoxel] * GetGradient(receptiveVoxel);
		}
		SetGradient(warp_voxel, smoothed_gradient);
	}

private:
	_CPU_AND_GPU_CODE_
	static inline Vector3f GetGradient(const TWarpVoxel& warp_voxel) {
		switch (TDirection) {
			case X:
				return warp_voxel.gradient0;
			case Y:
				return warp_voxel.gradient1;
			case Z:
				return warp_voxel.gradient0;
			default:
				return Vector3f(0.0);
		}
	}

	_CPU_AND_GPU_CODE_
	static inline void SetGradient(TWarpVoxel& warp_voxel, const Vector3f gradient) {
		switch (TDirection) {
			case X:
				warp_voxel.gradient1 = gradient;
				return;
			case Y:
				warp_voxel.gradient0 = gradient;
				return;
			case Z:
				warp_voxel.gradient1 = gradient;
				return;
		}
	}

	ITMLib::VoxelVolume<TWarpVoxel, TIndex>* warp_field;
	TWarpVoxel* warp_voxels;
	typename TIndex::IndexData* warp_index_data;
	typename TIndex::IndexCache warp_field_cache;

};

template<typename TWarpVoxel, bool hasCumulativeWarp>
struct AddFramewiseWarpToWarpWithClearStaticFunctor;

template<typename TWarpVoxel>
struct AddFramewiseWarpToWarpWithClearStaticFunctor<TWarpVoxel, true> {
	_CPU_AND_GPU_CODE_
	static inline void run(TWarpVoxel& warp) {
		warp.warp += warp.framewise_warp;
		warp.framewise_warp = Vector3f(0.0f);
	}
};

template<typename TWarpVoxel>
struct AddFramewiseWarpToWarpWithClearStaticFunctor<TWarpVoxel, false> {
	_CPU_AND_GPU_CODE_
	static inline void run(TWarpVoxel& warp) {
	}
};
template<typename TWarpVoxel, bool hasCumulativeWarp>
struct AddFramewiseWarpToWarpStaticFunctor;

template<typename TWarpVoxel>
struct AddFramewiseWarpToWarpStaticFunctor<TWarpVoxel, true> {
	_CPU_AND_GPU_CODE_
	static inline void run(TWarpVoxel& warp) {
		warp.warp += warp.framewise_warp;
	}
};

template<typename TVoxelCanonical>
struct AddFramewiseWarpToWarpStaticFunctor<TVoxelCanonical, false> {
	_CPU_AND_GPU_CODE_
	static inline void run(TVoxelCanonical& voxel) {
	}
};
