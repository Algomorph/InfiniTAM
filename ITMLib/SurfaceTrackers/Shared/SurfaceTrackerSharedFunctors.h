//  ================================================================
//  Created by Gregory Kramida on 7/12/18.
//  Copyright (c) 2018-2025 Gregory Kramida
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


#include "../../Utils/ITMMath.h"
#include "../../Objects/Scene/VoxelVolume.h"
#include "../../Utils/Configuration.h"
#include "SurfaceTrackerSharedRoutines.h"
#include "../../../ORUtils/PlatformIndependentAtomics.h"


#include "../../Engines/Traversal/CPU/VolumeTraversal_CPU_PlainVoxelArray.h"
#include "../../Engines/Traversal/CPU/VolumeTraversal_CPU_VoxelBlockHash.h"

#ifdef __CUDACC__
#include "../../Utils/ITMCUDAUtils.h"
#include "../../Engines/Traversal/CUDA/VolumeTraversal_CUDA_PlainVoxelArray.h"
#include "../../Engines/Traversal/CUDA/VolumeTraversal_CUDA_VoxelBlockHash.h"
#endif


template<typename TVoxel, bool hasCumulativeWarp>
struct WarpClearFunctor;

template<typename TWarp>
struct WarpClearFunctor<TWarp, true> {
	_CPU_AND_GPU_CODE_
	static inline void run(TWarp& voxel) {
		voxel.warp = Vector3f(0.0f);
	}
};

template<typename TWarp>
struct WarpClearFunctor<TWarp, false> {
	_CPU_AND_GPU_CODE_
	static inline void run(TWarp& voxel) {}
};

template<typename TWarp>
struct ClearOutFramewiseWarpStaticFunctor {
	_CPU_AND_GPU_CODE_
	static inline void run(TWarp& voxel) {
		voxel.framewise_warp = Vector3f(0.0f);
	}
};


template<typename TWarp>
struct ClearOutGradientStaticFunctor {
	_CPU_AND_GPU_CODE_
	static inline void run(TWarp& voxel) {
		voxel.gradient0 = Vector3f(0.0f);
		voxel.gradient1 = Vector3f(0.0f);
	}
};


template<typename TVoxel, typename TWarp, MemoryDeviceType TMemoryDeviceType>
struct WarpUpdateFunctor {
	WarpUpdateFunctor(float learningRate, float momentumWeight, bool gradientSmoothingEnabled) :
			gradientWeight(learningRate * (1.0f - momentumWeight)), momentumWeight(momentumWeight),
			gradientSmoothingEnabled(gradientSmoothingEnabled),
			maxFramewiseWarpPosition(0),
			maxWarpUpdatePosition(0) {
		INITIALIZE_ATOMIC(float, maxFramewiseWarpLength, 0.0f);
		INITIALIZE_ATOMIC(float, maxWarpUpdateLength, 0.0f);
	}

	~WarpUpdateFunctor() {
		CLEAN_UP_ATOMIC(maxFramewiseWarpLength);CLEAN_UP_ATOMIC(maxWarpUpdateLength);
	}

	_DEVICE_WHEN_AVAILABLE_
	void operator()(TVoxel& liveVoxel, TVoxel& canonicalVoxel, TWarp& warp, const Vector3i& position) {
		if (!VoxelIsConsideredForTracking(canonicalVoxel, liveVoxel)) return;
		Vector3f warpUpdate = -gradientWeight * (gradientSmoothingEnabled ?
		                                         warp.gradient1 : warp.gradient0);

		warp.warp_update = warpUpdate + momentumWeight * warp.warp_update;
		warp.framewise_warp += warp.warp_update;

		// update stats
		float framewiseWarpLength = ORUtils::length(warp.framewise_warp);
		float warpUpdateLength = ORUtils::length(warpUpdate);

#if !defined(__CUDACC__) && !defined(WITH_OPENMP)
		//single-threaded CPU version (for debugging max warp position)
		if (framewiseWarpLength > maxFramewiseWarpLength.load()) {
			maxFramewiseWarpLength.store(framewiseWarpLength);
			maxFramewiseWarpPosition = position;
		}
		if (warpUpdateLength > maxWarpUpdateLength.load()) {
			maxWarpUpdateLength.store(warpUpdateLength);
			maxWarpUpdatePosition = position;
		}
#else
		ATOMIC_MAX(maxFramewiseWarpLength, framewiseWarpLength);
		ATOMIC_MAX(maxWarpUpdateLength, warpUpdateLength);
#endif
	}

	DECLARE_ATOMIC_FLOAT(maxFramewiseWarpLength);
	DECLARE_ATOMIC_FLOAT(maxWarpUpdateLength);
	Vector3i maxFramewiseWarpPosition;
	Vector3i maxWarpUpdatePosition;

	_DEVICE_WHEN_AVAILABLE_
	void PrintWarp() {
#if !defined(__CUDACC__) && !defined(WITH_OPENMP)
		std::cout << ITMLib::green << "Max warp: [" << maxFramewiseWarpLength << " at " << maxFramewiseWarpPosition
		          << "] Max update: [" << maxWarpUpdateLength << " at " << maxWarpUpdatePosition << "]."
		          << ITMLib::reset
		          << std::endl;
#else
#endif
	}


private:
	const float gradientWeight;
	const float momentumWeight;
	const bool gradientSmoothingEnabled;
};


template<typename TVoxel, typename TWarp>
struct WarpHistogramFunctor {
	WarpHistogramFunctor(float maxWarpLength, float maxWarpUpdateLength) :
			maxWarpLength(maxWarpLength), maxWarpUpdateLength(maxWarpUpdateLength) {
	}

	static const int histBinCount = 10;

	void operator()(TVoxel& liveVoxel, TVoxel& canonicalVoxel, TWarp& warp) {
		if (!VoxelIsConsideredForTracking(canonicalVoxel, liveVoxel)) return;
		float framewiseWarpLength = ORUtils::length(warp.framewise_warp);
		float warpUpdateLength = ORUtils::length(warp.gradient0);
		const int histBinCount = WarpHistogramFunctor<TVoxel, TWarp>::histBinCount;
		int binIdx = 0;
		if (maxWarpLength > 0) {
			binIdx = ORUTILS_MIN(histBinCount - 1, (int) (framewiseWarpLength * histBinCount / maxWarpLength));
		}
		warpBins[binIdx]++;
		if (maxWarpUpdateLength > 0) {
			binIdx = ORUTILS_MIN(histBinCount - 1,
			                     (int) (warpUpdateLength * histBinCount / maxWarpUpdateLength));
		}
		updateBins[binIdx]++;
	}

	void PrintHistogram() {
		std::cout << "FW warp length histogram: ";
		for (int iBin = 0; iBin < histBinCount; iBin++) {
			std::cout << std::setfill(' ') << std::setw(7) << warpBins[iBin] << "  ";
		}
		std::cout << std::endl;
		std::cout << "Update length histogram: ";
		for (int iBin = 0; iBin < histBinCount; iBin++) {
			std::cout << std::setfill(' ') << std::setw(7) << updateBins[iBin] << "  ";
		}
		std::cout << std::endl;
	}


private:
	const float maxWarpLength;
	const float maxWarpUpdateLength;

	// <20%, 40%, 60%, 80%, 100%
	int warpBins[histBinCount] = {0};
	int updateBins[histBinCount] = {0};
};

enum TraversalDirection : int {
	X = 0, Y = 1, Z = 2
};

template<typename TVoxel, typename TWarp, typename TIndex, TraversalDirection TDirection>
struct GradientSmoothingPassFunctor {
	GradientSmoothingPassFunctor(ITMLib::VoxelVolume<TWarp, TIndex>* warpField) :
			warpField(warpField),
			warpVoxels(warpField->localVBA.GetVoxelBlocks()),
			warpIndexData(warpField->index.GetIndexData()),
			warpFieldCache() {}

	_CPU_AND_GPU_CODE_
	void operator()(TVoxel& liveVoxel, TVoxel& canonicalVoxel, TWarp& warp, Vector3i position) {
		const int sobolevFilterSize = 7;
		const float sobolevFilter1D[sobolevFilterSize] = {
				2.995861099047703036e-04f,
				4.410932423926419363e-03f,
				6.571314272194948847e-02f,
				9.956527876693953560e-01f,
				6.571314272194946071e-02f,
				4.410932423926422832e-03f,
				2.995861099045313996e-04f
		};

		int vmIndex = 0;
		if (!VoxelIsConsideredForTracking(canonicalVoxel, liveVoxel)) return;

		const auto directionIndex = (int) TDirection;

		Vector3i receptiveVoxelPosition = position;
		receptiveVoxelPosition[directionIndex] -= (sobolevFilterSize / 2);
		Vector3f smoothedGradient(0.0f);

		for (int iVoxel = 0; iVoxel < sobolevFilterSize; iVoxel++, receptiveVoxelPosition[directionIndex]++) {
#if !defined(__CUDACC__) && !defined(WITH_OPENMP)
			const TWarp& receptiveVoxel = readVoxel(warpVoxels, warpIndexData,
			                                        receptiveVoxelPosition, vmIndex, warpFieldCache);
#else
			const TWarp& receptiveVoxel = readVoxel(warpVoxels, warpIndexData,
			                                        receptiveVoxelPosition, vmIndex);
#endif
			smoothedGradient += sobolevFilter1D[iVoxel] * GetGradient(receptiveVoxel);
		}
		SetGradient(warp, smoothedGradient);
	}

private:
	_CPU_AND_GPU_CODE_
	static inline Vector3f GetGradient(const TWarp& voxel) {
		switch (TDirection) {
			case X:
				return voxel.gradient0;
			case Y:
				return voxel.gradient1;
			case Z:
				return voxel.gradient0;
			default:
				return Vector3f(0.0);
		}
	}

	_CPU_AND_GPU_CODE_
	static inline void SetGradient(TWarp& voxel, const Vector3f gradient) {
		switch (TDirection) {
			case X:
				voxel.gradient1 = gradient;
				return;
			case Y:
				voxel.gradient0 = gradient;
				return;
			case Z:
				voxel.gradient1 = gradient;
				return;
		}
	}

	ITMLib::VoxelVolume<TWarp, TIndex>* warpField;
	TWarp* warpVoxels;
	typename TIndex::IndexData* warpIndexData;
	int sourceSdfFieldIndex;
	typename TIndex::IndexCache warpFieldCache;

};


template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TDeviceType>
void SmoothWarpGradient_common(ITMLib::VoxelVolume<TVoxel, TIndex>* liveScene,
                               ITMLib::VoxelVolume<TVoxel, TIndex>* canonicalScene,
                               ITMLib::VoxelVolume<TWarp, TIndex>* warpField) {

	GradientSmoothingPassFunctor<TVoxel, TWarp, TIndex, X> passFunctorX(warpField);
	GradientSmoothingPassFunctor<TVoxel, TWarp, TIndex, Y> passFunctorY(warpField);
	GradientSmoothingPassFunctor<TVoxel, TWarp, TIndex, Z> passFunctorZ(warpField);

	ITMLib::ThreeVolumeTraversalEngine<TVoxel, TWarp, TIndex, TDeviceType>::
	template DualVoxelPositionTraversal(liveScene, canonicalScene, warpField, passFunctorX);
	ITMLib::ThreeVolumeTraversalEngine<TVoxel, TWarp, TIndex, TDeviceType>::
	template DualVoxelPositionTraversal(liveScene, canonicalScene, warpField, passFunctorY);
	ITMLib::ThreeVolumeTraversalEngine<TVoxel, TWarp, TIndex, TDeviceType>::
	template DualVoxelPositionTraversal(liveScene, canonicalScene, warpField, passFunctorZ);
}

template<typename TWarp, bool hasCumulativeWarp>
struct AddFramewiseWarpToWarpWithClearStaticFunctor;

template<typename TWarp>
struct AddFramewiseWarpToWarpWithClearStaticFunctor<TWarp, true> {
	_CPU_AND_GPU_CODE_
	static inline void run(TWarp& warp) {
		warp.warp += warp.framewise_warp;
		warp.framewise_warp = Vector3f(0.0f);
	}
};

template<typename TWarp>
struct AddFramewiseWarpToWarpWithClearStaticFunctor<TWarp, false> {
	_CPU_AND_GPU_CODE_
	static inline void run(TWarp& warp) {
	}
};
template<typename TWarp, bool hasCumulativeWarp>
struct AddFramewiseWarpToWarpStaticFunctor;

template<typename TWarp>
struct AddFramewiseWarpToWarpStaticFunctor<TWarp, true> {
	_CPU_AND_GPU_CODE_
	static inline void run(TWarp& warp) {
		warp.warp += warp.framewise_warp;
	}
};

template<typename TVoxelCanonical>
struct AddFramewiseWarpToWarpStaticFunctor<TVoxelCanonical, false> {
	_CPU_AND_GPU_CODE_
	static inline void run(TVoxelCanonical& voxel) {
	}
};

template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TDeviceType>
inline float UpdateWarps_common(
		ITMLib::VoxelVolume<TVoxel, TIndex>* canonicalScene,
		ITMLib::VoxelVolume<TVoxel, TIndex>* liveScene,
		ITMLib::VoxelVolume<TWarp, TIndex>* warpField,
		float learning_rate,
		bool gradeintSmoothingEnabled,
		bool print_histogram) {
	WarpUpdateFunctor<TVoxel, TWarp, TDeviceType>
			warpUpdateFunctor(learning_rate, ITMLib::configuration::get().non_rigid_tracking_parameters.momentum_weight, gradeintSmoothingEnabled);

	ITMLib::ThreeVolumeTraversalEngine<TVoxel, TWarp, TIndex, TDeviceType>::
	DualVoxelPositionTraversal(liveScene, canonicalScene, warpField, warpUpdateFunctor);

	//TODO: move histogram printing / logging to a separate function
	//don't compute histogram in CUDA version
#ifndef __CUDACC__
	if(print_histogram){
		WarpHistogramFunctor<TVoxel, TWarp>
				warpHistogramFunctor(warpUpdateFunctor.maxFramewiseWarpLength, warpUpdateFunctor.maxWarpUpdateLength);
		ITMLib::ThreeVolumeTraversalEngine<TVoxel, TWarp, TIndex, TDeviceType>::
		DualVoxelTraversal(liveScene, canonicalScene, warpField, warpHistogramFunctor);
		warpHistogramFunctor.PrintHistogram();
		warpUpdateFunctor.PrintWarp();
	}
#endif
	//return warpUpdateFunctor.maxWarpUpdateLength;
	return GET_ATOMIC_VALUE_CPU(warpUpdateFunctor.maxFramewiseWarpLength);
}

template<typename TVoxelCanonical, typename TIndex, MemoryDeviceType TDeviceType>
inline void
AddFramewiseWarpToWarp_common(ITMLib::VoxelVolume<TVoxelCanonical, TIndex>* canonicalScene, bool clearFramewiseWarp) {
	if (clearFramewiseWarp) {
		ITMLib::VolumeTraversalEngine<TVoxelCanonical, TIndex, TDeviceType>::
		template StaticVoxelTraversal<
				AddFramewiseWarpToWarpWithClearStaticFunctor<TVoxelCanonical, TVoxelCanonical::hasCumulativeWarp>>
		(canonicalScene);
	} else {
		ITMLib::VolumeTraversalEngine<TVoxelCanonical, TIndex, TDeviceType>::
		template StaticVoxelTraversal<
				AddFramewiseWarpToWarpStaticFunctor<TVoxelCanonical, TVoxelCanonical::hasCumulativeWarp>>
		(canonicalScene);
	}
};
