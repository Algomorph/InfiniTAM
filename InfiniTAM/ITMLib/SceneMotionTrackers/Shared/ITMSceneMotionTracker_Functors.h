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
#include "../../Objects/Scene/ITMVoxelVolume.h"
#include "../../Utils/ITMLibSettings.h"
#include "ITMSceneMotionTracker_Shared.h"
#include "../../../ORUtils/PlatformIndependentAtomics.h"


#include "../../Engines/Traversal/CPU/ITMSceneTraversal_CPU_PlainVoxelArray.h"
#include "../../Engines/Traversal/CPU/ITMSceneTraversal_CPU_VoxelBlockHash.h"

#ifdef __CUDACC__
#include "../../Utils/ITMCUDAUtils.h"
#include "../../Engines/Traversal/CUDA/ITMSceneTraversal_CUDA_PlainVoxelArray.h"
#include "../../Engines/Traversal/CUDA/ITMSceneTraversal_CUDA_VoxelBlockHash.h"
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
struct ClearOutFlowWarpStaticFunctor {
	_CPU_AND_GPU_CODE_
	static inline void run(TWarp& voxel) {
		voxel.flow_warp = Vector3f(0.0f);
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


template<typename TVoxel, typename TWarp>
struct WarpUpdateFunctor {
	WarpUpdateFunctor(float learningRate, bool gradientSmoothingEnabled) :
			learningRate(learningRate), gradientSmoothingEnabled(gradientSmoothingEnabled),
			maxFlowWarpPosition(0),
			maxWarpUpdatePosition(0) {
		INITIALIZE_ATOMIC(float, maxFlowWarpLength, 0.0f);
		INITIALIZE_ATOMIC(float, maxWarpUpdateLength, 0.0f);
	}

	~WarpUpdateFunctor() {
		CLEAN_UP_ATOMIC(maxFlowWarpLength);CLEAN_UP_ATOMIC(maxWarpUpdateLength);
	}

	_DEVICE_WHEN_AVAILABLE_
	void operator()(TVoxel& liveVoxel, TVoxel& canonicalVoxel, TWarp& warp, const Vector3i& position) {
		if (!VoxelIsConsideredForTracking(canonicalVoxel, liveVoxel)) return;
		Vector3f warpUpdate = -learningRate * (gradientSmoothingEnabled ?
		                                       warp.gradient1 : warp.gradient0);

		warp.warp_update = warpUpdate;
		warp.flow_warp += warpUpdate;

		// update stats
		float framewiseWarpLength = ORUtils::length(warp.flow_warp);
		float warpUpdateLength = ORUtils::length(warpUpdate);

#if !defined(__CUDACC__) && !defined(WITH_OPENMP)
		//single-threaded CPU version (for debugging max warp position)
		if (framewiseWarpLength > maxFlowWarpLength.load()) {
			maxFlowWarpLength.store(framewiseWarpLength);
			maxFlowWarpPosition = position;
		}
		if (warpUpdateLength > maxWarpUpdateLength.load()) {
			maxWarpUpdateLength.store(warpUpdateLength);
			maxWarpUpdatePosition = position;
		}
#else
		ATOMIC_MAX(maxFlowWarpLength, framewiseWarpLength);
		ATOMIC_MAX(maxWarpUpdateLength, warpUpdateLength);
#endif
	}

	DECLARE_ATOMIC_FLOAT(maxFlowWarpLength);
	DECLARE_ATOMIC_FLOAT(maxWarpUpdateLength);
	Vector3i maxFlowWarpPosition;
	Vector3i maxWarpUpdatePosition;

	_DEVICE_WHEN_AVAILABLE_
	void PrintWarp() {
#if !defined(__CUDACC__) && !defined(WITH_OPENMP)
		std::cout << ITMLib::green << "Max warp: [" << maxFlowWarpLength << " at " << maxFlowWarpPosition
		          << "] Max update: [" << maxWarpUpdateLength << " at " << maxWarpUpdatePosition << "]."
		          << ITMLib::reset
		          << std::endl;
#else
#endif
	}


private:
	const float learningRate;
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
		float framewiseWarpLength = ORUtils::length(warp.flow_warp);
		float warpUpdateLength = ORUtils::length(warp.gradient0);
		const int histBinCount = WarpHistogramFunctor<TVoxel, TWarp>::histBinCount;
		int binIdx = 0;
		if (maxWarpLength > 0) {
			binIdx = MIN(histBinCount - 1, (int) (framewiseWarpLength * histBinCount / maxWarpLength));
		}
		warpBins[binIdx]++;
		if (maxWarpUpdateLength > 0) {
			binIdx = MIN(histBinCount - 1,
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
	GradientSmoothingPassFunctor(ITMLib::ITMVoxelVolume<TWarp, TIndex>* warpField) :
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

	ITMLib::ITMVoxelVolume<TWarp, TIndex>* warpField;
	TWarp* warpVoxels;
	typename TIndex::IndexData* warpIndexData;
	int sourceSdfFieldIndex;
	typename TIndex::IndexCache warpFieldCache;

};


template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TDeviceType>
void SmoothWarpGradient_common(ITMLib::ITMVoxelVolume<TVoxel, TIndex>* liveScene,
                               ITMLib::ITMVoxelVolume<TVoxel, TIndex>* canonicalScene,
                               ITMLib::ITMVoxelVolume<TWarp, TIndex>* warpField) {

	GradientSmoothingPassFunctor<TVoxel, TWarp, TIndex, X> passFunctorX(warpField);
	GradientSmoothingPassFunctor<TVoxel, TWarp, TIndex, Y> passFunctorY(warpField);
	GradientSmoothingPassFunctor<TVoxel, TWarp, TIndex, Z> passFunctorZ(warpField);

	ITMLib::ITMDualSceneWarpTraversalEngine<TVoxel, TWarp, TIndex, TDeviceType>::
	template DualVoxelPositionTraversal(liveScene, canonicalScene, warpField, passFunctorX);
	ITMLib::ITMDualSceneWarpTraversalEngine<TVoxel, TWarp, TIndex, TDeviceType>::
	template DualVoxelPositionTraversal(liveScene, canonicalScene, warpField, passFunctorY);
	ITMLib::ITMDualSceneWarpTraversalEngine<TVoxel, TWarp, TIndex, TDeviceType>::
	template DualVoxelPositionTraversal(liveScene, canonicalScene, warpField, passFunctorZ);
}

template<typename TWarp, bool hasCumulativeWarp>
struct AddFlowWarpToWarpWithClearStaticFunctor;

template<typename TWarp>
struct AddFlowWarpToWarpWithClearStaticFunctor<TWarp, true> {
	_CPU_AND_GPU_CODE_
	static inline void run(TWarp& warp) {
		warp.warp += warp.flow_warp;
		warp.flow_warp = Vector3f(0.0f);
	}
};

template<typename TWarp>
struct AddFlowWarpToWarpWithClearStaticFunctor<TWarp, false> {
	_CPU_AND_GPU_CODE_
	static inline void run(TWarp& warp) {
	}
};
template<typename TWarp, bool hasCumulativeWarp>
struct AddFlowWarpToWarpStaticFunctor;

template<typename TWarp>
struct AddFlowWarpToWarpStaticFunctor<TWarp, true> {
	_CPU_AND_GPU_CODE_
	static inline void run(TWarp& warp) {
		warp.warp += warp.flow_warp;
	}
};

template<typename TVoxelCanonical>
struct AddFlowWarpToWarpStaticFunctor<TVoxelCanonical, false> {
	_CPU_AND_GPU_CODE_
	static inline void run(TVoxelCanonical& voxel) {
	}
};

template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TDeviceType>
inline float UpdateWarps_common(
		ITMLib::ITMVoxelVolume<TVoxel, TIndex>* canonicalScene,
		ITMLib::ITMVoxelVolume<TVoxel, TIndex>* liveScene,
		ITMLib::ITMVoxelVolume<TWarp, TIndex>* warpField,
		float gradientDescentLearningRate,
		bool gradeintSmoothingEnabled) {
	WarpUpdateFunctor<TVoxel, TWarp>
			warpUpdateFunctor(gradientDescentLearningRate, gradeintSmoothingEnabled);

	ITMLib::ITMDualSceneWarpTraversalEngine<TVoxel, TWarp, TIndex, TDeviceType>::
	DualVoxelPositionTraversal(liveScene, canonicalScene, warpField, warpUpdateFunctor);

	//TODO: move histogram printing / logging to a separate function
	//don't compute histogram in CUDA version
#ifndef __CUDACC__
	WarpHistogramFunctor<TVoxel, TWarp>
			warpHistogramFunctor(warpUpdateFunctor.maxFlowWarpLength, warpUpdateFunctor.maxWarpUpdateLength);
	ITMLib::ITMDualSceneWarpTraversalEngine<TVoxel, TWarp, TIndex, TDeviceType>::
	DualVoxelTraversal(liveScene, canonicalScene, warpField, warpHistogramFunctor);
	warpHistogramFunctor.PrintHistogram();
	warpUpdateFunctor.PrintWarp();
#endif
	//return warpUpdateFunctor.maxWarpUpdateLength;
	return GET_ATOMIC_VALUE_CPU(warpUpdateFunctor.maxFlowWarpLength);
}

template<typename TVoxelCanonical, typename TIndex, MemoryDeviceType TDeviceType>
inline void
AddFlowWarpToWarp_common(ITMLib::ITMVoxelVolume<TVoxelCanonical, TIndex>* canonicalScene, bool clearFlowWarp) {
	if (clearFlowWarp) {
		ITMLib::ITMSceneTraversalEngine<TVoxelCanonical, TIndex, TDeviceType>::
		template StaticVoxelTraversal<
				AddFlowWarpToWarpWithClearStaticFunctor<TVoxelCanonical, TVoxelCanonical::hasCumulativeWarp>>
		(canonicalScene);
	} else {
		ITMLib::ITMSceneTraversalEngine<TVoxelCanonical, TIndex, TDeviceType>::
		template StaticVoxelTraversal<
				AddFlowWarpToWarpStaticFunctor<TVoxelCanonical, TVoxelCanonical::hasCumulativeWarp>>
		(canonicalScene);
	}
};