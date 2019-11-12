//  ================================================================
//  Created by Gregory Kramida on 5/28/18.
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

#include <iomanip>
#include "SurfaceTracker_CPU.h"
#include "../../Engines/Traversal/CPU/ITMSceneTraversal_CPU_PlainVoxelArray.h"
#include "../Shared/ITMSceneMotionTracker_Functors.h"
#include "../Shared/ITMCalculateWarpGradientFunctor.h"

using namespace ITMLib;

// region ===================================== CALCULATE GRADIENT SMOOTHING ===========================================


template<typename TVoxel, typename TWarp>
void SurfaceTracker<TVoxel, TWarp, ITMPlainVoxelArray, MEMORYDEVICE_CPU>::CalculateWarpGradient(
		ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* canonicalScene,
		ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* liveScene,
		ITMVoxelVolume<TWarp, ITMPlainVoxelArray>* warpField) {

	ITMSceneTraversalEngine<TWarp, ITMPlainVoxelArray, MEMORYDEVICE_CPU>::template
	StaticVoxelTraversal<ClearOutGradientStaticFunctor<TWarp>>(warpField);


	ITMCalculateWarpGradientFunctor<TVoxel, TWarp, ITMPlainVoxelArray::IndexData, ITMPlainVoxelArray::IndexCache>
			calculateGradientFunctor(this->parameters, this->switches,
			                         liveScene->localVBA.GetVoxelBlocks(), liveScene->index.GetIndexData(),
			                         canonicalScene->localVBA.GetVoxelBlocks(), canonicalScene->index.GetIndexData(),
			                         warpField->localVBA.GetVoxelBlocks(), warpField->index.GetIndexData(),
			                         canonicalScene->sceneParams->voxelSize, canonicalScene->sceneParams->mu);

	ITMDualSceneWarpTraversalEngine<TVoxel, TWarp, ITMPlainVoxelArray, MEMORYDEVICE_CPU>::
	DualVoxelPositionTraversal(liveScene, canonicalScene, warpField, calculateGradientFunctor);

	calculateGradientFunctor.PrintStatistics();
}


// endregion ===========================================================================================================
// region ========================================== SOBOLEV GRADIENT SMOOTHING ========================================


template<typename TVoxel, typename TWarp>
void SurfaceTracker<TVoxel, TWarp, ITMPlainVoxelArray, MEMORYDEVICE_CPU>::SmoothWarpGradient(
		ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* canonicalScene,
		ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* liveScene,
		ITMVoxelVolume<TWarp, ITMPlainVoxelArray>* warpField) {

	if (this->switches.enableSobolevGradientSmoothing) {
		SmoothWarpGradient_common<TVoxel, TWarp, ITMPlainVoxelArray, MEMORYDEVICE_CPU>
		        (liveScene, canonicalScene, warpField);
	}
}

// endregion ===========================================================================================================

// region ======================================== APPLY WARP UPDATE TO THE WARP ITSELF ================================


template<typename TVoxel, typename TWarp>
float SurfaceTracker<TVoxel, TWarp, ITMPlainVoxelArray, MEMORYDEVICE_CPU>::UpdateWarps(
		ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* canonicalScene,
		ITMVoxelVolume<TVoxel, ITMPlainVoxelArray>* liveScene,
		ITMVoxelVolume<TWarp, ITMPlainVoxelArray>* warpField) {
	return UpdateWarps_common<TVoxel, TWarp, ITMPlainVoxelArray, MEMORYDEVICE_CPU>
			(canonicalScene, liveScene, warpField, this->parameters.gradientDescentLearningRate,
			 this->switches.enableSobolevGradientSmoothing);
}


//endregion ============================================================================================================


template<typename TVoxel, typename TWarp>
void ITMLib::SurfaceTracker<TVoxel, TWarp, ITMPlainVoxelArray, MEMORYDEVICE_CPU>::ResetWarps(
		ITMVoxelVolume<TWarp, ITMPlainVoxelArray>* warpField) {

	ITMSceneTraversalEngine<TWarp, ITMPlainVoxelArray, MEMORYDEVICE_CPU>::template
	StaticVoxelTraversal<WarpClearFunctor<TWarp, TWarp::hasCumulativeWarp>>(warpField);
}

template<typename TVoxel, typename TWarp>
void SurfaceTracker<TVoxel, TWarp, ITMPlainVoxelArray, MEMORYDEVICE_CPU>::ClearOutFlowWarp(
		ITMVoxelVolume<TWarp, ITMPlainVoxelArray>* warpField) {
	ITMSceneTraversalEngine<TWarp, ITMPlainVoxelArray, MEMORYDEVICE_CPU>::template
	StaticVoxelTraversal<ClearOutFlowWarpStaticFunctor<TWarp>>(warpField);
}

template<typename TVoxel, typename TWarp>
void SurfaceTracker<TVoxel, TWarp, ITMPlainVoxelArray, MEMORYDEVICE_CPU>::AddFlowWarpToWarp(
		ITMVoxelVolume<TWarp, ITMPlainVoxelArray>* warpField, bool clearFlowWarp) {

	AddFlowWarpToWarp_common<TWarp, ITMPlainVoxelArray, MEMORYDEVICE_CPU>
		(warpField, clearFlowWarp);
}

