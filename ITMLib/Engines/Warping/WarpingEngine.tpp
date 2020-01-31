//  ================================================================
//  Created by Gregory Kramida on 1/29/20.
//  Copyright (c) 2020 Gregory Kramida
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
#include "WarpingEngine.h"
#include "../Traversal/Interface/VolumeTraversal.h"
#include "../Traversal/Interface/TwoVolumeTraversal.h"
#include "../Indexing/Interface/IndexingEngine.h"
#include "WarpingFunctors.h"

using namespace ITMLib;

template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TMemoryDeviceType>
template<WarpType TWarpType>
void WarpingEngine<TVoxel, TWarp, TIndex, TMemoryDeviceType>::WarpScene(
		VoxelVolume<TWarp, TIndex>* warpField,
		VoxelVolume<TVoxel, TIndex>* sourceTSDF,
		VoxelVolume<TVoxel, TIndex>* targetTSDF) {

	// Clear out the flags at target volume
	FieldClearFunctor<TVoxel, TMemoryDeviceType> flagClearFunctor;
	VolumeTraversalEngine<TVoxel, TIndex, TMemoryDeviceType>::VoxelTraversal(targetTSDF, flagClearFunctor);

	// Allocate new blocks where necessary, filter based on flags from source
	IndexingEngine<TVoxel, TIndex, TMemoryDeviceType>::Instance().
			template AllocateFromWarpedVolume<TWarpType>(warpField, sourceTSDF, targetTSDF);

	TrilinearInterpolationFunctor<TVoxel, TWarp, TIndex, TWarpType, TMemoryDeviceType>
			trilinearInterpolationFunctor(sourceTSDF, warpField);

	// Interpolate to obtain the new live frame values (at target index)
	TwoVolumeTraversalEngine<TVoxel, TWarp, TIndex, TIndex, TMemoryDeviceType>::
	//DualVoxelPositionTraversal_DefaultForMissingSecondary(targetTSDF, warpField, trilinearInterpolationFunctor);
	DualVoxelPositionTraversal(targetTSDF, warpField, trilinearInterpolationFunctor);
}


template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TMemoryDeviceType>
void WarpingEngine<TVoxel, TWarp, TIndex, TMemoryDeviceType>::WarpVolume_CumulativeWarps(
		VoxelVolume<TWarp, TIndex>* warpField,
		VoxelVolume<TVoxel, TIndex>* sourceTSDF,
		VoxelVolume<TVoxel, TIndex>* targetTSDF) {
	this->template WarpScene<WARP_CUMULATIVE>(warpField, sourceTSDF, targetTSDF);
}

template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TMemoryDeviceType>
void WarpingEngine<TVoxel, TWarp, TIndex, TMemoryDeviceType>::WarpVolume_FramewiseWarps(
		VoxelVolume<TWarp, TIndex>* warpField,
		VoxelVolume<TVoxel, TIndex>* sourceTSDF,
		VoxelVolume<TVoxel, TIndex>* targetTSDF) {
	this->template WarpScene<WARP_FLOW>(warpField, sourceTSDF, targetTSDF);
}

template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TMemoryDeviceType>
void WarpingEngine<TVoxel, TWarp, TIndex, TMemoryDeviceType>::WarpVolume_WarpUpdates(
		VoxelVolume<TWarp, TIndex>* warpField,
		VoxelVolume<TVoxel, TIndex>* sourceTSDF,
		VoxelVolume<TVoxel, TIndex>* targetTSDF) {
	this->template WarpScene<WARP_UPDATE>(warpField, sourceTSDF, targetTSDF);
}

