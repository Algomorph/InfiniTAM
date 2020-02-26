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

//#define TRAVERSE_ALL_HASH_BLOCKS
template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TMemoryDeviceType>
template<WarpType TWarpType>
void WarpingEngine<TVoxel, TWarp, TIndex, TMemoryDeviceType>::WarpVolume(
		VoxelVolume<TWarp, TIndex>* warp_field,
		VoxelVolume<TVoxel, TIndex>* source_volume,
		VoxelVolume<TVoxel, TIndex>* target_volume) {

	// Clear out the flags at target volume
	FieldClearFunctor<TVoxel, TMemoryDeviceType> flagClearFunctor;
	VolumeTraversalEngine<TVoxel, TIndex, TMemoryDeviceType>::
#ifdef TRAVERSE_ALL_HASH_BLOCKS
	TraverseAll(target_volume, flagClearFunctor);
#else
	TraverseUtilized(target_volume, flagClearFunctor);
#endif


	TrilinearInterpolationFunctor<TVoxel, TWarp, TIndex, TWarpType, TMemoryDeviceType>
			trilinearInterpolationFunctor(source_volume, warp_field);

	// Interpolate to obtain the new live frame values (at target index)
	TwoVolumeTraversalEngine<TVoxel, TWarp, TIndex, TIndex, TMemoryDeviceType>::
#ifdef TRAVERSE_ALL_HASH_BLOCKS
	TraverseAllWithPosition(target_volume, warp_field, trilinearInterpolationFunctor);
#else
	TraverseUtilizedWithPosition(target_volume, warp_field, trilinearInterpolationFunctor);
#endif
}


template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TMemoryDeviceType>
void WarpingEngine<TVoxel, TWarp, TIndex, TMemoryDeviceType>::WarpVolume_CumulativeWarps(
		VoxelVolume<TWarp, TIndex>* warpField,
		VoxelVolume<TVoxel, TIndex>* sourceTSDF,
		VoxelVolume<TVoxel, TIndex>* targetTSDF) {
	this->template WarpVolume<WARP_CUMULATIVE>(warpField, sourceTSDF, targetTSDF);
}

template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TMemoryDeviceType>
void WarpingEngine<TVoxel, TWarp, TIndex, TMemoryDeviceType>::WarpVolume_FramewiseWarps(
		VoxelVolume<TWarp, TIndex>* warpField,
		VoxelVolume<TVoxel, TIndex>* sourceTSDF,
		VoxelVolume<TVoxel, TIndex>* targetTSDF) {
	this->template WarpVolume<WARP_FRAMEWISE>(warpField, sourceTSDF, targetTSDF);
}

template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TMemoryDeviceType>
void WarpingEngine<TVoxel, TWarp, TIndex, TMemoryDeviceType>::WarpVolume_WarpUpdates(
		VoxelVolume<TWarp, TIndex>* warpField,
		VoxelVolume<TVoxel, TIndex>* sourceTSDF,
		VoxelVolume<TVoxel, TIndex>* targetTSDF) {
	this->template WarpVolume<WARP_UPDATE>(warpField, sourceTSDF, targetTSDF);
}

