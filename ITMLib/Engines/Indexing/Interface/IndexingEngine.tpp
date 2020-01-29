//  ================================================================
//  Created by Gregory Kramida on 11/1/19.
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

#include "IndexingEngine.h"
#include "../../../Objects/Scene/ITMRepresentationAccess.h"
#include "../../EditAndCopy/Shared/EditAndCopyEngine_Shared.h"

using namespace ITMLib;


template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
void IndexingEngine<TVoxel, TIndex, TMemoryDeviceType>::AllocateFromDepth(ITMVoxelVolume<TVoxel, TIndex>* volume,
                                                                             const ITMView* view,
                                                                             const ITMTrackingState* trackingState,
                                                                             bool onlyUpdateVisibleList,
                                                                             bool resetVisibleList) {}


template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
void IndexingEngine<TVoxel, TIndex, TMemoryDeviceType>::AllocateFromDepth(ITMVoxelVolume<TVoxel, TIndex>* scene,
                                                                             const ITMView* view,
                                                                             const Matrix4f& depth_camera_matrix,
                                                                             bool onlyUpdateVisibleList,
                                                                             bool resetVisibleList) {}


template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
void IndexingEngine<TVoxel, TIndex, TMemoryDeviceType>::AllocateFromDepthAndSdfSpan(
		ITMVoxelVolume<TVoxel, TIndex>* targetVolume,
		const ITMRenderState* sourceRenderState,
		const ITMView* view,
		const Matrix4f& depth_camera_matrix,
		bool onlyUpdateAllocatedList, bool resetAllocatedList) {}


template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
template<typename TVoxelTarget, typename TVoxelSource>
void IndexingEngine<TVoxel, TIndex, TMemoryDeviceType>::AllocateUsingOtherVolume(
		ITMVoxelVolume<TVoxelTarget, TIndex>* targetVolume,
		ITMVoxelVolume<TVoxelSource, TIndex>* sourceVolume) {}

template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
template<WarpType TWarpType, typename TWarp>
void IndexingEngine<TVoxel, TIndex, TMemoryDeviceType>::AllocateFromWarpedVolume(
		ITMVoxelVolume<TWarp, TIndex>* warpField,
		ITMVoxelVolume<TVoxel, TIndex>* sourceTSDF,
		ITMVoxelVolume<TVoxel, TIndex>* targetTSDF) {}

template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
template<typename TVoxelTarget, typename TVoxelSource>
void IndexingEngine<TVoxel, TIndex, TMemoryDeviceType>::AllocateUsingOtherVolumeExpanded(
		ITMVoxelVolume<TVoxelTarget, TIndex>* targetVolume, ITMVoxelVolume<TVoxelSource, TIndex>* sourceVolume) {}

template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
template<typename TVoxelTarget, typename TVoxelSource>
void IndexingEngine<TVoxel, TIndex, TMemoryDeviceType>::AllocateUsingOtherVolumeAndSetVisibilityExpanded(
		ITMVoxelVolume<TVoxelTarget, TIndex>* targetVolume,
		ITMVoxelVolume<TVoxelSource, TIndex>* sourceVolume,
		ITMView* view, const Matrix4f& depth_camera_matrix) {}

