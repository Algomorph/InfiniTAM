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

//local
#include "IndexingEngine.h"
#include "../../../Objects/Volume/RepresentationAccess.h"
#include "../../EditAndCopy/Shared/EditAndCopyEngine_Shared.h"
#include "../../../Utils/Geometry/FrustumTrigonometry.h"

using namespace ITMLib;


template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
void IndexingEngine<TVoxel, TIndex, TMemoryDeviceType>::AllocateFromDepth(VoxelVolume<TVoxel, TIndex>* volume,
                                                                          const ITMView* view,
                                                                          const CameraTrackingState* tracking_state,
                                                                          bool onlyUpdateVisibleList,
                                                                          bool resetVisibleList) {}


template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
void IndexingEngine<TVoxel, TIndex, TMemoryDeviceType>::AllocateFromDepth(VoxelVolume<TVoxel, TIndex>* scene,
                                                                          const ITMView* view,
                                                                          const Matrix4f& depth_camera_matrix,
                                                                          bool onlyUpdateVisibleList,
                                                                          bool resetVisibleList) {}


template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
void IndexingEngine<TVoxel, TIndex, TMemoryDeviceType>::AllocateFromDepthAndSdfSpan(
		VoxelVolume<TVoxel, TIndex>* targetVolume,
		const CameraTrackingState* tracking_state,
		const ITMView* view,
		bool onlyUpdateAllocatedList, bool resetAllocatedList) {}


template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
template<typename TVoxelTarget, typename TVoxelSource>
void IndexingEngine<TVoxel, TIndex, TMemoryDeviceType>::AllocateUsingOtherVolume(
		VoxelVolume<TVoxelTarget, TIndex>* targetVolume,
		VoxelVolume<TVoxelSource, TIndex>* sourceVolume) {}

template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
template<WarpType TWarpType, typename TWarp>
void IndexingEngine<TVoxel, TIndex, TMemoryDeviceType>::AllocateFromWarpedVolume(
		VoxelVolume<TWarp, TIndex>* warpField,
		VoxelVolume<TVoxel, TIndex>* sourceTSDF,
		VoxelVolume<TVoxel, TIndex>* targetTSDF) {}

template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
template<typename TVoxelTarget, typename TVoxelSource>
void IndexingEngine<TVoxel, TIndex, TMemoryDeviceType>::AllocateUsingOtherVolumeExpanded(
		VoxelVolume<TVoxelTarget, TIndex>* targetVolume, VoxelVolume<TVoxelSource, TIndex>* sourceVolume) {}

template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
template<typename TVoxelTarget, typename TVoxelSource>
void IndexingEngine<TVoxel, TIndex, TMemoryDeviceType>::AllocateUsingOtherVolumeAndSetVisibilityExpanded(
		VoxelVolume<TVoxelTarget, TIndex>* targetVolume,
		VoxelVolume<TVoxelSource, TIndex>* sourceVolume,
		ITMView* view, const Matrix4f& depth_camera_matrix) {}

