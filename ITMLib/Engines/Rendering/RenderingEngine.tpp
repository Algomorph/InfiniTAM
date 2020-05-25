//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 5/25/20.
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

#include "Interface/RenderingEngine.h"

using namespace ITMLib;

template<class TVoxel, class TIndex, MemoryDeviceType TMemoryDeviceType>
void RenderingEngine<TVoxel, TIndex, TMemoryDeviceType>::FindVisibleBlocks(VoxelVolume<TVoxel, TIndex>* scene, const ORUtils::SE3Pose* pose,
                                                                           const Intrinsics* intrinsics, RenderState* renderState) const {

}

template<class TVoxel, class TIndex, MemoryDeviceType TMemoryDeviceType>
int RenderingEngine<TVoxel, TIndex, TMemoryDeviceType>::CountVisibleBlocks(
		const VoxelVolume<TVoxel, TIndex>* scene, const RenderState* renderState, int minBlockId, int maxBlockId) const {
	return 0;
}


template<class TVoxel, class TIndex, MemoryDeviceType TMemoryDeviceType>
void RenderingEngine<TVoxel, TIndex, TMemoryDeviceType>::CreateExpectedDepths(
		const VoxelVolume<TVoxel, TIndex>* scene, const ORUtils::SE3Pose* pose, const Intrinsics* intrinsics, RenderState* renderState) {

}

template<class TVoxel, class TIndex, MemoryDeviceType TMemoryDeviceType>
void RenderingEngine<TVoxel, TIndex, TMemoryDeviceType>::RenderImage(
		VoxelVolume<TVoxel, TIndex>* scene, const ORUtils::SE3Pose* pose, const Intrinsics* intrinsics, const RenderState* renderState,
		UChar4Image* outputImage, IRenderingEngine::RenderImageType type, IRenderingEngine::RenderRaycastSelection raycastType) {

}

template<class TVoxel, class TIndex, MemoryDeviceType TMemoryDeviceType>
void
RenderingEngine<TVoxel, TIndex, TMemoryDeviceType>::FindSurface(
		VoxelVolume<TVoxel, TIndex>* scene, const ORUtils::SE3Pose* pose, const Intrinsics* intrinsics, const RenderState* renderState) {

}

template<class TVoxel, class TIndex, MemoryDeviceType TMemoryDeviceType>
void RenderingEngine<TVoxel, TIndex, TMemoryDeviceType>::CreatePointCloud(
		VoxelVolume<TVoxel, TIndex>* scene, const View* view, CameraTrackingState* trackingState, RenderState* renderState, bool skipPoints) {

}

template<class TVoxel, class TIndex, MemoryDeviceType TMemoryDeviceType>
void RenderingEngine<TVoxel, TIndex, TMemoryDeviceType>::CreateICPMaps(
		VoxelVolume<TVoxel, TIndex>* scene, const View* view, CameraTrackingState* trackingState, RenderState* renderState) {

}

template<class TVoxel, class TIndex, MemoryDeviceType TMemoryDeviceType>
void RenderingEngine<TVoxel, TIndex, TMemoryDeviceType>::ForwardRender(
		const VoxelVolume<TVoxel, TIndex>* scene, const View* view, CameraTrackingState* trackingState, RenderState* renderState) {

}
