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

#include "RenderingEngine.h"
#include "Shared/RenderingEngine_Functors.h"
#include "Shared/RenderingEngine_Shared.h"
#include "RenderingEngine_Specialized.tpp"

using namespace ITMLib;

template<class TVoxel, class TIndex, MemoryDeviceType TMemoryDeviceType>
void RenderingEngine<TVoxel, TIndex, TMemoryDeviceType>::FindVisibleBlocks(VoxelVolume<TVoxel, TIndex>* volume, const ORUtils::SE3Pose* pose,
                                                                           const Intrinsics* intrinsics, RenderState* render_state) const {
	specialized_engine.FindVisibleBlocks(volume, pose, intrinsics, render_state);
}

template<class TVoxel, class TIndex, MemoryDeviceType TMemoryDeviceType>
int RenderingEngine<TVoxel, TIndex, TMemoryDeviceType>::CountVisibleBlocks(
		const VoxelVolume<TVoxel, TIndex>* volume, const RenderState* render_state, int min_block_list_id, int max_block_list_id) const {
	return specialized_engine.CountVisibleBlocks(volume, min_block_list_id, max_block_list_id);
}


template<class TVoxel, class TIndex, MemoryDeviceType TMemoryDeviceType>
void RenderingEngine<TVoxel, TIndex, TMemoryDeviceType>::CreateExpectedDepths(
		const VoxelVolume<TVoxel, TIndex>* volume, const ORUtils::SE3Pose* pose, const Intrinsics* intrinsics, RenderState* render_state) const {
	DIEWITHEXCEPTION_REPORTLOCATION("Not implemented");
}

template<class TVoxel, class TIndex, MemoryDeviceType TMemoryDeviceType>
void RenderingEngine<TVoxel, TIndex, TMemoryDeviceType>::RenderImage(
		VoxelVolume<TVoxel, TIndex>* volume, const ORUtils::SE3Pose* pose, const Intrinsics* intrinsics, const RenderState* render_state,
		UChar4Image* output_image, IRenderingEngine::RenderImageType type, IRenderingEngine::RenderRaycastSelection raycast_type) const {
	DIEWITHEXCEPTION_REPORTLOCATION("Not implemented");
}

template<class TVoxel, class TIndex, MemoryDeviceType TMemoryDeviceType>
void
RenderingEngine<TVoxel, TIndex, TMemoryDeviceType>::FindSurface(
		VoxelVolume<TVoxel, TIndex>* volume, const ORUtils::SE3Pose* pose, const Intrinsics* intrinsics, const RenderState* render_state) const {
	DIEWITHEXCEPTION_REPORTLOCATION("Not implemented");
}

template<class TVoxel, class TIndex, MemoryDeviceType TMemoryDeviceType>
void RenderingEngine<TVoxel, TIndex, TMemoryDeviceType>::CreatePointCloud(
		VoxelVolume<TVoxel, TIndex>* volume, const View* view, CameraTrackingState* camera_tracking_state, RenderState* render_state,
		bool skipPoints) const {
	DIEWITHEXCEPTION_REPORTLOCATION("Not implemented");
}

template<class TVoxel, class TIndex, MemoryDeviceType TMemoryDeviceType>
void RenderingEngine<TVoxel, TIndex, TMemoryDeviceType>::CreateICPMaps(
		VoxelVolume<TVoxel, TIndex>* volume, const View* view, CameraTrackingState* camera_tracking_state, RenderState* render_state) const {
	DIEWITHEXCEPTION_REPORTLOCATION("Not implemented");
}

template<class TVoxel, class TIndex, MemoryDeviceType TMemoryDeviceType>
void RenderingEngine<TVoxel, TIndex, TMemoryDeviceType>::ForwardRender(
		const VoxelVolume<TVoxel, TIndex>* volume, const View* view, CameraTrackingState* camera_tracking_state, RenderState* render_state) const {
	DIEWITHEXCEPTION_REPORTLOCATION("Not implemented");
}
