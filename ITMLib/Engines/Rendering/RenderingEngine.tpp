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
	return specialized_engine.CreateExpectedDepths(volume, pose, intrinsics, render_state);
}

template<class TVoxel, class TIndex, MemoryDeviceType TMemoryDeviceType>
void RenderingEngine<TVoxel, TIndex, TMemoryDeviceType>::RenderImage(
		VoxelVolume<TVoxel, TIndex>* volume, const ORUtils::SE3Pose* pose, const Intrinsics* intrinsics, const RenderState* render_state,
		UChar4Image* output_image, IRenderingEngine::RenderImageType type, IRenderingEngine::RenderRaycastSelection raycast_type) const {

}

template<class TVoxel, class TIndex, MemoryDeviceType TMemoryDeviceType>
void RenderingEngine<TVoxel, TIndex, TMemoryDeviceType>::FindSurface(
		VoxelVolume<TVoxel, TIndex>* volume, const ORUtils::SE3Pose* pose, const Intrinsics* intrinsics, const RenderState* render_state) const {
	GenericRaycast(volume, render_state->raycastResult->dimensions, pose->GetInvM(), intrinsics->projectionParamsSimple.all, render_state, false);
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

template<typename TIndex>
struct RaycastingTraits;
template<>
struct RaycastingTraits<VoxelBlockHash> {
	static constexpr bool has_visibility_information = true;
};
template<>
struct RaycastingTraits<PlainVoxelArray> {
	static constexpr bool has_visibility_information = false;
};

template<class TVoxel, class TIndex, MemoryDeviceType TMemoryDeviceType>
void RenderingEngine<TVoxel, TIndex, TMemoryDeviceType>::GenericRaycast (
		VoxelVolume<TVoxel, TIndex>* volume, const Vector2i& depth_image_size, const Matrix4f& depth_camera_inverse_pose,
		const Vector4f& depth_camera_projection_parameters, const RenderState* render_state, bool update_visible_list) const{

	bool update_visibility_information = update_visible_list && RaycastingTraits<TIndex>::has_visibility_information;

	if (update_visibility_information) {
		RaycastFunctor<TVoxel, TIndex, TMemoryDeviceType, true> functor(
				*volume, *render_state->renderingRangeImage, InvertProjectionParams(depth_camera_projection_parameters), depth_camera_inverse_pose);
		ImageTraversalEngine<TMemoryDeviceType>::template TraverseWithPosition<16, 12>(render_state->raycastResult, functor);
	} else {
		RaycastFunctor<TVoxel, TIndex, TMemoryDeviceType, false> functor(
				*volume, *render_state->renderingRangeImage, InvertProjectionParams(depth_camera_projection_parameters), depth_camera_inverse_pose);
		ImageTraversalEngine<TMemoryDeviceType>::template TraverseWithPosition<16, 12>(render_state->raycastResult, functor);
	}
}
