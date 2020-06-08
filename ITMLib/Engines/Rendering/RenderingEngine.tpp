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
	DIEWITHEXCEPTION_REPORTLOCATION("Not implemented");
}

template<class TVoxel, class TIndex, MemoryDeviceType TMemoryDeviceType>
void RenderingEngine<TVoxel, TIndex, TMemoryDeviceType>::FindSurface(
		VoxelVolume<TVoxel, TIndex>* volume, const ORUtils::SE3Pose* pose, const Intrinsics* intrinsics, const RenderState* render_state) const {
	GenericRaycast(volume, render_state->raycastResult->dimensions, pose->GetInvM(), intrinsics->projectionParamsSimple.all, render_state, false);
}

template<class TVoxel, class TIndex, MemoryDeviceType TMemoryDeviceType>
void RenderingEngine<TVoxel, TIndex, TMemoryDeviceType>::CreatePointCloud(
		VoxelVolume<TVoxel, TIndex>* volume, const View* view, CameraTrackingState* camera_tracking_state, RenderState* render_state) const {

	Matrix4f inverted_rgb_camera_matrix = camera_tracking_state->pose_d->GetInvM() * view->calib.trafo_rgb_to_depth.calib;
	GenericRaycast(volume, render_state->raycastResult->dimensions, inverted_rgb_camera_matrix, view->calib.intrinsics_rgb.projectionParamsSimple.all,
	               render_state, true);
	const Vector3f light_source = -Vector3f(inverted_rgb_camera_matrix.getColumn(2));
	RenderPointCloudFunctor<TVoxel, TIndex, TMemoryDeviceType> functor(*camera_tracking_state->pointCloud, *volume, light_source,
	                                                                   this->parameters.skip_points);
	ImageTraversalEngine<TMemoryDeviceType>::template TraverseWithPosition(render_state->raycastResult, functor);
	camera_tracking_state->pointCloud->noTotalPoints = functor.GetPointCount();
}

template<class TVoxel, class TIndex, MemoryDeviceType TMemoryDeviceType>
void RenderingEngine<TVoxel, TIndex, TMemoryDeviceType>::CreateICPMaps(
		VoxelVolume<TVoxel, TIndex>* volume, const View* view, CameraTrackingState* camera_tracking_state, RenderState* render_state) const {
	Vector2i map_dimensions = render_state->raycastResult->dimensions;
	Matrix4f inverted_depth_camera_pose = camera_tracking_state->pose_d->GetInvM();

	GenericRaycast(volume, map_dimensions, inverted_depth_camera_pose, view->calib.intrinsics_d.projectionParamsSimple.all, render_state, true);
	camera_tracking_state->pose_pointCloud->SetFrom(camera_tracking_state->pose_d);
	Vector3f light_source = -Vector3f(inverted_depth_camera_pose.getColumn(2));
	if (view->calib.intrinsics_d.FocalLengthSignsDiffer()) {
		ICPMapRenderFunctor<TMemoryDeviceType, true, true> functor(
				*camera_tracking_state, volume->GetParameters().voxel_size, *render_state, light_source);
		ImageTraversalEngine<TMemoryDeviceType>::template TraversePositionOnly<16, 12>(render_state->raycastResult, functor);
	} else {
		ICPMapRenderFunctor<TMemoryDeviceType, true, false> functor(
				*camera_tracking_state, volume->GetParameters().voxel_size, *render_state, light_source);
		ImageTraversalEngine<TMemoryDeviceType>::template TraversePositionOnly<16, 12>(render_state->raycastResult, functor);
	}
}

template<class TVoxel, class TIndex, MemoryDeviceType TMemoryDeviceType>
void RenderingEngine<TVoxel, TIndex, TMemoryDeviceType>::ForwardRender(
		const VoxelVolume<TVoxel, TIndex>* volume, const View* view, CameraTrackingState* camera_tracking_state, RenderState* render_state) const {

	const Matrix4f depth_camera_pose = camera_tracking_state->pose_d->GetM();
	const Vector4f& depth_camera_projection_parameters = view->calib.intrinsics_d.projectionParamsSimple.all;
	int* missing_point_indices = render_state->fwdProjMissingPoints->GetData(MEMORYDEVICE_CUDA);
	render_state->forwardProjection->Clear();

	float voxel_size = volume->GetParameters().voxel_size;
	ForwardProjectFunctor<TMemoryDeviceType> project_functor(*render_state->forwardProjection, voxel_size, depth_camera_projection_parameters,
	                                                         depth_camera_pose);
	ImageTraversalEngine<TMemoryDeviceType>::template Traverse(render_state->raycastResult, project_functor);

	FindMissingProjectionPointsFunctor<TMemoryDeviceType> find_missing_points_functor(*render_state->fwdProjMissingPoints,
	                                                                                  *render_state->forwardProjection,
	                                                                                  *render_state->renderingRangeImage);

	ImageTraversalEngine<TMemoryDeviceType>::template TraversePositionOnly(render_state->raycastResult, find_missing_points_functor);
	render_state->noFwdProjMissingPoints = find_missing_points_functor.GetMissingPointCount();


	RaycastMissingPointsFunctor<TVoxel, TIndex, false, TMemoryDeviceType> raycast_missing_points_functor(*volume,
	                                                                                                     view->calib.intrinsics_d.projectionParamsSimple.all,
	                                                                                                     camera_tracking_state->pose_d->GetInvM(),
	                                                                                                     *render_state->renderingRangeImage);

	ImageTraversalEngine<TMemoryDeviceType>::template TraverseSampleWithPixelCoordinates(
			render_state->noFwdProjMissingPoints, missing_point_indices, render_state->forwardProjection, raycast_missing_points_functor);
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
void RenderingEngine<TVoxel, TIndex, TMemoryDeviceType>::GenericRaycast(
		VoxelVolume<TVoxel, TIndex>* volume, const Vector2i& depth_image_size, const Matrix4f& camera_inverse_pose,
		const Vector4f& camera_projection_parameters, const RenderState* render_state, bool update_visible_list) const {

	bool update_visibility_information = update_visible_list && RaycastingTraits<TIndex>::has_visibility_information;

	if (update_visibility_information) {
		RaycastFunctor<TVoxel, TIndex, TMemoryDeviceType, true> functor(
				*volume, *render_state->renderingRangeImage, InvertProjectionParams(camera_projection_parameters), camera_inverse_pose);
		ImageTraversalEngine<TMemoryDeviceType>::template TraverseWithPosition<16, 12>(render_state->raycastResult, functor);
	} else {
		RaycastFunctor<TVoxel, TIndex, TMemoryDeviceType, false> functor(
				*volume, *render_state->renderingRangeImage, InvertProjectionParams(camera_projection_parameters), camera_inverse_pose);
		ImageTraversalEngine<TMemoryDeviceType>::template TraverseWithPosition<16, 12>(render_state->raycastResult, functor);
	}
}
