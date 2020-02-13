//  ================================================================
//  Created by Gregory Kramida on 2/3/20.
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
#include "DepthFusionEngine.h"
#include "../Indexing/Interface/IndexingEngine.h"
#include "../Traversal/Interface/VolumeTraversal.h"
#include "DepthFusionEngine_Shared.h"
#include "../../GlobalTemplateDefines.h"

using namespace ITMLib;

template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TMemoryDeviceType>
void DepthFusionEngine<TVoxel, TWarp, TIndex, TMemoryDeviceType>::UpdateVisibleList(
		VoxelVolume<TVoxel, TIndex>* scene, const ITMView* view, const CameraTrackingState* trackingState,
		const RenderState* renderState, bool resetVisibleList) {
	IndexingEngine<TVoxel, TIndex, TMemoryDeviceType>::Instance()
			.AllocateNearSurface(scene, view, trackingState, true, resetVisibleList);
}

template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TMemoryDeviceType>
void DepthFusionEngine<TVoxel, TWarp, TIndex, TMemoryDeviceType>::GenerateTsdfVolumeFromTwoSurfaces(
		VoxelVolume<TVoxel, TIndex>* volume, const ITMView* view,
		const CameraTrackingState* tracking_state) {
	volume->Reset();
	IndexingEngine<TVoxel, TIndex, TMemoryDeviceType>::Instance()
			.AllocateNearAndBetweenTwoSurfaces(volume, tracking_state, view);
	this->IntegrateDepthImageIntoTsdfVolume_Helper(volume, view, tracking_state->pose_d->GetM());
}


template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TMemoryDeviceType>
void DepthFusionEngine<TVoxel, TWarp, TIndex, TMemoryDeviceType>::GenerateTsdfVolumeFromSurface(
		VoxelVolume<TVoxel, TIndex>* volume, const ITMView* view, const CameraTrackingState* tracking_state) {
	volume->Reset();
	IndexingEngine<TVoxel, TIndex, TMemoryDeviceType>::Instance()
			.AllocateNearSurface(volume, view, tracking_state, false, false);
	this->IntegrateDepthImageIntoTsdfVolume_Helper(volume, view, tracking_state->pose_d->GetM());
}


template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TMemoryDeviceType>
void DepthFusionEngine<TVoxel, TWarp, TIndex, TMemoryDeviceType>::IntegrateDepthImageIntoTsdfVolume_Helper(
		VoxelVolume<TVoxel, TIndex>* volume, const ITMView* view, Matrix4f depth_camera_matrix) {
	if (volume->sceneParams->stop_integration_at_max_weight) {
		VoxelDepthIntegrationFunctor<TVoxel, TMemoryDeviceType, true> integration_functor(*(volume->sceneParams), view,
		                                                                                  depth_camera_matrix);
		VolumeTraversalEngine<TVoxel, TIndex, TMemoryDeviceType>::VoxelPositionTraversal(volume, integration_functor);
	} else {
		VoxelDepthIntegrationFunctor<TVoxel, TMemoryDeviceType, false> integration_functor(*(volume->sceneParams), view,
		                                                                                   depth_camera_matrix);
		VolumeTraversalEngine<TVoxel, TIndex, TMemoryDeviceType>::VoxelPositionTraversal(volume, integration_functor);
	}

}

template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TMemoryDeviceType>
void DepthFusionEngine<TVoxel, TWarp, TIndex, TMemoryDeviceType>::IntegrateDepthImageIntoTsdfVolume(
		VoxelVolume<TVoxel, TIndex>* volume, const ITMView* view, const CameraTrackingState* trackingState) {
	IntegrateDepthImageIntoTsdfVolume_Helper(volume, view, trackingState->pose_d->GetM());
}

template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TMemoryDeviceType>
void DepthFusionEngine<TVoxel, TWarp, TIndex, TMemoryDeviceType>::IntegrateDepthImageIntoTsdfVolume(
		VoxelVolume<TVoxel, TIndex>* volume, const ITMView* view) {
	IntegrateDepthImageIntoTsdfVolume_Helper(volume, view);
}

