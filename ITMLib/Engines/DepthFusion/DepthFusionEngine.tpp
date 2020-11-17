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
void DepthFusionEngine<TVoxel, TWarp, TIndex, TMemoryDeviceType>::IntegrateDepthImageIntoTsdfVolume_Helper(
		VoxelVolume<TVoxel, TIndex>* volume, const View* view, Matrix4f depth_camera_matrix) {
	switch(this->parameters.execution_mode){
		case OPTIMIZED:
			if (volume->GetParameters().stop_integration_at_max_weight) {
				if (this->parameters.use_surface_thickness_cutoff) {
					VoxelDepthIntegrationFunctor<TVoxel, TMemoryDeviceType, true, true, ExecutionMode::OPTIMIZED>
							integration_functor(volume->GetParameters(), view, depth_camera_matrix, this->parameters.surface_thickness);
					VolumeTraversalEngine<TVoxel, TIndex, TMemoryDeviceType>::TraverseAllWithPosition(volume, integration_functor);
				} else {
					VoxelDepthIntegrationFunctor<TVoxel, TMemoryDeviceType, true, false, ExecutionMode::OPTIMIZED>
							integration_functor(volume->GetParameters(), view, depth_camera_matrix, this->parameters.surface_thickness);
					VolumeTraversalEngine<TVoxel, TIndex, TMemoryDeviceType>::TraverseAllWithPosition(volume, integration_functor);
				}
			} else {
				if (this->parameters.use_surface_thickness_cutoff) {
					VoxelDepthIntegrationFunctor<TVoxel, TMemoryDeviceType, false, true, ExecutionMode::OPTIMIZED>
							integration_functor(volume->GetParameters(), view, depth_camera_matrix, this->parameters.surface_thickness);
					VolumeTraversalEngine<TVoxel, TIndex, TMemoryDeviceType>::TraverseAllWithPosition(volume, integration_functor);
				} else {
					VoxelDepthIntegrationFunctor<TVoxel, TMemoryDeviceType, false, false, ExecutionMode::OPTIMIZED>
							integration_functor(volume->GetParameters(), view, depth_camera_matrix, this->parameters.surface_thickness);
					VolumeTraversalEngine<TVoxel, TIndex, TMemoryDeviceType>::TraverseAllWithPosition(volume, integration_functor);
				}
			}
			break;
		case DIAGNOSTIC:
			if (volume->GetParameters().stop_integration_at_max_weight) {
				if (this->parameters.use_surface_thickness_cutoff) {
					VoxelDepthIntegrationFunctor<TVoxel, TMemoryDeviceType, true, true, ExecutionMode::DIAGNOSTIC>
							integration_functor(volume->GetParameters(), view, depth_camera_matrix, this->parameters.surface_thickness);
					VolumeTraversalEngine<TVoxel, TIndex, TMemoryDeviceType>::TraverseAllWithPosition(volume, integration_functor);
				} else {
					VoxelDepthIntegrationFunctor<TVoxel, TMemoryDeviceType, true, false, ExecutionMode::DIAGNOSTIC>
							integration_functor(volume->GetParameters(), view, depth_camera_matrix, this->parameters.surface_thickness);
					VolumeTraversalEngine<TVoxel, TIndex, TMemoryDeviceType>::TraverseAllWithPosition(volume, integration_functor);
				}
			} else {
				if (this->parameters.use_surface_thickness_cutoff) {
					VoxelDepthIntegrationFunctor<TVoxel, TMemoryDeviceType, false, true, ExecutionMode::DIAGNOSTIC>
							integration_functor(volume->GetParameters(), view, depth_camera_matrix, this->parameters.surface_thickness);
					VolumeTraversalEngine<TVoxel, TIndex, TMemoryDeviceType>::TraverseAllWithPosition(volume, integration_functor);
				} else {
					VoxelDepthIntegrationFunctor<TVoxel, TMemoryDeviceType, false, false, ExecutionMode::DIAGNOSTIC>
							integration_functor(volume->GetParameters(), view, depth_camera_matrix, this->parameters.surface_thickness);
					VolumeTraversalEngine<TVoxel, TIndex, TMemoryDeviceType>::TraverseAllWithPosition(volume, integration_functor);
				}
			}
			break;
		default:
			DIEWITHEXCEPTION_REPORTLOCATION("Unsupported execution mode.");
			break;
	}
}

template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TMemoryDeviceType>
void DepthFusionEngine<TVoxel, TWarp, TIndex, TMemoryDeviceType>::IntegrateDepthImageIntoTsdfVolume(
		VoxelVolume<TVoxel, TIndex>* volume, const View* view, const CameraTrackingState* trackingState) {
	IntegrateDepthImageIntoTsdfVolume_Helper(volume, view, trackingState->pose_d->GetM());
}

template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TMemoryDeviceType>
void DepthFusionEngine<TVoxel, TWarp, TIndex, TMemoryDeviceType>::IntegrateDepthImageIntoTsdfVolume(
		VoxelVolume<TVoxel, TIndex>* volume, const View* view) {
	IntegrateDepthImageIntoTsdfVolume_Helper(volume, view);
}

