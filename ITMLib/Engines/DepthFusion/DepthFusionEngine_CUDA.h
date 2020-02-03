//  ================================================================
//  Created by Gregory Kramida on 7/24/18.
//  Copyright (c) 2018-2025 Gregory Kramida
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

#include "DepthFusionEngine.h"
#include "../../Objects/Volume/PlainVoxelArray.h"
#include "../EditAndCopy/CUDA/EditAndCopyEngine_CUDA.h"
#include "../Indexing/VBH/CUDA/IndexingEngine_CUDA_VoxelBlockHash.h"

namespace ITMLib {
template<typename TVoxel, typename TWarp, typename TIndex>
class DepthFusionEngine_CUDA
		: public DepthFusionEngineInterface<TVoxel, TVoxel, TIndex> {
};

//region =================================== VOXEL BLOCK HASH ==========================================================

template<typename TVoxel, typename TWarp>
class DepthFusionEngine_CUDA<TVoxel, TWarp, VoxelBlockHash>
		: public DepthFusionEngineInterface<TVoxel, TWarp, VoxelBlockHash> {
public:
	DepthFusionEngine_CUDA() = default;
	~DepthFusionEngine_CUDA() = default;
	void UpdateVisibleList(VoxelVolume<TVoxel, VoxelBlockHash>* scene, const ITMView* view,
	                       const CameraTrackingState* trackingState, const RenderState* renderState,
	                       bool resetVisibleList) override;
	void GenerateTsdfVolumeFromView(VoxelVolume<TVoxel, VoxelBlockHash>* volume, const ITMView* view,
	                                const CameraTrackingState* trackingState) override;
	void GenerateTsdfVolumeFromView(VoxelVolume<TVoxel, VoxelBlockHash>* scene, const ITMView* view,
	                                const Matrix4f& depth_camera_matrix = Matrix4f::Identity()) override;
	void GenerateTsdfVolumeFromViewExpanded(VoxelVolume<TVoxel, VoxelBlockHash>* volume,
	                                        VoxelVolume<TVoxel, VoxelBlockHash>* temporaryAllocationVolume,
	                                        const ITMView* view,
	                                        const Matrix4f& depth_camera_matrix = Matrix4f::Identity()) override;

	void IntegrateDepthImageIntoTsdfVolume(VoxelVolume<TVoxel, VoxelBlockHash>* volume, const ITMView* view) override;
	void IntegrateDepthImageIntoTsdfVolume(VoxelVolume<TVoxel, VoxelBlockHash>* volume, const ITMView* view,
	                                       const CameraTrackingState* trackingState) override;
protected:
	void IntegrateDepthImageIntoTsdfVolume_Helper(VoxelVolume<TVoxel, VoxelBlockHash>* volume, const ITMView* view,
	                                              Matrix4f depth_camera_matrix = Matrix4f::Identity());

};

// endregion ===========================================================================================================
// region ==================================== PLAIN VOXEL ARRAY =======================================================

template<typename TVoxel, typename TWarp>
class DepthFusionEngine_CUDA<TVoxel, TWarp, PlainVoxelArray>
		: public DepthFusionEngineInterface<TVoxel, TWarp, PlainVoxelArray> {
public:
	void UpdateVisibleList(VoxelVolume<TVoxel, PlainVoxelArray>* volume, const ITMView* view,
	                       const CameraTrackingState* trackingState, const RenderState* renderState,
	                       bool resetVisibleList) override;
	void GenerateTsdfVolumeFromView(VoxelVolume<TVoxel, PlainVoxelArray>* volume, const ITMView* view,
	                                const CameraTrackingState* trackingState) override;
	void GenerateTsdfVolumeFromView(VoxelVolume<TVoxel, PlainVoxelArray>* volume, const ITMView* view,
	                                const Matrix4f& depth_camera_matrix = Matrix4f::Identity()) override;
	void GenerateTsdfVolumeFromViewExpanded(VoxelVolume<TVoxel, PlainVoxelArray>* volume,
	                                        VoxelVolume<TVoxel, PlainVoxelArray>* temporaryAllocationVolume,
	                                        const ITMView* view,
	                                        const Matrix4f& depth_camera_matrix = Matrix4f::Identity()) override;

	DepthFusionEngine_CUDA() = default;
	~DepthFusionEngine_CUDA() = default;
	void IntegrateDepthImageIntoTsdfVolume(VoxelVolume<TVoxel, PlainVoxelArray>* volume, const ITMView* view) override;
	void IntegrateDepthImageIntoTsdfVolume(VoxelVolume<TVoxel, PlainVoxelArray>* volume, const ITMView* view,
	                                       const CameraTrackingState* trackingState) override;
private:
	void IntegrateDepthImageIntoTsdfVolume_Helper(VoxelVolume<TVoxel, PlainVoxelArray>* volume, const ITMView* view,
	                                              Matrix4f depth_camera_matrix = Matrix4f::Identity());

};

// endregion ===========================================================================================================

}//namespace ITMLib