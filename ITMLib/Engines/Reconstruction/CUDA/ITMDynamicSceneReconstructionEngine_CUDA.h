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

#include "../Interface/ITMDynamicSceneReconstructionEngine.h"
#include "../../../Objects/Scene/PlainVoxelArray.h"
#include "../../VolumeEditAndCopy/CUDA/VolumeEditAndCopyEngine_CUDA.h"
#include "../../Indexing/VBH/CUDA/IndexingEngine_CUDA_VoxelBlockHash.h"

namespace ITMLib {
template<typename TVoxel, typename TWarp, typename TIndex>
class ITMDynamicSceneReconstructionEngine_CUDA
		: public ITMDynamicSceneReconstructionEngine<TVoxel, TVoxel, TIndex> {
};

//region =================================== VOXEL BLOCK HASH ==========================================================

template<typename TVoxel, typename TWarp>
class ITMDynamicSceneReconstructionEngine_CUDA<TVoxel, TWarp, VoxelBlockHash>
		: public ITMDynamicSceneReconstructionEngine<TVoxel, TWarp, VoxelBlockHash> {
public:
	ITMDynamicSceneReconstructionEngine_CUDA() = default;
	~ITMDynamicSceneReconstructionEngine_CUDA() = default;
	void UpdateVisibleList(ITMVoxelVolume<TVoxel, VoxelBlockHash>* scene, const ITMView* view,
	                       const ITMTrackingState* trackingState, const ITMRenderState* renderState,
	                       bool resetVisibleList) override;
	void GenerateTsdfVolumeFromView(ITMVoxelVolume<TVoxel, VoxelBlockHash>* volume, const ITMView* view,
	                                const ITMTrackingState* trackingState) override;
	void GenerateTsdfVolumeFromView(ITMVoxelVolume<TVoxel, VoxelBlockHash>* scene, const ITMView* view,
	                                const Matrix4f& depth_camera_matrix = Matrix4f::Identity()) override;
	void GenerateTsdfVolumeFromViewExpanded(ITMVoxelVolume<TVoxel, VoxelBlockHash>* volume,
	                                        ITMVoxelVolume<TVoxel, VoxelBlockHash>* temporaryAllocationVolume,
	                                        const ITMView* view,
	                                        const Matrix4f& depth_camera_matrix = Matrix4f::Identity()) override;
	void FuseOneTsdfVolumeIntoAnother(ITMVoxelVolume<TVoxel, VoxelBlockHash>* targetTsdfVolume,
	                              ITMVoxelVolume<TVoxel, VoxelBlockHash>* liveTsdfVolume) override;
	void WarpScene_CumulativeWarps(ITMVoxelVolume<TWarp, VoxelBlockHash>* warpField,
	                               ITMVoxelVolume<TVoxel, VoxelBlockHash>* sourceTSDF,
	                               ITMVoxelVolume<TVoxel, VoxelBlockHash>* targetTSDF) override;
	void WarpScene_FramewiseWarps(ITMVoxelVolume<TWarp, VoxelBlockHash>* warpField,
	                         ITMVoxelVolume<TVoxel, VoxelBlockHash>* sourceTSDF,
	                         ITMVoxelVolume<TVoxel, VoxelBlockHash>* targetTSDF) override;
	void WarpScene_WarpUpdates(ITMVoxelVolume<TWarp, VoxelBlockHash>* warpField,
	                           ITMVoxelVolume<TVoxel, VoxelBlockHash>* sourceTSDF,
	                           ITMVoxelVolume<TVoxel, VoxelBlockHash>* targetTSDF) override;
	void IntegrateDepthImageIntoTsdfVolume(ITMVoxelVolume<TVoxel, VoxelBlockHash>* volume, const ITMView* view) override;
	void IntegrateDepthImageIntoTsdfVolume(ITMVoxelVolume<TVoxel, VoxelBlockHash>* volume, const ITMView* view,
	                                      const ITMTrackingState* trackingState) override;
protected:
	void IntegrateDepthImageIntoTsdfVolume_Helper(ITMVoxelVolume<TVoxel, VoxelBlockHash>* volume, const ITMView* view,
	                                             Matrix4f depth_camera_matrix = Matrix4f::Identity());
	template<WarpType TWarpSource>
	void WarpScene(ITMVoxelVolume<TWarp, VoxelBlockHash>* warpField,
	               ITMVoxelVolume<TVoxel, VoxelBlockHash>* sourceTSDF,
	               ITMVoxelVolume<TVoxel, VoxelBlockHash>* targetTSDF);

private:
	VolumeEditAndCopyEngine_CUDA<TVoxel, VoxelBlockHash> liveSceneManager;
};

// endregion ===========================================================================================================
// region ==================================== PLAIN VOXEL ARRAY =======================================================

template<typename TVoxel, typename TWarp>
class ITMDynamicSceneReconstructionEngine_CUDA<TVoxel, TWarp, PlainVoxelArray>
		: public ITMDynamicSceneReconstructionEngine<TVoxel, TWarp, PlainVoxelArray> {
public:
	void UpdateVisibleList(ITMVoxelVolume<TVoxel, PlainVoxelArray>* volume, const ITMView* view,
	                       const ITMTrackingState* trackingState, const ITMRenderState* renderState,
	                       bool resetVisibleList) override;
	void GenerateTsdfVolumeFromView(ITMVoxelVolume<TVoxel, PlainVoxelArray>* volume, const ITMView* view,
	                                const ITMTrackingState* trackingState) override;
	void GenerateTsdfVolumeFromView(ITMVoxelVolume<TVoxel, PlainVoxelArray>* volume, const ITMView* view,
	                                const Matrix4f& depth_camera_matrix = Matrix4f::Identity()) override;
	void GenerateTsdfVolumeFromViewExpanded(ITMVoxelVolume<TVoxel, PlainVoxelArray>* volume,
	                                        ITMVoxelVolume<TVoxel, PlainVoxelArray>* temporaryAllocationVolume,
	                                        const ITMView* view,
	                                        const Matrix4f& depth_camera_matrix = Matrix4f::Identity()) override;
	void FuseOneTsdfVolumeIntoAnother(ITMVoxelVolume<TVoxel, PlainVoxelArray>* canonicalScene,
	                              ITMVoxelVolume<TVoxel, PlainVoxelArray>* liveScene) override;
	void WarpScene_CumulativeWarps(
			ITMVoxelVolume<TWarp, PlainVoxelArray>* warpField,
			ITMVoxelVolume<TVoxel, PlainVoxelArray>* sourceTSDF,
			ITMVoxelVolume<TVoxel, PlainVoxelArray>* targetTSDF) override;
	void WarpScene_FramewiseWarps(
			ITMVoxelVolume<TWarp, PlainVoxelArray>* warpField,
			ITMVoxelVolume<TVoxel, PlainVoxelArray>* sourceTSDF,
			ITMVoxelVolume<TVoxel, PlainVoxelArray>* targetTSDF) override;
	void WarpScene_WarpUpdates(
			ITMVoxelVolume<TWarp, PlainVoxelArray>* warpField,
			ITMVoxelVolume<TVoxel, PlainVoxelArray>* sourceTSDF,
			ITMVoxelVolume<TVoxel, PlainVoxelArray>* targetTSDF) override;

	ITMDynamicSceneReconstructionEngine_CUDA() = default;
	~ITMDynamicSceneReconstructionEngine_CUDA() = default;
	void IntegrateDepthImageIntoTsdfVolume(ITMVoxelVolume<TVoxel, PlainVoxelArray>* volume, const ITMView* view) override;
	void IntegrateDepthImageIntoTsdfVolume(ITMVoxelVolume<TVoxel, PlainVoxelArray>* volume, const ITMView* view,
	                                      const ITMTrackingState* trackingState) override;
private:
	VolumeEditAndCopyEngine_CUDA<TVoxel, PlainVoxelArray> liveSceneManager;
	void IntegrateDepthImageIntoTsdfVolume_Helper(ITMVoxelVolume<TVoxel, PlainVoxelArray>* volume, const ITMView* view,
	                                             Matrix4f depth_camera_matrix = Matrix4f::Identity());

	template<WarpType TWarpSource>
	void WarpScene(ITMVoxelVolume<TWarp, PlainVoxelArray>* warpField,
	               ITMVoxelVolume<TVoxel, PlainVoxelArray>* sourceTSDF,
	               ITMVoxelVolume<TVoxel, PlainVoxelArray>* targetTSDF);
};

// endregion ===========================================================================================================

}//namespace ITMLib