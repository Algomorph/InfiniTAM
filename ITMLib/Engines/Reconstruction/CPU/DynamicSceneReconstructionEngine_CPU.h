// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../../SurfaceTrackers/Interface/SurfaceTrackerInterface.h"
#include "../Interface/DynamicSceneReconstructionEngine.h"
#include "../../../Objects/Scene/PlainVoxelArray.h"
#include "../../EditAndCopy/CPU/EditAndCopyEngine_CPU.h"
#include "../../Indexing/VBH/CPU/IndexingEngine_CPU_VoxelBlockHash.h"
#include "../../Common/ITMWarpEnums.h"


namespace ITMLib {
template<typename TVoxel, typename TWarp, typename TIndex>
class DynamicSceneReconstructionEngine_CPU
		: public DynamicSceneReconstructionEngine<TVoxel, TWarp, TIndex> {
};

template<typename TVoxel, typename TWarp>
class DynamicSceneReconstructionEngine_CPU<TVoxel, TWarp, VoxelBlockHash>
		: public DynamicSceneReconstructionEngine<TVoxel, TWarp, VoxelBlockHash> {
public:
	DynamicSceneReconstructionEngine_CPU() = default;
	~DynamicSceneReconstructionEngine_CPU() = default;

	void UpdateVisibleList(ITMVoxelVolume<TVoxel, VoxelBlockHash>* scene, const ITMView* view,
	                       const ITMTrackingState* trackingState, const ITMRenderState* renderState,
	                       bool resetVisibleList) override;
	void GenerateTsdfVolumeFromView(ITMVoxelVolume<TVoxel, VoxelBlockHash>* volume, const ITMView* view,
	                                const ITMTrackingState* trackingState) override;
	void GenerateTsdfVolumeFromView(ITMVoxelVolume<TVoxel, VoxelBlockHash>* volume, const ITMView* view,
	                                const Matrix4f& depth_camera_matrix = Matrix4f::Identity()) override;
	void GenerateTsdfVolumeFromViewExpanded(ITMVoxelVolume<TVoxel, VoxelBlockHash>* volume,
	                                        ITMVoxelVolume<TVoxel, VoxelBlockHash>* temporaryAllocationVolume,
	                                        const ITMView* view,
	                                        const Matrix4f& depth_camera_matrix = Matrix4f::Identity()) override;
	void FuseOneTsdfVolumeIntoAnother(ITMVoxelVolume<TVoxel, VoxelBlockHash>* canonicalScene,
	                              ITMVoxelVolume<TVoxel, VoxelBlockHash>* liveScene) override;

	void IntegrateDepthImageIntoTsdfVolume(ITMVoxelVolume<TVoxel, VoxelBlockHash>* volume, const ITMView* view);
	void IntegrateDepthImageIntoTsdfVolume(ITMVoxelVolume<TVoxel, VoxelBlockHash>* volume, const ITMView* view,
	                                      const ITMTrackingState* trackingState);
private:
	void IntegrateDepthImageIntoTsdfVolume_Helper(ITMVoxelVolume<TVoxel, VoxelBlockHash>* volume, const ITMView* view,
	                                             Matrix4f depth_camera_matrix = Matrix4f::Identity());

};

template<typename TVoxel, typename TWarp>
class DynamicSceneReconstructionEngine_CPU<TVoxel, TWarp, PlainVoxelArray>
		: public DynamicSceneReconstructionEngine<TVoxel, TWarp, PlainVoxelArray> {
public:
	void UpdateVisibleList(ITMVoxelVolume<TVoxel, PlainVoxelArray>* scene, const ITMView* view,
	                       const ITMTrackingState* trackingState, const ITMRenderState* renderState,
	                       bool resetVisibleList) override;
	void GenerateTsdfVolumeFromView(ITMVoxelVolume<TVoxel, PlainVoxelArray>* volume, const ITMView* view,
	                                const ITMTrackingState* trackingState) override;
	void GenerateTsdfVolumeFromView(ITMVoxelVolume<TVoxel, PlainVoxelArray>* volume, const ITMView* view,
	                                const Matrix4f& depth_camera_matrix=Matrix4f::Identity()) override;
	void GenerateTsdfVolumeFromViewExpanded(ITMVoxelVolume<TVoxel, PlainVoxelArray>* volume,
	                                        ITMVoxelVolume<TVoxel, PlainVoxelArray>* temporaryAllocationVolume,
	                                        const ITMView* view,
	                                        const Matrix4f& depth_camera_matrix = Matrix4f::Identity()) override;
	void FuseOneTsdfVolumeIntoAnother(ITMVoxelVolume<TVoxel, PlainVoxelArray>* canonicalScene,
	                              ITMVoxelVolume<TVoxel, PlainVoxelArray>* liveScene) override;

	void IntegrateDepthImageIntoTsdfVolume(ITMVoxelVolume<TVoxel, PlainVoxelArray>* volume, const ITMView* view);
	void IntegrateDepthImageIntoTsdfVolume(ITMVoxelVolume<TVoxel, PlainVoxelArray>* volume, const ITMView* view,
	                                      const ITMTrackingState* trackingState);

	DynamicSceneReconstructionEngine_CPU() = default;
	~DynamicSceneReconstructionEngine_CPU() = default;
private:
	void IntegrateDepthImageIntoTsdfVolume_Helper(ITMVoxelVolume<TVoxel, PlainVoxelArray>* volume, const ITMView* view,
	                                             Matrix4f camera_depth_matrix = Matrix4f::Identity());

};
} // namespace ITMLib
