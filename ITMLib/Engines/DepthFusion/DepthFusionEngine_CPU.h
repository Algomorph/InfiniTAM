// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../SurfaceTrackers/Interface/SurfaceTrackerInterface.h"
#include "DepthFusionEngine.h"
#include "../../Objects/Volume/PlainVoxelArray.h"
#include "../EditAndCopy/CPU/EditAndCopyEngine_CPU.h"
#include "../Indexing/VBH/CPU/IndexingEngine_CPU_VoxelBlockHash.h"
#include "../Common/WarpType.h"


namespace ITMLib {
template<typename TVoxel, typename TWarp, typename TIndex>
class DepthFusionEngine_CPU
		: public DepthFusionEngineInterface<TVoxel, TWarp, TIndex> {
};

template<typename TVoxel, typename TWarp>
class DepthFusionEngine_CPU<TVoxel, TWarp, VoxelBlockHash>
		: public DepthFusionEngineInterface<TVoxel, TWarp, VoxelBlockHash> {
public:
	DepthFusionEngine_CPU() = default;
	~DepthFusionEngine_CPU() = default;

	void UpdateVisibleList(VoxelVolume<TVoxel, VoxelBlockHash>* scene, const ITMView* view,
	                       const CameraTrackingState* trackingState, const RenderState* renderState,
	                       bool resetVisibleList) override;
	void GenerateTsdfVolumeFromView(VoxelVolume<TVoxel, VoxelBlockHash>* volume, const ITMView* view,
	                                const CameraTrackingState* trackingState) override;
	void GenerateTsdfVolumeFromView(VoxelVolume<TVoxel, VoxelBlockHash>* volume, const ITMView* view,
	                                const Matrix4f& depth_camera_matrix = Matrix4f::Identity()) override;
	void GenerateTsdfVolumeFromViewExpanded(VoxelVolume<TVoxel, VoxelBlockHash>* volume,
	                                        VoxelVolume<TVoxel, VoxelBlockHash>* temporaryAllocationVolume,
	                                        const ITMView* view,
	                                        const Matrix4f& depth_camera_matrix = Matrix4f::Identity()) override;
	void IntegrateDepthImageIntoTsdfVolume(VoxelVolume<TVoxel, VoxelBlockHash>* volume, const ITMView* view);
	void IntegrateDepthImageIntoTsdfVolume(VoxelVolume<TVoxel, VoxelBlockHash>* volume, const ITMView* view,
	                                       const CameraTrackingState* trackingState);
private:
	void IntegrateDepthImageIntoTsdfVolume_Helper(VoxelVolume<TVoxel, VoxelBlockHash>* volume, const ITMView* view,
	                                              Matrix4f depth_camera_matrix = Matrix4f::Identity());

};

template<typename TVoxel, typename TWarp>
class DepthFusionEngine_CPU<TVoxel, TWarp, PlainVoxelArray>
		: public DepthFusionEngineInterface<TVoxel, TWarp, PlainVoxelArray> {
public:
	void UpdateVisibleList(VoxelVolume<TVoxel, PlainVoxelArray>* scene, const ITMView* view,
	                       const CameraTrackingState* trackingState, const RenderState* renderState,
	                       bool resetVisibleList) override;
	void GenerateTsdfVolumeFromView(VoxelVolume<TVoxel, PlainVoxelArray>* volume, const ITMView* view,
	                                const CameraTrackingState* trackingState) override;
	void GenerateTsdfVolumeFromView(VoxelVolume<TVoxel, PlainVoxelArray>* volume, const ITMView* view,
	                                const Matrix4f& depth_camera_matrix=Matrix4f::Identity()) override;
	void GenerateTsdfVolumeFromViewExpanded(VoxelVolume<TVoxel, PlainVoxelArray>* volume,
	                                        VoxelVolume<TVoxel, PlainVoxelArray>* temporaryAllocationVolume,
	                                        const ITMView* view,
	                                        const Matrix4f& depth_camera_matrix = Matrix4f::Identity()) override;
	void IntegrateDepthImageIntoTsdfVolume(VoxelVolume<TVoxel, PlainVoxelArray>* volume, const ITMView* view);
	void IntegrateDepthImageIntoTsdfVolume(VoxelVolume<TVoxel, PlainVoxelArray>* volume, const ITMView* view,
	                                       const CameraTrackingState* trackingState);

	DepthFusionEngine_CPU() = default;
	~DepthFusionEngine_CPU() = default;
private:
	void IntegrateDepthImageIntoTsdfVolume_Helper(VoxelVolume<TVoxel, PlainVoxelArray>* volume, const ITMView* view,
	                                              Matrix4f camera_depth_matrix = Matrix4f::Identity());

};
} // namespace ITMLib
