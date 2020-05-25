// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Interface/VisualizationEngine.h"

struct RenderingBlock;

namespace ITMLib
{
	template<class TVoxel, class TIndex>
	class VisualizationEngine_CUDA : public VisualizationEngine < TVoxel, TIndex >
	{
	private:
		uint *noTotalPoints_device;

	public:
		explicit VisualizationEngine_CUDA();
		~VisualizationEngine_CUDA();

		void FindVisibleBlocks(VoxelVolume<TVoxel,TIndex> *volume, const ORUtils::SE3Pose *pose, const Intrinsics *intrinsics, RenderState *renderState) const;
		int CountVisibleBlocks(const VoxelVolume<TVoxel,TIndex> *volume, const RenderState *renderState, int minBlockId, int maxBlockId) const;
		void CreateExpectedDepths(const VoxelVolume<TVoxel,TIndex> *volume, const ORUtils::SE3Pose *pose, const Intrinsics *intrinsics, RenderState *renderState) const;
		void RenderImage(VoxelVolume<TVoxel,TIndex> *scene, const ORUtils::SE3Pose *pose, const Intrinsics *intrinsics, const RenderState *renderState,
		                 UChar4Image *outputImage, IVisualizationEngine::RenderImageType type = IVisualizationEngine::RENDER_SHADED_GREYSCALE,
		                 IVisualizationEngine::RenderRaycastSelection raycastType = IVisualizationEngine::RENDER_FROM_NEW_RAYCAST) const;
		void FindSurface(VoxelVolume<TVoxel,TIndex> *scene, const ORUtils::SE3Pose *pose, const Intrinsics *intrinsics, const RenderState *renderState) const;
		void CreatePointCloud(VoxelVolume<TVoxel,TIndex> *scene, const View *view, CameraTrackingState *trackingState, RenderState *renderState, bool skipPoints) const;
		void CreateICPMaps(VoxelVolume<TVoxel,TIndex> *scene, const View *view, CameraTrackingState *trackingState, RenderState *renderState) const;
		void ForwardRender(const VoxelVolume<TVoxel,TIndex> *scene, const View *view, CameraTrackingState *trackingState, RenderState *renderState) const;
	};

	template<class TVoxel>
	class VisualizationEngine_CUDA<TVoxel, VoxelBlockHash> : public VisualizationEngine < TVoxel, VoxelBlockHash >
	{
	private:
		uint* noTotalPoints_device;
		RenderingBlock* rendering_block_list_device;
		uint* block_count_device;
		int* visible_block_count_device;
	public:
		explicit VisualizationEngine_CUDA();
		~VisualizationEngine_CUDA();

		void FindVisibleBlocks(VoxelVolume<TVoxel,VoxelBlockHash> *volume, const ORUtils::SE3Pose *pose, const Intrinsics *intrinsics, RenderState *renderState) const;
		int CountVisibleBlocks(const VoxelVolume<TVoxel,VoxelBlockHash> *volume, const RenderState *renderState, int minBlockId, int maxBlockId) const;
		void CreateExpectedDepths(const VoxelVolume<TVoxel,VoxelBlockHash> *volume, const ORUtils::SE3Pose *pose, const Intrinsics *intrinsics, RenderState *renderState) const;
		void RenderImage(VoxelVolume<TVoxel,VoxelBlockHash> *scene, const ORUtils::SE3Pose *pose, const Intrinsics *intrinsics, const RenderState *renderState,
		                 UChar4Image *outputImage, IVisualizationEngine::RenderImageType type = IVisualizationEngine::RENDER_SHADED_GREYSCALE,
		                 IVisualizationEngine::RenderRaycastSelection raycastType = IVisualizationEngine::RENDER_FROM_NEW_RAYCAST) const;
		void FindSurface(VoxelVolume<TVoxel,VoxelBlockHash> *scene, const ORUtils::SE3Pose *pose, const Intrinsics *intrinsics, const RenderState *renderState) const;
		void CreatePointCloud(VoxelVolume<TVoxel,VoxelBlockHash> *scene, const View *view, CameraTrackingState *trackingState, RenderState *renderState, bool skipPoints) const;
		void CreateICPMaps(VoxelVolume<TVoxel,VoxelBlockHash> *scene, const View *view, CameraTrackingState *trackingState, RenderState *renderState) const;
		void ForwardRender(const VoxelVolume<TVoxel,VoxelBlockHash> *scene, const View *view, CameraTrackingState *trackingState, RenderState *renderState) const;
	};
}
