// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Interface/VisualizationEngine.h"

namespace ITMLib
{
	template<class TVoxel, class TIndex>
	class VisualizationEngine_CPU : public VisualizationEngine < TVoxel, TIndex >
	{
	public:
		explicit VisualizationEngine_CPU(void) { }
		~VisualizationEngine_CPU(void) { }

		void FindVisibleBlocks(ITMVoxelVolume<TVoxel,TIndex> *scene, const ORUtils::SE3Pose *pose, const ITMIntrinsics *intrinsics, ITMRenderState *renderState) const;
		int CountVisibleBlocks(const ITMVoxelVolume<TVoxel,TIndex> *scene, const ITMRenderState *renderState, int minBlockId, int maxBlockId) const;
		void CreateExpectedDepths(const ITMVoxelVolume<TVoxel,TIndex> *scene, const ORUtils::SE3Pose *pose, const ITMIntrinsics *intrinsics, ITMRenderState *renderState) const;
		void RenderImage(ITMVoxelVolume<TVoxel,TIndex> *scene, const ORUtils::SE3Pose *pose, const ITMIntrinsics *intrinsics, const ITMRenderState *renderState,
		                 ITMUChar4Image *outputImage, IVisualizationEngine::RenderImageType type = IVisualizationEngine::RENDER_SHADED_GREYSCALE,
		                 IVisualizationEngine::RenderRaycastSelection raycastType = IVisualizationEngine::RENDER_FROM_NEW_RAYCAST) const;
		void FindSurface(ITMVoxelVolume<TVoxel,TIndex> *scene, const ORUtils::SE3Pose *pose, const ITMIntrinsics *intrinsics, const ITMRenderState *renderState) const;
		void CreatePointCloud(ITMVoxelVolume<TVoxel,TIndex> *scene, const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState, bool skipPoints) const;
		void CreateICPMaps(ITMVoxelVolume<TVoxel,TIndex> *scene, const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState) const;
		void ForwardRender(const ITMVoxelVolume<TVoxel,TIndex> *scene, const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState) const;
	};

	template<class TVoxel>
	class VisualizationEngine_CPU<TVoxel, VoxelBlockHash> : public VisualizationEngine < TVoxel, VoxelBlockHash >
	{
	public:
		explicit VisualizationEngine_CPU(void) { }
		~VisualizationEngine_CPU(void) { }

		void FindVisibleBlocks(ITMVoxelVolume<TVoxel,VoxelBlockHash> *scene, const ORUtils::SE3Pose *pose, const ITMIntrinsics *intrinsics, ITMRenderState *renderState) const;
		int CountVisibleBlocks(const ITMVoxelVolume<TVoxel,VoxelBlockHash> *scene, const ITMRenderState *renderState, int minBlockId, int maxBlockId) const;
		void CreateExpectedDepths(const ITMVoxelVolume<TVoxel,VoxelBlockHash> *scene, const ORUtils::SE3Pose *pose, const ITMIntrinsics *intrinsics, ITMRenderState *renderState) const;
		void RenderImage(ITMVoxelVolume<TVoxel,VoxelBlockHash> *scene, const ORUtils::SE3Pose *pose, const ITMIntrinsics *intrinsics, const ITMRenderState *renderState,
		                 ITMUChar4Image *outputImage, IVisualizationEngine::RenderImageType type = IVisualizationEngine::RENDER_SHADED_GREYSCALE,
		                 IVisualizationEngine::RenderRaycastSelection raycastType = IVisualizationEngine::RENDER_FROM_NEW_RAYCAST) const;
		void FindSurface(ITMVoxelVolume<TVoxel,VoxelBlockHash> *scene, const ORUtils::SE3Pose *pose, const ITMIntrinsics *intrinsics, const ITMRenderState *renderState) const;
		void CreatePointCloud(ITMVoxelVolume<TVoxel,VoxelBlockHash> *scene, const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState, bool skipPoints) const;
		void CreateICPMaps(ITMVoxelVolume<TVoxel,VoxelBlockHash> *scene, const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState) const;
		void ForwardRender(const ITMVoxelVolume<TVoxel,VoxelBlockHash> *scene, const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState) const;
	};
}
