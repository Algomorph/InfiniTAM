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

		void FindVisibleBlocks(VoxelVolume<TVoxel,TIndex> *scene, const ORUtils::SE3Pose *pose, const Intrinsics *intrinsics, RenderState *renderState) const;
		int CountVisibleBlocks(const VoxelVolume<TVoxel,TIndex> *scene, const RenderState *renderState, int minBlockId, int maxBlockId) const;
		void CreateExpectedDepths(const VoxelVolume<TVoxel,TIndex> *scene, const ORUtils::SE3Pose *pose, const Intrinsics *intrinsics, RenderState *renderState) const;
		void RenderImage(VoxelVolume<TVoxel,TIndex> *scene, const ORUtils::SE3Pose *pose, const Intrinsics *intrinsics, const RenderState *renderState,
		                 ITMUChar4Image *outputImage, IVisualizationEngine::RenderImageType type = IVisualizationEngine::RENDER_SHADED_GREYSCALE,
		                 IVisualizationEngine::RenderRaycastSelection raycastType = IVisualizationEngine::RENDER_FROM_NEW_RAYCAST) const;
		void FindSurface(VoxelVolume<TVoxel,TIndex> *scene, const ORUtils::SE3Pose *pose, const Intrinsics *intrinsics, const RenderState *renderState) const;
		void CreatePointCloud(VoxelVolume<TVoxel,TIndex> *scene, const ITMView *view, CameraTrackingState *trackingState, RenderState *renderState, bool skipPoints) const;
		void CreateICPMaps(VoxelVolume<TVoxel,TIndex> *scene, const ITMView *view, CameraTrackingState *trackingState, RenderState *renderState) const;
		void ForwardRender(const VoxelVolume<TVoxel,TIndex> *scene, const ITMView *view, CameraTrackingState *trackingState, RenderState *renderState) const;
	};

	template<class TVoxel>
	class VisualizationEngine_CPU<TVoxel, VoxelBlockHash> : public VisualizationEngine < TVoxel, VoxelBlockHash >
	{
	public:
		explicit VisualizationEngine_CPU(void) { }
		~VisualizationEngine_CPU(void) { }

		void FindVisibleBlocks(VoxelVolume<TVoxel,VoxelBlockHash> *scene, const ORUtils::SE3Pose *pose, const Intrinsics *intrinsics, RenderState *renderState) const;
		int CountVisibleBlocks(const VoxelVolume<TVoxel,VoxelBlockHash> *scene, const RenderState *renderState, int minBlockId, int maxBlockId) const;
		void CreateExpectedDepths(const VoxelVolume<TVoxel,VoxelBlockHash> *scene, const ORUtils::SE3Pose *pose, const Intrinsics *intrinsics, RenderState *renderState) const;
		void RenderImage(VoxelVolume<TVoxel,VoxelBlockHash> *scene, const ORUtils::SE3Pose *pose, const Intrinsics *intrinsics, const RenderState *renderState,
		                 ITMUChar4Image *outputImage, IVisualizationEngine::RenderImageType type = IVisualizationEngine::RENDER_SHADED_GREYSCALE,
		                 IVisualizationEngine::RenderRaycastSelection raycastType = IVisualizationEngine::RENDER_FROM_NEW_RAYCAST) const;
		void FindSurface(VoxelVolume<TVoxel,VoxelBlockHash> *scene, const ORUtils::SE3Pose *pose, const Intrinsics *intrinsics, const RenderState *renderState) const;
		void CreatePointCloud(VoxelVolume<TVoxel,VoxelBlockHash> *scene, const ITMView *view, CameraTrackingState *trackingState, RenderState *renderState, bool skipPoints) const;
		void CreateICPMaps(VoxelVolume<TVoxel,VoxelBlockHash> *scene, const ITMView *view, CameraTrackingState *trackingState, RenderState *renderState) const;
		void ForwardRender(const VoxelVolume<TVoxel,VoxelBlockHash> *scene, const ITMView *view, CameraTrackingState *trackingState, RenderState *renderState) const;
	};
}
