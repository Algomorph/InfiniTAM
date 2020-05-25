// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Interface/RenderingEngine.h"

namespace ITMLib
{
	template<class TVoxel, class TIndex>
	class VisualizationEngine_CPU : public RenderingEngineBase < TVoxel, TIndex >
	{
	public:
		explicit VisualizationEngine_CPU() { }
		~VisualizationEngine_CPU() { }

		void FindVisibleBlocks(VoxelVolume<TVoxel,TIndex> *volume, const ORUtils::SE3Pose *pose, const Intrinsics *intrinsics, RenderState *renderState) const;
		int CountVisibleBlocks(const VoxelVolume<TVoxel,TIndex> *volume, const RenderState *renderState, int minBlockId, int maxBlockId) const;
		void CreateExpectedDepths(const VoxelVolume<TVoxel,TIndex> *volume, const ORUtils::SE3Pose *pose, const Intrinsics *intrinsics, RenderState *renderState) const;
		void RenderImage(VoxelVolume<TVoxel,TIndex> *scene, const ORUtils::SE3Pose *pose, const Intrinsics *intrinsics, const RenderState *renderState,
		                 UChar4Image *outputImage, IRenderingEngine::RenderImageType type = IRenderingEngine::RENDER_SHADED_GREYSCALE,
		                 IRenderingEngine::RenderRaycastSelection raycastType = IRenderingEngine::RENDER_FROM_NEW_RAYCAST) const;
		void FindSurface(VoxelVolume<TVoxel,TIndex> *scene, const ORUtils::SE3Pose *pose, const Intrinsics *intrinsics, const RenderState *renderState) const;
		void CreatePointCloud(VoxelVolume<TVoxel,TIndex> *scene, const View *view, CameraTrackingState *trackingState, RenderState *renderState, bool skipPoints) const;
		void CreateICPMaps(VoxelVolume<TVoxel,TIndex> *scene, const View *view, CameraTrackingState *trackingState, RenderState *renderState) const;
		void ForwardRender(const VoxelVolume<TVoxel,TIndex> *scene, const View *view, CameraTrackingState *trackingState, RenderState *renderState) const;
	};

	template<class TVoxel>
	class VisualizationEngine_CPU<TVoxel, VoxelBlockHash> : public RenderingEngineBase < TVoxel, VoxelBlockHash >
	{
	public:
		explicit VisualizationEngine_CPU() { }
		~VisualizationEngine_CPU() { }

		void FindVisibleBlocks(VoxelVolume<TVoxel,VoxelBlockHash> *volume, const ORUtils::SE3Pose *pose, const Intrinsics *intrinsics, RenderState *renderState) const;
		int CountVisibleBlocks(const VoxelVolume<TVoxel,VoxelBlockHash> *volume, const RenderState *renderState, int minBlockId, int maxBlockId) const;
		void CreateExpectedDepths(const VoxelVolume<TVoxel,VoxelBlockHash> *volume, const ORUtils::SE3Pose *pose, const Intrinsics *intrinsics, RenderState *renderState) const;
		void RenderImage(VoxelVolume<TVoxel,VoxelBlockHash> *scene, const ORUtils::SE3Pose *pose, const Intrinsics *intrinsics, const RenderState *renderState,
		                 UChar4Image *outputImage, IRenderingEngine::RenderImageType type = IRenderingEngine::RENDER_SHADED_GREYSCALE,
		                 IRenderingEngine::RenderRaycastSelection raycastType = IRenderingEngine::RENDER_FROM_NEW_RAYCAST) const;
		void FindSurface(VoxelVolume<TVoxel,VoxelBlockHash> *scene, const ORUtils::SE3Pose *pose, const Intrinsics *intrinsics, const RenderState *renderState) const;
		void CreatePointCloud(VoxelVolume<TVoxel,VoxelBlockHash> *scene, const View *view, CameraTrackingState *trackingState, RenderState *renderState, bool skipPoints) const;
		void CreateICPMaps(VoxelVolume<TVoxel,VoxelBlockHash> *scene, const View *view, CameraTrackingState *trackingState, RenderState *renderState) const;
		void ForwardRender(const VoxelVolume<TVoxel,VoxelBlockHash> *scene, const View *view, CameraTrackingState *trackingState, RenderState *renderState) const;
	};
}
