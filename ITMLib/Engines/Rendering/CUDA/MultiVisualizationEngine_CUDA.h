// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Interface/MultiVisualizationEngine.h"

namespace ITMLib {
	struct RenderingBlock;
	template<class TVoxel, class TIndex>
	class MultiVisualizationEngine_CUDA : public MultiVisualizationEngine<TVoxel, TIndex>
	{
	private:

	public:
		MultiVisualizationEngine_CUDA();
		~MultiVisualizationEngine_CUDA();

		void PrepareRenderState(const VoxelMapGraphManager<TVoxel, TIndex> & sceneManager, RenderState *state);

		void CreateExpectedDepths(const VoxelMapGraphManager<TVoxel, TIndex> & sceneManager, const ORUtils::SE3Pose *pose, const Intrinsics *intrinsics, RenderState *renderState) const;

		void RenderImage(const ORUtils::SE3Pose *pose, const Intrinsics *intrinsics, RenderState *renderState, UChar4Image *outputImage, IRenderingEngine::RenderImageType type) const;
	};

	template<class TVoxel>
	class MultiVisualizationEngine_CUDA<TVoxel, VoxelBlockHash> : public MultiVisualizationEngine<TVoxel, VoxelBlockHash>
	{
	private:
		RenderingBlock *renderingBlockList_device;
		uint *noTotalBlocks_device;

	public:
		MultiVisualizationEngine_CUDA();
		~MultiVisualizationEngine_CUDA();

		void PrepareRenderState(const VoxelMapGraphManager<TVoxel, VoxelBlockHash> & sceneManager, RenderState *state);

		void CreateExpectedDepths(const VoxelMapGraphManager<TVoxel, VoxelBlockHash> & sceneManager, const ORUtils::SE3Pose *pose, const Intrinsics *intrinsics, RenderState *renderState) const;

		void RenderImage(const ORUtils::SE3Pose *pose, const Intrinsics *intrinsics, RenderState *renderState, UChar4Image *outputImage, IRenderingEngine::RenderImageType type) const;
	};
}