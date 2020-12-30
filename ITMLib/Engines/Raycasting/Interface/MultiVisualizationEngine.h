// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../MultiScene/VoxelMapGraphManager.h"
#include "RaycastingEngineInterface.h"

namespace ITMLib {

	template<class TVoxel, class TIndex>
	class MultiVisualizationEngine
	{
	public:
		virtual ~MultiVisualizationEngine() {}

		virtual void PrepareRenderState(const VoxelMapGraphManager<TVoxel, TIndex> & sceneManager, RenderState *state) = 0;

		//skip "FindVisibleBlocks"

		virtual void CreateExpectedDepths(const VoxelMapGraphManager<TVoxel, TIndex> & sceneManager, const ORUtils::SE3Pose *pose, const Intrinsics *intrinsics, RenderState *renderState) const = 0;
		virtual void RenderImage(const ORUtils::SE3Pose *pose, const Intrinsics *intrinsics, RenderState *renderState,
		                         UChar4Image *outputImage, IRaycastingEngine::RenderImageType type) const = 0;
	};
}

