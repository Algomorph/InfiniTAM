// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../MultiScene/ITMMapGraphManager.h"
#include "VisualizationEngine.h"

namespace ITMLib {

	template<class TVoxel, class TIndex>
	class MultiVisualizationEngine
	{
	public:
		virtual ~MultiVisualizationEngine(void) {}

		virtual void PrepareRenderState(const ITMVoxelMapGraphManager<TVoxel, TIndex> & sceneManager, ITMRenderState *state) = 0;

		//skip "FindVisibleBlocks"

		virtual void CreateExpectedDepths(const ITMVoxelMapGraphManager<TVoxel, TIndex> & sceneManager, const ORUtils::SE3Pose *pose, const ITMIntrinsics *intrinsics, ITMRenderState *renderState) const = 0;
		virtual void RenderImage(const ORUtils::SE3Pose *pose, const ITMIntrinsics *intrinsics, ITMRenderState *renderState,
			ITMUChar4Image *outputImage, IVisualizationEngine::RenderImageType type) const = 0;
	};
}

