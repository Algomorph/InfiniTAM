// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Interface/SceneReconstructionEngine.h"
#include "../../../Objects/Scene/PlainVoxelArray.h"

namespace ITMLib
{
	template<class TVoxel, class TIndex>
	class SceneReconstructionEngine_CPU : public SceneReconstructionEngine < TVoxel, TIndex >
	{};

	template<class TVoxel>
	class SceneReconstructionEngine_CPU<TVoxel, VoxelBlockHash> : public SceneReconstructionEngine < TVoxel, VoxelBlockHash >
	{

	public:
		void ResetScene(ITMVoxelVolume<TVoxel, VoxelBlockHash> *scene);

		void AllocateSceneFromDepth(ITMVoxelVolume<TVoxel, VoxelBlockHash> *scene, const ITMView *view, const ITMTrackingState *trackingState,
		                            const ITMRenderState *renderState, bool onlyUpdateVisibleList = false, bool resetVisibleList = false) override;

		void IntegrateIntoScene(ITMVoxelVolume<TVoxel, VoxelBlockHash> *scene, const ITMView *view, const ITMTrackingState *trackingState,
		                        const ITMRenderState *renderState);

		SceneReconstructionEngine_CPU() = default;
		~SceneReconstructionEngine_CPU() = default;
	};

	template<class TVoxel>
	class SceneReconstructionEngine_CPU<TVoxel, PlainVoxelArray> : public SceneReconstructionEngine < TVoxel, PlainVoxelArray >
	{
	public:
		void ResetScene(ITMVoxelVolume<TVoxel, PlainVoxelArray> *scene);

		void AllocateSceneFromDepth(ITMVoxelVolume<TVoxel, PlainVoxelArray> *scene, const ITMView *view, const ITMTrackingState *trackingState,
		                            const ITMRenderState *renderState, bool onlyUpdateVisibleList = false, bool resetVisibleList = false);

		void IntegrateIntoScene(ITMVoxelVolume<TVoxel, PlainVoxelArray> *scene, const ITMView *view, const ITMTrackingState *trackingState,
		                        const ITMRenderState *renderState);

		SceneReconstructionEngine_CPU(void);
		~SceneReconstructionEngine_CPU(void);
	};
}
