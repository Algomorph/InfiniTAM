// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Interface/SceneReconstructionEngine.h"
#include "../../../Objects/Scene/PlainVoxelArray.h"

namespace ITMLib
{
	template<class TVoxel, class TIndex>
	class SceneReconstructionEngine_CUDA : public SceneReconstructionEngine < TVoxel, TIndex >
	{};

	template<class TVoxel>
	class SceneReconstructionEngine_CUDA<TVoxel, VoxelBlockHash> : public SceneReconstructionEngine < TVoxel, VoxelBlockHash >
	{
	private:
		void *allocationTempData_device;
		void *allocationTempData_host;

	public:
		void ResetScene(ITMVoxelVolume<TVoxel, VoxelBlockHash> *scene);

		void AllocateSceneFromDepth(ITMVoxelVolume<TVoxel, VoxelBlockHash> *scene, const ITMView *view, const ITMTrackingState *trackingState,
		                            const ITMRenderState *renderState, bool onlyUpdateVisibleList = false, bool resetVisibleList = false);

		void IntegrateIntoScene(ITMVoxelVolume<TVoxel, VoxelBlockHash> *scene, const ITMView *view, const ITMTrackingState *trackingState,
		                        const ITMRenderState *renderState);

		SceneReconstructionEngine_CUDA(void);
		~SceneReconstructionEngine_CUDA(void);
	};

	template<class TVoxel>
	class SceneReconstructionEngine_CUDA<TVoxel, PlainVoxelArray> : public SceneReconstructionEngine < TVoxel, PlainVoxelArray >
	{
	public:
		void ResetScene(ITMVoxelVolume<TVoxel, PlainVoxelArray> *scene);

		void AllocateSceneFromDepth(ITMVoxelVolume<TVoxel, PlainVoxelArray> *scene, const ITMView *view, const ITMTrackingState *trackingState,
		                            const ITMRenderState *renderState, bool onlyUpdateVisibleList = false, bool resetVisibleList = false);

		void IntegrateIntoScene(ITMVoxelVolume<TVoxel, PlainVoxelArray> *scene, const ITMView *view, const ITMTrackingState *trackingState,
		                        const ITMRenderState *renderState);
	};
}
