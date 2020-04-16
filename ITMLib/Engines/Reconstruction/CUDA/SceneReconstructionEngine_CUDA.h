// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Interface/SceneReconstructionEngine.h"
#include "../../../Objects/Volume/PlainVoxelArray.h"

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
		void ResetScene(VoxelVolume<TVoxel, VoxelBlockHash> *volume);

		void AllocateSceneFromDepth(VoxelVolume<TVoxel, VoxelBlockHash> *volume, const View *view, const CameraTrackingState *trackingState,
		                            const RenderState *renderState, bool onlyUpdateVisibleList = false, bool resetVisibleList = false);

		void IntegrateIntoScene(VoxelVolume<TVoxel, VoxelBlockHash> *volume, const View *view, const CameraTrackingState *tracking_state,
		                        const RenderState *render_state);

		SceneReconstructionEngine_CUDA();
		~SceneReconstructionEngine_CUDA();
	};

	template<class TVoxel>
	class SceneReconstructionEngine_CUDA<TVoxel, PlainVoxelArray> : public SceneReconstructionEngine < TVoxel, PlainVoxelArray >
	{
	public:
		void ResetScene(VoxelVolume<TVoxel, PlainVoxelArray> *volume);

		void AllocateSceneFromDepth(VoxelVolume<TVoxel, PlainVoxelArray> *scene, const View *view, const CameraTrackingState *trackingState,
		                            const RenderState *renderState, bool onlyUpdateVisibleList = false, bool resetVisibleList = false);

		void IntegrateIntoScene(VoxelVolume<TVoxel, PlainVoxelArray> *volume, const View *view, const CameraTrackingState *trackingState,
		                        const RenderState *renderState);
	};
}
