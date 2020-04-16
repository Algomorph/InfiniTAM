// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Interface/SceneReconstructionEngine.h"
#include "../../../Objects/Volume/PlainVoxelArray.h"

namespace ITMLib
{
	template<class TVoxel, class TIndex>
	class SceneReconstructionEngine_CPU : public SceneReconstructionEngine < TVoxel, TIndex >
	{};

	template<class TVoxel>
	class SceneReconstructionEngine_CPU<TVoxel, VoxelBlockHash> : public SceneReconstructionEngine < TVoxel, VoxelBlockHash >
	{

	public:
		void ResetScene(VoxelVolume<TVoxel, VoxelBlockHash> *volume);

		void AllocateSceneFromDepth(VoxelVolume<TVoxel, VoxelBlockHash> *scene, const View *view, const CameraTrackingState *trackingState,
		                            const RenderState *renderState, bool onlyUpdateVisibleList = false, bool resetVisibleList = false) override;

		void IntegrateIntoScene(VoxelVolume<TVoxel, VoxelBlockHash> *scene, const View *view, const CameraTrackingState *trackingState,
		                        const RenderState *renderState);

		SceneReconstructionEngine_CPU() = default;
		~SceneReconstructionEngine_CPU() = default;
	};

	template<class TVoxel>
	class SceneReconstructionEngine_CPU<TVoxel, PlainVoxelArray> : public SceneReconstructionEngine < TVoxel, PlainVoxelArray >
	{
	public:
		void ResetScene(VoxelVolume<TVoxel, PlainVoxelArray> *scene);

		void AllocateSceneFromDepth(VoxelVolume<TVoxel, PlainVoxelArray> *scene, const View *view, const CameraTrackingState *trackingState,
		                            const RenderState *renderState, bool onlyUpdateVisibleList = false, bool resetVisibleList = false);

		void IntegrateIntoScene(VoxelVolume<TVoxel, PlainVoxelArray> *scene, const View *view, const CameraTrackingState *trackingState,
		                        const RenderState *renderState);

		SceneReconstructionEngine_CPU(void);
		~SceneReconstructionEngine_CPU(void);
	};
}
