// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Interface/ITMSwappingEngine.h"

namespace ITMLib
{
	template<class TVoxel, class TIndex>
	class ITMSwappingEngine_CUDA : public ITMSwappingEngine < TVoxel, TIndex >
	{
	public:
		void IntegrateGlobalIntoLocal(ITMVoxelVolume<TVoxel, TIndex> *scene, ITMRenderState *renderState) {}
		void SaveToGlobalMemory(ITMVoxelVolume<TVoxel, TIndex> *scene, ITMRenderState *renderState) {}
		void CleanLocalMemory(ITMVoxelVolume<TVoxel, TIndex> *scene, ITMRenderState *renderState) {}
	};

	template<class TVoxel>
	class ITMSwappingEngine_CUDA<TVoxel, ITMVoxelBlockHash> : public ITMSwappingEngine < TVoxel, ITMVoxelBlockHash >
	{
	private:
		int *noNeededEntries_device, *noAllocatedVoxelEntries_device;
		int *entriesToClean_device;

		int LoadFromGlobalMemory(ITMVoxelVolume<TVoxel, ITMVoxelBlockHash> *scene);

	public:
		void IntegrateGlobalIntoLocal(ITMVoxelVolume<TVoxel, ITMVoxelBlockHash> *scene, ITMRenderState *renderState);
		void SaveToGlobalMemory(ITMVoxelVolume<TVoxel, ITMVoxelBlockHash> *scene, ITMRenderState *renderState);
		void CleanLocalMemory(ITMVoxelVolume<TVoxel, ITMVoxelBlockHash> *scene, ITMRenderState *renderState);

		ITMSwappingEngine_CUDA(void);
		~ITMSwappingEngine_CUDA(void);
	};
}
