// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Interface/SwappingEngine.h"

namespace ITMLib
{
	template<class TVoxel, class TIndex>
	class SwappingEngine_CUDA : public SwappingEngine < TVoxel, TIndex >
	{
	public:
		explicit SwappingEngine_CUDA(const TIndex& index){};
		void IntegrateGlobalIntoLocal(VoxelVolume<TVoxel, TIndex> *scene, RenderState *renderState) {}
		void SaveToGlobalMemory(VoxelVolume<TVoxel, TIndex> *scene, RenderState *renderState) {}
		void CleanLocalMemory(VoxelVolume<TVoxel, TIndex> *scene, RenderState *renderState) {}
	};

	template<class TVoxel>
	class SwappingEngine_CUDA<TVoxel, VoxelBlockHash> : public SwappingEngine < TVoxel, VoxelBlockHash >
	{
	private:
		int *noNeededEntries_device, *noAllocatedVoxelEntries_device;
		int *entriesToClean_device;

		int LoadFromGlobalMemory(VoxelVolume<TVoxel, VoxelBlockHash> *volume);

	public:
		void IntegrateGlobalIntoLocal(VoxelVolume<TVoxel, VoxelBlockHash> *volume, RenderState *render_state);
		void SaveToGlobalMemory(VoxelVolume<TVoxel, VoxelBlockHash> *volume, RenderState *render_state);
		void CleanLocalMemory(VoxelVolume<TVoxel, VoxelBlockHash> *volume, RenderState *renderState);

		explicit SwappingEngine_CUDA(const VoxelBlockHash& index);
		~SwappingEngine_CUDA();
	};
}
