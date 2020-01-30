// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Interface/SwappingEngine.h"

namespace ITMLib
{
	template<class TVoxel, class TIndex>
	class SwappingEngine_CPU : public SwappingEngine < TVoxel, TIndex >
	{
	public:
		explicit SwappingEngine_CPU(const TIndex& index){};
		void IntegrateGlobalIntoLocal(ITMVoxelVolume<TVoxel, TIndex> *scene, ITMRenderState *renderState) {}
		void SaveToGlobalMemory(ITMVoxelVolume<TVoxel, TIndex> *scene, ITMRenderState *renderState) {}
		void CleanLocalMemory(ITMVoxelVolume<TVoxel, TIndex> *scene, ITMRenderState *renderState) {}
	};

	template<class TVoxel>
	class SwappingEngine_CPU<TVoxel, VoxelBlockHash> : public SwappingEngine < TVoxel, VoxelBlockHash >
	{
	private:
		int LoadFromGlobalMemory(ITMVoxelVolume<TVoxel, VoxelBlockHash> *scene);

	public:
		// This class is currently just for debugging purposes -- swaps CPU memory to CPU memory.
		// Potentially this could stream into the host memory from somewhere else (disk, database, etc.).

		void IntegrateGlobalIntoLocal(ITMVoxelVolume<TVoxel, VoxelBlockHash> *scene, ITMRenderState *renderState);
		void SaveToGlobalMemory(ITMVoxelVolume<TVoxel, VoxelBlockHash> *scene, ITMRenderState *renderState);
		void CleanLocalMemory(ITMVoxelVolume<TVoxel, VoxelBlockHash> *scene, ITMRenderState *renderState);

		explicit SwappingEngine_CPU(const VoxelBlockHash& index){};
		~SwappingEngine_CPU() = default;
	};
}
