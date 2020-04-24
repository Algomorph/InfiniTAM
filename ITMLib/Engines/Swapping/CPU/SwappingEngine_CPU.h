// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Interface/SwappingEngine.h"

namespace ITMLib
{
	template<class TVoxel, class TIndex>
	class SwappingEngine_CPU : public SwappingEngine < TVoxel, TIndex >
	{
	public:
		explicit SwappingEngine_CPU(typename TIndex::InitializationParameters index_parameters){};
		void IntegrateGlobalIntoLocal(VoxelVolume<TVoxel, TIndex> *scene, RenderState *renderState) {}
		void SaveToGlobalMemory(VoxelVolume<TVoxel, TIndex> *scene, RenderState *renderState) {}
		void CleanLocalMemory(VoxelVolume<TVoxel, TIndex> *scene, RenderState *renderState) {}
	};

	template<class TVoxel>
	class SwappingEngine_CPU<TVoxel, VoxelBlockHash> : public SwappingEngine < TVoxel, VoxelBlockHash >
	{
	private:
		int LoadFromGlobalMemory(VoxelVolume<TVoxel, VoxelBlockHash> *volume);

	public:
		// This class is currently just for debugging purposes -- swaps CPU memory to CPU memory.
		// Potentially this could stream into the host memory from somewhere else (disk, database, etc.).

		void IntegrateGlobalIntoLocal(VoxelVolume<TVoxel, VoxelBlockHash> *volume, RenderState *render_state);
		void SaveToGlobalMemory(VoxelVolume<TVoxel, VoxelBlockHash> *volume, RenderState *render_state);
		void CleanLocalMemory(VoxelVolume<TVoxel, VoxelBlockHash> *volume, RenderState *renderState);

		explicit SwappingEngine_CPU(typename VoxelBlockHash::InitializationParameters index){};
		~SwappingEngine_CPU() = default;
	};
}
