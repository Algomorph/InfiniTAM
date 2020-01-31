// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Interface/MultiMeshingEngine.h"
#include "../../../Objects/Scene/MultiSceneAccess.h"

namespace ITMLib
{
	template<class TVoxel, class TIndex>
	class MultiMeshingEngine_CPU : public MultiMeshingEngine<TVoxel, TIndex>
	{
	public:
		explicit MultiMeshingEngine_CPU(const TIndex& index){};
		void MeshScene(ITMMesh *mesh, const VoxelMapGraphManager<TVoxel, TIndex> & sceneManager) {}
	};

	template<class TVoxel>
	class MultiMeshingEngine_CPU<TVoxel, VoxelBlockHash> : public MultiMeshingEngine < TVoxel, VoxelBlockHash >
	{
	public:
		explicit MultiMeshingEngine_CPU(const VoxelBlockHash& index){};
		typedef typename MultiIndex<VoxelBlockHash>::IndexData MultiIndexData;
		typedef MultiVoxel<TVoxel> MultiVoxelData;
		typedef VoxelMapGraphManager<TVoxel, VoxelBlockHash> MultiSceneManager;
		void MeshScene(ITMMesh *mesh, const MultiSceneManager & sceneManager);
	};
}