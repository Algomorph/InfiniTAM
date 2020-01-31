// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Interface/ITMMultiMeshingEngine.h"
#include "../../../Objects/Scene/ITMMultiSceneAccess.h"

namespace ITMLib
{
	template<class TVoxel, class TIndex>
	class ITMMultiMeshingEngine_CPU : public ITMMultiMeshingEngine<TVoxel, TIndex>
	{
	public:
		explicit ITMMultiMeshingEngine_CPU(const TIndex& index){};
		void MeshScene(ITMMesh *mesh, const VoxelMapGraphManager<TVoxel, TIndex> & sceneManager) {}
	};

	template<class TVoxel>
	class ITMMultiMeshingEngine_CPU<TVoxel, VoxelBlockHash> : public ITMMultiMeshingEngine < TVoxel, VoxelBlockHash >
	{
	public:
		explicit ITMMultiMeshingEngine_CPU(const VoxelBlockHash& index){};
		typedef typename ITMMultiIndex<VoxelBlockHash>::IndexData MultiIndexData;
		typedef ITMMultiVoxel<TVoxel> MultiVoxelData;
		typedef VoxelMapGraphManager<TVoxel, VoxelBlockHash> MultiSceneManager;
		void MeshScene(ITMMesh *mesh, const MultiSceneManager & sceneManager);
	};
}