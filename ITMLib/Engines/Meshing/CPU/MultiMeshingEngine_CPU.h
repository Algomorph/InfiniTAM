// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Interface/MultiMeshingEngine.h"
#include "../../../Objects/Volume/MultiSceneAccess.h"

namespace ITMLib
{
	template<class TVoxel, class TIndex>
	class MultiMeshingEngine_CPU : public MultiMeshingEngine<TVoxel, TIndex>
	{
	public:
		explicit MultiMeshingEngine_CPU(const TIndex& index){};
		Mesh MeshScene(const VoxelMapGraphManager<TVoxel, TIndex> & sceneManager) override {
			DIEWITHEXCEPTION_REPORTLOCATION("Not implemented.");
		}
	};

	template<class TVoxel>
	class MultiMeshingEngine_CPU<TVoxel, VoxelBlockHash> : public MultiMeshingEngine < TVoxel, VoxelBlockHash >
	{
	public:
		explicit MultiMeshingEngine_CPU(const VoxelBlockHash& index){};
		typedef typename MultiIndex<VoxelBlockHash>::IndexData MultiIndexData;
		typedef MultiVoxel<TVoxel> MultiVoxelData;
		typedef VoxelMapGraphManager<TVoxel, VoxelBlockHash> MultiSceneManager;
		Mesh MeshScene(const MultiSceneManager & sceneManager);
	};
}