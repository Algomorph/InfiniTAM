// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Interface/MultiMeshingEngine.h"
#include "../../../Objects/Scene/MultiSceneAccess.h"

namespace ITMLib
{
	template<class TVoxel, class TIndex>
	class MultiMeshingEngine_CUDA : public MultiMeshingEngine<TVoxel, TIndex>
	{
	public:
		explicit MultiMeshingEngine_CUDA(const TIndex& index){};
		void MeshScene(ITMMesh *mesh, const VoxelMapGraphManager<TVoxel, TIndex> & sceneManager) {}
	};

	template<class TVoxel>
	class MultiMeshingEngine_CUDA<TVoxel, VoxelBlockHash> : public MultiMeshingEngine < TVoxel, VoxelBlockHash >
	{
	private:
		unsigned int  *noTriangles_device;
		Vector4s *visibleBlockGlobalPos_device;

	public:
		typedef typename MultiIndex<VoxelBlockHash>::IndexData MultiIndexData;
		typedef MultiVoxel<TVoxel> MultiVoxelData;
		typedef VoxelMapGraphManager<TVoxel, VoxelBlockHash> MultiSceneManager;

		MultiIndexData *indexData_device, indexData_host;
		MultiVoxelData *voxelData_device, voxelData_host;

		void MeshScene(ITMMesh *mesh, const MultiSceneManager & sceneManager);

		explicit MultiMeshingEngine_CUDA(const VoxelBlockHash& index);
		~MultiMeshingEngine_CUDA();
	};
}

