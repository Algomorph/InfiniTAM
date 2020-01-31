// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Interface/ITMMultiMeshingEngine.h"
#include "../../../Objects/Scene/ITMMultiSceneAccess.h"

namespace ITMLib
{
	template<class TVoxel, class TIndex>
	class ITMMultiMeshingEngine_CUDA : public ITMMultiMeshingEngine<TVoxel, TIndex>
	{
	public:
		explicit ITMMultiMeshingEngine_CUDA(const TIndex& index){};
		void MeshScene(ITMMesh *mesh, const VoxelMapGraphManager<TVoxel, TIndex> & sceneManager) {}
	};

	template<class TVoxel>
	class ITMMultiMeshingEngine_CUDA<TVoxel, VoxelBlockHash> : public ITMMultiMeshingEngine < TVoxel, VoxelBlockHash >
	{
	private:
		unsigned int  *noTriangles_device;
		Vector4s *visibleBlockGlobalPos_device;

	public:
		typedef typename ITMMultiIndex<VoxelBlockHash>::IndexData MultiIndexData;
		typedef ITMMultiVoxel<TVoxel> MultiVoxelData;
		typedef VoxelMapGraphManager<TVoxel, VoxelBlockHash> MultiSceneManager;

		MultiIndexData *indexData_device, indexData_host;
		MultiVoxelData *voxelData_device, voxelData_host;

		void MeshScene(ITMMesh *mesh, const MultiSceneManager & sceneManager);

		explicit ITMMultiMeshingEngine_CUDA(const VoxelBlockHash& index);
		~ITMMultiMeshingEngine_CUDA();
	};
}

