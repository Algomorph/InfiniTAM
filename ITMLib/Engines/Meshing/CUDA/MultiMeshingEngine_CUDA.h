// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Interface/MultiMeshingEngine.h"
#include "../../../Objects/Volume/MultiSceneAccess.h"

namespace ITMLib
{
	template<class TVoxel, class TIndex>
	class MultiMeshingEngine_CUDA : public MultiMeshingEngine<TVoxel, TIndex>
	{
	public:
		explicit MultiMeshingEngine_CUDA(const TIndex& index){};
		Mesh MeshScene(const VoxelMapGraphManager<TVoxel, TIndex> & manager) override {
			DIEWITHEXCEPTION_REPORTLOCATION("Not implemented");
			return Mesh();
		}
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
		typedef VoxelMapGraphManager<TVoxel, VoxelBlockHash> MultiVolumeManager;

		MultiIndexData *indexData_device, indexData_host;
		MultiVoxelData *voxelData_device, voxelData_host;

		Mesh MeshScene(const MultiVolumeManager & manager) override;

		explicit MultiMeshingEngine_CUDA(const VoxelBlockHash& index);
		~MultiMeshingEngine_CUDA();
	};
}

