// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Interface/MeshingEngine.h"
#include "../../../Objects/Volume/PlainVoxelArray.h"

namespace ITMLib
{
	template<class TVoxel, class TIndex>
	class MeshingEngine_CUDA : public MeshingEngine < TVoxel, TIndex >{};

	template<class TVoxel>
	class MeshingEngine_CUDA<TVoxel, VoxelBlockHash> : public MeshingEngine < TVoxel, VoxelBlockHash >
	{
	private:
		unsigned int  *noTriangles_device;
		Vector4s *visibleBlockGlobalPos_device;

	public:
		void MeshScene(Mesh *mesh, const VoxelVolume<TVoxel, VoxelBlockHash> *scene) override;

		explicit MeshingEngine_CUDA(const VoxelBlockHash& index);
		~MeshingEngine_CUDA();
	};

	template<class TVoxel>
	class MeshingEngine_CUDA<TVoxel, PlainVoxelArray> : public MeshingEngine < TVoxel, PlainVoxelArray >
	{
	public:
		void MeshScene(Mesh *mesh, const VoxelVolume<TVoxel, PlainVoxelArray> *scene) override;

		explicit MeshingEngine_CUDA(const PlainVoxelArray& index);
		~MeshingEngine_CUDA();
	};
}
