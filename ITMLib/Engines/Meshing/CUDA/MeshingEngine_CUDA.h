// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Interface/MeshingEngine.h"
#include "../../../Objects/Volume/PlainVoxelArray.h"

namespace ITMLib
{
	template<class TVoxel, class TIndex>
	class MeshingEngine_CUDA : public MeshingEngine < TVoxel, TIndex >{};

	template<class TVoxel>
	class MeshingEngine_CUDA<TVoxel, PlainVoxelArray> : public MeshingEngine < TVoxel, PlainVoxelArray >
	{
	public:
		//TODO: implement meshing for PVA (for completeness / consistency)
		Mesh MeshVolume(const VoxelVolume<TVoxel, PlainVoxelArray> *volume) override;

		explicit MeshingEngine_CUDA();
		~MeshingEngine_CUDA();
	};

	template<class TVoxel>
	class MeshingEngine_CUDA<TVoxel, VoxelBlockHash> : public MeshingEngine < TVoxel, VoxelBlockHash >
	{

	public:
		Mesh MeshVolume(const VoxelVolume<TVoxel, VoxelBlockHash> *volume) override;
		explicit MeshingEngine_CUDA();
		~MeshingEngine_CUDA();
	};

}
