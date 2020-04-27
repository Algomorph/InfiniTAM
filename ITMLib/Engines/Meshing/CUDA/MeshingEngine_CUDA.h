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
		void MeshScene(Mesh *mesh, const VoxelVolume<TVoxel, PlainVoxelArray> *scene) override;

		explicit MeshingEngine_CUDA();
		~MeshingEngine_CUDA();
	};

	template<class TVoxel>
	class MeshingEngine_CUDA<TVoxel, VoxelBlockHash> : public MeshingEngine < TVoxel, VoxelBlockHash >
	{
	private:
		unsigned int  *triangle_count_device;
		Vector4s *visible_block_positions_device;
		const bool streaming_mode_enabled;
		bool buffers_allocated;

	public:
		void MeshScene(Mesh *mesh, const VoxelVolume<TVoxel, VoxelBlockHash> *volume) override;
		void PreallocateTemporaryBuffers(int maximum_block_count);
		explicit MeshingEngine_CUDA();
		~MeshingEngine_CUDA();
	};

}
