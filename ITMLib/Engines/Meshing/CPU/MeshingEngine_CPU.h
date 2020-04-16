// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Interface/MeshingEngine.h"
#include "../../../Objects/Volume/PlainVoxelArray.h"

namespace ITMLib
{
	template<class TVoxel, class TIndex>
	class MeshingEngine_CPU : public MeshingEngine < TVoxel, TIndex >
	{
	public:
		explicit MeshingEngine_CPU(const TIndex& index) : MeshingEngine<TVoxel,TIndex>(index){};
		//TODO: implement meshing for PVA (for completeness / consistency)
		void MeshScene(Mesh *mesh, const VoxelVolume<TVoxel, TIndex> *scene) {
			DIEWITHEXCEPTION_REPORTLOCATION("Not implemented");
		}
	};

	template<class TVoxel>
	class MeshingEngine_CPU<TVoxel, VoxelBlockHash> : public MeshingEngine < TVoxel, VoxelBlockHash >
	{
	public:
		void MeshScene(Mesh *mesh, const VoxelVolume<TVoxel, VoxelBlockHash> *volume);

		explicit MeshingEngine_CPU(const VoxelBlockHash& index) :
				MeshingEngine<TVoxel,VoxelBlockHash>(index) { }
		~MeshingEngine_CPU() = default;
	};
}
