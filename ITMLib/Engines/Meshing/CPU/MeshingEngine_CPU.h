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
		explicit MeshingEngine_CPU() = default;
		//TODO: implement meshing for PVA (for completeness / consistency)
		Mesh MeshVolume(const VoxelVolume<TVoxel, TIndex> *volume) {
			DIEWITHEXCEPTION_REPORTLOCATION("Not implemented");
		}
	};

	template<class TVoxel>
	class MeshingEngine_CPU<TVoxel, VoxelBlockHash> : public MeshingEngine < TVoxel, VoxelBlockHash >
	{
	public:
		Mesh MeshVolume(const VoxelVolume<TVoxel, VoxelBlockHash> *volume);

		explicit MeshingEngine_CPU() = default;
		~MeshingEngine_CPU() = default;
	};
}
