// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "ITMMeshingEngine.h"
#include "../../../Objects/Meshing/ITMMesh.h"
#include "../../MultiScene/VoxelMapGraphManager.h"

namespace ITMLib
{
	template<class TVoxel, class TIndex>
	class ITMMultiMeshingEngine
	{
	public:
		virtual ~ITMMultiMeshingEngine(void) {}

		virtual void MeshScene(ITMMesh *mesh, const VoxelMapGraphManager<TVoxel, TIndex> & sceneManager) = 0;
	};
}