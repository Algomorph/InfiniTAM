// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM


#include "../../ITMLibDefines.h"

#include "../../Engines/Meshing/CUDA/MeshingEngine_CUDA.tcu"
#include "../../Engines/Meshing/CUDA/MultiMeshingEngine_CUDA.tcu"
#include "../../Engines/Reconstruction/CUDA/SceneReconstructionEngine_CUDA.tcu"
#include "../../Engines/Swapping/CUDA/SwappingEngine_CUDA.tcu"

#include "../../Engines/Visualization/CUDA/VisualizationEngine_CUDA.tcu"
#include "../../Engines/Visualization/CUDA/MultiVisualizationEngine_CUDA.tcu"

namespace ITMLib
{
	template class MeshingEngine_CUDA<ITMVoxel, VoxelBlockHash>;
	template class MultiMeshingEngine_CUDA<ITMVoxel, VoxelBlockHash>;

}
