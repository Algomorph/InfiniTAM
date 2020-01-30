// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM


#include "../../ITMLibDefines.h"

#include "../../Engines/Meshing/CUDA/ITMMeshingEngine_CUDA.tcu"
#include "../../Engines/Meshing/CUDA/ITMMultiMeshingEngine_CUDA.tcu"
#include "../../Engines/Reconstruction/CUDA/SceneReconstructionEngine_CUDA.tcu"
#include "../../Engines/Swapping/CUDA/SwappingEngine_CUDA.tcu"

#include "../../Engines/Visualization/CUDA/VisualizationEngine_CUDA.tcu"
#include "../../Engines/Visualization/CUDA/MultiVisualizationEngine_CUDA.tcu"

namespace ITMLib
{
	template class ITMMeshingEngine_CUDA<ITMVoxel, VoxelBlockHash>;
	template class ITMMultiMeshingEngine_CUDA<ITMVoxel, VoxelBlockHash>;

}
