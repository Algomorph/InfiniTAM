// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "../../ITMLibDefines.h"

#include "../../Engines/Meshing/CUDA/ITMMeshingEngine_CUDA.tcu"
#include "../../Engines/Meshing/CUDA/ITMMultiMeshingEngine_CUDA.tcu"
#include "../../Engines/Reconstruction/CUDA/SceneReconstructionEngine_CUDA.tcu"
#include "../../Engines/Reconstruction/CUDA/SurfelSceneReconstructionEngine_CUDA.tcu"
#include "../../Engines/Swapping/CUDA/ITMSwappingEngine_CUDA.tcu"
#include "../../Engines/Visualization/CUDA/ITMSurfelVisualizationEngine_CUDA.tcu"
#include "../../Engines/Visualization/CUDA/ITMVisualizationEngine_CUDA.tcu"
#include "../../Engines/Visualization/CUDA/ITMMultiVisualizationEngine_CUDA.tcu"

namespace ITMLib
{
	template class ITMMeshingEngine_CUDA<ITMVoxel, PlainVoxelArray>;
	template class ITMMultiMeshingEngine_CUDA<ITMVoxel, PlainVoxelArray>;
	template class SceneReconstructionEngine_CUDA<ITMVoxel, PlainVoxelArray>;
	template class ITMSwappingEngine_CUDA<ITMVoxel, PlainVoxelArray>;
	template class ITMVisualizationEngine_CUDA<ITMVoxel, PlainVoxelArray>;
	template class ITMMultiVisualizationEngine_CUDA<ITMVoxel, PlainVoxelArray>;

	template class SurfelSceneReconstructionEngine_CUDA<ITMSurfel_grey>;
	template class SurfelSceneReconstructionEngine_CUDA<ITMSurfel_rgb>;
	template class ITMSurfelVisualizationEngine_CUDA<ITMSurfel_grey>;
	template class ITMSurfelVisualizationEngine_CUDA<ITMSurfel_rgb>;

}
