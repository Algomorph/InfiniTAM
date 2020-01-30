// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "../../ITMLibDefines.h"
//Note: ".tpp" files have to be included for all explicit instantiations in order to link properly

#include "../../Engines/Main/ITMBasicEngine.tpp"
#include "../../Engines/Main/ITMBasicSurfelEngine.tpp"
#include "../../Engines/Main/ITMMultiEngine.tpp"
#include "../../Engines/Main/ITMDenseMapper.tpp"
#include "../../Engines/Main/ITMDenseSurfelMapper.tpp"
#include "../../Engines/Meshing/CPU/ITMMeshingEngine_CPU.tpp"
#include "../../Engines/Meshing/CPU/ITMMultiMeshingEngine_CPU.tpp"
#include "../../Engines/MultiScene/ITMMapGraphManager.tpp"
#include "../../Engines/Visualization/CPU/ITMMultiVisualizationEngine_CPU.tpp"
#include "../../Engines/Reconstruction/CPU/SceneReconstructionEngine_CPU.tpp"
#include "../../Engines/Reconstruction/CPU/SurfelSceneReconstructionEngine_CPU.tpp"
#include "../../Engines/Reconstruction/Interface/SurfelSceneReconstructionEngine.tpp"
#include "../../Engines/Swapping/CPU/ITMSwappingEngine_CPU.tpp"
#include "../../Engines/Visualization/CPU/ITMSurfelVisualisationEngine_CPU.tpp"
#include "../../Engines/Visualization/CPU/ITMVisualisationEngine_CPU.tpp"
#include "../../Engines/Visualization/Interface/ITMSurfelVisualisationEngine.tpp"
#include "../../Engines/Visualization/Interface/ITMVisualisationEngine.h"
#include "../../CameraTrackers/ITMCameraTrackerFactory.h"


namespace ITMLib
{
	//voxel fusion
	template class ITMBasicEngine<ITMVoxel, VoxelBlockHash>;
	template class ITMBasicSurfelEngine<ITMSurfel_grey>;
	template class ITMBasicSurfelEngine<ITMSurfel_rgb>;
	template class ITMMultiEngine<ITMVoxel, VoxelBlockHash>;
	template class ITMDenseMapper<ITMVoxel, VoxelBlockHash>;
	template class ITMVoxelMapGraphManager<ITMVoxel, VoxelBlockHash>;
	template class ITMVisualizationEngine_CPU<ITMVoxel, VoxelBlockHash>;
	template class ITMMeshingEngine_CPU<ITMVoxel, VoxelBlockHash>;
	template class ITMMultiMeshingEngine_CPU<ITMVoxel, VoxelBlockHash>;
	template class ITMSwappingEngine_CPU<ITMVoxel, VoxelBlockHash>;

	//surfel fusion
	template class ITMDenseSurfelMapper<ITMSurfel_grey>;
	template class ITMDenseSurfelMapper<ITMSurfel_rgb>;
	template class ITMSurfelVisualisationEngine<ITMSurfel_grey>;
	template class ITMSurfelVisualisationEngine<ITMSurfel_rgb>;
	template class ITMSurfelVisualizationEngine_CPU<ITMSurfel_grey>;
	template class ITMSurfelVisualizationEngine_CPU<ITMSurfel_rgb>;
}
