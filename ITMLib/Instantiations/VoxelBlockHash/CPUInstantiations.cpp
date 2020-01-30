// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "../../ITMLibDefines.h"
//Note: ".tpp" files have to be included for all explicit instantiations in order to link properly

#include "../../Engines/Main/BasicEngine.tpp"
#include "../../Engines/Main/BasicSurfelEngine.tpp"
#include "../../Engines/Main/MultiEngine.tpp"
#include "../../Engines/Main/DenseMapper.tpp"
#include "../../Engines/Main/DenseSurfelMapper.tpp"
#include "../../Engines/Meshing/CPU/ITMMeshingEngine_CPU.tpp"
#include "../../Engines/Meshing/CPU/ITMMultiMeshingEngine_CPU.tpp"
#include "../../Engines/MultiScene/ITMMapGraphManager.tpp"
#include "../../Engines/Visualization/CPU/MultiVisualizationEngine_CPU.tpp"
#include "../../Engines/Reconstruction/CPU/SceneReconstructionEngine_CPU.tpp"
#include "../../Engines/Reconstruction/CPU/SurfelSceneReconstructionEngine_CPU.tpp"
#include "../../Engines/Reconstruction/Interface/SurfelSceneReconstructionEngine.tpp"
#include "../../Engines/Swapping/CPU/ITMSwappingEngine_CPU.tpp"
#include "../../Engines/Visualization/CPU/SurfelVisualizationEngine_CPU.tpp"
#include "../../Engines/Visualization/CPU/VisualizationEngine_CPU.tpp"
#include "../../Engines/Visualization/Interface/SurfelVisualizationEngine.tpp"
#include "../../Engines/Visualization/Interface/VisualizationEngine.h"
#include "../../CameraTrackers/ITMCameraTrackerFactory.h"


namespace ITMLib
{
	//voxel fusion
	template class BasicEngine<ITMVoxel, VoxelBlockHash>;
	template class BasicSurfelEngine<ITMSurfel_grey>;
	template class BasicSurfelEngine<ITMSurfel_rgb>;
	template class MultiEngine<ITMVoxel, VoxelBlockHash>;
	template class DenseMapper<ITMVoxel, VoxelBlockHash>;
	template class ITMVoxelMapGraphManager<ITMVoxel, VoxelBlockHash>;

	template class ITMMeshingEngine_CPU<ITMVoxel, VoxelBlockHash>;
	template class ITMMultiMeshingEngine_CPU<ITMVoxel, VoxelBlockHash>;
	template class ITMSwappingEngine_CPU<ITMVoxel, VoxelBlockHash>;

	//surfel fusion
	template class DenseSurfelMapper<ITMSurfel_grey>;
	template class DenseSurfelMapper<ITMSurfel_rgb>;

}
