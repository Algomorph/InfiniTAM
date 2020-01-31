// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "../../ITMLibDefines.h"
//Note: ".tpp" files have to be included for all explicit instantiations in order to link properly

#include "../../Engines/Main/BasicVoxelEngine.tpp"
#include "../../Engines/Main/BasicSurfelEngine.tpp"
#include "../../Engines/Main/Mappers/DenseMapper.tpp"
#include "../../Engines/Main/Mappers/DenseSurfelMapper.tpp"
#include "../../Engines/Meshing/CPU/ITMMeshingEngine_CPU.tpp"
#include "../../Engines/Meshing/CPU/ITMMultiMeshingEngine_CPU.tpp"
#include "../../Engines/MultiScene/VoxelMapGraphManager.tpp"
#include "../../Engines/Visualization/CPU/MultiVisualizationEngine_CPU.tpp"
#include "../../Engines/Reconstruction/CPU/SceneReconstructionEngine_CPU.tpp"
#include "../../Engines/Reconstruction/Interface/SurfelSceneReconstructionEngine.tpp"
#include "../../Engines/Swapping/CPU/SwappingEngine_CPU.tpp"
#include "../../Engines/Visualization/Interface/SurfelVisualizationEngine.tpp"
#include "../../Engines/Visualization/Interface/VisualizationEngine.h"
#include "../../CameraTrackers/ITMCameraTrackerFactory.h"


namespace ITMLib
{

	template class ITMMeshingEngine_CPU<ITMVoxel, VoxelBlockHash>;
	template class ITMMultiMeshingEngine_CPU<ITMVoxel, VoxelBlockHash>;

}
