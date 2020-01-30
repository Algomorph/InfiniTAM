#include "../../ITMLibDefines.h"

#include "../../Engines/Visualization/CUDA/SurfelVisualizationEngine_CUDA.tcu"
#include "../../Engines/Reconstruction/CUDA/SurfelSceneReconstructionEngine_CUDA.tcu"

namespace ITMLib
{
	template class SurfelSceneReconstructionEngine_CUDA<ITMSurfel_grey>;
	template class SurfelSceneReconstructionEngine_CUDA<ITMSurfel_rgb>;
	template class SurfelVisualizationEngine_CUDA<ITMSurfel_grey>;
	template class SurfelVisualizationEngine_CUDA<ITMSurfel_rgb>;

}