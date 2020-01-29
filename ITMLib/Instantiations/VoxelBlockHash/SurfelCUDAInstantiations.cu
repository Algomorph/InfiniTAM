#include "../../ITMLibDefines.h"

#include "../../Engines/Visualization/CUDA/ITMSurfelVisualizationEngine_CUDA.tcu"
#include "../../Engines/Reconstruction/CUDA/SurfelSceneReconstructionEngine_CUDA.tcu"

namespace ITMLib
{
	template class SurfelSceneReconstructionEngine_CUDA<ITMSurfel_grey>;
	template class SurfelSceneReconstructionEngine_CUDA<ITMSurfel_rgb>;
	template class ITMSurfelVisualizationEngine_CUDA<ITMSurfel_grey>;
	template class ITMSurfelVisualizationEngine_CUDA<ITMSurfel_rgb>;

}