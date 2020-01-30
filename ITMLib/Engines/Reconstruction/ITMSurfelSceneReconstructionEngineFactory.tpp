// InfiniTAM: Surffuse. Copyright (c) Torr Vision Group and the authors of InfiniTAM, 2016.

#include "ITMSurfelSceneReconstructionEngineFactory.h"

#include "CPU/SurfelSceneReconstructionEngine_CPU.h"

#ifndef COMPILE_WITHOUT_CUDA
#include "CUDA/SurfelSceneReconstructionEngine_CUDA.h"
#endif

namespace ITMLib {

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

template<typename TSurfel>
SurfelSceneReconstructionEngine<TSurfel>*
ITMSurfelSceneReconstructionEngineFactory<TSurfel>::make_surfel_scene_reconstruction_engine(
		const Vector2i& depthImageSize, MemoryDeviceType deviceType) {
	SurfelSceneReconstructionEngine<TSurfel>* reconstruction_engine = NULL;

	if (deviceType == MEMORYDEVICE_CUDA) {
#ifdef COMPILE_WITHOUT_CUDA
		throw std::runtime_error(
				"Error: CUDA support not currently available. Reconfigure in CMake with the WITH_CUDA option set to on.");
#else
		reconstruction_engine = new SurfelSceneReconstructionEngine_CUDA<TSurfel>(depthImageSize);

#endif
	} else {
		reconstruction_engine = new SurfelSceneReconstructionEngine_CPU<TSurfel>(depthImageSize);
	}

	return reconstruction_engine;
}

}
