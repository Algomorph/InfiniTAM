// InfiniTAM: Surffuse. Copyright (c) Torr Vision Group and the authors of InfiniTAM, 2016.

#pragma once

#include "Interface/SurfelSceneReconstructionEngine.h"
#include "CPU/SurfelSceneReconstructionEngine_CPU.h"
#include "../../Utils/Configuration/Configuration.h"

#ifndef COMPILE_WITHOUT_CUDA
#include "CUDA/SurfelSceneReconstructionEngine_CUDA.h"
#endif

namespace ITMLib
{
  /**
   * \brief An instantiation of this struct can be used to construct surfel scene reconstruction engines.
   */
  struct SurfelSceneReconstructionEngineFactory
  {
    //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

    /**
     * \brief Makes a surfel scene reconstruction engine.
     *
     * \param depthImageSize  The size of the depth images that will be fused into the scene.
     * \param deviceType      The device on which the surfel scene reconstruction engine should operate.
     * \return                The surfel scene reconstruction engine.
     */
    template <typename TSurfel>
    static SurfelSceneReconstructionEngine<TSurfel>* Build(const Vector2i& depthImageSize, MemoryDeviceType deviceType){
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
  };
}
