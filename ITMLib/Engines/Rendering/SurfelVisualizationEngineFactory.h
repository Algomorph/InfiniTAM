// InfiniTAM: Surffuse. Copyright (c) Torr Vision Group and the authors of InfiniTAM, 2016.

#pragma once

#include "Interface/SurfelVisualizationEngine.h"
#include "../../Utils/Configuration/Configuration.h"
#include "CPU/SurfelVisualizationEngine_CPU.h"
#ifndef COMPILE_WITHOUT_CUDA
#include "CUDA/SurfelVisualizationEngine_CUDA.h"
#endif

namespace ITMLib
{
  /**
   * \brief An instantiation of this struct can be used to construct surfel Rendering engines.
   */
  struct SurfelVisualizationEngineFactory
  {
    //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

    /**
     * \brief Makes a surfel Rendering engine.
     *
     * \param deviceType  The device on which the surfel Rendering engine should operate.
     * \return            The surfel Rendering engine.
     */
    template <typename TSurfel>
    static SurfelVisualizationEngine<TSurfel> *Build(MemoryDeviceType deviceType){
	    SurfelVisualizationEngine<TSurfel> *visualization_engine = nullptr;

	    if(deviceType == MEMORYDEVICE_CUDA)
	    {
#ifndef COMPILE_WITHOUT_CUDA
		    visualization_engine = new SurfelVisualizationEngine_CUDA<TSurfel>;
#else
		    throw std::runtime_error("Error: CUDA support not currently available. Reconfigure in CMake with the WITH_CUDA option set to on.");
#endif
	    }
	    else
	    {
		    visualization_engine = new SurfelVisualizationEngine_CPU<TSurfel>;
	    }

	    return visualization_engine;
    }
  };
}
