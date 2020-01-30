// InfiniTAM: Surffuse. Copyright (c) Torr Vision Group and the authors of InfiniTAM, 2016.

#pragma once

#include "Interface/ITMSurfelVisualisationEngine.h"
#include "../../Utils/Configuration.h"
#include "CPU/ITMSurfelVisualizationEngine_CPU.h"
#ifndef COMPILE_WITHOUT_CUDA
#include "CUDA/ITMSurfelVisualizationEngine_CUDA.h"
#endif

namespace ITMLib
{
  /**
   * \brief An instantiation of this struct can be used to construct surfel visualisation engines.
   */
  struct ITMSurfelVisualizationEngineFactory
  {
    //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

    /**
     * \brief Makes a surfel visualisation engine.
     *
     * \param deviceType  The device on which the surfel visualisation engine should operate.
     * \return            The surfel visualisation engine.
     */
    template <typename TSurfel>
    static ITMSurfelVisualisationEngine<TSurfel> *Build(MemoryDeviceType deviceType){
	    ITMSurfelVisualisationEngine<TSurfel> *visualisationEngine = nullptr;

	    if(deviceType == MEMORYDEVICE_CUDA)
	    {
#ifndef COMPILE_WITHOUT_CUDA
		    visualisationEngine = new ITMSurfelVisualizationEngine_CUDA<TSurfel>;
#else
		    throw std::runtime_error("Error: CUDA support not currently available. Reconfigure in CMake with the WITH_CUDA option set to on.");
#endif
	    }
	    else
	    {
		    visualisationEngine = new ITMSurfelVisualizationEngine_CPU<TSurfel>;
	    }

	    return visualisationEngine;
    }
  };
}
