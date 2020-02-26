// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "CPU/SwappingEngine_CPU.h"
#ifndef COMPILE_WITHOUT_CUDA
#include "CUDA/SwappingEngine_CUDA.h"
#endif
#ifdef COMPILE_WITH_METAL
#include "Metal/SwappingEngine_Metal.h"
#endif

namespace ITMLib
{

/**
 * \brief This struct provides functions that can be used to construct swapping engines.
 */
struct SwappingEngineFactory
{
  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

  /**
   * \brief Makes a swapping engine.
   *
   * \param deviceType  The device on which the swapping engine should operate.
   */
  template <typename TVoxel, typename TIndex>
  static SwappingEngine<TVoxel,TIndex> *Build(MemoryDeviceType deviceType, const TIndex& index)
  {
    SwappingEngine<TVoxel,TIndex> *swapping_engine = NULL;

    switch(deviceType)
    {
      case MEMORYDEVICE_CPU:
	      swapping_engine = new SwappingEngine_CPU<TVoxel,TIndex>(index);
        break;
      case MEMORYDEVICE_CUDA:
#ifndef COMPILE_WITHOUT_CUDA
        swapping_engine = new SwappingEngine_CUDA<TVoxel,TIndex>(index);
#endif
        break;
      case MEMORYDEVICE_METAL:
#ifdef COMPILE_WITH_METAL
        swapping_engine = new SwappingEngine_CPU<TVoxelCanonical,TIndex>;
#endif
        break;
    }

    return swapping_engine;
  }
};

}
