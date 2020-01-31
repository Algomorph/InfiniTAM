// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "LowLevelEngineFactory.h"

#include "CPU/LowLevelEngine_CPU.h"
#ifndef COMPILE_WITHOUT_CUDA
#include "CUDA/ITMLowLevelEngine_CUDA.h"
#endif

namespace ITMLib
{

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

LowLevelEngine *LowLevelEngineFactory::MakeLowLevelEngine(MemoryDeviceType deviceType)
{
  LowLevelEngine *lowLevelEngine = NULL;

  switch(deviceType)
  {
    case MEMORYDEVICE_CPU:
      lowLevelEngine = new LowLevelEngine_CPU();
      break;
    case MEMORYDEVICE_CUDA:
#ifndef COMPILE_WITHOUT_CUDA
      lowLevelEngine = new ITMLowLevelEngine_CUDA();
#endif
      break;
    case MEMORYDEVICE_METAL:
#ifdef COMPILE_WITH_METAL
      lowLevelEngine = new ITMLowLevelEngine_CPU();
#endif
      break;
  }

  return lowLevelEngine;
}

}
