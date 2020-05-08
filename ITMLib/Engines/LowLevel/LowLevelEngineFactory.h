// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "Interface/LowLevelEngine.h"
#include "../../Utils/Configuration/Configuration.h"
#include "../../../ORUtils/MemoryDeviceType.h"

namespace ITMLib
{

/**
 * \brief This struct provides functions that can be used to construct low-level engines.
 */
struct LowLevelEngineFactory
{
  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

  /**
   * \brief Makes a low-level engine.
   *
   * \param deviceType  The device on which the low-level engine should operate.
   */
  static LowLevelEngine *MakeLowLevelEngine(MemoryDeviceType deviceType);
};

}
