#pragma once

#include "Interface/PreprocessingEngineInterface.h"
#include "../../Utils/Configuration/Configuration.h"
#include "../../../ORUtils/MemoryDeviceType.h"

namespace ITMLib{

/**
 * \brief This struct provides functions that can be used to construct low-level engines.
 */
struct PreprocessingEngineFactory
{
  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

  /**
   * \brief Makes a low-level engine.
   *
   * \param device_type  The device on which the low-level engine should operate.
   */
  static PreprocessingEngineInterface *Build(MemoryDeviceType device_type);
};

} // namespace ITMLib
