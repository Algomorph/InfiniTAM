#pragma once

#include "Interface/ImageProcessingEngineInterface.h"
#include "../../Utils/Configuration/Configuration.h"
#include "../../../ORUtils/MemoryDeviceType.h"

namespace ITMLib{

/**
 * \brief This struct provides functions that can be used to construct low-level engines.
 */
struct ImageProcessingEngineFactory
{
  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

  /**
   * \brief Makes a low-level engine.
   *
   * \param device_type  The device on which the low-level engine should operate.
   */
  static ImageProcessingEngineInterface* Build(MemoryDeviceType device_type);
};

} // namespace ITMLib
