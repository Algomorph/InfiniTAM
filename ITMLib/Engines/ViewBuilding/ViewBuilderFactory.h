// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "Interface/ViewBuilder.h"
#include "../../Utils/Configuration.h"

namespace ITMLib
{

/**
 * \brief This struct provides functions that can be used to construct view builders.
 */
struct ViewBuilderFactory
{
  //#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

  /**
   * \brief Makes a view builder.
   *
   * \param calib       The joint RGBD calibration parameters.
   * \param device_type  The device on which the view builder should operate.
   */
  static ViewBuilder *Build(const RGBDCalib& calib, MemoryDeviceType device_type);
  static ViewBuilder *Build(const std::string& calibration_path, MemoryDeviceType device_type);
};

}
