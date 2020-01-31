// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "ViewBuilderFactory.h"

#include "CPU/ViewBuilder_CPU.h"
#ifndef COMPILE_WITHOUT_CUDA
#include "CUDA/ViewBuilder_CUDA.h"
#endif

namespace ITMLib
{

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

ViewBuilder *ViewBuilderFactory::MakeViewBuilder(const RGBDCalib& calib, MemoryDeviceType deviceType)
{
  ViewBuilder *viewBuilder = nullptr;

  switch(deviceType)
  {
    case MEMORYDEVICE_CPU:
      viewBuilder = new ViewBuilder_CPU(calib);
      break;
    case MEMORYDEVICE_CUDA:
#ifndef COMPILE_WITHOUT_CUDA
      viewBuilder = new ViewBuilder_CUDA(calib);
#endif
      break;
    case MEMORYDEVICE_METAL:
#ifdef COMPILE_WITH_METAL
      viewBuilder = new ViewBuilder_CPU(calib);
#endif
      break;
  }

  return viewBuilder;
}

ViewBuilder *ViewBuilderFactory::MakeViewBuilder(const std::string& calibration_path, MemoryDeviceType deviceType)
{
	ViewBuilder *viewBuilder = nullptr;

	RGBDCalib calibrationData;
	readRGBDCalib(calibration_path.c_str(), calibrationData);

	switch(deviceType)
	{
		case MEMORYDEVICE_CPU:
			viewBuilder = new ViewBuilder_CPU(calibrationData);
			break;
		case MEMORYDEVICE_CUDA:
#ifndef COMPILE_WITHOUT_CUDA
			viewBuilder = new ViewBuilder_CUDA(calibrationData);
#endif
			break;
		case MEMORYDEVICE_METAL:
#ifdef COMPILE_WITH_METAL
			viewBuilder = new ViewBuilder_CPU(calibrationData);
#endif
			break;
	}

	return viewBuilder;
}

}
