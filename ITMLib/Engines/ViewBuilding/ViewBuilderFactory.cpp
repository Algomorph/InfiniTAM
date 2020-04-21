// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "ViewBuilderFactory.h"

#include "CPU/ViewBuilder_CPU.h"

#ifndef COMPILE_WITHOUT_CUDA

#include "CUDA/ViewBuilder_CUDA.h"

#endif

namespace ITMLib {

//#################### PUBLIC STATIC MEMBER FUNCTIONS ####################

ViewBuilder* ViewBuilderFactory::Build(const RGBDCalib& calib, MemoryDeviceType device_type) {
	ViewBuilder* view_builder = nullptr;

	switch (device_type) {
		case MEMORYDEVICE_CPU:
			view_builder = new ViewBuilder_CPU(calib);
			break;
		case MEMORYDEVICE_CUDA:
#ifndef COMPILE_WITHOUT_CUDA
			view_builder = new ViewBuilder_CUDA(calib);
#endif
			break;
		case MEMORYDEVICE_METAL:
#ifdef COMPILE_WITH_METAL
			view_builder = new ViewBuilder_CPU(calib);
#endif
			break;
	}

	return view_builder;
}

ViewBuilder* ViewBuilderFactory::Build(const std::string& calibration_path, MemoryDeviceType device_type) {
	ViewBuilder* view_builder = nullptr;

	RGBDCalib calibrationData;
	readRGBDCalib(calibration_path.c_str(), calibrationData);

	switch (device_type) {
		case MEMORYDEVICE_CPU:
			view_builder = new ViewBuilder_CPU(calibrationData);
			break;
		case MEMORYDEVICE_CUDA:
#ifndef COMPILE_WITHOUT_CUDA
			view_builder = new ViewBuilder_CUDA(calibrationData);
#endif
			break;
		case MEMORYDEVICE_METAL:
#ifdef COMPILE_WITH_METAL
			view_builder = new ViewBuilder_CPU(calibrationData);
#endif
			break;
	}

	return view_builder;
}

} // namespace ITMLib
