#include "ImageProcessingEngineFactory.h"
#include "ImageProcessingEngine.h"

#include "CPU/LowLevelEngine_CPU.h"
#ifndef COMPILE_WITHOUT_CUDA
#include "CUDA/LowLevelEngine_CUDA.h"
#endif

namespace ITMLib {


ImageProcessingEngineInterface* ImageProcessingEngineFactory::Build(MemoryDeviceType device_type) {
	ImageProcessingEngineInterface* preprocessing_engine = nullptr;

	switch (device_type) {
		case MEMORYDEVICE_CPU:
			preprocessing_engine = new ImageProcessingEngine<MEMORYDEVICE_CPU>();
			break;
		case MEMORYDEVICE_CUDA:
#ifdef COMPILE_WITHOUT_CUDA
			DIEWITHEXCEPTION_REPORTLOCATION("Trying to instantiate a CUDA-based ImageProcessingEngine while code not compiled with CUDA support.");
#else
			preprocessing_engine = new ImageProcessingEngine<MEMORYDEVICE_CUDA>();
#endif
			break;
		case MEMORYDEVICE_METAL:
#ifdef COMPILE_WITH_METAL
			preprocessing_engine = new ImageProcessingEngine<MEMORYDEVICE_CPU>();
#else
			DIEWITHEXCEPTION_REPORTLOCATION("Trying to instantiate a Metal-based ImageProcessingEngine while code not compiled with Metal support.");
#endif
			break;
		default:
			DIEWITHEXCEPTION_REPORTLOCATION("Unsupported device type.");
			break;
	}

	return preprocessing_engine;
}

ImageProcessingEngineInterface* ImageProcessingEngineFactory::BuildLegacy(MemoryDeviceType device_type) {
	ImageProcessingEngineInterface* preprocessing_engine = nullptr;

	switch (device_type) {
		case MEMORYDEVICE_CPU:
			preprocessing_engine = new LowLevelEngine_CPU();
			break;
		case MEMORYDEVICE_CUDA:
#ifdef COMPILE_WITHOUT_CUDA
			DIEWITHEXCEPTION_REPORTLOCATION("Trying to instantiate a CUDA-based ImageProcessingEngine while code not compiled with CUDA support.");
#else
			preprocessing_engine = new LowLevelEngine_CUDA();
#endif
			break;
		case MEMORYDEVICE_METAL:
#ifdef COMPILE_WITH_METAL
			preprocessing_engine = new LowLevelEngine_CPU();
#else
			DIEWITHEXCEPTION_REPORTLOCATION("Trying to instantiate a Metal-based ImageProcessingEngine while code not compiled with Metal support.");
#endif
			break;
		default:
			DIEWITHEXCEPTION_REPORTLOCATION("Unsupported device type.");
			break;
	}

	return preprocessing_engine;
}

} // namespace ITMLib
