#pragma once

#include "ImageProcessingEngineInterface.h"
#include "../../Utils/Configuration/Configuration.h"
#include "../../../ORUtils/MemoryDeviceType.h"

namespace ITMLib {

/**
 * \brief This struct provides functions that can be used to construct low-level engines.
 */
struct ImageProcessingEngineFactory {

// static functions
	/**
	 * \brief Make an image-processing engine using the provided device type
	 * \param device_type the memory device that the constructed image processor will use
	 * \return a raw pointer reference to the newly-constructed engine
	 */
	static ImageProcessingEngineInterface* Build(MemoryDeviceType device_type);
	/**
	 * \brief Makes a low-level engine.
	 *
	 * \param device_type  The device on which the low-level engine should operate.
	 */
	static ImageProcessingEngineInterface* BuildLegacy(MemoryDeviceType device_type);


};

} // namespace ITMLib
