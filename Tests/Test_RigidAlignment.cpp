//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 11/27/20.
//  Copyright (c) 2020 Gregory Kramida
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at

//  http://www.apache.org/licenses/LICENSE-2.0

//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
//  ================================================================
#define BOOST_TEST_MODULE RigidAlignment

#include "TestUtilities/GenericTestModuleHeader.h"
#include "../ITMLib/CameraTrackers/CameraTrackerFactory.h"
#include "../ITMLib/Engines/ImageProcessing/ImageProcessingEngineFactory.h"
#include "../ITMLib/Objects/Misc/IMUCalibrator.h"

template<MemoryDeviceType TMemoryDeviceType>
void GenericRgbTrackerTest() {
	constexpr const char* preset_rrbb = "type=rgb,levels=rrbb";
	constexpr const char* preset_rrrbb = "type=rgb,levels=rrrbb";
	constexpr const char* preset_rrbrb = "type=rgb,levels=rrbrb";

	ImageProcessingEngineInterface* image_processing_engine = ImageProcessingEngineFactory::Build(TMemoryDeviceType);

	IMUCalibrator* imu_calibrator = new ITMIMUCalibrator_iPad();

	CameraTracker* tracker = CameraTrackerFactory::Instance().Make(TMemoryDeviceType, preset_rrbb, teddy::frame_image_size, teddy::frame_image_size,
	                                                               image_processing_engine, imu_calibrator,
	                                                               configuration::Get().general_voxel_volume_parameters);

	View* view;
	UpdateView(&view,
	           std::string(teddy::frame_115_depth_path),
	           std::string(teddy::frame_115_color_path),
	           std::string(teddy::calibration_path),
	           TMemoryDeviceType);



	delete image_processing_engine;
	delete imu_calibrator;
}

template<MemoryDeviceType TMemoryDeviceType>
void GenericIcpTrackerTest() {

}

template<MemoryDeviceType TMemoryDeviceType>
void GenericExtendedTrackerTest() {

}

template<MemoryDeviceType TMemoryDeviceType>
void GenericFileTrackerTest() {

}

template<MemoryDeviceType TMemoryDeviceType>
void GenericImuIcpTrackerTest() {

}

template<MemoryDeviceType TMemoryDeviceType>
void GenericImuExtendedTrackerTest() {

}

template<MemoryDeviceType TMemoryDeviceType>
void GenericForceFailTrackerTest() {

}

BOOST_AUTO_TEST_CASE(Test_RgbTracker_CPU) {
	GenericRgbTrackerTest<MEMORYDEVICE_CPU>();
}