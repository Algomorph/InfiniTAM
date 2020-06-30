//  ================================================================
//  Created by Gregory Kramida on 12/17/19.
//  Copyright (c) 2019 Gregory Kramida
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
#include "SnoopyTestUtilities.h"
#include "TestUtilities.h"

namespace snoopy_test_utilities {

std::string SnoopyCalibrationPath() {
	return STATIC_TEST_DATA_PREFIX "TestData/calibration/snoopy_calib.txt";
}

std::string Frame00DepthPath() {
	return STATIC_TEST_DATA_PREFIX "TestData/frames/snoopy_depth_000000.png";
}

std::string Frame00ColorPath() {
	return STATIC_TEST_DATA_PREFIX "TestData/frames/snoopy_color_000000.png";
}

std::string Frame00MaskPath() {
	return STATIC_TEST_DATA_PREFIX "TestData/frames/snoopy_omask_000000.png";
}

std::string Frame16DepthPath() {
	return STATIC_TEST_DATA_PREFIX "TestData/frames/snoopy_depth_000016.png";
}

std::string Frame16ColorPath() {
	return STATIC_TEST_DATA_PREFIX "TestData/frames/snoopy_color_000016.png";
}

std::string Frame16MaskPath() {
	return STATIC_TEST_DATA_PREFIX "TestData/frames/snoopy_omask_000016.png";
}

std::string Frame17DepthPath() {
	return STATIC_TEST_DATA_PREFIX "TestData/frames/snoopy_depth_000017.png";
}

std::string Frame17ColorPath() {
	return STATIC_TEST_DATA_PREFIX "TestData/frames/snoopy_color_000017.png";
}

std::string Frame17MaskPath() {
	return STATIC_TEST_DATA_PREFIX "TestData/frames/snoopy_omask_000017.png";
}

template std::string PartialVolume00Path<PlainVoxelArray>();
template std::string PartialVolume00Path<VoxelBlockHash>();
template std::string PartialVolume16Path<PlainVoxelArray>();
template std::string PartialVolume16Path<VoxelBlockHash>();
template std::string PartialVolume17Path<PlainVoxelArray>();
template std::string PartialVolume17Path<VoxelBlockHash>();

template std::string FullVolume16Path<PlainVoxelArray>();
template std::string FullVolume16Path<VoxelBlockHash>();
template std::string FullVolume17Path<PlainVoxelArray>();
template std::string FullVolume17Path<VoxelBlockHash>();

template<>
PlainVoxelArray::InitializationParameters InitializationParameters_Fr00<PlainVoxelArray>() {
	return {Vector3i(512, 112, 360), Vector3i(-512, -24, 152)};
}
template<>
VoxelBlockHash::InitializationParameters InitializationParameters_Fr00<VoxelBlockHash>() {
	return {0x0800, 0x20000};
}

template<>
PlainVoxelArray::InitializationParameters InitializationParameters_Fr16andFr17<PlainVoxelArray>() {
	return {Vector3i(88, 104, 160), Vector3i(-72, -24, 160)};
}
template<>
VoxelBlockHash::InitializationParameters InitializationParameters_Fr16andFr17<VoxelBlockHash>() {
	return {0x0800, 0x20000};
}



template void
Load<PlainVoxelArray>(VoxelVolume<TSDFVoxel, PlainVoxelArray>** volume, Frame frame,
                      MemoryDeviceType device_type,
                      PlainVoxelArray::InitializationParameters initialization_parameters = InitializationParameters_Fr16andFr17<PlainVoxelArray>(),
                      configuration::SwappingMode swapping_mode = configuration::SWAPPINGMODE_DISABLED);

template void
Load<VoxelBlockHash>(VoxelVolume<TSDFVoxel, VoxelBlockHash>** volume, Frame frame,
                     MemoryDeviceType device_type,
                     VoxelBlockHash::InitializationParameters initialization_parameters = InitializationParameters_Fr16andFr17<VoxelBlockHash>(),
                     configuration::SwappingMode swapping_mode = configuration::SWAPPINGMODE_DISABLED);


} // namespace snoopy16and17utilities