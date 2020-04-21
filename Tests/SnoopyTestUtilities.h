//  ================================================================
//  Created by Gregory Kramida on 12/1/19.
//  Copyright (c)  2019 Gregory Kramida
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

#pragma once

#include "../ITMLib/Objects/Volume/PlainVoxelArray.h"
#include "../ITMLib/Objects/Volume/VoxelBlockHash.h"
#include "TestUtilities.h"

using namespace ITMLib;

namespace snoopy_test_utilities {

enum Frame {
	FRAME00,
	FRAME16,
	FRAME17
};

std::string SnoopyCalibrationPath();

std::string Frame00DepthPath();
std::string Frame00ColorPath();
std::string Frame00MaskPath();

std::string Frame16DepthPath();
std::string Frame16ColorPath();
std::string Frame16MaskPath();

std::string Frame17DepthPath();
std::string Frame17ColorPath();
std::string Frame17MaskPath();

template<typename TIndex>
std::string PartialVolume00Path() {
	return "TestData/volumes/" + test_utilities::IndexString<TIndex>() + "/snoopy_partial_frame_00.dat";
}
extern template std::string PartialVolume00Path<PlainVoxelArray>();
extern template std::string PartialVolume00Path<VoxelBlockHash>();

template<typename TIndex>
std::string PartialVolume16Path() {
	return "TestData/volumes/" + test_utilities::IndexString<TIndex>() + "/snoopy_partial_frame_16.dat";
}
extern template std::string PartialVolume16Path<PlainVoxelArray>();
extern template std::string PartialVolume16Path<VoxelBlockHash>();

template<typename TIndex>
std::string PartialVolume17Path() {
	return "TestData/volumes/" + test_utilities::IndexString<TIndex>() + "/snoopy_partial_frame_17.dat";
}
extern template std::string PartialVolume17Path<PlainVoxelArray>();
extern template std::string PartialVolume17Path<VoxelBlockHash>();



template<typename TIndex>
std::string FullVolume16Path() {
	return "TestData/volumes/" + test_utilities::IndexString<TIndex>() + "/snoopy_full_frame_16.dat";
}
extern template std::string FullVolume16Path<PlainVoxelArray>();
extern template std::string FullVolume16Path<VoxelBlockHash>();

template<typename TIndex>
std::string FullVolume17Path() {
	return "TestData/volumes/" + test_utilities::IndexString<TIndex>() + "/snoopy_full_frame_17.dat";
}
extern template std::string FullVolume17Path<PlainVoxelArray>();
extern template std::string FullVolume17Path<VoxelBlockHash>();

template<typename TIndex>
typename TIndex::InitializationParameters InitializationParameters_Fr00();
template<>
PlainVoxelArray::InitializationParameters InitializationParameters_Fr00<PlainVoxelArray>();
template<>
VoxelBlockHash::InitializationParameters InitializationParameters_Fr00<VoxelBlockHash>();
extern template PlainVoxelArray::InitializationParameters InitializationParameters_Fr00<PlainVoxelArray>();
extern template VoxelBlockHash::InitializationParameters InitializationParameters_Fr00<VoxelBlockHash>();

template<typename TIndex>
typename TIndex::InitializationParameters InitializationParameters_Fr16andFr17();
template<>
PlainVoxelArray::InitializationParameters InitializationParameters_Fr16andFr17<PlainVoxelArray>();
template<>
VoxelBlockHash::InitializationParameters InitializationParameters_Fr16andFr17<VoxelBlockHash>();
extern template PlainVoxelArray::InitializationParameters InitializationParameters_Fr16andFr17<PlainVoxelArray>();
extern template VoxelBlockHash::InitializationParameters InitializationParameters_Fr16andFr17<VoxelBlockHash>();


template<typename TIndex>
void Load(VoxelVolume<TSDFVoxel, TIndex>** volume, Frame frame, MemoryDeviceType device_type,
          typename TIndex::InitializationParameters initialization_parameters = InitializationParameters_Fr16andFr17<TIndex>(),
          configuration::SwappingMode swapping_mode = configuration::SWAPPINGMODE_DISABLED) {
	std::string path;
	switch (frame) {
		case FRAME00:
			path = PartialVolume00Path<TIndex>();
		case FRAME16:
			path = PartialVolume16Path<TIndex>();
		case FRAME17:
			path = PartialVolume17Path<TIndex>();
	}
}
extern template void
Load<PlainVoxelArray>(VoxelVolume<TSDFVoxel, PlainVoxelArray>** volume, Frame frame,
                                             MemoryDeviceType device_type,
                                             PlainVoxelArray::InitializationParameters initialization_parameters = InitializationParameters_Fr16andFr17<PlainVoxelArray>(),
                                             configuration::SwappingMode swapping_mode = configuration::SWAPPINGMODE_DISABLED);
extern template void
Load<VoxelBlockHash>(VoxelVolume<TSDFVoxel, VoxelBlockHash>** volume, Frame frame,
                                            MemoryDeviceType device_type,
                                            VoxelBlockHash::InitializationParameters initialization_parameters = InitializationParameters_Fr16andFr17<VoxelBlockHash>(),
                                            configuration::SwappingMode swapping_mode = configuration::SWAPPINGMODE_DISABLED);

} // namespace snoopy16and17utilities


