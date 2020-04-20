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

namespace snoopy16and17utilities {

enum Frame {
	FRAME16,
	FRAME17
};

std::string Frame16DepthPath();
std::string Frame16ColorPath();
std::string Frame16MaskPath();

std::string Frame17DepthPath();
std::string Frame17ColorPath();
std::string Frame17MaskPath();

template<typename TIndex>
std::string PartialVolume16Path() {
	return "TestData/" + test_utilities::IndexString<TIndex>() + "/snoopy_partial_frame_16_";
}
extern template std::string PartialVolume16Path<PlainVoxelArray>();
extern template std::string PartialVolume16Path<VoxelBlockHash>();

template<typename TIndex>
std::string PartialVolume17Path() {
	return "TestData/" + test_utilities::IndexString<TIndex>() + "/snoopy_partial_frame_17_";
}
extern template std::string PartialVolume17Path<PlainVoxelArray>();
extern template std::string PartialVolume17Path<VoxelBlockHash>();

template<typename TIndex>
typename TIndex::InitializationParameters InitializationParameters();
template<>
PlainVoxelArray::InitializationParameters InitializationParameters<PlainVoxelArray>();
template<>
VoxelBlockHash::InitializationParameters InitializationParameters<VoxelBlockHash>();
extern template PlainVoxelArray::InitializationParameters InitializationParameters<PlainVoxelArray>();
extern template VoxelBlockHash::InitializationParameters InitializationParameters<VoxelBlockHash>();


template<typename TIndex>
void Load(VoxelVolume<TSDFVoxel, TIndex>** volume, Frame frame, MemoryDeviceType device_type,
          typename TIndex::InitializationParameters initialization_parameters = InitializationParameters<TIndex>(),
          configuration::SwappingMode swapping_mode = configuration::SWAPPINGMODE_DISABLED) {
	std::string path;
	switch (frame) {
		case FRAME16:
			path = PartialVolume16Path<TIndex>();
		case FRAME17:
			path = PartialVolume17Path<TIndex>();
	}
}
extern template void
Load<PlainVoxelArray>(VoxelVolume<TSDFVoxel, PlainVoxelArray>** volume, Frame frame,
                                             MemoryDeviceType device_type,
                                             PlainVoxelArray::InitializationParameters initialization_parameters = InitializationParameters<PlainVoxelArray>(),
                                             configuration::SwappingMode swapping_mode = configuration::SWAPPINGMODE_DISABLED);
extern template void
Load<VoxelBlockHash>(VoxelVolume<TSDFVoxel, VoxelBlockHash>** volume, Frame frame,
                                            MemoryDeviceType device_type,
                                            VoxelBlockHash::InitializationParameters initialization_parameters = InitializationParameters<VoxelBlockHash>(),
                                            configuration::SwappingMode swapping_mode = configuration::SWAPPINGMODE_DISABLED);

} // namespace snoopy16and17utilities


