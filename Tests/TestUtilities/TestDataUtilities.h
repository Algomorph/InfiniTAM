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

#include "../../ITMLib/Objects/Volume/PlainVoxelArray.h"
#include "../../ITMLib/Objects/Volume/VoxelBlockHash.h"
#include "TestUtilities.h"
#include "../../../ORUtils/CompileTimeStringConcatenation.h"

using namespace ITMLib;

namespace test {
namespace snoopy {

static const Vector2i frame_image_size = Vector2i(640, 480);

enum Frame {
	FRAME00,
	FRAME16,
	FRAME17
};

// calibration
static constexpr const std::string_view calibration_file_name = "snoopy_calib.txt";
static constexpr const std::string_view calibration_path = ORUtils::join_v<static_calibration_directory, calibration_file_name>;

// frames
static constexpr const std::string_view frame_00_depth_file_name = "snoopy_depth_000000.png";
static constexpr const std::string_view frame_00_depth_path = ORUtils::join_v<static_frames_directory, frame_00_depth_file_name>;
static constexpr const std::string_view frame_00_color_file_name = "snoopy_color_000000.png";
static constexpr const std::string_view frame_00_color_path = ORUtils::join_v<static_frames_directory, frame_00_color_file_name>;
static constexpr const std::string_view frame_00_mask_file_name = "snoopy_omask_000000.png";
static constexpr const std::string_view frame_00_mask_path = ORUtils::join_v<static_frames_directory, frame_00_mask_file_name>;

static constexpr const std::string_view frame_16_depth_file_name = "snoopy_depth_000016.png";
static constexpr const std::string_view frame_16_depth_path = ORUtils::join_v<static_frames_directory, frame_16_depth_file_name>;
static constexpr const std::string_view frame_16_color_file_name = "snoopy_color_000016.png";
static constexpr const std::string_view frame_16_color_path = ORUtils::join_v<static_frames_directory, frame_16_color_file_name>;
static constexpr const std::string_view frame_16_mask_file_name = "snoopy_omask_000016.png";
static constexpr const std::string_view frame_16_mask_path = ORUtils::join_v<static_frames_directory, frame_16_mask_file_name>;

static constexpr const std::string_view frame_17_depth_file_name = "snoopy_depth_000017.png";
static constexpr const std::string_view frame_17_depth_path = ORUtils::join_v<static_frames_directory, frame_17_depth_file_name>;
static constexpr const std::string_view frame_17_color_file_name = "snoopy_color_000017.png";
static constexpr const std::string_view frame_17_color_path = ORUtils::join_v<static_frames_directory, frame_17_color_file_name>;
static constexpr const std::string_view frame_17_mask_file_name = "snoopy_omask_000017.png";
static constexpr const std::string_view frame_17_mask_path = ORUtils::join_v<static_frames_directory, frame_17_mask_file_name>;

static constexpr const std::string_view frame_18_depth_file_name = "snoopy_depth_000018.png";
static constexpr const std::string_view frame_18_depth_path = ORUtils::join_v<static_frames_directory, frame_18_depth_file_name>;
static constexpr const std::string_view frame_18_color_file_name = "snoopy_color_000018.png";
static constexpr const std::string_view frame_18_color_path = ORUtils::join_v<static_frames_directory, frame_18_color_file_name>;
static constexpr const std::string_view frame_18_mask_file_name = "snoopy_omask_000018.png";
static constexpr const std::string_view frame_18_mask_path = ORUtils::join_v<static_frames_directory, frame_18_mask_file_name>;

// videos
static constexpr const std::string_view frames_16_to_18_depth_file_name = "snoopy_depth_16-18.avi";
static constexpr const std::string_view frames_16_to_18_depth_path = ORUtils::join_v<static_videos_directory, frames_16_to_18_depth_file_name>;
static constexpr const std::string_view frames_16_to_18_color_file_name = "snoopy_color_16-18.avi";
static constexpr const std::string_view frames_16_to_18_color_path = ORUtils::join_v<static_videos_directory, frames_16_to_18_color_file_name>;
static constexpr const std::string_view frames_16_to_18_color_YUV422P_file_name = "snoopy_color_16-18_yuv422p.avi";
static constexpr const std::string_view frames_16_to_18_color_YUV422P_path = ORUtils::join_v<static_videos_directory, frames_16_to_18_color_YUV422P_file_name>;
static constexpr const std::string_view frames_16_to_18_depth_GRAY16LE_file_name = "snoopy_depth_16-18_gray16le.avi";
static constexpr const std::string_view frames_16_to_18_depth_GRAY16LE_path = ORUtils::join_v<static_videos_directory, frames_16_to_18_depth_GRAY16LE_file_name>;


template<typename TIndex>
std::string PartialVolume00Path() {
	return std::string(test::generated_volume_directory) + test::IndexString<TIndex>() + "/snoopy_partial_frame_00.dat";
}

extern template std::string PartialVolume00Path<PlainVoxelArray>();
extern template std::string PartialVolume00Path<VoxelBlockHash>();

template<typename TIndex>
std::string PartialVolume16Path() {
	return std::string(test::generated_volume_directory) + test::IndexString<TIndex>() + "/snoopy_partial_frame_16.dat";
}

extern template std::string PartialVolume16Path<PlainVoxelArray>();
extern template std::string PartialVolume16Path<VoxelBlockHash>();

template<typename TIndex>
std::string PartialVolume17Path() {
	return std::string(test::generated_volume_directory) + test::IndexString<TIndex>() + "/snoopy_partial_frame_17.dat";
}

extern template std::string PartialVolume17Path<PlainVoxelArray>();
extern template std::string PartialVolume17Path<VoxelBlockHash>();


template<typename TIndex>
std::string FullVolume16Path() {
	return std::string(test::generated_volume_directory) + test::IndexString<TIndex>() + "/snoopy_full_frame_16.dat";
}

extern template std::string FullVolume16Path<PlainVoxelArray>();
extern template std::string FullVolume16Path<VoxelBlockHash>();

template<typename TIndex>
std::string FullVolume17Path() {
	return std::string(test::generated_volume_directory) + test::IndexString<TIndex>() + "/snoopy_full_frame_17.dat";
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
                      PlainVoxelArray::InitializationParameters initialization_parameters,
                      configuration::SwappingMode swapping_mode);
extern template void
Load<VoxelBlockHash>(VoxelVolume<TSDFVoxel, VoxelBlockHash>** volume, Frame frame,
                     MemoryDeviceType device_type,
                     VoxelBlockHash::InitializationParameters initialization_parameters,
                     configuration::SwappingMode swapping_mode);

VoxelVolumeParameters DefaultVolumeParameters();


} // namespace snoopy

namespace teddy {

static const Vector2i frame_image_size = Vector2i(640, 480);

static constexpr const std::string_view calibration_file_name = "teddy_calib.txt";
static constexpr const std::string_view calibration_path = ORUtils::join_v<static_calibration_directory, calibration_file_name>;

static constexpr const std::string_view frame_115_depth_file_name = "teddy_depth_000115.png";
static constexpr const std::string_view frame_115_depth_path = ORUtils::join_v<static_frames_directory, frame_115_depth_file_name>;
static constexpr const std::string_view frame_115_color_file_name = "teddy_color_000115.png";
static constexpr const std::string_view frame_115_color_path = ORUtils::join_v<static_frames_directory, frame_115_color_file_name>;

static constexpr const std::string_view frame_116_depth_file_name = "teddy_depth_000116.png";
static constexpr const std::string_view frame_116_depth_path = ORUtils::join_v<static_frames_directory, frame_116_depth_file_name>;
static constexpr const std::string_view frame_116_color_file_name = "teddy_color_000116.png";
static constexpr const std::string_view frame_116_color_path = ORUtils::join_v<static_frames_directory, frame_116_color_file_name>;

static constexpr const std::string_view frame_200_depth_file_name = "teddy_depth_000200.png";
static constexpr const std::string_view frame_200_depth_path = ORUtils::join_v<static_frames_directory, frame_200_depth_file_name>;
static constexpr const std::string_view frame_200_color_file_name = "teddy_color_000200.png";
static constexpr const std::string_view frame_200_color_path = ORUtils::join_v<static_frames_directory, frame_200_color_file_name>;

template<typename TIndex>
std::string Volume115Path() {
	return std::string(test::generated_volume_directory) + test::IndexString<TIndex>() + "/teddy_frame_115.dat";
}

extern template std::string Volume115Path<PlainVoxelArray>();
extern template std::string Volume115Path<VoxelBlockHash>();

template<typename TIndex>
typename TIndex::InitializationParameters InitializationParameters();

template<>
typename VoxelBlockHash::InitializationParameters InitializationParameters<VoxelBlockHash>();
extern template VoxelBlockHash::InitializationParameters InitializationParameters<VoxelBlockHash>();
template<>
typename PlainVoxelArray::InitializationParameters InitializationParameters<PlainVoxelArray>();
extern template PlainVoxelArray::InitializationParameters InitializationParameters<PlainVoxelArray>();


VoxelVolumeParameters DefaultVolumeParameters();


} // namespace teddy

} // namespace test


