//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 4/28/20.
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
#pragma once

//stdlib
#include <string>

namespace ITMLib {

namespace telemetry {
// region ============================== DEFINE CONSTANTS ==============================================================

// where to save the data within the output directory
inline const std::string warp_iteration_2D_slices_folder_name = "bucket_interest_region_2D_iteration_slices";
inline const std::string live_iteration_2D_slices_folder_name = "bucket_interest_region_live_slices";
inline const std::string canonical_scene_rasterized_folder_name = "canonical_rasterized";
inline const std::string live_scene_rasterized_folder_name = "live_rasterized";
inline const std::string frame_folder_prefix = "/Frame_";

// endregion ================================== END CONSTANT DEFINITIONS ===============================================

std::string GetAndCreateOutputFolderForFrame(int frame_index);
} // namespace telemetry
} // namespace ITMLib