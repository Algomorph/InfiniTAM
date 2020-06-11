//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 6/9/20.
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
//local
#include "../Metacoding/DeferrableSerializableStruct.h"

namespace ITMLib {

#define AUTOMATIC_RUN_SETTINGS_STRUCT_DESCRIPTION AutomaticRunSettings, "automatic_run_settings", \
    (int, index_of_frame_to_end_before, 0, PRIMITIVE, "This number of frames will be processed automatically after the program is launched (launches automatic run)."), \
    (int, index_of_frame_to_start_at, 0, PRIMITIVE, "Index of the first frame (or frame set) to read from disk (or, how many frames to skip). The remaining frames will be read in order."), \
    (bool, load_volume_and_camera_matrix_before_processing, false, PRIMITIVE, "When this is set to true, the program will attempt to load the volume for the index_of_frame_to_start_with from the corresponding subfolder within output_folder."), \
    (bool, save_volumes_and_camera_matrix_after_processing, false, PRIMITIVE, "Whether to save volume(s) after automatic processing"), \
    (bool, exit_after_automatic_processing, false, PRIMITIVE, "Whether to exit the program after the automatic run.")

DECLARE_DEFERRABLE_SERIALIZABLE_STRUCT(AUTOMATIC_RUN_SETTINGS_STRUCT_DESCRIPTION);

} // namespace ITMLib