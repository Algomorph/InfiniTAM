//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 5/8/20.
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
#include "../../Utils/Metacoding/DeferrableSerializableStruct.h"
#include "../../Utils/Geometry/CardinalAxesAndPlanes.h"

namespace ITMLib {

#define TELEMETRY_SETTINGS_STRUCT_DESCRIPTION TelemetrySettings, "telemetry_settings", \
    (bool, record_volume_memory_usage, false, PRIMITIVE, "Whether to record information required to debug memory" \
    " usage, e.g. used block locations for the VoxelBlockHash index."), \
    (bool, record_surface_tracking_optimization_energies, false, PRIMITIVE, "Whether to record optimization energies " \
    "for each iteration of the surface tracking optimization in a separate file. Only works when non_rigid_tracking_parameters.functor_type " \
    "parameter is set to \"slavcheva_diagnostic\""),                                   \
    (bool, record_surface_tracking_additional_statistics, false, PRIMITIVE, "Whether to additional statistics (warp update length, etc.) " \
    "for each iteration of the surface tracking optimization in a separate file. Only works when non_rigid_tracking_parameters.functor_type " \
    "parameter is set to \"slavcheva_diagnostic\""), \
    (bool, record_frame_meshes, false, PRIMITIVE, "Whether to record three meshes at every frame: (a) from live volume " \
    "after camera tracking and before surface tracking, (b) from live volume after surface tracking, and (c) from " \
    "canonical volume after fusion."), \
    (bool, record_warp_update_length_histograms, false, PRIMITIVE, "Whether record a histogram of warp lengths at every iteration."), \
    (bool, use_warp_update_length_histogram_manual_max, false, PRIMITIVE, "Enables setting an upper bound manually for warp update length "\
	"histograms via -telemetry_settings.warp_update_length_histogram_max."), \
    (float, warp_update_length_histogram_max, 0.00005, PRIMITIVE, "A manual bound for warp update length histogram (in meters). "\
    "Has effect only when -telemetry_settings.use_warp_update_length_histogram_manual_max is set to true."),                         \
    (int, warp_update_length_histogram_bin_count, 16, PRIMITIVE, \
    "Bin count for warp update length histogram when -telemetry_settings.record_warp_update_length_histograms "\
    "or -logging_settings.log_warp_update_length_histograms or both are used."),\
    (bool, use_CPU_for_mesh_recording, false, PRIMITIVE, "Whether to ALWAYS use CPU & regular RAM when recording mesh telemetry. For CUDA runs, this will reduce GPU memory usage."), \
    (bool, record_camera_matrices, false, PRIMITIVE, "Whether to record estimated camera trajectory matrices in world space.")


DECLARE_DEFERRABLE_SERIALIZABLE_STRUCT(TELEMETRY_SETTINGS_STRUCT_DESCRIPTION);


} // namespace ITMLib