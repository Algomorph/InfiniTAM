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

#include "../Metacoding/Metacoding.h"

namespace ITMLib {

#define VERBOSITY_LEVEL_ENUM_DESCRIPTION VerbosityLevel, \
    (VERBOSITY_SILENT, "silent", "SILENT", "VERBOSITY_SILENT"), \
    (VERBOSITY_FATAL, "fatal", "FATAL", "VERBOSITY_FATAL"), \
    (VERBOSITY_ERROR, "error", "ERROR", "VERBOSITY_ERROR"), \
    (VERBOSITY_WARNING, "warning", "WARNING", "VERBOSITY_WARNING"), \
    (VERBOSITY_INFO, "info", "information", "INFO", "VERBOSITY_INFO"), \
    (VERBOSITY_TOP_LEVEL, "top_level", "TOP_LEVEL", "Top-level operations", "VERBOSITY_TOP_LEVEL", "top-level", "top-level operations"), \
    (VERBOSITY_PER_FRAME, "per_frame", "PER_FRAME", "Per-frame operations", "VERBOSITY_PER_FRAME", "per-frame", "per-frame operations"), \
    (VERBOSITY_PER_ITERATION, "per_iteration", "PER_ITERATION", "Per-iteration operations", "VERBOSITY_PER_ITERATION", "per-iteration", "per-iteration operations"), \
    (VERBOSITY_FOCUS_SPOTS, "focus_spots", "FOCUS_SPOTS", "focus_voxel", "Interesting details", "trouble spots"), \
    (VERBOSITY_DEBUG, "debug", "DEBUG", "VERBOSITY_DEBUG")

DECLARE_SERIALIZABLE_ENUM(VERBOSITY_LEVEL_ENUM_DESCRIPTION);

#define LOGGING_SETTINGS_STRUCT_DESCRIPTION LoggingSettings, \
    (VerbosityLevel, verbosity_level, VERBOSITY_PER_FRAME, ENUM, "Verbosity level. "),\
    (bool, log_to_disk, false, PRIMITIVE, "Print log to text file, in the output path (preserves older log files). "\
    "Can be used in combination with stdout."), \
    (bool, log_to_stdout, true, PRIMITIVE, "Print log to stdout. Can be used in combination with disk logging."), \
    (bool, log_benchmarks, false, PRIMITIVE, "Whether to log runtime benchmarks after automatic run."),\
    (bool, log_volume_statistics, false, PRIMITIVE, "Whether to output various volume statistics after some operations" \
    " (used only when verbosity_level is set at or above PER_FRAME)."),\
    (bool, log_trajectory_quaternions, false, PRIMITIVE, "Whether to log estimated camera trajectory quaternions"), \
	(bool, log_iteration_number, false, PRIMITIVE, "Whether to log iteration number " \
	"(per-iteration verbosity only, diagnostic level_set_evolution.execution_mode only)."), \
    (bool, log_surface_tracking_procedure_names, false, PRIMITIVE,                    \
    "Whether to log names of surface tracking procedures as they are triggered " \
	"(per-iteration verbosity only, diagnostic level_set_evolution.execution_mode only)."), \
    (bool, log_max_gradient_length_position, false, PRIMITIVE, "Log position of the voxel with the greatest gradient length " \
    "(works only if level_set_alignment_parameters.termination.warp_length_termination_threshold_type is set to \"maximum\"."), \
    (bool, log_gradient_length_statistic, false, PRIMITIVE, \
    "Whether to log average warp update length (per-iteration verbosity only, diagnostic level_set_evolution.execution_mode only)."), \
    (bool, log_surface_tracking_optimization_energies, false, PRIMITIVE, "Whether to log optimization energies " \
    "for each iteration of the surface tracking optimization. Only works when level_set_evolution.execution_mode " \
    "parameter is set to \"diagnostic\""),                   \
    (bool, log_additional_surface_tracking_stats, false, PRIMITIVE, "Whether to log additional statistics " \
    "accumulated during each surface tracking iteration. Only works when level_set_evolution.execution_mode " \
    "parameter is set to \"diagnostic\""),\
    (bool, log_warp_update_length_histograms, false, PRIMITIVE, \
    "Whether to log warp update length histograms (per-iteration verbosity only, diagnostic level_set_evolution.execution_mode only). "\
	"The number of bins is controlled by -telemetry_settings.warp_update_length_histogram_bin_count. " \
	"A manual histogram maximum (if any) can also be set through telemetry_settings."), \
    (bool, log_voxel_hash_block_usage, false, PRIMITIVE, \
    "Whether to log counts of utilized voxel hash blocks for all volumes involved (along with upper bounds).")

DECLARE_SERIALIZABLE_STRUCT(LOGGING_SETTINGS_STRUCT_DESCRIPTION);

} // namespace ITMLib