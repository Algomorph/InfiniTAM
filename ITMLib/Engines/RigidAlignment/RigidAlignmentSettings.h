//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 1/12/21.
//  Copyright (c) 2021 Gregory Kramida
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
#include "RigidAlignmentOptimizationType.h"
#include "../../Utils/Metacoding/DeferrableSerializableStruct.h"

namespace ITMLib {

extern const std::vector<RigidAlignmentOptimizationType> default_levels;
//TODO: verify the stuff in descriptions below [that appears in square brackets with a question mark at the end?]
#define RIGID_ALIGNMENT_SETTINGS_STRUCT_DESCRIPTION RigidAlignmentSettings, "rigid_alignment_settings", \
    (std::vector<RigidAlignmentOptimizationType>, levels, default_levels, DYNAMIC_VECTOR, \
    "Optimization types for each resolution hierarchy level. The type for each level can be set to \"rotation\" (\"r\"),  "\
    "\"translation\" (\"t\"), or \"both\" (\"b\"), which determines which part of the rigid transformation will be optimized at "\
    "each level, with levels ordered from from fine/large to coarse/small."), \
    (bool, use_depth, true, PRIMITIVE, "When set to false, pixel depth values will not be taken into account during optimization, and "\
    "\"use_color\" must be set to true."), \
    (bool, use_color, true, PRIMITIVE, "When set to false, pixel color values will not be taken into account during optimization, and "\
    "\"use_depth\" must be set to true."),  \
    (float, color_weight, 0.3f, PRIMITIVE, "When both depth and color pixel values are used during optimization, this weight will scale" \
    " the contributions to the rigid transform (errors [which ones?] and jacobians) from the color-based optimization."), \
    (float, min_update_threshold, 1e-4f, PRIMITIVE, "Update distance [between what and what?] threshold for convergence."), \
    (float, distance_outlier_threshold_fine, 0.004f, PRIMITIVE, "Distance threshold that is used to designate outlier points on the finest level."), \
    (float, distance_outlier_threshold_coarse, 0.1f, PRIMITIVE, "Distance threshold that is used to designate outlier points on the coarsest level."), \
    (float, color_outlier_threshold_fine, 0.145f, PRIMITIVE, "Color distance [intensity difference ?] threshold that is used to designate outlier" \
    " points on the finest level."), \
    (float, color_outlier_threshold_coarse, 0.005f, PRIMITIVE, "Color distance [intensity difference ?] threshold that is used to designate outlier" \
    " points on the coarsest level."), \
    (float, min_color_gradient, 0.01f, PRIMITIVE, "Minimum color gradient [intensity gradient magnitude?] for a pixel to be used for alignment." \
    " points on the coarsest level."), \
    (int, iteration_count_fine, 20, PRIMITIVE, "Number of iterations to optimize on the finer level."), \
    (int, iteration_count_coarse, 20, PRIMITIVE, "Number of iterations to optimize on the coarser level."), \
    (float, tukey_cutoff, 8.0f, PRIMITIVE, "Coff for the Tukey m-estimator."), \
    (int, frames_to_skip, 20, PRIMITIVE, "Number of frames to skip before a depth pixel is used for tracking -- [this is inaccurate!!?]."), \
    (int, frames_to_weight, 50, PRIMITIVE, "Number of frames to accumulate each pixel's weight before it's used to the maximum extent -- [this is inaccurate!!?]."), \
    (float, failure_detection_threshold, 3.0f, PRIMITIVE, "Threshold [of what?] for failure detection.")

DECLARE_DEFERRABLE_SERIALIZABLE_STRUCT(RIGID_ALIGNMENT_SETTINGS_STRUCT_DESCRIPTION);


} // namespace ITMLib