//  ================================================================
//  Created by Gregory Kramida on 11/13/19.
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
#pragma once

//boost
#include <boost/program_options.hpp>
#include <boost/property_tree/ptree.hpp>

//local
#include "../../../Utils/Metacoding/DeferrableSerializableStruct.h"
#include "../../../Utils/Metacoding/SerializableStruct.h"
#include "../../../Utils/Enums/ExecutionMode.h"

namespace ITMLib {

#define WEIGHTS_STRUCT_DESCRIPTION LevelSetAlignmentWeights, \
        (float, learning_rate, 0.1f, PRIMITIVE, "Used in level set alignment optimization. Gradient descent step magnitude / learning rate."), \
        (float, Killing_dampening_factor, 0.1f, PRIMITIVE, "Used in level set alignment optimization when the Killing regularization term is enabled."), \
        (float, weight_data_term, 1.0f, PRIMITIVE, "Used in level set alignment optimization when the data term is enabled."), \
        (float, weight_smoothing_term, 0.2f, PRIMITIVE, "Used in level set alignment optimization when the smoothness regularization term is enabled."), \
        (float, weight_level_set_term, 0.2f, PRIMITIVE, \
            "Used in level set alignment optimization when the level set regularization term is enabled." \
            " Greater values penalize deformations resulting in non-SDF-like voxel grid."), \
        (float, epsilon, 1e-5f, PRIMITIVE, "Small value to avoid division by zero when computing level set term in level set alignment optimization.")

DECLARE_SERIALIZABLE_STRUCT(WEIGHTS_STRUCT_DESCRIPTION);

#define SWITCHES_STRUCT_DESCRIPTION LevelSetAlignmentSwitches, \
        (bool, enable_data_term, true, PRIMITIVE, "Whether to enable or disable data term of Slavcheva-based level set alignment energy."), \
        (bool, enable_level_set_term, false, PRIMITIVE, "Whether to enable or disable level-set of Slavcheva-based level set alignment energy. (see KillingFusion by Slavcheva et. all.)"), \
        (bool, enable_smoothing_term, true, PRIMITIVE, \
                "Whether to enable or disable smoothing regularization term of Slavcheva-based dynamic surface " \
                "tracking energy. When rigidity-enforcement factor is enabled, acts as Killing term in KillingFusion,"\
                " when it is not, acts as Tikhonov term in SobolevFusion (both articles by Slavcheva et al.)"), \
        (bool, enable_Killing_field, false, PRIMITIVE, "Whether to enable or disable the non-isometric-motion-penalizing Killing field portion of the smoothing term of Slavcheva-based level-set alignment energy (see KillingFusion by Slavcheva et. all."), \
        (bool, enable_sobolev_gradient_smoothing, true, PRIMITIVE, "Whether to enable or disable Sobolev-space gradient smoothing of Slavcheva-based level set alignment (see SobolevFusion article by Slavcheva et al.).")

DECLARE_SERIALIZABLE_STRUCT(SWITCHES_STRUCT_DESCRIPTION);


#define TERMINATION_CONDITIONS_STRUCT_DESCRIPTION LevelSetAlignmentTerminationConditions, \
        (int, max_iteration_count, 200, PRIMITIVE, "Maximum iteration count, after which the non-rigid alignment is cut off."), \
        (int, min_iteration_count, 10, PRIMITIVE, "Minimum iteration count, after which all other termination conditions are enabled."), \
        (float, update_length_threshold, 1e-6f, PRIMITIVE, "Update length threshold factor, in voxels ('1/[voxel size] * factor'). Depending on settings, can be" \
        " \"mean update length\" or \"max update length\". When the mean vector update in calculating voxel motion doesn't exceed this"\
        " threshold, the non-rigid alignment optimization is terminated.")


DECLARE_SERIALIZABLE_STRUCT(TERMINATION_CONDITIONS_STRUCT_DESCRIPTION);

#define LEVEL_SET_EVOLUTION_PARAMETERS_STRUCT_DESCRIPTION LevelSetAlignmentParameters, "level_set_evolution", \
    (ExecutionMode, execution_mode, ExecutionMode::OPTIMIZED, ENUM, "Whether to use optimized or diagnostic mode."), \
    (LevelSetAlignmentWeights, weights, LevelSetAlignmentWeights(), STRUCT, "Level set evolution weights / rates / factors"), \
    (LevelSetAlignmentSwitches, switches, LevelSetAlignmentSwitches(), STRUCT, "Level set evolution switches for turning different terms on and off."), \
    (LevelSetAlignmentTerminationConditions, termination, LevelSetAlignmentTerminationConditions(), STRUCT, "Level set evolution termination parameters.") \

DECLARE_DEFERRABLE_SERIALIZABLE_STRUCT(LEVEL_SET_EVOLUTION_PARAMETERS_STRUCT_DESCRIPTION);

std::ostream& operator<<(std::ostream& stream, const LevelSetAlignmentParameters& parameters);

}//namespace ITMLib
