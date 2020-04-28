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
#include "../../Utils/Serialization/Serialization.h"

namespace ITMLib {

class SlavchevaSurfaceTracker {
public:
#define PARAMETERS_STRUCT_DESCRIPTION Parameters, \
        (float, learning_rate, 0.1f, PRIMITIVE, "Used in dynamic surface tracking optimization. Gradient descent step magnitude / learning rate."), \
        (float, rigidity_enforcement_factor, 0.1f, PRIMITIVE, "Used in dynamic surface tracking optimization when the Killing regularization term is enabled."), \
        (float, weight_data_term, 1.0f, PRIMITIVE, "Used in dynamic surface tracking optimization when the data term is enabled."), \
        (float, weight_smoothing_term, 0.2f, PRIMITIVE, "Used in dynamic surface tracking optimization when the smoothness regularization term is enabled."), \
        (float, weight_level_set_term, 0.2f, PRIMITIVE, \
        	"Used in dynamic surface tracking optimization when the level set regularization term is enabled." \
			" Greater values penalize deformations resulting in non-SDF-like voxel grid."), \
        (float, epsilon, 1e-5f, PRIMITIVE, "Small value to avoid division by zero when computing level set term in dynamic surface tracking optimization.")

	DECLARE_SERIALIZABLE_STRUCT(PARAMETERS_STRUCT_DESCRIPTION);

#define SWITCHES_STRUCT_DESCRIPTION Switches, \
        (bool, enable_data_term, true, PRIMITIVE, "Whether to enable or disable data term of Slavcheva-based dynamic surface tracking energy."), \
        (bool, enable_level_set_term, false, PRIMITIVE, "Whether to enable or disable level-set of Slavcheva-based dynamic surface tracking energy. (see KillingFusion by Slavcheva et. all.)"), \
        (bool, enable_smoothing_term, true, PRIMITIVE, \
        		"Whether to enable or disable smoothing regularization term of Slavcheva-based dynamic surface " \
		        "tracking energy. When rigidity-enforcement factor is enabled, acts as Killing term in KillingFusion,"\
		        " when it is not, acts as Tikhonov term in SobolevFusion (both articles by Slavcheva et al.)"), \
        (bool, enable_killing_rigidity_enforcement_term, false, PRIMITIVE, "Whether to enable or disable the non-isometric motion penalizing portion of the Killing term of Slavcheva-based dynamic surface tracking energy (see KillingFusion by Slavcheva et. all."), \
        (bool, enable_sobolev_gradient_smoothing, true, PRIMITIVE, "Whether to enable or disable Sobolev-space gradient smoothing of Slavcheva-based dynamic surface tracking (see SobolevFusion article by Slavcheva et al.).")

	DECLARE_SERIALIZABLE_STRUCT( SWITCHES_STRUCT_DESCRIPTION );

	const Parameters
	parameters;
	const Switches switches;

	explicit SlavchevaSurfaceTracker();
	SlavchevaSurfaceTracker(Switches switches, Parameters parameters = Parameters());

private:
	void PrintSettings();
};
}//namespace ITMLib
