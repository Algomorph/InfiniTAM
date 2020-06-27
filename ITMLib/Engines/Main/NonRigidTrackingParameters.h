//  ================================================================
//  Created by Gregory Kramida on 1/2/20.
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

#include "../../Utils/Metacoding/Metacoding.h"
#include "../../SurfaceTrackers/WarpGradientFunctors/WarpGradientFunctor.h"


namespace ITMLib {

GENERATE_SERIALIZABLE_STRUCT(NonRigidTrackingParameters,
                             (GradientFunctorType, functor_type, ITMLib::TRACKER_SLAVCHEVA_DIAGNOSTIC, ENUM,
                             		"blah"),\
                             (int, max_iteration_threshold, 200, PRIMITIVE, "Maximum iteration count, after which the non-rigid alignment is cut off."),\
                             (float, max_update_length_threshold, 0.0001f, PRIMITIVE, "Maximum update length threshold, in meters. When the greatest"\
                                 " vector update in calculating voxel motion doesn't exceed this threshold, the non-rigid alignment optimization is terminated."),\
                             (float, momentum_weight, 0.5f, PRIMITIVE, "blah"));

} // namespace ITMLib
