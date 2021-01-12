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
#include "RigidAlignmentSettings.h"

namespace ITMLib {

const std::vector<RigidAlignmentOptimizationType> default_levels = {ROTATION, ROTATION, TRANSLATION, TRANSLATION};

DEFINE_DEFERRABLE_SERIALIZABLE_STRUCT(RIGID_ALIGNMENT_SETTINGS_STRUCT_DESCRIPTION);


} // namespace ITMLib
