//  ================================================================
//  Created by Gregory Kramida on 11/8/19.
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

//stdlib
#include <unordered_map>
#include <string>

//local
#include "LevelSetEvolutionParameters.h"
#include "../../Utils/Configuration/Configuration.h"
#include "../../Utils/FileIO/JSON_Utilities.h"
#include "../../Utils/CPPPrintHelpers.h"

namespace ITMLib{

DEFINE_SERIALIZABLE_STRUCT(WEIGHTS_STRUCT_DESCRIPTION);
DEFINE_SERIALIZABLE_STRUCT(SWITCHES_STRUCT_DESCRIPTION);
DEFINE_SERIALIZABLE_STRUCT(TERMINATION_CONDITIONS_STRUCT_DESCRIPTION);
DEFINE_DEFERRABLE_SERIALIZABLE_STRUCT(LEVEL_SET_EVOLUTION_PARAMETERS_STRUCT_DESCRIPTION);

} // namespace ITMLib