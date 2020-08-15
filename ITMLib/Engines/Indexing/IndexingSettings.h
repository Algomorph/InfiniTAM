//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 5/14/20.
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

#include "../../Utils/Metacoding/DeferrableSerializableStruct.h"
#include "../../Utils/Enums/ExecutionMode.h"

namespace ITMLib {
#define INDEXING_SETTINGS_STRUCT_DESCRIPTION IndexingSettings, "indexing_settings", \
    (ExecutionMode, execution_mode, OPTIMIZED, ENUM, "Set to \"diagnostic\" for recording telemetry while performing " \
	 "index allocations, or \"optimized\" for unhindered, optimized execution of index allocations.")

DECLARE_DEFERRABLE_SERIALIZABLE_STRUCT(INDEXING_SETTINGS_STRUCT_DESCRIPTION);
}