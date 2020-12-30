//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 6/5/20.
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

namespace ITMLib {
#define RAYCASTING_SETTINGS_STRUCT_DESCRIPTION RaycastingSettings, "raycasting_settings", \
    (bool, skip_points, false, PRIMITIVE, "Skips every other point when using the color renderer for creating a point cloud")


DECLARE_DEFERRABLE_SERIALIZABLE_STRUCT(RAYCASTING_SETTINGS_STRUCT_DESCRIPTION);
}


