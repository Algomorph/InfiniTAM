//  ================================================================
//  Created by Gregory Kramida on 10/16/19.
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

#include "../../../ORUtils/PlatformIndependence.h"

namespace ITMLib{

//_CPU_AND_GPU_CONSTANT_
constexpr const char* red = "\033[0;31m";
//_CPU_AND_GPU_CONSTANT_
constexpr const char* green = "\033[0;32m";
//_CPU_AND_GPU_CONSTANT_
constexpr const char* blue = "\033[0;34m";
//_CPU_AND_GPU_CONSTANT_
constexpr const char* yellow = "\033[0;33m";
//_CPU_AND_GPU_CONSTANT_
constexpr const char* cyan = "\033[0;36m";

//_CPU_AND_GPU_CONSTANT_
constexpr const char* bright_red = "\033[0;91m";
//_CPU_AND_GPU_CONSTANT_
constexpr const char* bright_cyan = "\033[0;96m";
//_CPU_AND_GPU_CONSTANT_
constexpr const char* reset = "\033[0m";

} // namespace ITMLib