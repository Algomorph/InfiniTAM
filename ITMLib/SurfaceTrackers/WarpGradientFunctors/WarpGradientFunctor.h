//  ================================================================
//  Created by Gregory Kramida on 11/18/19.
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

#include "../../Utils/Metacoding/Metacoding.h"
#include "../../../ORUtils/MemoryDeviceType.h"
#include "../../Utils/Enums/ExecutionMode.h"

namespace ITMLib {

template< typename TTSDFVoxel, typename TWarpVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType, ExecutionMode TGradientFunctorType>
struct WarpGradientFunctor;

} // namespace ITMLib