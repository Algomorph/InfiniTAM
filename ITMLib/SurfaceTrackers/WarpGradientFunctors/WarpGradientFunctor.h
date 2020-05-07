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

namespace ITMLib {

#define GRADIENT_FUNCTOR_TYPE_ENUM_DESCRIPTION GradientFunctorType, \
    (TRACKER_SLAVCHEVA_DIAGNOSTIC, "slavcheva_diagnostic", "SLAVCHEVA_DIAGNOSTIC"),\
    (TRACKER_SLAVCHEVA_OPTIMIZED, "slavcheva_optimized", "SLAVCHEVA_OPTIMIZED")

DECLARE_SERIALIZABLE_ENUM(GRADIENT_FUNCTOR_TYPE_ENUM_DESCRIPTION)

template< typename TTSDFVoxel, typename TWarpVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType, GradientFunctorType TGradientFunctorType>
struct WarpGradientFunctor;
} // namespace ITMLib

DEFINE_INLINE_SERIALIZABLE_ENUM(ITMLib::GRADIENT_FUNCTOR_TYPE_ENUM_DESCRIPTION)