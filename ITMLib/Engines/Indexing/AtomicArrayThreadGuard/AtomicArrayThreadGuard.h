//  ================================================================
//  Created by Gregory Kramida on 2/6/20.
//  Copyright (c)  2020 Gregory Kramida
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

//local
#include "../../../../ORUtils/MemoryDeviceType.h"

namespace ITMLib{

/**
 * \brief A simplistic and not very safe protection mechanism for multiple threads working on a fixed number of shared items.
 * \details Assumes the same thread will always lock & release on the same item index. The locks use optimistic retrys and are not re-entrant.
 * \tparam TMemoryDeviceType - CPU or CUDA currently supported.
 */
template<MemoryDeviceType TMemoryDeviceType>
class AtomicArrayThreadGuard;

}
