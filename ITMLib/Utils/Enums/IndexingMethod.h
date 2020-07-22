//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 7/22/20.
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
#include "../Metacoding/SerializableEnum.h"

namespace ITMLib {

#define INDEXING_METHOD_ENUM_DESCRIPTION IndexingMethod, \
    (INDEX_HASH, "hash", "HASH"), \
    (INDEX_ARRAY, "array", "ARRAY")

DECLARE_SERIALIZABLE_ENUM(INDEXING_METHOD_ENUM_DESCRIPTION);

} // namespace ITMLib