//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 5/29/20.
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

#include "../../../Utils/Math.h"

namespace ITMLib{

static const CONSTPTR(int) MAX_RENDERING_BLOCKS = 65536*4;

struct RenderingBlock {
	Vector2s upper_left;
	Vector2s lower_right;
	Vector2f z_range;
};

} // namespace ITMLib