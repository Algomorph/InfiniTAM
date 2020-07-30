//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 7/23/20.
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

//stdlib
#include <vector>
#include <cassert>

//local
#include "ImageCombination.h"
#include "Image.h"
#include "Vector.h"

namespace ORUtils {

template Image<Vector4<unsigned char>> ConcatenateImages<Vector4<unsigned char>>(std::vector<std::reference_wrapper<Image<Vector4<unsigned char>>>> images, int axis);

} // namespace ORUtils