//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 7/18/20.
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
#include "Image.h"
#include "Vector.h"

namespace ORUtils{


template<typename T>
void DrawTextOnImage(Image<T>& image, const std::string& text, int x, int y, bool lower_right_corner_offset);

template<>
void DrawTextOnImage<Vector4<unsigned char>>(Image<Vector4<unsigned char>>& image, const std::string& text, int x, int y, bool lower_right_corner_offset);

} // namespace ORUtils