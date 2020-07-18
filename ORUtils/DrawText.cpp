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
#include "DrawText.h"
#include "Vector.h"
#include "wft/wft.hpp"
#include <ORUtilsConfig.h>

namespace ORUtils {

template<>
void DrawText<Vector4<unsigned char>>(Image<Vector4<unsigned char>>& image, const std::string& text, int x, int y, bool lower_right_corner_offset) {
	Image<Vector4<unsigned char>>* image_to_modify;
	bool allocated_temporary_image = false;
	if (image.GetAccessMode() == MEMORYDEVICE_CPU) {
		image_to_modify = &image;
	} else {
		allocated_temporary_image = true;
		image_to_modify = new Image<Vector4<unsigned char>>(image, MEMORYDEVICE_CPU);
	}

	int image_width = image.dimensions.width, image_height = image.dimensions.height;
	Vector4<unsigned char>* image_data = image_to_modify->GetData(MEMORYDEVICE_CPU);
	wft::font font_hack_regular{RESOURCE_DIRECTORY "fonts/Hack-Regular.ttf"};
	auto bitmap = font_hack_regular.make_bitmap_text(text);
	int i_bit = 0;
	int& row_image = y;
	int& col_image = x;

	if (lower_right_corner_offset) {
		col_image = image_width - bitmap.width - x;
		row_image = image_width - bitmap.height - y;
	}

	for (int row_bitmap = 0; row_bitmap < bitmap.height; row_bitmap++, row_image++) {
		for (int col_bitmap = 0; col_bitmap < bitmap.width; col_bitmap++, i_bit++, col_image++) {
			if (row_image < image_height && col_image < image_width) {
				unsigned char bit = bitmap.Bits[i_bit];
				Vector4<unsigned char>& pixel = image_data[row_image * image_width + col_image];
				pixel.r = bit;
				pixel.g = bit;
				pixel.b = bit;
			}
		}
	}

	if (allocated_temporary_image) {
		image.SetFrom(*image_to_modify, MemoryCopyDirection::CPU_TO_CUDA);
		delete image_to_modify;
	}
}

} // namespace ORUtils