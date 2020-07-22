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
#include <memory>

namespace ORUtils {

static auto font_hack_regular = [](){
	return std::make_shared<wft::face>(RESOURCE_DIRECTORY "fonts/Hack-Regular.ttf");}();
static auto font_liberation_mono = [](){
	return std::make_shared<wft::face>(RESOURCE_DIRECTORY "fonts/LiberationMono-Regular.ttf");}();

template<>
void DrawTextOnImage<Vector4<unsigned char>>(Image<Vector4<unsigned char>>& image, const std::string& text, int font_size, int offset_x, int offset_y, bool lower_right_corner_offset) {
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
	//wft::font font_hack_regular{RESOURCE_DIRECTORY "fonts/Hack-Regular.ttf"};
	font_hack_regular->set_size(0, font_size);
	auto bitmap = font_hack_regular->make_bitmap_text(text);
	int i_bit = 0;
	if (lower_right_corner_offset) {
		offset_x = image_width - bitmap.width - offset_x;
		offset_y = image_height - bitmap.height - offset_y;
	}
	int& y_image = offset_y;
	for (int row_bitmap = 0; row_bitmap < bitmap.height; row_bitmap++, y_image++) {
		int x_image = offset_x;
		for (int col_bitmap = 0; col_bitmap < bitmap.width; col_bitmap++, i_bit++, x_image++) {
			if (offset_y < image_height && offset_x < image_width) {
				unsigned char bit = bitmap.data[i_bit];
				if(bit != 0){
					Vector4<unsigned char>& pixel = image_data[y_image * image_width + x_image];
					pixel.r = pixel.r + bit < pixel.r ? 255 : pixel.r + bit;
					pixel.g = pixel.g + bit < pixel.g ? 255 : pixel.g + bit;;
					pixel.b = pixel.b + bit < pixel.b ? 255 : pixel.b + bit;;
				}
			}
		}
	}

	if (allocated_temporary_image) {
		image.SetFrom(*image_to_modify, MemoryCopyDirection::CPU_TO_CUDA);
		delete image_to_modify;
	}
}

} // namespace ORUtils