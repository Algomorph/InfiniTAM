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
#pragma once
//stdlib
#include <vector>
#include <cassert>
#include <functional>

//local
#include "ImageCombination.h"
#include "Image.h"
#include "Vector.h"

namespace ORUtils{

template<typename TPixel>
Image<TPixel> ConcatenateImages(std::vector<std::reference_wrapper<Image<TPixel>>> images, int axis = 0) {
	assert(axis >= 0 && axis < 3);
	assert(images.size() > 1);
	bool have_first_value = false;
	int shared_dimension;
	int combined_other_dimension = 0;
	int other_axis = (axis + 1) % 2;
	MemoryDeviceType access_device_mode;
	for (auto& image : images) {
		if (have_first_value) {
			assert(image.get().dimensions.values[axis] == shared_dimension);
			assert(image.get().GetAccessMode() == access_device_mode);
		} else {
			shared_dimension = image.get().dimensions.values[axis];
			have_first_value = true;
			access_device_mode = image.get().GetAccessMode();
		}
		combined_other_dimension += image.get().dimensions.values[other_axis];
	}
	Vector2<int> concatenated_image_dimensions;
	concatenated_image_dimensions.values[axis] = shared_dimension;
	concatenated_image_dimensions.values[other_axis] = combined_other_dimension;

#ifdef COMPILE_WITHOUT_CUDA
	#define RECO_UTILS_MEMCPY( destination, source, size ) memcpy(destination, source, size)
	assert(access_device_mode != MEMORYDEVICE_CUDA);
#else
	cudaMemcpyKind memcpy_kind;
	switch (access_device_mode) {
		case MEMORYDEVICE_CPU: {
			memcpy_kind = cudaMemcpyHostToHost;
		}
			break;
		case MEMORYDEVICE_CUDA: {
			memcpy_kind = cudaMemcpyDeviceToDevice;
		}
			break;
		default:
			DIEWITHEXCEPTION_REPORTLOCATION("Unsupported device type for concatenation.");
	}

#define RECO_UTILS_MEMCPY( destination, source, size ) cudaMemcpy((destination), (source), (size), memcpy_kind);
#endif


	Image<TPixel> concatenated_image(concatenated_image_dimensions, access_device_mode);
	TPixel* concatenated_image_data = concatenated_image.GetData(access_device_mode);

	switch (axis) {
		case 0:
			// data order is preserved, so we can simply copy/paste
			for (auto& image : images) {
				RECO_UTILS_MEMCPY(concatenated_image_data, image.get().GetData(access_device_mode), image.get().size() * sizeof(TPixel));
				concatenated_image_data += image.get().size();
			}
			break;
		case 1:{
			// row data is interleaved, so we have to go row-by-row and mix data from each image in each row
			for (int row = 0; row < shared_dimension; row++) {
				for (auto& image: images){
					int image_width = image.get().dimensions.width;
					RECO_UTILS_MEMCPY(concatenated_image_data, image.get().GetData(access_device_mode)[image_width * row], image_width * sizeof(TPixel));
					concatenated_image_data += image_width;
				}
			}
		}
			break;
		default:;
	}
#undef RECO_UTILS_MEMCPY

	return concatenated_image;
}

extern template Image<Vector4<unsigned char>> ConcatenateImages<Vector4<unsigned char>>(std::vector<std::reference_wrapper<Image<Vector4<unsigned char>>>> images, int axis = 0);

} // namespace ORUtils