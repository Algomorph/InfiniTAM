//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 1/3/21.
//  Copyright (c) 2021 Gregory Kramida
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

// === ORUtils ===
#include "../../../ORUtils/CrossPlatformMacros.h"

// local
#include "ImageProcessingEngine.h"
#include "ImageProcessingEngine_Functors.h"
#include "../Traversal/Interface/TwoImageTraversal.h"
#include "../Traversal/Interface/ImageTraversal.h"


using namespace ITMLib;


template<MemoryDeviceType TMemoryDeviceType>
ImageProcessingEngine<TMemoryDeviceType>::ImageProcessingEngine() {}

template<MemoryDeviceType TMemoryDeviceType>
ImageProcessingEngine<TMemoryDeviceType>::~ImageProcessingEngine() {}


template<MemoryDeviceType TMemoryDeviceType>
void ImageProcessingEngine<TMemoryDeviceType>::ConvertColorToIntensity(FloatImage& image_out, const UChar4Image& image_in) const {
	image_out.ChangeDims(image_in.dimensions);
	TwoImageTraversalEngine<TMemoryDeviceType>::Traverse(
			[=]_DEVICE_WHEN_AVAILABLE_(float& pixel_value_out, const Vector4u& pixel_value_in) {
				pixel_value_out = (0.299f * pixel_value_in.r + 0.587f * pixel_value_in.g +
				                   0.114f * pixel_value_in.b) / 255.f;
			},
			image_out, image_in
	);
}

template<MemoryDeviceType TMemoryDeviceType>
void ImageProcessingEngine<TMemoryDeviceType>::FilterIntensity(FloatImage& image_out, const FloatImage& image_in) const {
	image_out.ChangeDims(image_in.dimensions);
	image_out.Clear();

	const float* image_data_in = image_in.GetData(TMemoryDeviceType);
	float* image_data_out = image_out.GetData(TMemoryDeviceType);

	const int image_width = image_in.dimensions.width;
	const int image_height = image_in.dimensions.height;
	ImageTraversalEngine<TMemoryDeviceType>::TraversePositionOnly(
			[=]_DEVICE_WHEN_AVAILABLE_(int index, int x, int y) {
				if (x >= image_width - 2 || y >= image_height - 2 || x <= 1 || y <= 1) return;
				float& pixel_out = image_data_out[index];
				pixel_out += image_data_in[index];
				pixel_out += image_data_in[index + 1];
				pixel_out += image_data_in[index + image_width];
				pixel_out += image_data_in[index + image_width + 1];
				pixel_out /= 4.f;
			},
			image_in
	);
}

template<MemoryDeviceType TMemoryDeviceType>
void ImageProcessingEngine<TMemoryDeviceType>::FilterSubsample(UChar4Image& image_out, const UChar4Image& image_in) const {
	const Vector2i old_dimensions = image_in.dimensions;
	const Vector2i new_dimensions(old_dimensions.width / 2, old_dimensions.height / 2);
	image_out.ChangeDims(new_dimensions);

	const Vector4u* image_data_in = image_in.GetData(TMemoryDeviceType);
	Vector4u* image_data_out = image_out.GetData(TMemoryDeviceType);
	const int old_image_width = old_dimensions.width;
	ImageTraversalEngine<TMemoryDeviceType>::TraversePositionOnly(
			[=]_DEVICE_WHEN_AVAILABLE_(int index, int x, int y) {
				Vector4u& pixel_out = image_data_out[index];

				int source_x = x * 2, source_y = y * 2;
				const Vector4u& pixel_in1 = image_data_in[(source_x + 0) + (source_y + 0) * old_image_width];
				const Vector4u& pixel_in2 = image_data_in[(source_x + 1) + (source_y + 0) * old_image_width];
				const Vector4u& pixel_in3 = image_data_in[(source_x + 0) + (source_y + 1) * old_image_width];
				const Vector4u& pixel_in4 = image_data_in[(source_x + 1) + (source_y + 1) * old_image_width];

				pixel_out.r = (pixel_in1.r + pixel_in2.r + pixel_in3.r + pixel_in4.r) / 4;
				pixel_out.g = (pixel_in1.g + pixel_in2.g + pixel_in3.g + pixel_in4.g) / 4;
				pixel_out.b = (pixel_in1.b + pixel_in2.b + pixel_in3.b + pixel_in4.b) / 4;
				pixel_out.a = (pixel_in1.a + pixel_in2.a + pixel_in3.a + pixel_in4.a) / 4;
			},
			image_out
	);

}

template<MemoryDeviceType TMemoryDeviceType>
void ImageProcessingEngine<TMemoryDeviceType>::FilterSubsample(FloatImage& image_out, const FloatImage& image_in) const {
	const Vector2i old_dimensions = image_in.dimensions;
	const Vector2i new_dimensions(old_dimensions.width / 2, old_dimensions.height / 2);
	image_out.ChangeDims(new_dimensions);

	const float* image_data_in = image_in.GetData(TMemoryDeviceType);
	float* image_data_out = image_out.GetData(TMemoryDeviceType);
	const int old_image_width = old_dimensions.width;

	ImageTraversalEngine<TMemoryDeviceType>::TraversePositionOnly(
			[=]_DEVICE_WHEN_AVAILABLE_(int index, int x, int y) {
				if (x > new_dimensions.width - 2 || y > new_dimensions.height - 2 || x < 1 || y < 1) return;

				int source_x = x * 2, source_y = y * 2;

				float& pixel_out = image_data_out[index];
				pixel_out += image_data_in[(source_x + 0) + (source_y + 0) * old_image_width];
				pixel_out += image_data_in[(source_x + 1) + (source_y + 0) * old_image_width];
				pixel_out += image_data_in[(source_x + 0) + (source_y + 1) * old_image_width];
				pixel_out += image_data_in[(source_x + 1) + (source_y + 1) * old_image_width];
				pixel_out /= 4.f;
			},
			image_out
	);
}

template<MemoryDeviceType TMemoryDeviceType>
void ImageProcessingEngine<TMemoryDeviceType>::FilterSubsampleWithHoles(FloatImage& image_out, const FloatImage& image_in) const {
	const Vector2i old_dimensions = image_in.dimensions;
	const Vector2i new_dimensions(old_dimensions.width / 2, old_dimensions.height / 2);
	image_out.ChangeDims(new_dimensions);

	const float* image_data_in = image_in.GetData(TMemoryDeviceType);
	float* image_data_out = image_out.GetData(TMemoryDeviceType);
	const int old_image_width = old_dimensions.width;

	ImageTraversalEngine<TMemoryDeviceType>::TraversePositionOnly(
			[=]_DEVICE_WHEN_AVAILABLE_(int index, int x, int y) {
				int source_x = x * 2, source_y = y * 2;
				float& pixel_out = image_data_out[index];
				float pixel_in, no_good_pixels = 0.0f;

				pixel_in = image_data_in[(source_x + 0) + (source_y + 0) * old_image_width];
				if (pixel_in > 0.0f) { pixel_out += pixel_in; no_good_pixels++; }

				pixel_in = image_data_in[(source_x + 1) + (source_y + 0) * old_image_width];
				if (pixel_in > 0.0f) { pixel_out += pixel_in; no_good_pixels++; }

				pixel_in = image_data_in[(source_x + 0) + (source_y + 1) * old_image_width];
				if (pixel_in > 0.0f) { pixel_out += pixel_in; no_good_pixels++; }

				pixel_in = image_data_in[(source_x + 1) + (source_y + 1) * old_image_width];
				if (pixel_in > 0.0f) { pixel_out += pixel_in; no_good_pixels++; }

				if (no_good_pixels > 0) pixel_out /= no_good_pixels;
			},
			image_out
	);
}

template<MemoryDeviceType TMemoryDeviceType>
void ImageProcessingEngine<TMemoryDeviceType>::FilterSubsampleWithHoles(Float4Image& image_out, const Float4Image& image_in) const {
	DIEWITHEXCEPTION_REPORTLOCATION("Not implemented.");
}

template<MemoryDeviceType TMemoryDeviceType>
void ImageProcessingEngine<TMemoryDeviceType>::GradientX(Short4Image& grad_out, const UChar4Image& image_in) const {
	DIEWITHEXCEPTION_REPORTLOCATION("Not implemented.");
}

template<MemoryDeviceType TMemoryDeviceType>
void ImageProcessingEngine<TMemoryDeviceType>::GradientY(Short4Image& grad_out, const UChar4Image& image_in) const {
	DIEWITHEXCEPTION_REPORTLOCATION("Not implemented.");
}

template<MemoryDeviceType TMemoryDeviceType>
void ImageProcessingEngine<TMemoryDeviceType>::GradientXY(Float2Image& grad_out, const FloatImage& image_in) const {
	DIEWITHEXCEPTION_REPORTLOCATION("Not implemented.");
}

template<MemoryDeviceType TMemoryDeviceType>
int ImageProcessingEngine<TMemoryDeviceType>::CountValidDepths(const FloatImage& image_in) const {
	return 0;
}
