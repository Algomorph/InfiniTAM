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
#include "CountValidDepths.h"
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
	Vector4u * image_data_out = image_out.GetData(TMemoryDeviceType);
	const int old_image_width = old_dimensions.width;
	ImageTraversalEngine<TMemoryDeviceType>::TraversePositionOnly(
			[=]_DEVICE_WHEN_AVAILABLE_(int index, int x, int y) {
				Vector4u & pixel_out = image_data_out[index];

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
	image_out.Clear();

	const float* image_data_in = image_in.GetData(TMemoryDeviceType);
	float* image_data_out = image_out.GetData(TMemoryDeviceType);
	const int old_image_width = old_dimensions.width;

	ImageTraversalEngine<TMemoryDeviceType>::TraversePositionOnly(
			[=]_DEVICE_WHEN_AVAILABLE_(int index, int x, int y) {
				int source_x = x * 2, source_y = y * 2;
				float& pixel_out = image_data_out[index];
				float pixel_in, good_pixel_count = 0.0f;

				pixel_in = image_data_in[(source_x + 0) + (source_y + 0) * old_image_width];
				if (pixel_in > 0.0f) {
					pixel_out += pixel_in;
					good_pixel_count++;
				}

				pixel_in = image_data_in[(source_x + 1) + (source_y + 0) * old_image_width];
				if (pixel_in > 0.0f) {
					pixel_out += pixel_in;
					good_pixel_count++;
				}

				pixel_in = image_data_in[(source_x + 0) + (source_y + 1) * old_image_width];
				if (pixel_in > 0.0f) {
					pixel_out += pixel_in;
					good_pixel_count++;
				}

				pixel_in = image_data_in[(source_x + 1) + (source_y + 1) * old_image_width];
				if (pixel_in > 0.0f) {
					pixel_out += pixel_in;
					good_pixel_count++;
				}

				if (good_pixel_count > 0) pixel_out /= good_pixel_count;
			},
			image_out
	);
}

template<MemoryDeviceType TMemoryDeviceType>
void ImageProcessingEngine<TMemoryDeviceType>::FilterSubsampleWithHoles(Float4Image& image_out, const Float4Image& image_in) const {
	const Vector2i old_dimensions = image_in.dimensions;
	const Vector2i new_dimensions(old_dimensions.width / 2, old_dimensions.height / 2);
	image_out.ChangeDims(new_dimensions);
	image_out.Clear();

	const Vector4f* image_data_in = image_in.GetData(TMemoryDeviceType);
	Vector4f* image_data_out = image_out.GetData(TMemoryDeviceType);
	const int old_image_width = old_dimensions.width;

	ImageTraversalEngine<TMemoryDeviceType>::TraversePositionOnly(
			[=]_DEVICE_WHEN_AVAILABLE_(int index, int x, int y) {
				int source_x = x * 2, source_y = y * 2;
				Vector4f& pixel_out = image_data_out[index];
				Vector4f pixel_in;
				float good_pixel_count = 0.0f;

				pixel_in = image_data_in[(source_x + 0) + (source_y + 0) * old_image_width];
				if (pixel_in.w > 0.0f) {
					pixel_out += pixel_in;
					good_pixel_count++;
				}

				pixel_in = image_data_in[(source_x + 1) + (source_y + 0) * old_image_width];
				if (pixel_in.w > 0.0f) {
					pixel_out += pixel_in;
					good_pixel_count++;
				}

				pixel_in = image_data_in[(source_x + 0) + (source_y + 1) * old_image_width];
				if (pixel_in.w > 0.0f) {
					pixel_out += pixel_in;
					good_pixel_count++;
				}

				pixel_in = image_data_in[(source_x + 1) + (source_y + 1) * old_image_width];
				if (pixel_in.w > 0.0f) {
					pixel_out += pixel_in;
					good_pixel_count++;
				}

				if (good_pixel_count > 0) {
					pixel_out /= good_pixel_count;
				} else {
					pixel_out.w = -1.0f;
				};
			},
			image_out
	);
}

//TODO: rename to PerChannelSobelGradientX
template<MemoryDeviceType TMemoryDeviceType>
void ImageProcessingEngine<TMemoryDeviceType>::GradientX(Short4Image& gradient_out, const UChar4Image& image_in) const {
	gradient_out.ChangeDims(image_in.dimensions);
	gradient_out.Clear();
	Vector4s* gradient_data = gradient_out.GetData(TMemoryDeviceType);
	const Vector4u* image_data = image_in.GetData(TMemoryDeviceType);
	const Vector2i image_dimensions = image_in.dimensions;
	const int image_width = image_dimensions.width;

	ImageTraversalEngine<TMemoryDeviceType>::TraversePositionOnly(
			[=]_DEVICE_WHEN_AVAILABLE_(int index, int x, int y) {
				if (x < 1 || x > image_width - 2 || y < 1 || y > image_dimensions.height - 2) return;
				Vector4s& gradient_pixel_out = gradient_data[index];

				const short alpha = 2 * 255;

				const Vector4u& pixel_at_x_plus_one_y_minus_one = image_data[index - image_width + 1];
				const Vector4u& pixel_at_x_minus_one_y_minus_one = image_data[index - image_width - 1];

				const Vector4s gradient_at_y_minus_one_row(pixel_at_x_plus_one_y_minus_one.r - pixel_at_x_minus_one_y_minus_one.r,
				                                           pixel_at_x_plus_one_y_minus_one.g - pixel_at_x_minus_one_y_minus_one.g,
				                                           pixel_at_x_plus_one_y_minus_one.b - pixel_at_x_minus_one_y_minus_one.b,
				                                           alpha);

				const Vector4u& pixel_at_x_plus_one_y = image_data[index + 1];
				const Vector4u& pixel_at_x_minus_one_y = image_data[index - 1];

				const Vector4s gradient_at_row_y(pixel_at_x_plus_one_y.r - pixel_at_x_minus_one_y.r,
				                                 pixel_at_x_plus_one_y.g - pixel_at_x_minus_one_y.g,
				                                 pixel_at_x_plus_one_y.b - pixel_at_x_minus_one_y.b,
				                                 alpha);

				const Vector4u& pixel_at_x_plus_one_y_plus_one = image_data[index + image_width + 1];
				const Vector4u& pixel_at_x_minus_one_y_plus_one = image_data[index + image_width - 1];

				const Vector4s gradient_at_y_plus_one_row(pixel_at_x_plus_one_y_plus_one.r - pixel_at_x_minus_one_y_plus_one.r,
				                                          pixel_at_x_plus_one_y_plus_one.g - pixel_at_x_minus_one_y_plus_one.g,
				                                          pixel_at_x_plus_one_y_plus_one.b - pixel_at_x_minus_one_y_plus_one.b,
				                                          alpha);

				gradient_pixel_out.r = (gradient_at_y_minus_one_row.r + 2 * gradient_at_row_y.r + gradient_at_y_plus_one_row.r) / 8;
				gradient_pixel_out.g = (gradient_at_y_minus_one_row.g + 2 * gradient_at_row_y.g + gradient_at_y_plus_one_row.g) / 8;
				gradient_pixel_out.b = (gradient_at_y_minus_one_row.b + 2 * gradient_at_row_y.b + gradient_at_y_plus_one_row.b) / 8;
				gradient_pixel_out.a = (gradient_at_y_minus_one_row.a + 2 * gradient_at_row_y.a + gradient_at_y_plus_one_row.a) / 8;
			},
			gradient_out
	);
}

//TODO: rename to PerChannelSobelGradientY
template<MemoryDeviceType TMemoryDeviceType>
void ImageProcessingEngine<TMemoryDeviceType>::GradientY(Short4Image& gradient_out, const UChar4Image& image_in) const {
	gradient_out.ChangeDims(image_in.dimensions);
	gradient_out.Clear();
	Vector4s* gradient_data = gradient_out.GetData(TMemoryDeviceType);
	const Vector4u* image_data = image_in.GetData(TMemoryDeviceType);
	const Vector2i image_dimensions = image_in.dimensions;
	const int image_width = image_dimensions.width;

	ImageTraversalEngine<TMemoryDeviceType>::TraversePositionOnly(
			[=]_DEVICE_WHEN_AVAILABLE_(int index, int x, int y) {
				if (x < 1 || x > image_width - 2 || y < 1 || y > image_dimensions.height - 2) return;
				Vector4s& gradient_pixel_out = gradient_data[index];

				const short alpha = 2 * 255;

				const Vector4u& pixel_at_x_minus_one_y_plus_one = image_data[index + image_width - 1];
				const Vector4u& pixel_at_x_minus_one_y_minus_one = image_data[index - image_width - 1];

				const Vector4s gradient_at_x_minus_one_column(pixel_at_x_minus_one_y_plus_one.r - pixel_at_x_minus_one_y_minus_one.r,
				                                              pixel_at_x_minus_one_y_plus_one.g - pixel_at_x_minus_one_y_minus_one.g,
				                                              pixel_at_x_minus_one_y_plus_one.b - pixel_at_x_minus_one_y_minus_one.b,
				                                              alpha);

				const Vector4u& pixel_at_x_y_plus_one = image_data[index + image_width];
				const Vector4u& pixel_at_x_y_minus_one = image_data[index - image_width];

				const Vector4s gradient_at_column_x(pixel_at_x_y_plus_one.r - pixel_at_x_y_minus_one.r,
				                                    pixel_at_x_y_plus_one.g - pixel_at_x_y_minus_one.g,
				                                    pixel_at_x_y_plus_one.b - pixel_at_x_y_minus_one.b,
				                                    alpha);

				const Vector4u& pixel_at_x_plus_one_y_plus_one = image_data[index + image_width + 1];
				const Vector4u& pixel_at_x_plus_one_y_minus_one = image_data[index - image_width + 1];

				const Vector4s gradient_at_x_plus_one_row(pixel_at_x_plus_one_y_plus_one.r - pixel_at_x_plus_one_y_minus_one.r,
				                                          pixel_at_x_plus_one_y_plus_one.g - pixel_at_x_plus_one_y_minus_one.g,
				                                          pixel_at_x_plus_one_y_plus_one.b - pixel_at_x_plus_one_y_minus_one.b,
				                                          alpha);

				gradient_pixel_out.r = (gradient_at_x_minus_one_column.r + 2 * gradient_at_column_x.r + gradient_at_x_plus_one_row.r) / 8;
				gradient_pixel_out.g = (gradient_at_x_minus_one_column.g + 2 * gradient_at_column_x.g + gradient_at_x_plus_one_row.g) / 8;
				gradient_pixel_out.b = (gradient_at_x_minus_one_column.b + 2 * gradient_at_column_x.b + gradient_at_x_plus_one_row.b) / 8;
				gradient_pixel_out.a = (gradient_at_x_minus_one_column.a + 2 * gradient_at_column_x.a + gradient_at_x_plus_one_row.a) / 8;
			},
			gradient_out
	);
}

//TODO: rename to SobelGradientXY
template<MemoryDeviceType TMemoryDeviceType>
void ImageProcessingEngine<TMemoryDeviceType>::GradientXY(Float2Image& gradient_out, const FloatImage& image_in) const {
	gradient_out.ChangeDims(image_in.dimensions);
	gradient_out.Clear();
	Vector2f* gradient_data = gradient_out.GetData(TMemoryDeviceType);
	const float* image_data = image_in.GetData(TMemoryDeviceType);
	const Vector2i image_dimensions = image_in.dimensions;
	const int image_width = image_dimensions.width;

	ImageTraversalEngine<TMemoryDeviceType>::TraversePositionOnly(
			[=]_DEVICE_WHEN_AVAILABLE_(int index, int x, int y) {
				if (x < 1 || x > image_width - 2 || y < 1 || y > image_dimensions.height - 2) return;
				Vector2f& gradient_pixel_out = gradient_data[index];

				const float& pixel_at_x_plus_one_y_plus_one = image_data[index + image_width + 1];
				const float& pixel_at_x_plus_one_y = image_data[index + 1];
				const float& pixel_at_x_plus_one_y_minus_one = image_data[index - image_width + 1];

				const float& pixel_at_x_y_plus_one = image_data[index + image_width];
				const float& pixel_at_x_y_minus_one = image_data[index - image_width];

				const float& pixel_at_x_minus_one_y_plus_one = image_data[index + image_width - 1];
				const float& pixel_at_x_minus_one_y = image_data[index - 1];
				const float& pixel_at_x_minus_one_y_minus_one = image_data[index - image_width - 1];

				const float dx_below = pixel_at_x_plus_one_y_minus_one - pixel_at_x_minus_one_y_minus_one;
				const float dx_center = pixel_at_x_plus_one_y - pixel_at_x_minus_one_y;
				const float dx_above = pixel_at_x_plus_one_y_plus_one - pixel_at_x_minus_one_y_plus_one;

				const float dy_left = pixel_at_x_minus_one_y_plus_one - pixel_at_x_minus_one_y_minus_one;
				const float dy_center = pixel_at_x_y_plus_one - pixel_at_x_y_minus_one;
				const float dy_right = pixel_at_x_plus_one_y_plus_one - pixel_at_x_plus_one_y_minus_one;

				gradient_pixel_out.x = (dx_below + 2.f * dx_center + dx_above) / 8.f;
				gradient_pixel_out.y = (dy_left + 2.f * dy_center + dy_right) / 8.f;
			},
			gradient_out
	);
}

template<MemoryDeviceType TMemoryDeviceType>
int ImageProcessingEngine<TMemoryDeviceType>::CountValidDepths(const FloatImage& image_in) const {
	return internal::CountValidDepths<TMemoryDeviceType>(image_in);
}
