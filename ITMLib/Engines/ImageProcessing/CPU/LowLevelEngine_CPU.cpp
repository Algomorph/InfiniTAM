// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "LowLevelEngine_CPU.h"

#include "../Shared/LowLevelEngine_Shared.h"

using namespace ITMLib;

LowLevelEngine_CPU::LowLevelEngine_CPU() { }
LowLevelEngine_CPU::~LowLevelEngine_CPU() { }

void LowLevelEngine_CPU::CopyImage(UChar4Image& image_out, const UChar4Image& image_in) const
{
	Vector4u *dest = image_out.GetData(MEMORYDEVICE_CPU);
	const Vector4u *src = image_in.GetData(MEMORYDEVICE_CPU);

	memcpy(dest, src, image_in.size() * sizeof(Vector4u));
}

void LowLevelEngine_CPU::CopyImage(FloatImage& image_out, const FloatImage& image_in) const
{
	float *dest = image_out.GetData(MEMORYDEVICE_CPU);
	const float *src = image_in.GetData(MEMORYDEVICE_CPU);

	memcpy(dest, src, image_in.size() * sizeof(float));
}

void LowLevelEngine_CPU::CopyImage(Float4Image& image_out, const Float4Image& image_in) const
{
	Vector4f *dest = image_out.GetData(MEMORYDEVICE_CPU);
	const Vector4f *src = image_in.GetData(MEMORYDEVICE_CPU);

	memcpy(dest, src, image_in.size() * sizeof(Vector4f));
}

void LowLevelEngine_CPU::ConvertColorToIntensity(FloatImage& image_out, const UChar4Image& image_in) const
{
	const Vector2i dims = image_in.dimensions;
	image_out.ChangeDims(dims);

	float *dest = image_out.GetData(MEMORYDEVICE_CPU);
	const Vector4u *src = image_in.GetData(MEMORYDEVICE_CPU);

	for (int y = 0; y < dims.y; y++) for (int x = 0; x < dims.x; x++)
		convertColourToIntensity(dest, x, y, dims, src);
}

void LowLevelEngine_CPU::FilterIntensity(FloatImage& image_out, const FloatImage& image_in) const
{
	Vector2i dims = image_in.dimensions;

	image_out.ChangeDims(dims);
	image_out.Clear(0);

	const float *imageData_in = image_in.GetData(MEMORYDEVICE_CPU);
	float *imageData_out = image_out.GetData(MEMORYDEVICE_CPU);

	for (int y = 2; y < dims.y - 2; y++) for (int x = 2; x < dims.x - 2; x++)
		boxFilter2x2(imageData_out, x, y, dims, imageData_in, x, y, dims);
}

void LowLevelEngine_CPU::FilterSubsample(UChar4Image& image_out, const UChar4Image& image_in) const
{
	Vector2i oldDims = image_in.dimensions;
	Vector2i newDims; newDims.x = image_in.dimensions.x / 2; newDims.y = image_in.dimensions.y / 2;

	image_out.ChangeDims(newDims);

	const Vector4u *imageData_in = image_in.GetData(MEMORYDEVICE_CPU);
	Vector4u *imageData_out = image_out.GetData(MEMORYDEVICE_CPU);

	for (int y = 0; y < newDims.y; y++) for (int x = 0; x < newDims.x; x++)
		filterSubsample(imageData_out, x, y, newDims, imageData_in, oldDims);
}

void LowLevelEngine_CPU::FilterSubsample(FloatImage& image_out, const FloatImage& image_in) const
{
	Vector2i oldDims = image_in.dimensions;
	Vector2i newDims(image_in.dimensions.x / 2, image_in.dimensions.y / 2);

	image_out.ChangeDims(newDims);
	image_out.Clear();

	const float *imageData_in = image_in.GetData(MEMORYDEVICE_CPU);
	float *imageData_out = image_out.GetData(MEMORYDEVICE_CPU);

	for (int y = 1; y < newDims.y - 1; y++) for (int x = 1; x < newDims.x - 1; x++)
		boxFilter2x2(imageData_out, x, y, newDims, imageData_in, x * 2, y * 2, oldDims);
}

void LowLevelEngine_CPU::FilterSubsampleWithHoles(FloatImage& image_out, const FloatImage& image_in) const
{
	Vector2i oldDims = image_in.dimensions;
	Vector2i newDims; newDims.x = image_in.dimensions.x / 2; newDims.y = image_in.dimensions.y / 2;

	image_out.ChangeDims(newDims);

	const float *imageData_in = image_in.GetData(MEMORYDEVICE_CPU);
	float *imageData_out = image_out.GetData(MEMORYDEVICE_CPU);

	for (int y = 0; y < newDims.y; y++) for (int x = 0; x < newDims.x; x++)
		filterSubsampleWithHoles(imageData_out, x, y, newDims, imageData_in, oldDims);
}

void LowLevelEngine_CPU::FilterSubsampleWithHoles(Float4Image& image_out, const Float4Image& image_in) const
{
	Vector2i oldDims = image_in.dimensions;
	Vector2i newDims; newDims.x = image_in.dimensions.x / 2; newDims.y = image_in.dimensions.y / 2;

	image_out.ChangeDims(newDims);

	const Vector4f *imageData_in = image_in.GetData(MEMORYDEVICE_CPU);
	Vector4f *imageData_out = image_out.GetData(MEMORYDEVICE_CPU);

	for (int y = 0; y < newDims.y; y++) for (int x = 0; x < newDims.x; x++)
		filterSubsampleWithHoles(imageData_out, x, y, newDims, imageData_in, oldDims);
}

void LowLevelEngine_CPU::GradientX(Short4Image& grad_out, const UChar4Image& image_in) const
{
	grad_out.ChangeDims(image_in.dimensions);
	Vector2i imgSize = image_in.dimensions;

	Vector4s *grad = grad_out.GetData(MEMORYDEVICE_CPU);
	const Vector4u *image = image_in.GetData(MEMORYDEVICE_CPU);

	memset(grad, 0, imgSize.x * imgSize.y * sizeof(Vector4s));

	for (int y = 1; y < imgSize.y - 1; y++) for (int x = 1; x < imgSize.x - 1; x++)
		gradientX(grad, x, y, image, imgSize);
}

void LowLevelEngine_CPU::GradientY(Short4Image& grad_out, const UChar4Image& image_in) const
{
	grad_out.ChangeDims(image_in.dimensions);
	Vector2i imgSize = image_in.dimensions;

	Vector4s *grad = grad_out.GetData(MEMORYDEVICE_CPU);
	const Vector4u *image = image_in.GetData(MEMORYDEVICE_CPU);

	memset(grad, 0, imgSize.x * imgSize.y * sizeof(Vector4s));

	for (int y = 1; y < imgSize.y - 1; y++) for (int x = 1; x < imgSize.x - 1; x++)
		gradientY(grad, x, y, image, imgSize);
}

void LowLevelEngine_CPU::GradientXY(Float2Image& grad_out, const FloatImage& image_in) const
{
	Vector2i imgSize = image_in.dimensions;
	grad_out.ChangeDims(imgSize);
	grad_out.Clear();

	Vector2f *grad = grad_out.GetData(MEMORYDEVICE_CPU);
	const float *image = image_in.GetData(MEMORYDEVICE_CPU);

	for (int y = 1; y < imgSize.y - 1; y++) for (int x = 1; x < imgSize.x - 1; x++)
		gradientXY(grad, x, y, image, imgSize);
}

int LowLevelEngine_CPU::CountValidDepths(const FloatImage& image_in) const
{
	int noValidPoints = 0;
	const float *imageData_in = image_in.GetData(MEMORYDEVICE_CPU);

	for (int i = 0; i < image_in.dimensions.x * image_in.dimensions.y; ++i) if (imageData_in[i] > 0.0) noValidPoints++;

	return noValidPoints;
}
