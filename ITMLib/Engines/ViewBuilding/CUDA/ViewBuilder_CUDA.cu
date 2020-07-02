// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "ViewBuilder_CUDA.h"

#include "../Shared/ViewBuilder_Shared.h"
#include "../../../../ORUtils/CUDADefines.h"
#include "../../../../ORUtils/MemoryBlock.h"

using namespace ITMLib;
using namespace ORUtils;

ViewBuilder_CUDA::ViewBuilder_CUDA(const RGBDCalib& calib): ViewBuilder(calib) { }
ViewBuilder_CUDA::~ViewBuilder_CUDA() { }

//---------------------------------------------------------------------------
//
// kernel function declaration 
//
//---------------------------------------------------------------------------


__global__ void convertDisparityToDepth_device(float *depth_out, const short *depth_in, Vector2f disparityCalibParams, float fx_depth, Vector2i imgSize);
__global__ void convertDepthAffineToFloat_device(float *d_out, const short *d_in, Vector2i imgSize, Vector2f depthCalibParams);
__global__ void filterDepth_device(float *imageData_out, const float *imageData_in, Vector2i imgDims);
__global__ void thresholdDepth_device(float *imageData_out, const float *imageData_in, Vector2i imgDims);
__global__ void ComputeNormalAndWeight_device(const float* depth_in, Vector4f* normal_out, float *sigmaL_out, Vector2i imgDims, Vector4f intrinsic);

//---------------------------------------------------------------------------
//
// host methods
//
//---------------------------------------------------------------------------

void ViewBuilder_CUDA::UpdateView(View** view_ptr, UChar4Image* rgbImage, ShortImage* raw_depth_image, bool useThresholdFilter,
                                  bool useBilateralFilter, bool modelSensorNoise, bool storePreviousImage)
{
	if (*view_ptr == nullptr)
	{
		*view_ptr = new View(calibration_information, rgbImage->dimensions, raw_depth_image->dimensions, true);

		if (modelSensorNoise)
		{
			(*view_ptr)->depth_normal = new Float4Image(raw_depth_image->dimensions, true, true);
			(*view_ptr)->depth_uncertainty = new FloatImage(raw_depth_image->dimensions, true, true);
		}
	}

	View *view = *view_ptr;

	if (storePreviousImage)
	{
		if (!view->rgb_prev) view->rgb_prev = new UChar4Image(rgbImage->dimensions, true, true);
		else view->rgb_prev->SetFrom(view->rgb, MemoryCopyDirection::CUDA_TO_CUDA);
	}	

	view->rgb.SetFrom(*rgbImage, MemoryCopyDirection::CPU_TO_CUDA);
	view->short_raw_disparity_image.SetFrom(*raw_depth_image, MemoryCopyDirection::CPU_TO_CUDA);

	switch (view->calibration_information.disparityCalib.GetType())
	{
	case DisparityCalib::TRAFO_KINECT:
		this->ConvertDisparityToDepth(view->depth, view->short_raw_disparity_image, view->calibration_information.intrinsics_d, view->calibration_information.disparityCalib.GetParams());
		break;
	case DisparityCalib::TRAFO_AFFINE:
		this->ConvertDepthAffineToFloat(view->depth, view->short_raw_disparity_image, view->calibration_information.disparityCalib.GetParams());
		break;
	default:
		break;
	}

	if (useBilateralFilter)
	{
		//5 steps of bilateral filtering
		this->DepthFiltering(view->float_raw_disparity_image, view->depth);
		this->DepthFiltering(view->depth, view->float_raw_disparity_image);
		this->DepthFiltering(view->float_raw_disparity_image, view->depth);
		this->DepthFiltering(view->depth, view->float_raw_disparity_image);
		this->DepthFiltering(view->float_raw_disparity_image, view->depth);
		view->depth.SetFrom(view->float_raw_disparity_image, MemoryCopyDirection::CUDA_TO_CUDA);
	}

	if (modelSensorNoise)
	{
		this->ComputeNormalAndWeights(*view->depth_normal, *view->depth_uncertainty, view->depth, view->calibration_information.intrinsics_d.projectionParamsSimple.all);
	}
}

void ViewBuilder_CUDA::UpdateView(View** view_ptr, UChar4Image* rgbImage, ShortImage* depthImage, bool useThresholdFilter,
                                  bool useBilateralFilter, IMUMeasurement* imuMeasurement, bool modelSensorNoise,
                                  bool storePreviousImage)
{
	if (*view_ptr == nullptr) 
	{
		*view_ptr = new ViewIMU(calibration_information, rgbImage->dimensions, depthImage->dimensions, true);

		if (modelSensorNoise)
		{
			(*view_ptr)->depth_normal = new Float4Image(depthImage->dimensions, true, true);
			(*view_ptr)->depth_uncertainty = new FloatImage(depthImage->dimensions, true, true);
		}
	}

	ViewIMU* imuView = (ViewIMU*)(*view_ptr);
	imuView->imu->SetFrom(imuMeasurement);

	this->UpdateView(view_ptr, rgbImage, depthImage, false, useBilateralFilter, modelSensorNoise, storePreviousImage);
}

void ViewBuilder_CUDA::ConvertDisparityToDepth(FloatImage& depth_out, const ShortImage& depth_in, const Intrinsics& depth_camera_intrinsics,
                                               Vector2f disparity_calibration_parameters)
{
	Vector2i imgSize = depth_in.dimensions;

	const short *d_in = depth_in.GetData(MEMORYDEVICE_CUDA);
	float *d_out = depth_out.GetData(MEMORYDEVICE_CUDA);

	float fx_depth = depth_camera_intrinsics.projectionParamsSimple.fx;

	dim3 blockSize(16, 16);
	dim3 gridSize((int)ceil((float)imgSize.x / (float)blockSize.x), (int)ceil((float)imgSize.y / (float)blockSize.y));

	convertDisparityToDepth_device <<<gridSize, blockSize >>>(d_out, d_in, disparity_calibration_parameters, fx_depth, imgSize);
	ORcudaKernelCheck;
}

void ViewBuilder_CUDA::ConvertDepthAffineToFloat(FloatImage&depth_out, const ShortImage& depth_in, Vector2f depth_calibration_parameters)
{
	Vector2i imgSize = depth_in.dimensions;

	const short *d_in = depth_in.GetData(MEMORYDEVICE_CUDA);
	float *d_out = depth_out.GetData(MEMORYDEVICE_CUDA);

	dim3 blockSize(16, 16);
	dim3 gridSize((int)ceil((float)imgSize.x / (float)blockSize.x), (int)ceil((float)imgSize.y / (float)blockSize.y));

	convertDepthAffineToFloat_device <<<gridSize, blockSize >>>(d_out, d_in, imgSize, depth_calibration_parameters);
	ORcudaKernelCheck;
}

void ViewBuilder_CUDA::DepthFiltering(FloatImage& image_out, const FloatImage& image_in)
{
	Vector2i imgDims = image_in.dimensions;

	const float *imageData_in = image_in.GetData(MEMORYDEVICE_CUDA);
	float *imageData_out = image_out.GetData(MEMORYDEVICE_CUDA);

	dim3 blockSize(16, 16);
	dim3 gridSize((int)ceil((float)imgDims.x / (float)blockSize.x), (int)ceil((float)imgDims.y / (float)blockSize.y));

	filterDepth_device <<<gridSize, blockSize >>>(imageData_out, imageData_in, imgDims);
	ORcudaKernelCheck;
}

void ViewBuilder_CUDA::ComputeNormalAndWeights(Float4Image& normal_out, FloatImage& sigma_z_out, const FloatImage& depth_in, Vector4f depth_camera_projection_parameters)
{
	Vector2i imgDims = depth_in.dimensions;

	const float *depthData_in = depth_in.GetData(MEMORYDEVICE_CUDA);

	float *sigmaZData_out = sigma_z_out.GetData(MEMORYDEVICE_CUDA);
	Vector4f *normalData_out = normal_out.GetData(MEMORYDEVICE_CUDA);

	dim3 blockSize(16, 16);
	dim3 gridSize((int)ceil((float)imgDims.x / (float)blockSize.x), (int)ceil((float)imgDims.y / (float)blockSize.y));

	ComputeNormalAndWeight_device <<<gridSize, blockSize >>>(depthData_in, normalData_out, sigmaZData_out, imgDims, depth_camera_projection_parameters);
	ORcudaKernelCheck;
}

void ViewBuilder_CUDA::ThresholdFiltering(FloatImage& image_out, const FloatImage& image_in) {
	Vector2i imgDims = image_in.dimensions;

	const float *imageData_in = image_in.GetData(MEMORYDEVICE_CUDA);
	float *imageData_out = image_out.GetData(MEMORYDEVICE_CUDA);

	dim3 blockSize(16, 16);
	dim3 gridSize((int)ceil((float)imgDims.x / (float)blockSize.x), (int)ceil((float)imgDims.y / (float)blockSize.y));

	thresholdDepth_device <<<gridSize, blockSize >>>(imageData_out, imageData_in, imgDims);
	ORcudaKernelCheck;
}

//---------------------------------------------------------------------------
//
// kernel function implementation
//
//---------------------------------------------------------------------------

__global__ void convertDisparityToDepth_device(float *d_out, const short *d_in, Vector2f disparityCalibParams, float fx_depth, Vector2i imgSize)
{
	int x = threadIdx.x + blockIdx.x * blockDim.x;
	int y = threadIdx.y + blockIdx.y * blockDim.y;

	if ((x >= imgSize.x) || (y >= imgSize.y)) return;

	convertDisparityToDepth(d_out, x, y, d_in, disparityCalibParams, fx_depth, imgSize);
}

__global__ void convertDepthAffineToFloat_device(float *d_out, const short *d_in, Vector2i imgSize, Vector2f depthCalibParams)
{
	int x = threadIdx.x + blockIdx.x * blockDim.x;
	int y = threadIdx.y + blockIdx.y * blockDim.y;

	if ((x >= imgSize.x) || (y >= imgSize.y)) return;

	convertDepthAffineToFloat(d_out, x, y, d_in, imgSize, depthCalibParams);
}

__global__ void filterDepth_device(float *imageData_out, const float *imageData_in, Vector2i imgDims)
{
	int x = threadIdx.x + blockIdx.x * blockDim.x, y = threadIdx.y + blockIdx.y * blockDim.y;

	if (x < 2 || x > imgDims.x - 2 || y < 2 || y > imgDims.y - 2) return;

	filterDepth(imageData_out, imageData_in, x, y, imgDims);
}

__global__ void thresholdDepth_device(float *imageData_out, const float *imageData_in, Vector2i imgDims)
{
	int x = threadIdx.x + blockIdx.x * blockDim.x, y = threadIdx.y + blockIdx.y * blockDim.y;

	if (x < 2 || x > imgDims.x - 2 || y < 2 || y > imgDims.y - 2) return;

	thresholdDepth(imageData_out, imageData_in, x, y, imgDims);
}

__global__ void ComputeNormalAndWeight_device(const float* depth_in, Vector4f* normal_out, float *sigmaZ_out, Vector2i imgDims, Vector4f intrinsic)
{
	int x = threadIdx.x + blockIdx.x * blockDim.x, y = threadIdx.y + blockIdx.y * blockDim.y;
	int idx = x + y * imgDims.x;

	if (x < 2 || x > imgDims.x - 2 || y < 2 || y > imgDims.y - 2)
	{
		normal_out[idx].w = -1.0f;
		sigmaZ_out[idx] = -1;
		return;
	}
	else
	{
		computeNormalAndWeight(depth_in, normal_out, sigmaZ_out, x, y, imgDims, intrinsic);
	}
}

