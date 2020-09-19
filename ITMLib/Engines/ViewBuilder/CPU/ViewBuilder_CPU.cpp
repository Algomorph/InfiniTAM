// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "ViewBuilder_CPU.h"

#include "../Shared/ViewBuilder_Shared.h"
#include "../../../../ORUtils/MetalContext.h"

using namespace ITMLib;
using namespace ORUtils;

ViewBuilder_CPU::ViewBuilder_CPU(const RGBD_CalibrationInformation& calib) : ViewBuilder(calib) {}

ViewBuilder_CPU::~ViewBuilder_CPU() {}

void ViewBuilder_CPU::UpdateView(View** view_ptr, UChar4Image* rgbImage, ShortImage* rawDepthImage, bool useThresholdFilter,
                                 bool useBilateralFilter, bool modelSensorNoise, bool storePreviousImage) {
	if (*view_ptr == nullptr) {
		*view_ptr = new View(calibration_information, rgbImage->dimensions, rawDepthImage->dimensions, false);
		if (modelSensorNoise) {
			(*view_ptr)->depth_normal = new Float4Image(rawDepthImage->dimensions, true, false);
			(*view_ptr)->depth_uncertainty = new FloatImage(rawDepthImage->dimensions, true, false);
		}
	}
	View* view = *view_ptr;

	if (storePreviousImage) {
		if (!view->rgb_prev) view->rgb_prev = new UChar4Image(rgbImage->dimensions, true, false);
		else view->rgb_prev->SetFrom(view->rgb, MemoryCopyDirection::CPU_TO_CPU);
	}

	view->rgb.SetFrom(*rgbImage, MemoryCopyDirection::CPU_TO_CPU);
	view->short_raw_disparity_image.SetFrom(*rawDepthImage, MemoryCopyDirection::CPU_TO_CPU);

	switch (view->calibration_information.disparityCalib.GetType()) {
		case DisparityCalib::TRAFO_KINECT:
			this->ConvertDisparityToDepth(view->depth, view->short_raw_disparity_image, view->calibration_information.intrinsics_d,
			                              view->calibration_information.disparityCalib.GetParams());
			break;
		case DisparityCalib::TRAFO_AFFINE:
			this->ConvertDepthAffineToFloat(view->depth, view->short_raw_disparity_image, view->calibration_information.disparityCalib.GetParams());
			break;
		default:
			break;
	}

	if (useThresholdFilter) {
		this->ThresholdFiltering(view->float_raw_disparity_image, view->depth);
		view->depth.SetFrom(view->float_raw_disparity_image, MemoryCopyDirection::CPU_TO_CPU);
	}

	if (useBilateralFilter) {
		//5 steps of bilateral filtering
		this->DepthFiltering(view->float_raw_disparity_image, view->depth);
		this->DepthFiltering(view->depth, view->float_raw_disparity_image);
		this->DepthFiltering(view->float_raw_disparity_image, view->depth);
		this->DepthFiltering(view->depth, view->float_raw_disparity_image);
		this->DepthFiltering(view->float_raw_disparity_image, view->depth);
		view->depth.SetFrom(view->float_raw_disparity_image, MemoryCopyDirection::CPU_TO_CPU);
	}

	if (useThresholdFilter) {
		this->ThresholdFiltering(view->float_raw_disparity_image, view->depth);
		view->depth.SetFrom(view->float_raw_disparity_image, MemoryCopyDirection::CPU_TO_CPU);
	}

	if (modelSensorNoise) {
		this->ComputeNormalAndWeights(*view->depth_normal, *view->depth_uncertainty, view->depth,
		                              view->calibration_information.intrinsics_d.projectionParamsSimple.all);
	}
}

void ViewBuilder_CPU::UpdateView(View** view_ptr, UChar4Image* rgbImage, ShortImage* depthImage, bool useThresholdFilter,
                                 bool useBilateralFilter, IMUMeasurement* imuMeasurement, bool modelSensorNoise,
                                 bool storePreviousImage) {
	if (*view_ptr == nullptr) {
		*view_ptr = new ViewIMU(calibration_information, rgbImage->dimensions, depthImage->dimensions, false);

		if (modelSensorNoise) {
			(*view_ptr)->depth_normal = new Float4Image(depthImage->dimensions, true, false);
			(*view_ptr)->depth_uncertainty = new FloatImage(depthImage->dimensions, true, false);
		}
	}

	ViewIMU* imuView = (ViewIMU*) (*view_ptr);
	imuView->imu->SetFrom(imuMeasurement);

	this->UpdateView(view_ptr, rgbImage, depthImage, false, useBilateralFilter, modelSensorNoise, storePreviousImage);
}

void ViewBuilder_CPU::ConvertDisparityToDepth(FloatImage& depth_out, const ShortImage& depth_in, const Intrinsics& depthIntrinsics,
                                              Vector2f disparityCalibParams) {
	auto image_dimensions = depth_in.dimensions;

	const short* d_in = depth_in.GetData(MEMORYDEVICE_CPU);
	float* d_out = depth_out.GetData(MEMORYDEVICE_CPU);

	float fx_depth = depthIntrinsics.projectionParamsSimple.fx;

	for (int y = 0; y < image_dimensions.y; y++)
		for (int x = 0; x < image_dimensions.x; x++)
			convertDisparityToDepth(d_out, x, y, d_in, disparityCalibParams, fx_depth, image_dimensions);
}

void ViewBuilder_CPU::ConvertDepthAffineToFloat(FloatImage& depth_out, const ShortImage& depth_in, const Vector2f depthCalibParams) {
	auto image_dimensions = depth_in.dimensions;

	const short* d_in = depth_in.GetData(MEMORYDEVICE_CPU);
	float* d_out = depth_out.GetData(MEMORYDEVICE_CPU);

	for (int y = 0; y < image_dimensions.y; y++) {
		for (int x = 0; x < image_dimensions.x; x++) {
			convertDepthAffineToFloat(d_out, x, y, d_in, image_dimensions, depthCalibParams);
		}
	}
}

void ViewBuilder_CPU::DepthFiltering(FloatImage& image_out, const FloatImage& image_in) {
	Vector2i imgSize = image_in.dimensions;

	image_out.Clear();

	float* imout = image_out.GetData(MEMORYDEVICE_CPU);
	const float* imin = image_in.GetData(MEMORYDEVICE_CPU);

	for (int y = 2; y < imgSize.y - 2; y++)
		for (int x = 2; x < imgSize.x - 2; x++)
			filterDepth(imout, imin, x, y, imgSize);
}

void ViewBuilder_CPU::ComputeNormalAndWeights(Float4Image& normal_out, FloatImage& sigma_z_out, const FloatImage& depth_in,
                                              Vector4f camera_projection_parameters) {
	Vector2i imgDims = depth_in.dimensions;

	const float* depth_data_in = depth_in.GetData(MEMORYDEVICE_CPU);

	float* sigma_z_data_out = sigma_z_out.GetData(MEMORYDEVICE_CPU);
	Vector4f* normal_data_out = normal_out.GetData(MEMORYDEVICE_CPU);

	for (int y = 2; y < imgDims.y - 2; y++)
		for (int x = 2; x < imgDims.x - 2; x++)
			computeNormalAndWeight(depth_data_in, normal_data_out, sigma_z_data_out, x, y, imgDims, camera_projection_parameters);
}

void ViewBuilder_CPU::ThresholdFiltering(FloatImage& image_out, const FloatImage& image_in) {
	Vector2i imgSize = image_in.dimensions;

	image_out.Clear();

	float* imout = image_out.GetData(MEMORYDEVICE_CPU);
	const float* imin = image_in.GetData(MEMORYDEVICE_CPU);

	for (int y = 2; y < imgSize.y - 2; y++)
		for (int x = 2; x < imgSize.x - 2; x++)
			thresholdDepth(imout, imin, x, y, imgSize);
}

