// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "ViewBuilder_CPU.h"

#include "../Shared/ViewBuilder_Shared.h"
#include "../../../../ORUtils/MetalContext.h"

using namespace ITMLib;
using namespace ORUtils;

ViewBuilder_CPU::ViewBuilder_CPU(const RGBDCalib& calib): ViewBuilder(calib) { }
ViewBuilder_CPU::~ViewBuilder_CPU() { }

void ViewBuilder_CPU::UpdateView(View** view_ptr, UChar4Image* rgbImage, ShortImage* rawDepthImage, bool useThresholdFilter,
                                 bool useBilateralFilter, bool modelSensorNoise, bool storePreviousImage)
{
	if (*view_ptr == nullptr)
	{
		*view_ptr = new View(calibration_information, rgbImage->dimensions, rawDepthImage->dimensions, false);
		//TODO: This is very bad coding practice, assumes that there's ever only one ViewBuilder updating a single view... \
		// Most likely, these "short_raw_disparity_image" and "float_raw_disparity_image" should be a part of View itself, while this class should have no state but parameters
		if (this->short_raw_disparity_image != NULL) delete this->short_raw_disparity_image;
		this->short_raw_disparity_image = new ShortImage(rawDepthImage->dimensions, true, false);
		if (this->float_raw_disparity_image != NULL) delete this->float_raw_disparity_image;
		this->float_raw_disparity_image = new FloatImage(rawDepthImage->dimensions, true, false);

		if (modelSensorNoise)
		{
			(*view_ptr)->depth_normal = new Float4Image(rawDepthImage->dimensions, true, false);
			(*view_ptr)->depthUncertainty = new FloatImage(rawDepthImage->dimensions, true, false);
		}
	}
	View *view = *view_ptr;

	if (storePreviousImage)
	{
		if (!view->rgb_prev) view->rgb_prev = new UChar4Image(rgbImage->dimensions, true, false);
		else view->rgb_prev->SetFrom(*view->rgb, MemoryCopyDirection::CPU_TO_CPU);
	}

	view->rgb->SetFrom(*rgbImage, MemoryCopyDirection::CPU_TO_CPU);
	this->short_raw_disparity_image->SetFrom(*rawDepthImage, MemoryCopyDirection::CPU_TO_CPU);

	switch (view->calibration_information.disparityCalib.GetType())
	{
	case DisparityCalib::TRAFO_KINECT:
		this->ConvertDisparityToDepth(view->depth, this->short_raw_disparity_image, &(view->calibration_information.intrinsics_d), view->calibration_information.disparityCalib.GetParams());
		break;
	case DisparityCalib::TRAFO_AFFINE:
		this->ConvertDepthAffineToFloat(view->depth, this->short_raw_disparity_image, view->calibration_information.disparityCalib.GetParams());
		break;
	default:
		break;
	}

	if (useThresholdFilter){
		this->ThresholdFiltering(this->float_raw_disparity_image, view->depth);
		view->depth->SetFrom(*this->float_raw_disparity_image, MemoryCopyDirection::CPU_TO_CPU);
	}

	if (useBilateralFilter)
	{
		//5 steps of bilateral filtering
		this->DepthFiltering(this->float_raw_disparity_image, view->depth);
		this->DepthFiltering(view->depth, this->float_raw_disparity_image);
		this->DepthFiltering(this->float_raw_disparity_image, view->depth);
		this->DepthFiltering(view->depth, this->float_raw_disparity_image);
		this->DepthFiltering(this->float_raw_disparity_image, view->depth);
		view->depth->SetFrom(*this->float_raw_disparity_image, MemoryCopyDirection::CPU_TO_CPU);
	}

	if (modelSensorNoise)
	{
		this->ComputeNormalAndWeights(view->depth_normal, view->depthUncertainty, view->depth, view->calibration_information.intrinsics_d.projectionParamsSimple.all);
	}
}

void ViewBuilder_CPU::UpdateView(View** view_ptr, UChar4Image* rgbImage, ShortImage* depthImage, bool useThresholdFilter,
                                 bool useBilateralFilter, IMUMeasurement* imuMeasurement, bool modelSensorNoise,
                                 bool storePreviousImage)
{
	if (*view_ptr == NULL)
	{
		*view_ptr = new ViewIMU(calibration_information, rgbImage->dimensions, depthImage->dimensions, false);
		if (this->short_raw_disparity_image != NULL) delete this->short_raw_disparity_image;
		this->short_raw_disparity_image = new ShortImage(depthImage->dimensions, true, false);
		if (this->float_raw_disparity_image != NULL) delete this->float_raw_disparity_image;
		this->float_raw_disparity_image = new FloatImage(depthImage->dimensions, true, false);

		if (modelSensorNoise)
		{
			(*view_ptr)->depth_normal = new Float4Image(depthImage->dimensions, true, false);
			(*view_ptr)->depthUncertainty = new FloatImage(depthImage->dimensions, true, false);
		}
	}

	ViewIMU* imuView = (ViewIMU*)(*view_ptr);
	imuView->imu->SetFrom(imuMeasurement);

	this->UpdateView(view_ptr, rgbImage, depthImage, false, useBilateralFilter, modelSensorNoise, storePreviousImage);
}

void ViewBuilder_CPU::ConvertDisparityToDepth(FloatImage *depth_out, const ShortImage *depth_in, const Intrinsics *depthIntrinsics,
                                              Vector2f disparityCalibParams)
{
	Vector2i imgSize = depth_in->dimensions;

	const short *d_in = depth_in->GetData(MEMORYDEVICE_CPU);
	float *d_out = depth_out->GetData(MEMORYDEVICE_CPU);

	float fx_depth = depthIntrinsics->projectionParamsSimple.fx;

	for (int y = 0; y < imgSize.y; y++) for (int x = 0; x < imgSize.x; x++)
		convertDisparityToDepth(d_out, x, y, d_in, disparityCalibParams, fx_depth, imgSize);
}

void ViewBuilder_CPU::ConvertDepthAffineToFloat(FloatImage *depth_out, const ShortImage *depth_in, const Vector2f depthCalibParams)
{
	Vector2i imgSize = depth_in->dimensions;

	const short *d_in = depth_in->GetData(MEMORYDEVICE_CPU);
	float *d_out = depth_out->GetData(MEMORYDEVICE_CPU);

	for (int y = 0; y < imgSize.y; y++){
		for (int x = 0; x < imgSize.x; x++){
			convertDepthAffineToFloat(d_out, x, y, d_in, imgSize, depthCalibParams);
		}
	}
}

void ViewBuilder_CPU::DepthFiltering(FloatImage *image_out, const FloatImage *image_in)
{
	Vector2i imgSize = image_in->dimensions;

	image_out->Clear();

	float *imout = image_out->GetData(MEMORYDEVICE_CPU);
	const float *imin = image_in->GetData(MEMORYDEVICE_CPU);

	for (int y = 2; y < imgSize.y - 2; y++) for (int x = 2; x < imgSize.x - 2; x++)
		filterDepth(imout, imin, x, y, imgSize);
}

void ViewBuilder_CPU::ComputeNormalAndWeights(Float4Image *normal_out, FloatImage *sigmaZ_out, const FloatImage *depth_in, Vector4f intrinsic)
{
	Vector2i imgDims = depth_in->dimensions;

	const float *depthData_in = depth_in->GetData(MEMORYDEVICE_CPU);

	float *sigmaZData_out = sigmaZ_out->GetData(MEMORYDEVICE_CPU);
	Vector4f *normalData_out = normal_out->GetData(MEMORYDEVICE_CPU);

	for (int y = 2; y < imgDims.y - 2; y++) for (int x = 2; x < imgDims.x - 2; x++)
		computeNormalAndWeight(depthData_in, normalData_out, sigmaZData_out, x, y, imgDims, intrinsic);
}

void ViewBuilder_CPU::ThresholdFiltering(FloatImage* image_out, const FloatImage* image_in) {
	Vector2i imgSize = image_in->dimensions;

	image_out->Clear();

	float *imout = image_out->GetData(MEMORYDEVICE_CPU);
	const float *imin = image_in->GetData(MEMORYDEVICE_CPU);

	for (int y = 2; y < imgSize.y - 2; y++) for (int x = 2; x < imgSize.x - 2; x++)
			thresholdDepth(imout, imin, x, y, imgSize);

}

