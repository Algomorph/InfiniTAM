// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "ViewBuilder_CPU.h"

#include "../Shared/ViewBuilder_Shared.h"
#include "../../../../ORUtils/MetalContext.h"

using namespace ITMLib;
using namespace ORUtils;

ViewBuilder_CPU::ViewBuilder_CPU(const RGBDCalib& calib): ViewBuilder(calib) { }
ViewBuilder_CPU::~ViewBuilder_CPU() { }

void ViewBuilder_CPU::UpdateView(View** view_ptr, ITMUChar4Image* rgbImage, ITMShortImage* rawDepthImage, bool useThresholdFilter,
                                 bool useBilateralFilter, bool modelSensorNoise, bool storePreviousImage)
{
	if (*view_ptr == NULL)
	{
		*view_ptr = new View(calib, rgbImage->dimensions, rawDepthImage->dimensions, false);
		//TODO: This is very bad coding practice, assumes that there's ever only one ViewBuilder updating a single view... \
		// Most likely, these "shortImage" and "floatImage" should be a part of View itself, while this class should have no state but parameters
		if (this->shortImage != NULL) delete this->shortImage;
		this->shortImage = new ITMShortImage(rawDepthImage->dimensions, true, false);
		if (this->floatImage != NULL) delete this->floatImage;
		this->floatImage = new ITMFloatImage(rawDepthImage->dimensions, true, false);

		if (modelSensorNoise)
		{
			(*view_ptr)->depthNormal = new ITMFloat4Image(rawDepthImage->dimensions, true, false);
			(*view_ptr)->depthUncertainty = new ITMFloatImage(rawDepthImage->dimensions, true, false);
		}
	}
	View *view = *view_ptr;

	if (storePreviousImage)
	{
		if (!view->rgb_prev) view->rgb_prev = new ITMUChar4Image(rgbImage->dimensions, true, false);
		else view->rgb_prev->SetFrom(*view->rgb, MemoryCopyDirection::CPU_TO_CPU);
	}

	view->rgb->SetFrom(*rgbImage, MemoryCopyDirection::CPU_TO_CPU);
	this->shortImage->SetFrom(*rawDepthImage, MemoryCopyDirection::CPU_TO_CPU);

	switch (view->calib.disparityCalib.GetType())
	{
	case DisparityCalib::TRAFO_KINECT:
		this->ConvertDisparityToDepth(view->depth, this->shortImage, &(view->calib.intrinsics_d), view->calib.disparityCalib.GetParams());
		break;
	case DisparityCalib::TRAFO_AFFINE:
		this->ConvertDepthAffineToFloat(view->depth, this->shortImage, view->calib.disparityCalib.GetParams());
		break;
	default:
		break;
	}

	if (useThresholdFilter){
		this->ThresholdFiltering(this->floatImage, view->depth);
		view->depth->SetFrom(*this->floatImage,MemoryCopyDirection::CPU_TO_CPU);
	}

	if (useBilateralFilter)
	{
		//5 steps of bilateral filtering
		this->DepthFiltering(this->floatImage, view->depth);
		this->DepthFiltering(view->depth, this->floatImage);
		this->DepthFiltering(this->floatImage, view->depth);
		this->DepthFiltering(view->depth, this->floatImage);
		this->DepthFiltering(this->floatImage, view->depth);
		view->depth->SetFrom(*this->floatImage, MemoryCopyDirection::CPU_TO_CPU);
	}

	if (modelSensorNoise)
	{
		this->ComputeNormalAndWeights(view->depthNormal, view->depthUncertainty, view->depth, view->calib.intrinsics_d.projectionParamsSimple.all);
	}
}

void ViewBuilder_CPU::UpdateView(View** view_ptr, ITMUChar4Image* rgbImage, ITMShortImage* depthImage, bool useThresholdFilter,
                                 bool useBilateralFilter, IMUMeasurement* imuMeasurement, bool modelSensorNoise,
                                 bool storePreviousImage)
{
	if (*view_ptr == NULL)
	{
		*view_ptr = new ViewIMU(calib, rgbImage->dimensions, depthImage->dimensions, false);
		if (this->shortImage != NULL) delete this->shortImage;
		this->shortImage = new ITMShortImage(depthImage->dimensions, true, false);
		if (this->floatImage != NULL) delete this->floatImage;
		this->floatImage = new ITMFloatImage(depthImage->dimensions, true, false);

		if (modelSensorNoise)
		{
			(*view_ptr)->depthNormal = new ITMFloat4Image(depthImage->dimensions, true, false);
			(*view_ptr)->depthUncertainty = new ITMFloatImage(depthImage->dimensions, true, false);
		}
	}

	ViewIMU* imuView = (ViewIMU*)(*view_ptr);
	imuView->imu->SetFrom(imuMeasurement);

	this->UpdateView(view_ptr, rgbImage, depthImage, false, useBilateralFilter, modelSensorNoise, storePreviousImage);
}

void ViewBuilder_CPU::ConvertDisparityToDepth(ITMFloatImage *depth_out, const ITMShortImage *depth_in, const Intrinsics *depthIntrinsics,
                                              Vector2f disparityCalibParams)
{
	Vector2i imgSize = depth_in->dimensions;

	const short *d_in = depth_in->GetData(MEMORYDEVICE_CPU);
	float *d_out = depth_out->GetData(MEMORYDEVICE_CPU);

	float fx_depth = depthIntrinsics->projectionParamsSimple.fx;

	for (int y = 0; y < imgSize.y; y++) for (int x = 0; x < imgSize.x; x++)
		convertDisparityToDepth(d_out, x, y, d_in, disparityCalibParams, fx_depth, imgSize);
}

void ViewBuilder_CPU::ConvertDepthAffineToFloat(ITMFloatImage *depth_out, const ITMShortImage *depth_in, const Vector2f depthCalibParams)
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

void ViewBuilder_CPU::DepthFiltering(ITMFloatImage *image_out, const ITMFloatImage *image_in)
{
	Vector2i imgSize = image_in->dimensions;

	image_out->Clear();

	float *imout = image_out->GetData(MEMORYDEVICE_CPU);
	const float *imin = image_in->GetData(MEMORYDEVICE_CPU);

	for (int y = 2; y < imgSize.y - 2; y++) for (int x = 2; x < imgSize.x - 2; x++)
		filterDepth(imout, imin, x, y, imgSize);
}

void ViewBuilder_CPU::ComputeNormalAndWeights(ITMFloat4Image *normal_out, ITMFloatImage *sigmaZ_out, const ITMFloatImage *depth_in, Vector4f intrinsic)
{
	Vector2i imgDims = depth_in->dimensions;

	const float *depthData_in = depth_in->GetData(MEMORYDEVICE_CPU);

	float *sigmaZData_out = sigmaZ_out->GetData(MEMORYDEVICE_CPU);
	Vector4f *normalData_out = normal_out->GetData(MEMORYDEVICE_CPU);

	for (int y = 2; y < imgDims.y - 2; y++) for (int x = 2; x < imgDims.x - 2; x++)
		computeNormalAndWeight(depthData_in, normalData_out, sigmaZData_out, x, y, imgDims, intrinsic);
}

void ViewBuilder_CPU::ThresholdFiltering(ITMFloatImage* image_out, const ITMFloatImage* image_in) {
	Vector2i imgSize = image_in->dimensions;

	image_out->Clear();

	float *imout = image_out->GetData(MEMORYDEVICE_CPU);
	const float *imin = image_in->GetData(MEMORYDEVICE_CPU);

	for (int y = 2; y < imgSize.y - 2; y++) for (int x = 2; x < imgSize.x - 2; x++)
			thresholdDepth(imout, imin, x, y, imgSize);

}

