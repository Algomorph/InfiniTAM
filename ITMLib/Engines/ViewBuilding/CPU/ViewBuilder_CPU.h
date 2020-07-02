// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Interface/ViewBuilder.h"

namespace ITMLib
{
	class ViewBuilder_CPU : public ViewBuilder
	{
	public:
		void ConvertDisparityToDepth(FloatImage& depth_out, const ShortImage& disp_in, const Intrinsics& depthIntrinsics,
		                             Vector2f disparityCalibParams);
		void ConvertDepthAffineToFloat(FloatImage& depth_out, const ShortImage& depth_in, Vector2f depthCalibParams);

		void ThresholdFiltering(FloatImage& image_out, const FloatImage& image_in) override;
		void DepthFiltering(FloatImage& image_out, const FloatImage& image_in);
		void ComputeNormalAndWeights(Float4Image& normal_out, FloatImage& sigma_z_out, const FloatImage& depth_in, Vector4f camera_projection_parameters);

		void UpdateView(View** view, UChar4Image* rgbImage, ShortImage* rawDepthImage, bool useThresholdFilter,
		                bool useBilateralFilter, bool modelSensorNoise, bool storePreviousImage);
		void UpdateView(View** view, UChar4Image* rgbImage, ShortImage* depthImage, bool useThresholdFilter,
		                bool useBilateralFilter, IMUMeasurement* imuMeasurement, bool modelSensorNoise,
		                bool storePreviousImage);

		ViewBuilder_CPU(const RGBDCalib& calib);
		~ViewBuilder_CPU();
	};
}

