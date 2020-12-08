// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Interface/ViewBuilder.h"

namespace ITMLib
{
	class ViewBuilder_CUDA : public ViewBuilder
	{
	public:
		void ConvertDisparityToDepth(FloatImage& depth_out, const ShortImage& depth_in, const Intrinsics& depth_camera_intrinsics,
		                             Vector2f disparity_calibration_parameters);
		void ConvertDepthAffineToFloat(FloatImage& depth_out, const ShortImage& depth_in, Vector2f depth_calibration_parameters);

		void ThresholdFiltering(FloatImage& image_out, const FloatImage& image_in) override;
		void DepthFiltering(FloatImage& image_out, const FloatImage& image_in);
		void ComputeNormalAndWeights(Float4Image& normal_out, FloatImage& sigma_z_out, const FloatImage& depth_in, Vector4f depth_camera_projection_parameters);

		void UpdateView(View** view, UChar4Image* rgbImage, ShortImage* raw_depth_image, bool useThresholdFilter,
		                bool useBilateralFilter, bool modelSensorNoise, bool storePreviousImage);
		void UpdateView(View** view, UChar4Image* rgbImage, ShortImage* depthImage, bool useThresholdFilter,
		                bool useBilateralFilter, IMUMeasurement* imuMeasurement, bool modelSensorNoise,
		                bool storePreviousImage);

		ViewBuilder_CUDA(const RGBD_CalibrationInformation& calib);
		~ViewBuilder_CUDA();
	};
}
