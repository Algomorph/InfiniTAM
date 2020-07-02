// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../../Objects/Camera/RGBDCalib.h"
#include "../../../Objects/Views/ViewIMU.h"

namespace ITMLib
{
	/** \brief
	*/
	class ViewBuilder
	{
	protected:
		const RGBDCalib calibration_information;
		ShortImage *short_raw_disparity_image;
		FloatImage *float_raw_disparity_image;

	public:
		virtual void ConvertDisparityToDepth(FloatImage *depth_out, const ShortImage *disp_in, const Intrinsics *depthIntrinsics,
		                                     Vector2f disparityCalibParams) = 0;
		virtual void ConvertDepthAffineToFloat(FloatImage *depth_out, const ShortImage *depth_in, Vector2f depthCalibParams) = 0;

		/** \brief Find discontinuities in a depth image by removing all pixels for
	     * which the maximum difference between the center pixel and it's neighbours
	     * is higher than a threshold
	     *  \param[in] image_out output image
	     *  \param[in] image_in input image
	     */
		virtual void ThresholdFiltering(FloatImage *image_out, const FloatImage *image_in) = 0;
		virtual void DepthFiltering(FloatImage *image_out, const FloatImage *image_in) = 0;
		virtual void ComputeNormalAndWeights(Float4Image *normal_out, FloatImage *sigmaZ_out, const FloatImage *depth_in, Vector4f intrinsic) = 0;

		virtual void UpdateView(View** view, UChar4Image* rgbImage, ShortImage* rawDepthImage, bool useThresholdFilter,
		                        bool useBilateralFilter, bool modelSensorNoise, bool storePreviousImage) = 0;
		virtual void UpdateView(View** view, UChar4Image* rgbImage, ShortImage* depthImage, bool useThresholdFilter,
		                        bool useBilateralFilter, IMUMeasurement* imuMeasurement, bool modelSensorNoise,
		                        bool storePreviousImage) = 0;

		ViewBuilder(const RGBDCalib& calib_)
		: calibration_information(calib_)
		{
			this->short_raw_disparity_image = NULL;
			this->float_raw_disparity_image = NULL;
		}

		virtual ~ViewBuilder()
		{
			if (this->short_raw_disparity_image != NULL) delete this->short_raw_disparity_image;
			if (this->float_raw_disparity_image != NULL) delete this->float_raw_disparity_image;
		}
	};
}
