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
		const RGBDCalib calib;
		ITMShortImage *shortImage;
		ITMFloatImage *floatImage;

	public:
		virtual void ConvertDisparityToDepth(ITMFloatImage *depth_out, const ITMShortImage *disp_in, const Intrinsics *depthIntrinsics,
			Vector2f disparityCalibParams) = 0;
		virtual void ConvertDepthAffineToFloat(ITMFloatImage *depth_out, const ITMShortImage *depth_in, Vector2f depthCalibParams) = 0;

		/** \brief Find discontinuities in a depth image by removing all pixels for
	     * which the maximum difference between the center pixel and it's neighbours
	     * is higher than a threshold
	     *  \param[in] image_out output image
	     *  \param[in] image_in input image
	     */
		virtual void ThresholdFiltering(ITMFloatImage *image_out, const ITMFloatImage *image_in) = 0;
		virtual void DepthFiltering(ITMFloatImage *image_out, const ITMFloatImage *image_in) = 0;
		virtual void ComputeNormalAndWeights(ITMFloat4Image *normal_out, ITMFloatImage *sigmaZ_out, const ITMFloatImage *depth_in, Vector4f intrinsic) = 0;

		virtual void UpdateView(View** view, ITMUChar4Image* rgbImage, ITMShortImage* rawDepthImage, bool useThresholdFilter,
		                        bool useBilateralFilter, bool modelSensorNoise, bool storePreviousImage) = 0;
		virtual void UpdateView(View** view, ITMUChar4Image* rgbImage, ITMShortImage* depthImage, bool useThresholdFilter,
		                        bool useBilateralFilter, IMUMeasurement* imuMeasurement, bool modelSensorNoise,
		                        bool storePreviousImage) = 0;

		ViewBuilder(const RGBDCalib& calib_)
		: calib(calib_)
		{
			this->shortImage = NULL;
			this->floatImage = NULL;
		}

		virtual ~ViewBuilder()
		{
			if (this->shortImage != NULL) delete this->shortImage;
			if (this->floatImage != NULL) delete this->floatImage;
		}
	};
}
