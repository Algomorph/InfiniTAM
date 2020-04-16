// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Camera/CalibIO.h"
#include "../../Utils/ImageTypes.h"

namespace ITMLib
{
	/** \brief
	    Represents a single "view", i.e. RGB and depth images along
	    with all intrinsic and relative calibration information
	*/
	class View
	{
	public:
		/// Intrinsic calibration information for the view.
		const RGBDCalib calib;

		/// RGB colour image for the current frame.
		ITMUChar4Image *rgb; 

		/// RGB colour image for the previous frame.
		ITMUChar4Image *rgb_prev; 

		/// Float valued depth image, if available according to @ref inputImageType.
		ITMFloatImage *depth;

		/// surface normal of depth image
		// allocated when needed
		ITMFloat4Image *depthNormal;

		/// uncertainty (std) in each pixel of depth value based on sensor noise model
		/// allocated when needed
		ITMFloatImage *depthUncertainty;

		// confidence based on distance from center
		ITMFloatImage *depthConfidence;

		View(const RGBDCalib& calibration, Vector2i imgSize_rgb, Vector2i imgSize_d, bool useGPU)
		: calib(calibration)
		{
			this->rgb = new ITMUChar4Image(imgSize_rgb, true, useGPU);
			this->rgb_prev = NULL;
			this->depth = new ITMFloatImage(imgSize_d, true, useGPU);
			this->depthNormal = NULL;
			this->depthUncertainty = NULL;
			this->depthConfidence = new ITMFloatImage(imgSize_d, true, useGPU);
		}

		virtual ~View()
		{
			delete rgb;
			delete rgb_prev;

			delete depth;
			delete depthConfidence;

			delete depthNormal;
			delete depthUncertainty;
		}

		// Suppress the default copy constructor and assignment operator
		View(const View&);
		View& operator=(const View&);
	};
}
