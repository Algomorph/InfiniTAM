// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "View.h"
#include "../Misc/IMUMeasurement.h"

namespace ITMLib
{
	/** \brief
	    Represents a single "view", i.e. RGB and depth images along
	    with all intrinsic and relative calibration information
	*/
	class ViewIMU : public View
	{
	public:
		IMUMeasurement *imu;

		ViewIMU(const RGBDCalib& calibration, Vector2i imgSize_rgb, Vector2i imgSize_d, bool useGPU)
		 : View(calibration, imgSize_rgb, imgSize_d, useGPU)
		{
			imu = new IMUMeasurement();
		}

		~ViewIMU(void) { delete imu; }

		// Suppress the default copy constructor and assignment operator
		ViewIMU(const ViewIMU&);
		ViewIMU& operator=(const ViewIMU&);
	};
}
