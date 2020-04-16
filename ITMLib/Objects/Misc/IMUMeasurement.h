// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../Utils/Math.h"

namespace ITMLib
{
	class IMUMeasurement
	{
	public:
		Matrix3f R;

		IMUMeasurement()
		{
			this->R.setIdentity();
		}

		IMUMeasurement(const Matrix3f & R)
		{
			this->R = R;
		}

		void SetFrom(const IMUMeasurement *measurement)
		{
			this->R = measurement->R;
		}

		~IMUMeasurement() { }

		// Suppress the default copy constructor and assignment operator
		IMUMeasurement(const IMUMeasurement&);
		IMUMeasurement& operator=(const IMUMeasurement&);
	};
}
