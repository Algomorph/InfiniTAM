// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../ITMLib/Objects/Misc/IMUMeasurement.h"

namespace InputSource {

class IMUSourceEngine
{
private:
	static const int BUF_SIZE = 2048;
	char imuMask[BUF_SIZE];

	ITMLib::IMUMeasurement *cached_imu;

	void loadIMUIntoCache();
	int cachedFrameNo;
	int currentFrameNo;

public:
	IMUSourceEngine(const char *imuMask);
	~IMUSourceEngine() { }

	bool hasMoreMeasurements();
	void getMeasurement(ITMLib::IMUMeasurement *imu);
};

}
