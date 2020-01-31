// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../Utils/ITMMath.h"
#include "../../../ORUtils/Image.h"

namespace ITMLib
{
	class PointCloud
	{
	public:
		uint noTotalPoints;

		ORUtils::Image<Vector4f> *locations, *colours;

		explicit PointCloud(Vector2i imgSize, MemoryDeviceType memoryType)
		{
			this->noTotalPoints = 0;

			locations = new ORUtils::Image<Vector4f>(imgSize, memoryType);
			colours = new ORUtils::Image<Vector4f>(imgSize, memoryType);
		}

		void UpdateHostFromDevice()
		{
			this->locations->UpdateHostFromDevice();
			this->colours->UpdateHostFromDevice();
		}

		void UpdateDeviceFromHost()
		{
			this->locations->UpdateDeviceFromHost();
			this->colours->UpdateDeviceFromHost();
		}

		~PointCloud()
		{
			delete locations;
			delete colours;
		}

		// Suppress the default copy constructor and assignment operator
		PointCloud(const PointCloud&);
		PointCloud& operator=(const PointCloud&);
	};
}
