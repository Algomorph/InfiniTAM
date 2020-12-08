// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../Utils/Math.h"
#include "../../../ORUtils/Image.h"

namespace ITMLib
{
	class PointCloud
	{
	public:
		uint point_count;

		ORUtils::Image<Vector4f> *locations, *colors;

		explicit PointCloud(Vector2i imgSize, MemoryDeviceType memoryType)
		{
			this->point_count = 0;

			locations = new ORUtils::Image<Vector4f>(imgSize, memoryType);
			colors = new ORUtils::Image<Vector4f>(imgSize, memoryType);
		}

		void UpdateHostFromDevice()
		{
			this->locations->UpdateHostFromDevice();
			this->colors->UpdateHostFromDevice();
		}

		void UpdateDeviceFromHost()
		{
			this->locations->UpdateDeviceFromHost();
			this->colors->UpdateDeviceFromHost();
		}

		~PointCloud()
		{
			delete locations;
			delete colors;
		}

		// Suppress the default copy constructor and assignment operator
		PointCloud(const PointCloud&);
		PointCloud& operator=(const PointCloud&);
	};
}
