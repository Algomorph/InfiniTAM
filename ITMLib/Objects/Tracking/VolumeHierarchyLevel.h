// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "TrackerIterationType.h"
#include "../../Utils/Math.h"
#include "../../../ORUtils/Image.h"

namespace ITMLib
{
	class VolumeHierarchyLevel
	{
	public:
		int levelId;

		TrackerIterationType iterationType;

		ORUtils::Image<Vector4f> *pointsMap;
		ORUtils::Image<Vector4f> *normalsMap;
		Vector4f intrinsics;

		bool manageData;

		VolumeHierarchyLevel(Vector2i imgSize, int levelId, TrackerIterationType iterationType, MemoryDeviceType memoryType, bool skipAllocation = false)
		{
			this->manageData = !skipAllocation;
			this->levelId = levelId;
			this->iterationType = iterationType;

			if (!skipAllocation) {
				this->pointsMap = new ORUtils::Image<Vector4f>(imgSize, memoryType);
				this->normalsMap = new ORUtils::Image<Vector4f>(imgSize, memoryType);
			}
		}

		void UpdateHostFromDevice()
		{ 
			this->pointsMap->UpdateHostFromDevice();
			this->normalsMap->UpdateHostFromDevice();
		}

		void UpdateDeviceFromHost()
		{ 
			this->pointsMap->UpdateDeviceFromHost();
			this->normalsMap->UpdateDeviceFromHost();
		}

		~VolumeHierarchyLevel(void)
		{
			if (manageData) {
				delete pointsMap;
				delete normalsMap;
			}
		}

		// Suppress the default copy constructor and assignment operator
		VolumeHierarchyLevel(const VolumeHierarchyLevel&);
		VolumeHierarchyLevel& operator=(const VolumeHierarchyLevel&);
	};
}
