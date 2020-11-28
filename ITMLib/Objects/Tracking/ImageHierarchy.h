// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "TrackerIterationType.h"
#include "../../Utils/Math.h"
#include "../../../ORUtils/MemoryBlock.h"

namespace ITMLib
{
	template <typename TLevelType> class ImageHierarchy
	{
	private:
		int noLevels;
		TLevelType **levels;

	public:
		ImageHierarchy(Vector2i imgSize, TrackerIterationType *trackingRegime, int hierarchy_level_count,
		               MemoryDeviceType memoryType, bool skipAllocationForLevel0 = false)
		{
			this->noLevels = hierarchy_level_count;

			levels = new TLevelType*[hierarchy_level_count];

			for (int i = hierarchy_level_count - 1; i >= 0; i--)
				levels[i] = new TLevelType(imgSize, i, trackingRegime[i], memoryType, (i == 0) && skipAllocationForLevel0);
		}

		void UpdateHostFromDevice()
		{ for (int i = 0; i < noLevels; i++) this->levels[i]->UpdateHostFromDevice(); }

		void UpdateDeviceFromHost()
		{ for (int i = 0; i < noLevels; i++) this->levels[i]->UpdateDeviceFromHost(); }

		int GetNoLevels() const { return noLevels; }

		TLevelType * GetLevel(int level) const
		{
			return level >= 0 && level < noLevels ? levels[level] : nullptr;
		}

		~ImageHierarchy()
		{
			for (int i = 0; i < noLevels; i++) delete levels[i];
			delete [] levels;
		}

		// Suppress the default copy constructor and assignment operator
		ImageHierarchy(const ImageHierarchy&);
		ImageHierarchy& operator=(const ImageHierarchy&);
	};
}
