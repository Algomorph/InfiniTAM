// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../Utils/Math.h"
#include "../../../ORUtils/Image.h"
#include "../../../ORUtils/OStreamWrapper.h"
#include "../../../ORUtils/IStreamWrapper.h"

namespace ITMLib {
class PointCloud {
public:
	uint point_count;

	ORUtils::Image<Vector4f> locations, colors;
private:
	MemoryDeviceType memory_type;
public:
	explicit PointCloud(Vector2i image_size, MemoryDeviceType memory_type);

	void UpdateHostFromDevice();

	void UpdateDeviceFromHost();

	friend bool operator==(const PointCloud& rhs, const PointCloud& lhs);
	friend ORUtils::OStreamWrapper& operator<<(ORUtils::OStreamWrapper& o_stream_wrapper, const PointCloud& point_cloud);
	friend ORUtils::IStreamWrapper& operator>>(ORUtils::IStreamWrapper& i_stream_wrapper, PointCloud& point_cloud);

};

} // namespace ITMLib
