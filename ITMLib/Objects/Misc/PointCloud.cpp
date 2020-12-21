//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 12/15/20.
//  Copyright (c) 2020 Gregory Kramida
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at

//  http://www.apache.org/licenses/LICENSE-2.0

//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
//  ================================================================

#include "PointCloud.h"
#include "../../../ORUtils/MemoryBlockPersistence/MemoryBlockPersistence.h"

using namespace ITMLib;

PointCloud::PointCloud(Vector2i image_size, MemoryDeviceType memory_type):
locations(image_size, memory_type), colors(image_size, memory_type), memory_type(memory_type) {
	this->point_count = 0;
}

void PointCloud::UpdateHostFromDevice() {
	this->locations.UpdateHostFromDevice();
	this->colors.UpdateHostFromDevice();
}

void PointCloud::UpdateDeviceFromHost() {
	this->locations.UpdateDeviceFromHost();
	this->colors.UpdateDeviceFromHost();
}


namespace ITMLib{
bool operator==(const PointCloud& right_hand_point_cloud, const PointCloud& left_hand_point_cloud) {
	return right_hand_point_cloud.locations == left_hand_point_cloud.locations && right_hand_point_cloud.colors == left_hand_point_cloud.colors;
}

ORUtils::OStreamWrapper& operator<<(ORUtils::OStreamWrapper& o_stream_wrapper, const PointCloud& point_cloud){
	ORUtils::MemoryBlockPersistence::SaveImage(o_stream_wrapper, point_cloud.locations);
	ORUtils::MemoryBlockPersistence::SaveImage(o_stream_wrapper, point_cloud.colors);
	o_stream_wrapper.OStream().write(reinterpret_cast<const char*>(&point_cloud.point_count), sizeof(unsigned int));
	return o_stream_wrapper;
}
ORUtils::IStreamWrapper& operator>>(ORUtils::IStreamWrapper& i_stream_wrapper, PointCloud& point_cloud) {
	point_cloud.locations = ORUtils::MemoryBlockPersistence::LoadImage<Vector4f>(i_stream_wrapper, point_cloud.memory_type);
	point_cloud.colors = ORUtils::MemoryBlockPersistence::LoadImage<Vector4f>(i_stream_wrapper, point_cloud.memory_type);
	i_stream_wrapper.IStream().read(reinterpret_cast<char *>(&point_cloud.point_count), sizeof (unsigned int));
	return i_stream_wrapper;
}

}//namespace ITMLib

