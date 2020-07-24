//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 4/17/20.
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

#include "PlainVoxelArray.h"

namespace ITMLib {

//_DEBUG
DEFINE_PATHLESS_SERIALIZABLE_STRUCT(GRID_ALIGNED_BOX_STRUCT_DESCRIPTION);

//=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@=@

PlainVoxelArray::PlainVoxelArray()
		: memory_type(MEMORYDEVICE_NONE),
		  index_data() {}

PlainVoxelArray::PlainVoxelArray(PlainVoxelArray::InitializationParameters info, MemoryDeviceType memory_type) :
		memory_type(memory_type),
		index_data(1, true, true) {
	*(index_data.GetData(MEMORYDEVICE_CPU)) = info;
	index_data.UpdateDeviceFromHost();
}

PlainVoxelArray::PlainVoxelArray(MemoryDeviceType memory_type, Vector3i size, Vector3i offset)
		: PlainVoxelArray({size, offset}, memory_type) {}

PlainVoxelArray::PlainVoxelArray(const PlainVoxelArray& other, MemoryDeviceType memory_type) :
		PlainVoxelArray({other.GetVolumeSize(), other.GetVolumeOffset()}, memory_type) {
	this->SetFrom(other);
}

int PlainVoxelArray::GetVoxelBlockSize() {
	return index_data.GetData(MEMORYDEVICE_CPU)->size.x *
	       index_data.GetData(MEMORYDEVICE_CPU)->size.y *
	       index_data.GetData(MEMORYDEVICE_CPU)->size.z;
}

unsigned int PlainVoxelArray::GetMaxVoxelCount() const {
	return static_cast<unsigned int>(index_data.GetData(MEMORYDEVICE_CPU)->size.x) *
	       static_cast<unsigned int>(index_data.GetData(MEMORYDEVICE_CPU)->size.y) *
	       static_cast<unsigned int>(index_data.GetData(MEMORYDEVICE_CPU)->size.z);
}

void PlainVoxelArray::Save(ORUtils::OStreamWrapper& file) const {
	index_data.UpdateHostFromDevice();
	PlainVoxelArray::IndexData data = *index_data.GetData(MEMORYDEVICE_CPU);
	file.OStream().write(reinterpret_cast<const char* >(&data.size.x), sizeof(int));
	file.OStream().write(reinterpret_cast<const char* >(&data.size.y), sizeof(int));
	file.OStream().write(reinterpret_cast<const char* >(&data.size.z), sizeof(int));
	file.OStream().write(reinterpret_cast<const char* >(&data.offset.x), sizeof(int));
	file.OStream().write(reinterpret_cast<const char* >(&data.offset.y), sizeof(int));
	file.OStream().write(reinterpret_cast<const char* >(&data.offset.z), sizeof(int));
}

void PlainVoxelArray::Load(ORUtils::IStreamWrapper& file) {
	PlainVoxelArray::IndexData data;
	file.IStream().read(reinterpret_cast<char* >(&data.size.x), sizeof(int));
	file.IStream().read(reinterpret_cast<char* >(&data.size.y), sizeof(int));
	file.IStream().read(reinterpret_cast<char* >(&data.size.z), sizeof(int));
	file.IStream().read(reinterpret_cast<char* >(&data.offset.x), sizeof(int));
	file.IStream().read(reinterpret_cast<char* >(&data.offset.y), sizeof(int));
	file.IStream().read(reinterpret_cast<char* >(&data.offset.z), sizeof(int));
	*index_data.GetData(MEMORYDEVICE_CPU) = data;
	index_data.UpdateDeviceFromHost();
}


} // namespace ITMLib

