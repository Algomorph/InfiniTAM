// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#ifndef __METALC__

#include "../../Utils/Math.h"
#include "../../Utils/Serialization/Serialization.h"
#include "../../../ORUtils/MemoryBlock.h"

namespace ITMLib {

GENERATE_PATHLESS_SERIALIZABLE_STRUCT(
		GridAlignedBox,
		(Vector3i, size, Vector3i(512), VECTOR, "Size, in voxels."),
		(Vector3i, offset, Vector3i(-256, -256, 0), VECTOR, "Offset of the lower left front corner of the volume, in voxels")
);

/** \brief
This is the central class for the original fixed size volume
representation. It contains the data needed on the CPU and
a pointer to the data structure on the GPU.
*/
class PlainVoxelArray {
public:

	typedef GridAlignedBox IndexData;
	typedef GridAlignedBox InitializationParameters;
	struct IndexCache {
	};

private:
	ORUtils::MemoryBlock<IndexData>* index_data;

public:
	const MemoryDeviceType memory_type;

	PlainVoxelArray(PlainVoxelArray::InitializationParameters info, MemoryDeviceType memoryType) :
			memory_type(memoryType),
			index_data(new ORUtils::MemoryBlock<IndexData>(1, true, true)) {
		*(index_data->GetData(MEMORYDEVICE_CPU)) = info;
		index_data->UpdateDeviceFromHost();
	}

	explicit PlainVoxelArray(MemoryDeviceType memoryType, Vector3i size = Vector3i(512),
	                            Vector3i offset = Vector3i(-256, -256, 0)) : PlainVoxelArray({size, offset},
	                                                                                            memoryType) {}

	PlainVoxelArray(const PlainVoxelArray& other, MemoryDeviceType memoryType) :
			PlainVoxelArray({other.GetVolumeSize(), other.GetVolumeOffset()}, memoryType) {
		this->SetFrom(other);
	}

	void SetFrom(const PlainVoxelArray& other) {
		MemoryCopyDirection memoryCopyDirection = determineMemoryCopyDirection(this->memory_type, other.memory_type);
		this->index_data->SetFrom(other.index_data, memoryCopyDirection);
	}

	~PlainVoxelArray() {
		delete index_data;
	}

	/** Maximum number of total entries. */
	int GetAllocatedBlockCount() { return 1; }

	int GetVoxelBlockSize() {
		return index_data->GetData(MEMORYDEVICE_CPU)->size.x *
		       index_data->GetData(MEMORYDEVICE_CPU)->size.y *
		       index_data->GetData(MEMORYDEVICE_CPU)->size.z;
	}

	unsigned int GetMaxVoxelCount() const {
		return static_cast<unsigned int>(index_data->GetData(MEMORYDEVICE_CPU)->size.x) *
		       static_cast<unsigned int>(index_data->GetData(MEMORYDEVICE_CPU)->size.y) *
		       static_cast<unsigned int>(index_data->GetData(MEMORYDEVICE_CPU)->size.z);
	}

	Vector3i GetVolumeSize() const { return index_data->GetData(MEMORYDEVICE_CPU)->size; }

	Vector3i GetVolumeOffset() const { return index_data->GetData(MEMORYDEVICE_CPU)->offset; }

	/**Get the memory type used for storage.**/
	MemoryDeviceType GetMemoryType() const {
		return this->memory_type;
	}

	const IndexData* GetIndexData() const { return index_data->GetData(memory_type); }

	IndexData* GetIndexData() { return index_data->GetData(memory_type); }

	void SaveToDirectory(const std::string& outputDirectory) const {
	}

#ifndef _MSC_VER
#pragma clang diagnostic push
#pragma ide diagnostic ignored "MemberFunctionCanBeStatic"
#endif

	void LoadFromDirectory(const std::string& outputDirectory) {
	}
#ifndef _MSC_VER
#pragma clang diagnostic pop
#endif
#ifdef COMPILE_WITH_METAL
	const void *getIndexData_MB() const { return index_data->GetMetalBuffer(); }
#endif

	// Suppress the default copy constructor and assignment operator
	PlainVoxelArray(const PlainVoxelArray&) = delete;
	PlainVoxelArray& operator=(const PlainVoxelArray&) = delete;
};
}

#endif