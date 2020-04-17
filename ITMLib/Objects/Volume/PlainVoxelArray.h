// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#ifndef __METALC__

#include "../../Utils/Math.h"
#include "../../Utils/Serialization/Serialization.h"
#include "../../../ORUtils/MemoryBlock.h"
#include "../../../ORUtils/MemoryBlockPersister.h"

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
	ORUtils::MemoryBlock<IndexData> index_data;

public:
	const MemoryDeviceType memory_type;

	PlainVoxelArray();

	PlainVoxelArray(PlainVoxelArray::InitializationParameters info, MemoryDeviceType memory_type);

	explicit PlainVoxelArray(MemoryDeviceType memory_type, Vector3i size = Vector3i(512),
	                         Vector3i offset = Vector3i(-256, -256, 0));

	PlainVoxelArray(const PlainVoxelArray& other, MemoryDeviceType memory_type);

	void SetFrom(const PlainVoxelArray& other) {
		MemoryCopyDirection memoryCopyDirection = determineMemoryCopyDirection(this->memory_type, other.memory_type);
		this->index_data.SetFrom(other.index_data, memoryCopyDirection);
	}

	/** Maximum number of total entries. */
	int GetMaximumBlockCount() const { return 1; }
	int GetUtilizedBlockCount() const { return 1; }
	int GetVoxelBlockSize();

	unsigned int GetMaxVoxelCount() const;

	Vector3i GetVolumeSize() const { return index_data.GetData(MEMORYDEVICE_CPU)->size; }

	Vector3i GetVolumeOffset() const { return index_data.GetData(MEMORYDEVICE_CPU)->offset; }

	/**Get the memory type used for storage.**/
	MemoryDeviceType GetMemoryType() const { return this->memory_type;}

	const IndexData* GetIndexData() const { return index_data.GetData(memory_type); }

	IndexData* GetIndexData() { return index_data.GetData(memory_type); }

	void Save(ORUtils::MemoryBlockOStreamWrapper& file) const;

	void Load(ORUtils::MemoryBlockIStreamWrapper& file);

#ifdef COMPILE_WITH_METAL
	const void *getIndexData_MB() const { return index_data.GetMetalBuffer(); }
#endif

	// Suppress the default copy constructor and assignment operator
	PlainVoxelArray(const PlainVoxelArray&) = delete;
	PlainVoxelArray& operator=(const PlainVoxelArray&) = delete;
};
}// namespace ITMLib

#endif