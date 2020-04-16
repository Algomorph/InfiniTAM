// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "GlobalCache.h"
#include "../../Utils/VoxelVolumeParameters.h"

namespace ITMLib
{
/** \brief
Represents the 3D world model as collection of voxel blocks, i.e. regular 3D grid
*/
template<class TVoxel, class TIndex>
class VoxelVolume {
public:
	/** Volume parameters like voxel size etc. */
	const VoxelVolumeParameters* parameters;

	/**
	 * \brief An indexing method for access to the volume's voxels.
	 * \details For instance, if VoxelBlockHash is used as TIndex, this is a hash table to reference the 8x8x8
	 * blocks. If it's an PlainVoxelArray, it's just a dense regular 3D array. */
	TIndex index;

	/** "Global" content -- stored on in host memory only */
	GlobalCache<TVoxel, TIndex>* global_cache;

	VoxelVolume(const VoxelVolumeParameters *volume_parameters, bool use_swapping, MemoryDeviceType memory_type,
	            typename TIndex::InitializationParameters index_parameters = typename TIndex::InitializationParameters());
	explicit VoxelVolume(MemoryDeviceType memoryDeviceType, typename TIndex::InitializationParameters indexParameters = typename TIndex::InitializationParameters());
	VoxelVolume(const VoxelVolume& other, MemoryDeviceType _memoryType);
	~VoxelVolume(){
		if (global_cache != nullptr) delete global_cache;
	}

	void Reset();
	void SetFrom(const VoxelVolume& other);
	void SaveToDirectory(const std::string &outputDirectory) const;
	void LoadFromDirectory(const std::string &outputDirectory);
	TVoxel* GetVoxelBlocks();
	const TVoxel* GetVoxelBlocks() const;
	TVoxel GetValueAt(const Vector3i& pos);
	TVoxel GetValueAt(int x, int y, int z){
		Vector3i pos(x,y,z);
		return GetValueAt(pos);
	}

	/** Return whether this scene is using swapping mechanism or not. **/
	bool Swapping() const{
		return this->global_cache != nullptr;
	}

	//TODO: restore
	// Suppress the default copy constructor and assignment operator (C++11 way)
	VoxelVolume(const VoxelVolume&) = delete;
	//ITMVoxelVolume(ITMVoxelVolume&&) noexcept = default;
	VoxelVolume& operator=(const VoxelVolume&) = delete;
private:
	/** Current local content of the 8x8x8 stored on host or device depending on memory_type*/
	ORUtils::MemoryBlock<TVoxel> voxels;
};

}//end namespace ITMLib