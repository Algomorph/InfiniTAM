//  ================================================================
//  Created by Gregory Kramida on 5/8/19.
//  Copyright (c) 2019-2000 Gregory Kramida
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
	GlobalCache<TVoxel, TIndex> global_cache;

	VoxelVolume(const VoxelVolumeParameters *volume_parameters, bool use_swapping, MemoryDeviceType memory_type,
	            typename TIndex::InitializationParameters index_parameters = typename TIndex::InitializationParameters());
	explicit VoxelVolume(MemoryDeviceType memory_type, typename TIndex::InitializationParameters index_parameters = typename TIndex::InitializationParameters());
	VoxelVolume(const VoxelVolume& other, MemoryDeviceType memory_type);

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
	bool SwappingEnabled() const{
		return swapping_enabled;
	}

private:
	/** Current local content of the 8x8x8 stored on host or device depending on memory_type*/
	ORUtils::MemoryBlock<TVoxel> voxels;
	const bool swapping_enabled;
};

}//end namespace ITMLib