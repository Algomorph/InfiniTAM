//  ================================================================
//  Created by Gregory Kramida on 5/8/19.
//  Copyright (c) 2019 Gregory Kramida
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

#include "VoxelVolume.h"
#include "../../Engines/VolumeFileIO/VolumeFileIOEngine.h"
#include "../../Utils/Configuration.h"
#include "../../Engines/EditAndCopy/CPU/EditAndCopyEngine_CPU.h"
#ifndef COMPILE_WITHOUT_CUDA
#include "../../Engines/EditAndCopy/CUDA/EditAndCopyEngine_CUDA.h"
#endif

namespace ITMLib {

/**
 * \brief Generate a new scene (Caution: does not reset / initialize the voxel storage itself)
 * \param volume_parameters volume parameters (\see VoxelVolumeParameters definition)
 * \param use_swapping whether or not to use the GPU<--> CPU swapping mechanism
 * When on, keeps a larger global scene in main memory and a smaller, working part in VRAM, and continuously updates
 * the former from the latter
 * \param memory_type DRAM to use -- GPU or CPU
 * \param index_parameters parameters to initialize the index (each index type has its own type)
 */
template<typename TVoxel, typename TIndex>
VoxelVolume<TVoxel,TIndex>::VoxelVolume(const VoxelVolumeParameters* volume_parameters, bool use_swapping, MemoryDeviceType memory_type,
                                        typename TIndex::InitializationParameters index_parameters)
	: parameters(volume_parameters),
	  index(index_parameters, memory_type),
	  voxels(index.GetMaxVoxelCount(), memory_type)
{
	if (use_swapping) global_cache = new GlobalCache<TVoxel,TIndex>(this->index);
	else global_cache = nullptr;
}


template<class TVoxel, class TIndex>
VoxelVolume<TVoxel, TIndex>::VoxelVolume(MemoryDeviceType memoryDeviceType,
                                         typename TIndex::InitializationParameters indexParameters) :
	VoxelVolume(&configuration::get().general_voxel_volume_parameters,
			configuration::get().swapping_mode == configuration::SWAPPINGMODE_ENABLED,
			    memoryDeviceType, indexParameters) {}

template<class TVoxel, class TIndex>
VoxelVolume<TVoxel, TIndex>::VoxelVolume(const VoxelVolume& other, MemoryDeviceType _memoryType)
	: parameters(other.parameters),
	  index(other.index, _memoryType),
	  voxels(other.voxels),
	  global_cache(nullptr)
	{
    if(other.global_cache != nullptr){
	    this->global_cache = new GlobalCache<TVoxel,TIndex>(*other.global_cache);
    }
}
template<class TVoxel, class TIndex>
void VoxelVolume<TVoxel, TIndex>::Reset(){
	switch (this->index.memory_type) {
		case MEMORYDEVICE_CPU:
			EditAndCopyEngine_CPU<TVoxel, TIndex>::Inst().ResetVolume(this);
			break;
#ifndef COMPILE_WITHOUT_CUDA
		case MEMORYDEVICE_CUDA:
			EditAndCopyEngine_CUDA<TVoxel, TIndex>::Inst().ResetVolume(this);
			break;
#endif
		default:
			DIEWITHEXCEPTION_REPORTLOCATION("Unsupported device type.");
	}
}

template<class TVoxel, class TIndex>
void VoxelVolume<TVoxel, TIndex>::SetFrom(const VoxelVolume& other) {
	index.SetFrom(other.index);
	MemoryCopyDirection memory_copy_direction = determineMemoryCopyDirection(this->index.memory_type, other.index.memory_type);
	voxels.SetFrom(other.voxels, memory_copy_direction);
	if(other.global_cache != nullptr){
		delete this->global_cache;
		global_cache = new GlobalCache<TVoxel, TIndex>(*other.global_cache);
	}else{
		global_cache = nullptr;
	}
}

template<class TVoxel, class TIndex>
void VoxelVolume<TVoxel, TIndex>::SaveToDirectory(const std::string& outputDirectory) const {
	VolumeFileIOEngine<TVoxel,TIndex>::SaveToDirectoryCompact(this, outputDirectory);
}

template<class TVoxel, class TIndex>
void VoxelVolume<TVoxel, TIndex>::LoadFromDirectory(const std::string& outputDirectory) {
	VolumeFileIOEngine<TVoxel,TIndex>::LoadFromDirectoryCompact(this, outputDirectory);
}

template<class TVoxel, class TIndex>
TVoxel VoxelVolume<TVoxel, TIndex>::GetValueAt(const Vector3i& pos) {
	switch (this->index.memory_type) {
		case MEMORYDEVICE_CPU:
			return EditAndCopyEngine_CPU<TVoxel, TIndex>::Inst().ReadVoxel(this, pos);
#ifndef COMPILE_WITHOUT_CUDA
		case MEMORYDEVICE_CUDA:
			return EditAndCopyEngine_CUDA<TVoxel, TIndex>::Inst().ReadVoxel(this, pos);
#endif
		default:
			DIEWITHEXCEPTION_REPORTLOCATION("Unsupported device type.");
	}
}

template<class TVoxel, class TIndex>
TVoxel* VoxelVolume<TVoxel, TIndex>::GetVoxelBlocks() {
	return this->voxels.GetData(this->index.memory_type);
}

template<class TVoxel, class TIndex>
const TVoxel* VoxelVolume<TVoxel, TIndex>::GetVoxelBlocks() const{
	return this->voxels.GetData(this->index.memory_type);
}


}  // namespace ITMLib