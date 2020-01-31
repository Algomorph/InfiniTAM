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

#include "ITMVoxelVolume.h"
#include "../../Engines/VolumeFileIO/VolumeFileIOEngine.h"
#include "../../Utils/Configuration.h"
#include "../../Engines/EditAndCopy/CPU/EditAndCopyEngine_CPU.h"
#ifndef COMPILE_WITHOUT_CUDA
#include "../../Engines/EditAndCopy/CUDA/EditAndCopyEngine_CUDA.h"
#endif

namespace ITMLib {

/**
 * \brief Generate a new scene (Caution: does not reset / initialize the voxel storage itself)
 * \param _sceneParams scene parameters (\see ITMSceneParams definition)
 * \param _useSwapping whether or not to use the GPU<--> CPU swapping mechanism
 * When on, keeps a larger global scene in main memory and a smaller, working part in VRAM, and continuously updates
 * the former from the latter
 * \param _memoryType DRAM to use -- GPU or CPU
 * \param size (optional) size of the scene -- affects only bounded index types, such as PlainVoxelArray
 * \param offset (optional) offset of the scene -- affects only bounded index types, such as PlainVoxelArray
 */
template<typename TVoxel, typename TIndex>
ITMVoxelVolume<TVoxel,TIndex>::ITMVoxelVolume(const VoxelVolumeParameters* _sceneParams, bool _useSwapping, MemoryDeviceType _memoryType,
                                              typename TIndex::InitializationParameters indexParameters)
	: sceneParams(_sceneParams),
		index(indexParameters, _memoryType),
	  localVBA(_memoryType, index.GetAllocatedBlockCount(), index.GetVoxelBlockSize())
{
	if (_useSwapping) globalCache = new ITMGlobalCache<TVoxel,TIndex>(this->index);
	else globalCache = nullptr;
}


template<class TVoxel, class TIndex>
ITMVoxelVolume<TVoxel, TIndex>::ITMVoxelVolume(MemoryDeviceType memoryDeviceType,
		typename TIndex::InitializationParameters indexParameters) :
	ITMVoxelVolume(&configuration::get().general_voxel_volume_parameters,
			configuration::get().swapping_mode == configuration::SWAPPINGMODE_ENABLED,
			memoryDeviceType, indexParameters) {}

template<class TVoxel, class TIndex>
ITMVoxelVolume<TVoxel, TIndex>::ITMVoxelVolume(const ITMVoxelVolume& other, MemoryDeviceType _memoryType)
	: sceneParams(other.sceneParams),
	index(other.index,_memoryType),
	localVBA(other.localVBA, _memoryType),
    globalCache(nullptr)
	{
    if(other.globalCache != nullptr){
	    this->globalCache = new ITMGlobalCache<TVoxel,TIndex>(*other.globalCache);
    }
}
template<class TVoxel, class TIndex>
void ITMVoxelVolume<TVoxel, TIndex>::Reset(){
	switch (this->index.memoryType) {
		case MEMORYDEVICE_CPU:
			EditAndCopyEngine_CPU<TVoxel, TIndex>::Inst().ResetScene(this);
			break;
#ifndef COMPILE_WITHOUT_CUDA
		case MEMORYDEVICE_CUDA:
			EditAndCopyEngine_CUDA<TVoxel, TIndex>::Inst().ResetScene(this);
			break;
#endif
		default:
			DIEWITHEXCEPTION_REPORTLOCATION("Unsupported device type.");
	}
}

template<class TVoxel, class TIndex>
void ITMVoxelVolume<TVoxel, TIndex>::SetFrom(const ITMVoxelVolume& other) {
	index.SetFrom(other.index);
	localVBA.SetFrom(other.localVBA);
	if(other.globalCache != nullptr){
		delete this->globalCache;
		globalCache = new ITMGlobalCache<TVoxel, TIndex>(*other.globalCache);
	}else{
		globalCache = nullptr;
	}
}

template<class TVoxel, class TIndex>
void ITMVoxelVolume<TVoxel, TIndex>::SaveToDirectory(const std::string& outputDirectory) const {
	VolumeFileIOEngine<TVoxel,TIndex>::SaveToDirectoryCompact(this, outputDirectory);
}

template<class TVoxel, class TIndex>
void ITMVoxelVolume<TVoxel, TIndex>::LoadFromDirectory(const std::string& outputDirectory) {
	VolumeFileIOEngine<TVoxel,TIndex>::LoadFromDirectoryCompact(this, outputDirectory);
}

template<class TVoxel, class TIndex>
TVoxel ITMVoxelVolume<TVoxel, TIndex>::GetValueAt(const Vector3i& pos) {
	switch (this->index.memoryType) {
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


}  // namespace ITMLib