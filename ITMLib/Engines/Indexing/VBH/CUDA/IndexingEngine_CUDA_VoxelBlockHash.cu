//  ================================================================
//  Created by Gregory Kramida on 11/1/19.
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

//local
#include "../../../../GlobalTemplateDefines.h"
#include "../../../Traversal/CUDA/ImageTraversal_CUDA.h"
#include "../../../Traversal/CUDA/TwoImageTraversal_CUDA.h"
#include "../../../Traversal/CUDA/HashTableTraversal_CUDA.h"
#include "../../../Traversal/CUDA/VolumeTraversal_CUDA_VoxelBlockHash.h"
#include "../IndexingEngine_VoxelBlockHash.tpp"
#include "IndexingEngine_CUDA_VoxelBlockHash.tcu"

namespace ITMLib {

template
class IndexingEngine_VoxelBlockHash<TSDFVoxel, MEMORYDEVICE_CUDA,
		IndexingEngine<TSDFVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA>>;
template
class IndexingEngine_VoxelBlockHash<WarpVoxel, MEMORYDEVICE_CUDA,
		IndexingEngine<WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA>>;
template
class IndexingEngine<TSDFVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA>;
template
class IndexingEngine<WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA>;

template void IndexingEngine<TSDFVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA>::AllocateUsingOtherVolume(
		ITMLib::VoxelVolume<WarpVoxel, VoxelBlockHash>* target_volume,
		ITMLib::VoxelVolume<TSDFVoxel, VoxelBlockHash>* source_volume);
template void IndexingEngine<TSDFVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA>::AllocateUsingOtherVolume(
		ITMLib::VoxelVolume<TSDFVoxel, VoxelBlockHash>* target_volume,
		ITMLib::VoxelVolume<TSDFVoxel, VoxelBlockHash>* source_volume);
template void IndexingEngine<WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA>::AllocateUsingOtherVolume(
		ITMLib::VoxelVolume<WarpVoxel, VoxelBlockHash>* target_volume,
		ITMLib::VoxelVolume<WarpVoxel, VoxelBlockHash>* source_volume);

template void IndexingEngine<TSDFVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA>::AllocateUsingOtherVolume_Bounded(
		ITMLib::VoxelVolume<WarpVoxel, VoxelBlockHash>* targetVolume,
		ITMLib::VoxelVolume<TSDFVoxel, VoxelBlockHash>* sourceVolume,
		const Extent3D& bounds);
template void IndexingEngine<TSDFVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA>::AllocateUsingOtherVolume_Bounded(
		ITMLib::VoxelVolume<TSDFVoxel, VoxelBlockHash>* targetVolume,
		ITMLib::VoxelVolume<TSDFVoxel, VoxelBlockHash>* sourceVolume,
		const Extent3D& bounds);
template void IndexingEngine<WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA>::AllocateUsingOtherVolume_Bounded(
		ITMLib::VoxelVolume<WarpVoxel, VoxelBlockHash>* target_volume,
		ITMLib::VoxelVolume<WarpVoxel, VoxelBlockHash>* source_volume,
		const Extent3D& bounds);

template void IndexingEngine<TSDFVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA>::AllocateUsingOtherVolume_OffsetAndBounded(
		ITMLib::VoxelVolume<WarpVoxel, VoxelBlockHash>* targetVolume,
		ITMLib::VoxelVolume<TSDFVoxel, VoxelBlockHash>* sourceVolume,
		const Extent3D& source_bounds, const Vector3i& target_offset);
template void IndexingEngine<TSDFVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA>::AllocateUsingOtherVolume_OffsetAndBounded(
		ITMLib::VoxelVolume<TSDFVoxel, VoxelBlockHash>* targetVolume,
		ITMLib::VoxelVolume<TSDFVoxel, VoxelBlockHash>* sourceVolume,
		const Extent3D& source_bounds, const Vector3i& target_offset);
template void IndexingEngine<WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA>::AllocateUsingOtherVolume_OffsetAndBounded(
		ITMLib::VoxelVolume<WarpVoxel, VoxelBlockHash>* targetVolume,
		ITMLib::VoxelVolume<WarpVoxel, VoxelBlockHash>* sourceVolume,
		const Extent3D& source_bounds, const Vector3i& target_offset);


}//namespace ITMLib