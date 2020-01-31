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

#include "IndexingEngine_CPU_VoxelBlockHash.tpp"
#include "../IndexingEngine_VoxelBlockHash.tpp"
#include "../../../../ITMLibDefines.h"

namespace ITMLib {
template
class IndexingEngine<ITMVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>;
template
class IndexingEngine<ITMWarp, VoxelBlockHash, MEMORYDEVICE_CPU>;

template void IndexingEngine<ITMVoxel,VoxelBlockHash,MEMORYDEVICE_CPU>::AllocateUsingOtherVolume(
		ITMLib::VoxelVolume<ITMWarp, VoxelBlockHash>* targetVolume,
		ITMLib::VoxelVolume<ITMVoxel, VoxelBlockHash>* sourceVolume);
template void IndexingEngine<ITMVoxel,VoxelBlockHash,MEMORYDEVICE_CPU>::AllocateUsingOtherVolume(
		ITMLib::VoxelVolume<ITMVoxel, VoxelBlockHash>* targetVolume,
		ITMLib::VoxelVolume<ITMVoxel, VoxelBlockHash>* sourceVolume);
template void IndexingEngine<ITMWarp,VoxelBlockHash,MEMORYDEVICE_CPU>::AllocateUsingOtherVolume(
		ITMLib::VoxelVolume<ITMWarp, VoxelBlockHash>* targetVolume,
		ITMLib::VoxelVolume<ITMWarp, VoxelBlockHash>* sourceVolume);
template void IndexingEngine<ITMVoxel,VoxelBlockHash,MEMORYDEVICE_CPU>::AllocateUsingOtherVolumeExpanded(
		ITMLib::VoxelVolume<ITMWarp, VoxelBlockHash>* targetVolume,
		ITMLib::VoxelVolume<ITMVoxel, VoxelBlockHash>* sourceVolume);
template void IndexingEngine<ITMVoxel,VoxelBlockHash,MEMORYDEVICE_CPU>::AllocateUsingOtherVolumeExpanded(
		ITMLib::VoxelVolume<ITMVoxel, VoxelBlockHash>* targetVolume,
		ITMLib::VoxelVolume<ITMVoxel, VoxelBlockHash>* sourceVolume);
template void IndexingEngine<ITMVoxel,VoxelBlockHash,MEMORYDEVICE_CPU>::AllocateUsingOtherVolumeAndSetVisibilityExpanded(
		ITMLib::VoxelVolume<ITMWarp, VoxelBlockHash>* targetVolume,
		ITMLib::VoxelVolume<ITMVoxel, VoxelBlockHash>* sourceVolume,
		ITMView* view, const Matrix4f& depth_camera_matrix);
template void IndexingEngine<ITMVoxel,VoxelBlockHash,MEMORYDEVICE_CPU>::AllocateUsingOtherVolumeAndSetVisibilityExpanded(
		ITMLib::VoxelVolume<ITMVoxel, VoxelBlockHash>* targetVolume,
		ITMLib::VoxelVolume<ITMVoxel, VoxelBlockHash>* sourceVolume,
		ITMView* view, const Matrix4f& depth_camera_matrix);

template void IndexingEngine_VoxelBlockHash<ITMVoxel, MEMORYDEVICE_CPU,
		IndexingEngine<ITMVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>>::
AllocateFromWarpedVolume<WarpType::WARP_CUMULATIVE>(
		VoxelVolume<ITMWarp, VoxelBlockHash>* warpField,
		VoxelVolume<ITMVoxel, VoxelBlockHash>* sourceTSDF,
		VoxelVolume<ITMVoxel, VoxelBlockHash>* targetTSDF
);
template void IndexingEngine_VoxelBlockHash<ITMVoxel, MEMORYDEVICE_CPU,
		IndexingEngine<ITMVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>>::
AllocateFromWarpedVolume<WarpType::WARP_FLOW>(
		VoxelVolume<ITMWarp, VoxelBlockHash>* warpField,
		VoxelVolume<ITMVoxel, VoxelBlockHash>* sourceTSDF,
		VoxelVolume<ITMVoxel, VoxelBlockHash>* targetTSDF
);
template void IndexingEngine_VoxelBlockHash<ITMVoxel, MEMORYDEVICE_CPU,
		IndexingEngine<ITMVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>>::
AllocateFromWarpedVolume<WarpType::WARP_UPDATE>(
		VoxelVolume<ITMWarp, VoxelBlockHash>* warpField,
		VoxelVolume<ITMVoxel, VoxelBlockHash>* sourceTSDF,
		VoxelVolume<ITMVoxel, VoxelBlockHash>* targetTSDF
);

}//namespace ITMLib