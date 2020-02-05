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
#include "../../../Traversal/CPU/VolumeTraversal_CPU_VoxelBlockHash.h"
#include "IndexingEngine_CPU_VoxelBlockHash.tpp"
#include "../IndexingEngine_VoxelBlockHash.tpp"
#include "../../../../GlobalTemplateDefines.h"

namespace ITMLib {
template
class IndexingEngine<TSDFVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>;
template
class IndexingEngine<WarpVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>;

template void IndexingEngine<TSDFVoxel,VoxelBlockHash,MEMORYDEVICE_CPU>::AllocateUsingOtherVolume(
		ITMLib::VoxelVolume<WarpVoxel, VoxelBlockHash>* targetVolume,
		ITMLib::VoxelVolume<TSDFVoxel, VoxelBlockHash>* sourceVolume);
template void IndexingEngine<TSDFVoxel,VoxelBlockHash,MEMORYDEVICE_CPU>::AllocateUsingOtherVolume(
		ITMLib::VoxelVolume<TSDFVoxel, VoxelBlockHash>* targetVolume,
		ITMLib::VoxelVolume<TSDFVoxel, VoxelBlockHash>* sourceVolume);
template void IndexingEngine<WarpVoxel,VoxelBlockHash,MEMORYDEVICE_CPU>::AllocateUsingOtherVolume(
		ITMLib::VoxelVolume<WarpVoxel, VoxelBlockHash>* targetVolume,
		ITMLib::VoxelVolume<WarpVoxel, VoxelBlockHash>* sourceVolume);
template void IndexingEngine<TSDFVoxel,VoxelBlockHash,MEMORYDEVICE_CPU>::AllocateUsingOtherVolumeExpanded(
		ITMLib::VoxelVolume<WarpVoxel, VoxelBlockHash>* targetVolume,
		ITMLib::VoxelVolume<TSDFVoxel, VoxelBlockHash>* sourceVolume);
template void IndexingEngine<TSDFVoxel,VoxelBlockHash,MEMORYDEVICE_CPU>::AllocateUsingOtherVolumeExpanded(
		ITMLib::VoxelVolume<TSDFVoxel, VoxelBlockHash>* targetVolume,
		ITMLib::VoxelVolume<TSDFVoxel, VoxelBlockHash>* sourceVolume);
template void IndexingEngine<TSDFVoxel,VoxelBlockHash,MEMORYDEVICE_CPU>::AllocateUsingOtherVolumeAndSetVisibilityExpanded(
		ITMLib::VoxelVolume<WarpVoxel, VoxelBlockHash>* targetVolume,
		ITMLib::VoxelVolume<TSDFVoxel, VoxelBlockHash>* sourceVolume,
		ITMView* view, const Matrix4f& depth_camera_matrix);
template void IndexingEngine<TSDFVoxel,VoxelBlockHash,MEMORYDEVICE_CPU>::AllocateUsingOtherVolumeAndSetVisibilityExpanded(
		ITMLib::VoxelVolume<TSDFVoxel, VoxelBlockHash>* targetVolume,
		ITMLib::VoxelVolume<TSDFVoxel, VoxelBlockHash>* sourceVolume,
		ITMView* view, const Matrix4f& depth_camera_matrix);

template void IndexingEngine_VoxelBlockHash<TSDFVoxel, MEMORYDEVICE_CPU,
		IndexingEngine<TSDFVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>>::
AllocateFromWarpedVolume<WarpType::WARP_CUMULATIVE>(
		VoxelVolume<WarpVoxel, VoxelBlockHash>* warpField,
		VoxelVolume<TSDFVoxel, VoxelBlockHash>* sourceTSDF,
		VoxelVolume<TSDFVoxel, VoxelBlockHash>* targetTSDF
);
template void IndexingEngine_VoxelBlockHash<TSDFVoxel, MEMORYDEVICE_CPU,
		IndexingEngine<TSDFVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>>::
AllocateFromWarpedVolume<WarpType::WARP_FLOW>(
		VoxelVolume<WarpVoxel, VoxelBlockHash>* warpField,
		VoxelVolume<TSDFVoxel, VoxelBlockHash>* sourceTSDF,
		VoxelVolume<TSDFVoxel, VoxelBlockHash>* targetTSDF
);
template void IndexingEngine_VoxelBlockHash<TSDFVoxel, MEMORYDEVICE_CPU,
		IndexingEngine<TSDFVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>>::
AllocateFromWarpedVolume<WarpType::WARP_UPDATE>(
		VoxelVolume<WarpVoxel, VoxelBlockHash>* warpField,
		VoxelVolume<TSDFVoxel, VoxelBlockHash>* sourceTSDF,
		VoxelVolume<TSDFVoxel, VoxelBlockHash>* targetTSDF
);

}//namespace ITMLib