//  ================================================================
//  Created by Gregory Kramida on 11/15/19.
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

#include "../../Interface/IndexingEngine.tpp"
#include "../../../../GlobalTemplateDefines.h"

namespace ITMLib {
template
class IndexingEngine<TSDFVoxel, PlainVoxelArray, MEMORYDEVICE_CUDA>;
template
class IndexingEngine<WarpVoxel, PlainVoxelArray, MEMORYDEVICE_CUDA>;


template void IndexingEngine<TSDFVoxel,PlainVoxelArray,MEMORYDEVICE_CUDA>::AllocateUsingOtherVolume(
		ITMLib::VoxelVolume<WarpVoxel, PlainVoxelArray>* targetVolume,
		ITMLib::VoxelVolume<TSDFVoxel, PlainVoxelArray>* sourceVolume);
template void IndexingEngine<TSDFVoxel,PlainVoxelArray,MEMORYDEVICE_CUDA>::AllocateUsingOtherVolume(
		ITMLib::VoxelVolume<TSDFVoxel, PlainVoxelArray>* targetVolume,
		ITMLib::VoxelVolume<TSDFVoxel, PlainVoxelArray>* sourceVolume);
template void IndexingEngine<TSDFVoxel,PlainVoxelArray,MEMORYDEVICE_CUDA>::AllocateUsingOtherVolumeExpanded(
		ITMLib::VoxelVolume<WarpVoxel, PlainVoxelArray>* targetVolume,
		ITMLib::VoxelVolume<TSDFVoxel, PlainVoxelArray>* sourceVolume);
template void IndexingEngine<TSDFVoxel,PlainVoxelArray,MEMORYDEVICE_CUDA>::AllocateUsingOtherVolumeExpanded(
		ITMLib::VoxelVolume<TSDFVoxel, PlainVoxelArray>* targetVolume,
		ITMLib::VoxelVolume<TSDFVoxel, PlainVoxelArray>* sourceVolume);
template void IndexingEngine<TSDFVoxel,PlainVoxelArray,MEMORYDEVICE_CUDA>::AllocateUsingOtherVolumeAndSetVisibilityExpanded(
		ITMLib::VoxelVolume<WarpVoxel, PlainVoxelArray>* targetVolume,
		ITMLib::VoxelVolume<TSDFVoxel, PlainVoxelArray>* sourceVolume,
		ITMView* view, const Matrix4f& depth_camera_matrix);
template void IndexingEngine<TSDFVoxel,PlainVoxelArray,MEMORYDEVICE_CUDA>::AllocateUsingOtherVolumeAndSetVisibilityExpanded(
		ITMLib::VoxelVolume<TSDFVoxel, PlainVoxelArray>* targetVolume,
		ITMLib::VoxelVolume<TSDFVoxel, PlainVoxelArray>* sourceVolume,
		ITMView* view, const Matrix4f& depth_camera_matrix);


template void IndexingEngine<TSDFVoxel, PlainVoxelArray, MEMORYDEVICE_CUDA>::
AllocateFromWarpedVolume<WarpType::WARP_CUMULATIVE>(
		VoxelVolume<WarpVoxel, PlainVoxelArray>* warpField,
		VoxelVolume<TSDFVoxel, PlainVoxelArray>* sourceTSDF,
		VoxelVolume<TSDFVoxel, PlainVoxelArray>* targetTSDF
);
template void IndexingEngine<TSDFVoxel, PlainVoxelArray, MEMORYDEVICE_CUDA>::
AllocateFromWarpedVolume<WarpType::WARP_FLOW>(
		VoxelVolume<WarpVoxel, PlainVoxelArray>* warpField,
		VoxelVolume<TSDFVoxel, PlainVoxelArray>* sourceTSDF,
		VoxelVolume<TSDFVoxel, PlainVoxelArray>* targetTSDF
);
template void IndexingEngine<TSDFVoxel, PlainVoxelArray, MEMORYDEVICE_CUDA>::
AllocateFromWarpedVolume<WarpType::WARP_UPDATE>(
		VoxelVolume<WarpVoxel, PlainVoxelArray>* warpField,
		VoxelVolume<TSDFVoxel, PlainVoxelArray>* sourceTSDF,
		VoxelVolume<TSDFVoxel, PlainVoxelArray>* targetTSDF
);
} //namespace ITMLib