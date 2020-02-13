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
class IndexingEngine<TSDFVoxel, PlainVoxelArray, MEMORYDEVICE_CPU>;
template
class IndexingEngine<WarpVoxel, PlainVoxelArray, MEMORYDEVICE_CPU>;


template void IndexingEngine<TSDFVoxel, PlainVoxelArray, MEMORYDEVICE_CPU>::AllocateUsingOtherVolume(
		ITMLib::VoxelVolume<WarpVoxel, PlainVoxelArray>* targetVolume,
		ITMLib::VoxelVolume<TSDFVoxel, PlainVoxelArray>* sourceVolume);
template void IndexingEngine<TSDFVoxel, PlainVoxelArray, MEMORYDEVICE_CPU>::AllocateUsingOtherVolume(
		ITMLib::VoxelVolume<TSDFVoxel, PlainVoxelArray>* targetVolume,
		ITMLib::VoxelVolume<TSDFVoxel, PlainVoxelArray>* sourceVolume);

template void IndexingEngine<TSDFVoxel, PlainVoxelArray, MEMORYDEVICE_CPU>::AllocateUsingOtherVolume_Bounded(
		ITMLib::VoxelVolume<WarpVoxel, PlainVoxelArray>* targetVolume,
		ITMLib::VoxelVolume<TSDFVoxel, PlainVoxelArray>* sourceVolume,
		const Extent3D& bounds);
template void IndexingEngine<TSDFVoxel, PlainVoxelArray, MEMORYDEVICE_CPU>::AllocateUsingOtherVolume_Bounded(
		ITMLib::VoxelVolume<TSDFVoxel, PlainVoxelArray>* targetVolume,
		ITMLib::VoxelVolume<TSDFVoxel, PlainVoxelArray>* sourceVolume,
		const Extent3D& bounds);

template void IndexingEngine<TSDFVoxel, PlainVoxelArray, MEMORYDEVICE_CPU>::AllocateUsingOtherVolume_OffsetAndBounded(
		ITMLib::VoxelVolume<WarpVoxel, PlainVoxelArray>* targetVolume,
		ITMLib::VoxelVolume<TSDFVoxel, PlainVoxelArray>* sourceVolume,
		const Extent3D& source_bounds, const Vector3i& target_offset);
template void IndexingEngine<TSDFVoxel, PlainVoxelArray, MEMORYDEVICE_CPU>::AllocateUsingOtherVolume_OffsetAndBounded(
		ITMLib::VoxelVolume<TSDFVoxel, PlainVoxelArray>* targetVolume,
		ITMLib::VoxelVolume<TSDFVoxel, PlainVoxelArray>* sourceVolume,
		const Extent3D& source_bounds, const Vector3i& target_offset);
} //namespace ITMLib