//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 11/28/20.
//  Copyright (c) 2020 Gregory Kramida
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

#include "IndexingEngine_PlainVoxelArray.tpp"
#include "../../../Objects/Volume/VoxelTypes.h"

namespace ITMLib::internal{
template void AllocateUsingOtherVolume<MEMORYDEVICE_CUDA, WarpVoxel, TSDFVoxel_f_flags>(
		ITMLib::VoxelVolume<WarpVoxel, PlainVoxelArray>* target_volume,
		ITMLib::VoxelVolume<TSDFVoxel_f_flags, PlainVoxelArray>* source_volume);
template void AllocateUsingOtherVolume<MEMORYDEVICE_CUDA, TSDFVoxel_f_flags, TSDFVoxel_f_flags>(
		ITMLib::VoxelVolume<TSDFVoxel_f_flags, PlainVoxelArray>* target_volume,
		ITMLib::VoxelVolume<TSDFVoxel_f_flags, PlainVoxelArray>* source_volume);
template void AllocateUsingOtherVolume<MEMORYDEVICE_CUDA, TSDFVoxel_f_rgb, TSDFVoxel_f_rgb>(
		ITMLib::VoxelVolume<TSDFVoxel_f_rgb, PlainVoxelArray>* target_volume,
		ITMLib::VoxelVolume<TSDFVoxel_f_rgb, PlainVoxelArray>* source_volume);
template void AllocateUsingOtherVolume<MEMORYDEVICE_CUDA, WarpVoxel, WarpVoxel>(
		ITMLib::VoxelVolume<WarpVoxel, PlainVoxelArray>* target_volume,
		ITMLib::VoxelVolume<WarpVoxel, PlainVoxelArray>* source_volume);

template void AllocateUsingOtherVolume_Bounded<MEMORYDEVICE_CUDA, WarpVoxel, TSDFVoxel_f_flags>(
		ITMLib::VoxelVolume<WarpVoxel, PlainVoxelArray>* targetVolume,
		ITMLib::VoxelVolume<TSDFVoxel_f_flags, PlainVoxelArray>* sourceVolume,
		const Extent3Di& bounds);
template void AllocateUsingOtherVolume_Bounded<MEMORYDEVICE_CUDA, TSDFVoxel_f_flags, TSDFVoxel_f_flags>(
		ITMLib::VoxelVolume<TSDFVoxel_f_flags, PlainVoxelArray>* targetVolume,
		ITMLib::VoxelVolume<TSDFVoxel_f_flags, PlainVoxelArray>* sourceVolume,
		const Extent3Di& bounds);
template void AllocateUsingOtherVolume_Bounded<MEMORYDEVICE_CUDA, TSDFVoxel_f_rgb, TSDFVoxel_f_rgb>(
		ITMLib::VoxelVolume<TSDFVoxel_f_rgb, PlainVoxelArray>* targetVolume,
		ITMLib::VoxelVolume<TSDFVoxel_f_rgb, PlainVoxelArray>* sourceVolume,
		const Extent3Di& bounds);
template void AllocateUsingOtherVolume_Bounded<MEMORYDEVICE_CUDA, WarpVoxel, WarpVoxel>(
		ITMLib::VoxelVolume<WarpVoxel, PlainVoxelArray>* target_volume,
		ITMLib::VoxelVolume<WarpVoxel, PlainVoxelArray>* source_volume,
		const Extent3Di& bounds);

template void AllocateUsingOtherVolume_OffsetAndBounded<MEMORYDEVICE_CUDA, WarpVoxel, TSDFVoxel_f_flags>(
		ITMLib::VoxelVolume<WarpVoxel, PlainVoxelArray>* targetVolume,
		ITMLib::VoxelVolume<TSDFVoxel_f_flags, PlainVoxelArray>* sourceVolume,
		const Extent3Di& source_bounds, const Vector3i& target_offset);
template void AllocateUsingOtherVolume_OffsetAndBounded<MEMORYDEVICE_CUDA, TSDFVoxel_f_flags, TSDFVoxel_f_flags>(
		ITMLib::VoxelVolume<TSDFVoxel_f_flags, PlainVoxelArray>* targetVolume,
		ITMLib::VoxelVolume<TSDFVoxel_f_flags, PlainVoxelArray>* sourceVolume,
		const Extent3Di& source_bounds, const Vector3i& target_offset);
template void AllocateUsingOtherVolume_OffsetAndBounded<MEMORYDEVICE_CUDA, TSDFVoxel_f_rgb, TSDFVoxel_f_rgb>(
		ITMLib::VoxelVolume<TSDFVoxel_f_rgb, PlainVoxelArray>* targetVolume,
		ITMLib::VoxelVolume<TSDFVoxel_f_rgb, PlainVoxelArray>* sourceVolume,
		const Extent3Di& source_bounds, const Vector3i& target_offset);
template void AllocateUsingOtherVolume_OffsetAndBounded<MEMORYDEVICE_CUDA, WarpVoxel, WarpVoxel>(
		ITMLib::VoxelVolume<WarpVoxel, PlainVoxelArray>* target_volume,
		ITMLib::VoxelVolume<WarpVoxel, PlainVoxelArray>* source_volume,
		const Extent3Di& source_bounds, const Vector3i& target_offset);
} //namespace ITMLib