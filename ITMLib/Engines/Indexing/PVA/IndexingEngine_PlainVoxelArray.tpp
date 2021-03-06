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
#pragma once

//local
#include "IndexingEngine_PlainVoxelArray.h"
#include "../../../Objects/Volume/RepresentationAccess.h"
#include "../../EditAndCopy/Shared/EditAndCopyEngine_Shared.h"
#include "../../../Utils/Geometry/FrustumTrigonometry.h"

using namespace ITMLib;


template<typename TVoxel, MemoryDeviceType TMemoryDeviceType, ExecutionMode TExecutionMode>
void IndexingEngine<TVoxel, PlainVoxelArray, TMemoryDeviceType, TExecutionMode>::AllocateNearSurface(VoxelVolume<TVoxel, PlainVoxelArray>* volume,
                                                                            const View* view,
                                                                            const CameraTrackingState* tracking_state) {}


template<typename TVoxel, MemoryDeviceType TMemoryDeviceType, ExecutionMode TExecutionMode>
void IndexingEngine<TVoxel, PlainVoxelArray, TMemoryDeviceType, TExecutionMode>::AllocateNearSurface(VoxelVolume<TVoxel, PlainVoxelArray>* scene,
                                                                            const View* view,
                                                                            const Matrix4f& depth_camera_matrix) {}


template<typename TVoxel, MemoryDeviceType TMemoryDeviceType, ExecutionMode TExecutionMode>
void IndexingEngine<TVoxel, PlainVoxelArray, TMemoryDeviceType, TExecutionMode>::AllocateNearAndBetweenTwoSurfaces(
		VoxelVolume<TVoxel, PlainVoxelArray>* targetVolume, const View* view, const CameraTrackingState* tracking_state) {}

template<typename TVoxel, MemoryDeviceType TMemoryDeviceType, ExecutionMode TExecutionMode>
void IndexingEngine<TVoxel, PlainVoxelArray, TMemoryDeviceType, TExecutionMode>::ResetUtilizedBlockList(VoxelVolume<TVoxel, PlainVoxelArray>* volume) {}
template<typename TVoxel, MemoryDeviceType TMemoryDeviceType, ExecutionMode TExecutionMode>
void IndexingEngine<TVoxel, PlainVoxelArray, TMemoryDeviceType, TExecutionMode>::ResetVisibleBlockList(VoxelVolume<TVoxel, PlainVoxelArray>* volume) {}

template<typename TVoxel, MemoryDeviceType TMemoryDeviceType, ExecutionMode TExecutionMode>
void IndexingEngine<TVoxel, PlainVoxelArray, TMemoryDeviceType, TExecutionMode>::AllocateGridAlignedBox(VoxelVolume<TVoxel, PlainVoxelArray>* volume,
                                                                               const Extent3Di& box) {}


namespace ITMLib{
namespace internal{
template<MemoryDeviceType TMemoryDeviceType, typename TVoxelTarget, typename TVoxelSource>
void AllocateUsingOtherVolume(VoxelVolume<TVoxelTarget, PlainVoxelArray>* target_volume,
                              VoxelVolume<TVoxelSource, PlainVoxelArray>* source_volume){}

template<MemoryDeviceType TMemoryDeviceType, typename TVoxelTarget, typename TVoxelSource>
void AllocateUsingOtherVolume_Bounded(VoxelVolume<TVoxelTarget, PlainVoxelArray>* target_volume,
                                      VoxelVolume<TVoxelSource, PlainVoxelArray>* source_volume,
                                      const Extent3Di& bounds){}
template<MemoryDeviceType TMemoryDeviceType, typename TVoxelTarget, typename TVoxelSource>
void AllocateUsingOtherVolume_OffsetAndBounded(VoxelVolume<TVoxelTarget, PlainVoxelArray>* target_volume,
                                               VoxelVolume<TVoxelSource, PlainVoxelArray>* source_volume,
                                               const Extent3Di& source_bounds, const Vector3i& target_offset){}
} // namespace internal
} // namespace ITMLib




