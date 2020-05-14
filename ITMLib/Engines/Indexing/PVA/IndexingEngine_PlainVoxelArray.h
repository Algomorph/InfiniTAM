//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 5/11/20.
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
#pragma once
#include "../Interface/IndexingEngine.h"

namespace ITMLib{

template<typename TVoxel, MemoryDeviceType TMemoryDeviceType, ExecutionMode TExecutionMode>
class IndexingEngine<TVoxel, PlainVoxelArray, TMemoryDeviceType, TExecutionMode>:
		public IndexingEngineInterface<TVoxel, PlainVoxelArray> {

public: // member functions
	IndexingEngine() = default;
	static IndexingEngine& Instance() {
		static IndexingEngine instance;
		return instance;
	}

	IndexingEngine(IndexingEngine const&) = delete;
	void operator=(IndexingEngine const&) = delete;

	void ResetUtilizedBlockList(VoxelVolume<TVoxel, PlainVoxelArray>* volume) override;
	void ResetVisibleBlockList(VoxelVolume<TVoxel, PlainVoxelArray>* volume) override;
	void AllocateNearSurface(VoxelVolume<TVoxel, PlainVoxelArray>* volume, const View* view,
	                         const CameraTrackingState* tracking_state) override;
	void AllocateNearSurface(VoxelVolume<TVoxel, PlainVoxelArray>* volume, const View* view,
	                         const Matrix4f& depth_camera_matrix = Matrix4f::Identity()) override;
	void AllocateNearAndBetweenTwoSurfaces(VoxelVolume<TVoxel, PlainVoxelArray>* targetVolume,
	                                       const View* view,
	                                       const CameraTrackingState* tracking_state) override;
	void AllocateGridAlignedBox(VoxelVolume<TVoxel, PlainVoxelArray>* volume, const Extent3Di& box) override;

};

} // namespace ITMLib