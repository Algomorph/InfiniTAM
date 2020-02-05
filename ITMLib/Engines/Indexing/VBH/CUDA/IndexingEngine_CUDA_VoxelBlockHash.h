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

#include "../../Interface/IndexingEngine.h"
#include "../IndexingEngine_VoxelBlockHash.h"
#include "../../../Common/WarpType.h"

namespace ITMLib {

template<typename TVoxel>
class IndexingEngine<TVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA> :
		public IndexingEngine_VoxelBlockHash<TVoxel, MEMORYDEVICE_CUDA,
		IndexingEngine<TVoxel, VoxelBlockHash, MEMORYDEVICE_CUDA> > {
private:
	IndexingEngine() = default;

	void SetVisibilityToVisibleAtPreviousFrameAndUnstreamed(HashBlockVisibility* hashBlockVisibilityTypes,
	                                                        const int* visibleBlockHashCodes,
	                                                        int visibleHashBlockCount);
public:
	static IndexingEngine& Instance() {
		static IndexingEngine instance; // Guaranteed to be destroyed.
		// Instantiated on first use.
		return instance;
	}

	IndexingEngine(IndexingEngine const&) = delete;
	void operator=(IndexingEngine const&) = delete;

	void AllocateFromDepth(VoxelVolume<TVoxel, VoxelBlockHash>* volume, const ITMView* view,
	                       const CameraTrackingState* tracking_state, bool onlyUpdateVisibleList, bool resetVisibleList) override;

	void AllocateFromDepth(VoxelVolume<TVoxel, VoxelBlockHash>* volume, const ITMView* view,
	                       const Matrix4f& depth_camera_matrix = Matrix4f::Identity(),
	                       bool only_update_visible_list = false, bool resetVisibleList = false) override;

	void AllocateFromDepthAndSdfSpan(VoxelVolume<TVoxel, VoxelBlockHash>* volume,
	                                 const CameraTrackingState* tracking_state,
	                                 const ITMView* view,
	                                 bool only_update_utilized_list = false, bool reset_utilized_list = false) override;

	void AllocateHashEntriesUsingLists(VoxelVolume <TVoxel, VoxelBlockHash>* volume) override;

	void AllocateHashEntriesUsingLists_SetVisibility(VoxelVolume <TVoxel, VoxelBlockHash>* volume) override;

	void BuildVisibilityList(VoxelVolume<TVoxel, VoxelBlockHash>* volume, const ITMView* view,
	                         const Matrix4f& depth_camera_matrix = Matrix4f::Identity());

	HashEntry FindHashEntry(const VoxelBlockHash& index, const Vector3s& coordinates) override;
	HashEntry FindHashEntry(const VoxelBlockHash& index, const Vector3s& coordinates, int& hashCode);
	bool AllocateHashBlockAt(VoxelVolume<TVoxel, VoxelBlockHash>* volume, Vector3s at, int& hashCode) override;

	template<typename TVoxelATarget, typename TVoxelASource>
	void AllocateUsingOtherVolume(VoxelVolume <TVoxelATarget, VoxelBlockHash>* targetVolume,
	                              VoxelVolume <TVoxelASource, VoxelBlockHash>* sourceVolume);

	template<typename TVoxelTarget, typename TVoxelSource>
	void AllocateUsingOtherVolumeExpanded(VoxelVolume<TVoxelTarget, VoxelBlockHash>* targetVolume,
	                                      VoxelVolume<TVoxelSource, VoxelBlockHash>* sourceVolume);

	template<typename TVoxelTarget, typename TVoxelSource>
	void AllocateUsingOtherVolumeAndSetVisibilityExpanded(VoxelVolume<TVoxelTarget, VoxelBlockHash>* targetVolume,
	                                                      VoxelVolume<TVoxelSource, VoxelBlockHash>* sourceVolume,
	                                                      ITMView* view, const Matrix4f& depth_camera_matrix = Matrix4f::Identity());
};
} //namespace ITMLib

