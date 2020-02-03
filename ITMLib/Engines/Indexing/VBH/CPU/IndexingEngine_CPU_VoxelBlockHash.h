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

#include "../IndexingEngine_VoxelBlockHash.h"


namespace ITMLib {

template<typename TVoxel>
class IndexingEngine<TVoxel, VoxelBlockHash, MEMORYDEVICE_CPU> :
		public IndexingEngine_VoxelBlockHash<TVoxel, MEMORYDEVICE_CPU,
				IndexingEngine<TVoxel, VoxelBlockHash, MEMORYDEVICE_CPU>> {
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
	                       const CameraTrackingState* trackingState, bool onlyUpdateVisibleList,
	                       bool resetVisibleList) override;

	void AllocateFromDepth(VoxelVolume<TVoxel, VoxelBlockHash>* volume, const ITMView* view,
	                       const Matrix4f& depth_camera_matrix = Matrix4f::Identity(),
	                       bool onlyUpdateVisibleList = false, bool resetVisibleList = false) override;

	void AllocateFromDepthAndSdfSpan(VoxelVolume<TVoxel, VoxelBlockHash>* volume,
	                                 const RenderState* sourceRenderState,
	                                 const ITMView* view,
	                                 const Matrix4f& depth_camera_matrix = Matrix4f::Identity(),
	                                 bool onlyUpdateAllocatedList = false, bool resetAllocatedList = false) override;


	void BuildVisibilityList(VoxelVolume<TVoxel, VoxelBlockHash>* volume, const ITMView* view,
	                         const Matrix4f& depth_camera_matrix = Matrix4f::Identity());

	void AllocateHashEntriesUsingLists(VoxelVolume<TVoxel, VoxelBlockHash>* volume) override;

	void AllocateHashEntriesUsingLists_SetVisibility(VoxelVolume<TVoxel, VoxelBlockHash>* volume) override;

	HashEntry FindHashEntry(const VoxelBlockHash& index, const Vector3s& coordinates) override;
	HashEntry FindHashEntry(const VoxelBlockHash& index, const Vector3s& coordinates, int& hashCode);


	bool AllocateHashBlockAt(VoxelVolume<TVoxel, VoxelBlockHash>* volume, Vector3s at, int& hashCode) override;

	template<typename TVoxelTarget, typename TVoxelSource>
	void AllocateUsingOtherVolume(VoxelVolume<TVoxelTarget, VoxelBlockHash>* targetVolume,
	                              VoxelVolume<TVoxelSource, VoxelBlockHash>* sourceVolume);
	/**
	 * \brief Allocate the same blocks in the target volume as are allocated in the source volume, plus an additional
	 * one-ring of blocks around them. Does not modify previously-existing allocation in the target volume.
	 * \tparam TVoxelTarget voxel type of the target volume
	 * \tparam TVoxelSource voxel type of the source volume
	 * \param targetVolume target volume
	 * \param sourceVolume source volume
	 */
	template<typename TVoxelTarget, typename TVoxelSource>
	void AllocateUsingOtherVolumeExpanded(VoxelVolume<TVoxelTarget, VoxelBlockHash>* targetVolume,
	                                      VoxelVolume<TVoxelSource, VoxelBlockHash>* sourceVolume);

	/**
	 * \brief Allocate the same blocks in the target volume as are allocated in the source volume, plus an additional
	 * one-ring of blocks around them, and set their visibility relative to the provided view assuming the provided
	 * depth camera matrix. Does not modify previously-existing allocation in the target volume.
	 * \tparam TVoxelTarget voxel type of the target volume
	 * \tparam TVoxelSource voxel type of the source volume
	 * \param targetVolume target volume
	 * \param sourceVolume source volume
	 * \param view the view (assumed to be the view that was used to allocate blocks in the source volume)
	 * \param depth_camera_matrix transformation of the camera from world origin to it's position that corresponds to
	 * the provided view
	 */
	template<typename TVoxelTarget, typename TVoxelSource>
	void AllocateUsingOtherVolumeAndSetVisibilityExpanded(VoxelVolume<TVoxelTarget, VoxelBlockHash>* targetVolume,
	                                                      VoxelVolume<TVoxelSource, VoxelBlockHash>* sourceVolume,
	                                                      ITMView* view,
	                                                      const Matrix4f& depth_camera_matrix = Matrix4f::Identity());

};
} //namespace ITMLib
