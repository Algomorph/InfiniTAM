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
#include "../../../Objects/Volume/VoxelVolume.h"
#include "../../../Utils/HashBlockProperties.h"
#include "../../../Objects/Views/ITMView.h"
#include "../../../Objects/Tracking/CameraTrackingState.h"
#include "../../../Objects/RenderStates/RenderState.h"
#include "../../Common/WarpType.h"
#include "../../../../ORUtils/MemoryDeviceType.h"
#include "../../../GlobalTemplateDefines.h"
#include "../../../Objects/Volume/VoxelBlockHash.h"


//ORUtils

namespace ITMLib {

/**
 * \brief A utility for allocating additional space within or around a voxel volume (expanding it) based on various inputs
 * \details Note: if a volume index has strict bounds, as in the case of plain voxel array, does not grow or shrink those bounds.
 * \tparam TVoxel type of voxels
 * \tparam TIndex type of index
 */
template<typename TVoxel, typename TIndex>
class IndexingEngineInterface {

	/**
	 * \brief Given a view with a new depth image, compute the
		visible blocks, allocate them and update the hash
		table, as well as the visible block list,
	    so that the new image data can be integrated.
	 * \param volume [out] the volume whose hash needs additional allocations
	 * \param view [in] a view with a new depth image
	 * \param tracking_state [in] tracking state that corresponds to the given view
	 * \param only_update_utilized_block_list [in] whether we want to allocate only the hash entry blocks currently visible
	 * \param reset_utilized_block_list  [in] reset visibility list prior to the rest of the operation
	 */
	virtual void
	AllocateNearSurface(VoxelVolume<TVoxel, TIndex>* volume, const ITMView* view,
	                    const CameraTrackingState* tracking_state,
	                    bool only_update_utilized_block_list, bool reset_utilized_block_list) = 0;

	/**
	 * \brief Given a view with a new depth image, compute the
		visible blocks, allocate them and update the hash
		table so that the new image data can be integrated.
	 * \param volume [out] the volume whose hash needs additional allocations
	 * \param view [in] a view with a new depth image
	 * \param depth_camera_matrix [in] transformation of the camera from world origin (initial position) to
	 * where the camera was at the given view's frame
	 * \param only_update_utilized_block_list [in] whether we want to allocate only the hash entry blocks currently visible
	 * \param reset_utilized_block_list  [in] reset visibility list upon completion
	 */
	virtual void
	AllocateNearSurface(VoxelVolume<TVoxel, TIndex>* volume, const ITMView* view,
	                    const Matrix4f& depth_camera_matrix = Matrix4f::Identity(),
	                    bool only_update_utilized_block_list = false, bool reset_utilized_block_list = false) = 0;


	/**
	 * \brief Given a view with a new depth image and a previous (source) volume, ray trace to determine which blocks
	 * span the region between the narrow band based on the depth and the intersection of the source scene narrow band with
	 * the camera frustum. Then allocate these blocks in the target scene and update the hash table so that the new
	 * image data can be integrated.
	 * \details Does nothing for a plain-voxel-array volume
	 * \param volume [out] the volume whose hash needs additional allocations
	 * \param sourceRenderState [in] contains information of the other surface used to determine the allocation bounds
	 * \param view [in] a view with a new depth image
	 * \param depth_camera_matrix [in] transformation of the camera from world origin (initial position) to
	 * where the camera was at the given view's frame
	 * \param onlyUpdateAllocatedList [in] whether we want to skip allocation and just update the list with hashes of
	 * the blocks that fall within the span.
	 * \param resetAllocatedList  [in] reset allocated list before the operation.
	 */
	virtual void
	AllocateNearAndBetweenTwoSurfaces(VoxelVolume<TVoxel, TIndex>* volume,
	                                  const CameraTrackingState* tracking_state,
	                                  const ITMView* view) = 0;
	/**
	 * \brief Allocate all hash blocks at given coordinates
	 * \param volume - volume, where to allocate
	 * \param block_coordinates coordinates of blocks to allocate (in blocks, not voxels)
	 */
	virtual void
	AllocateBlockList(VoxelVolume<TVoxel, TIndex>* volume, ORUtils::MemoryBlock<Vector3s> block_coordinates, int new_block_count) = 0;

};

template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
class IndexingEngine :
		public IndexingEngineInterface<TVoxel, TIndex> {
private:
	IndexingEngine() = default;
public:
	static IndexingEngine& Instance() {
		static IndexingEngine instance; // Guaranteed to be destroyed.
		// Instantiated on first use.
		return instance;
	}

	IndexingEngine(IndexingEngine const&) = delete;
	void operator=(IndexingEngine const&) = delete;

	virtual void AllocateNearSurface(VoxelVolume<TVoxel, TIndex>* volume, const ITMView* view,
	                                 const CameraTrackingState* tracking_state, bool onlyUpdateVisibleList,
	                                 bool resetVisibleList) override;

	virtual void AllocateNearSurface(VoxelVolume<TVoxel, TIndex>* volume, const ITMView* view,
	                                 const Matrix4f& depth_camera_matrix = Matrix4f::Identity(),
	                                 bool only_update_visible_list = false, bool reset_visible_list = false) override;

	virtual void AllocateNearAndBetweenTwoSurfaces(VoxelVolume<TVoxel, TIndex>* targetVolume,
	                                               const CameraTrackingState* tracking_state,
	                                               const ITMView* view) override;

	void
	AllocateBlockList(VoxelVolume<TVoxel, TIndex>* volume, ORUtils::MemoryBlock<Vector3s> block_coordinates, int new_block_count) override;


	template<typename TVoxelTarget, typename TVoxelSource>
	void AllocateUsingOtherVolume(VoxelVolume<TVoxelTarget, TIndex>* targetVolume,
	                              VoxelVolume<TVoxelSource, TIndex>* sourceVolume);

	template<typename TVoxelTarget, typename TVoxelSource>
	void AllocateUsingOtherVolumeExpanded(VoxelVolume<TVoxelTarget, TIndex>* targetVolume,
	                                      VoxelVolume<TVoxelSource, TIndex>* sourceVolume);


	template<typename TVoxelTarget, typename TVoxelSource>
	void AllocateUsingOtherVolumeAndSetVisibilityExpanded(VoxelVolume<TVoxelTarget, TIndex>* targetVolume,
	                                                      VoxelVolume<TVoxelSource, TIndex>* sourceVolume,
	                                                      ITMView* view,
	                                                      const Matrix4f& depth_camera_matrix = Matrix4f::Identity());

	template<WarpType TWarpType, typename TWarp>
	void AllocateFromWarpedVolume(
			VoxelVolume<TWarp, TIndex>* warpField,
			VoxelVolume<TVoxel, TIndex>* sourceTSDF,
			VoxelVolume<TVoxel, TIndex>* targetTSDF);
};

}//namespace ITMLib

