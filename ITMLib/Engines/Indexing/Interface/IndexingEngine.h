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
#include "../../../Objects/Views/View.h"
#include "../../../Objects/Tracking/CameraTrackingState.h"
#include "../../../Objects/RenderStates/RenderState.h"
#include "../../../Utils/WarpType.h"
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
public:
	/**
	 * \brief Given a view with a new depth image, compute_allocated the
		visible blocks, allocate them and update the hash
		table, as well as the visible block list,
	    so that the new image data can be integrated.
	 * \param volume [out] the volume whose hash needs additional allocations
	 * \param view [in] a view with a new depth image
	 * \param tracking_state [in] tracking state that corresponds to the given view
	 * \param only_update_utilized_block_list [in] whether we want to allocate only the hash entry blocks currently visible
	 * \param reset_utilized_block_list  [in] reset utilized block list before the rest of the operation
	 */
	virtual void
	AllocateNearSurface(VoxelVolume<TVoxel, TIndex>* volume, const View* view,
	                    const CameraTrackingState* tracking_state) = 0;

	/**
	 * \brief Given a view with a new depth image, compute_allocated the
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
	AllocateNearSurface(VoxelVolume<TVoxel, TIndex>* volume, const View* view, const Matrix4f& depth_camera_matrix = Matrix4f::Identity()) = 0;

	virtual void ResetUtilizedBlockList(VoxelVolume<TVoxel, TIndex>* volume) = 0;

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
	                                  const View* view,
	                                  const CameraTrackingState* tracking_state) = 0;


	/**
	 * \brief Allocates (at least) enough space to fit the given
	 * grid-aligned box (within volume allocation bounds, if any, i.e. for array).
	 * \param volume target volume
	 * \param box (voxel-)grid-aligned box where to allocate (in voxel coordinates).
	 */
	virtual void
	AllocateGridAlignedBox(VoxelVolume<TVoxel, TIndex>* volume, const Extent3Di& box) = 0;

};


struct IndexingEngineFactory;
template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
class IndexingEngine :
		public IndexingEngineInterface<TVoxel, TIndex> {
	friend IndexingEngineFactory;
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

	virtual void ResetUtilizedBlockList(VoxelVolume<TVoxel, TIndex>* volume) override;

	virtual void AllocateNearSurface(VoxelVolume<TVoxel, TIndex>* volume, const View* view,
	                                 const CameraTrackingState* tracking_state) override;

	virtual void AllocateNearSurface(VoxelVolume<TVoxel, TIndex>* volume, const View* view,
	                                 const Matrix4f& depth_camera_matrix = Matrix4f::Identity()) override;

	virtual void AllocateNearAndBetweenTwoSurfaces(VoxelVolume<TVoxel, TIndex>* targetVolume,
	                                               const View* view,
	                                               const CameraTrackingState* tracking_state) override;

	void
	AllocateGridAlignedBox(VoxelVolume<TVoxel, TIndex>* volume, const Extent3Di& box) override;

};


namespace internal{
template<MemoryDeviceType TMemoryDeviceType, typename TVoxelTarget, typename TVoxelSource>
void AllocateUsingOtherVolume(VoxelVolume<TVoxelTarget, PlainVoxelArray>* target_volume,
                              VoxelVolume<TVoxelSource, PlainVoxelArray>* source_volume);

template<MemoryDeviceType TMemoryDeviceType, typename TVoxelTarget, typename TVoxelSource>
void AllocateUsingOtherVolume_Bounded(VoxelVolume<TVoxelTarget, PlainVoxelArray>* target_volume,
                                      VoxelVolume<TVoxelSource, PlainVoxelArray>* source_volume,
                                      const Extent3Di& bounds);
template<MemoryDeviceType TMemoryDeviceType, typename TVoxelTarget, typename TVoxelSource>
void AllocateUsingOtherVolume_OffsetAndBounded(VoxelVolume<TVoxelTarget, PlainVoxelArray>* target_volume,
                                               VoxelVolume<TVoxelSource, PlainVoxelArray>* source_volume,
                                               const Extent3Di& source_bounds, const Vector3i& target_offset);

template<MemoryDeviceType TMemoryDeviceType, typename TVoxelTarget, typename TVoxelSource>
void AllocateUsingOtherVolume(VoxelVolume<TVoxelTarget, VoxelBlockHash>* target_volume,
                              VoxelVolume<TVoxelSource, VoxelBlockHash>* source_volume);

template<MemoryDeviceType TMemoryDeviceType, typename TVoxelTarget, typename TVoxelSource>
void AllocateUsingOtherVolume_Bounded(VoxelVolume<TVoxelTarget, VoxelBlockHash>* target_volume,
                                      VoxelVolume<TVoxelSource, VoxelBlockHash>* source_volume,
                                      const Extent3Di& bounds);
template<MemoryDeviceType TMemoryDeviceType, typename TVoxelTarget, typename TVoxelSource>
void AllocateUsingOtherVolume_OffsetAndBounded(VoxelVolume<TVoxelTarget, VoxelBlockHash>* target_volume,
                                               VoxelVolume<TVoxelSource, VoxelBlockHash>* source_volume,
                                               const Extent3Di& source_bounds, const Vector3i& target_offset);
}// namespace internal

template<typename TVoxelTarget, typename TVoxelSource, typename TIndex>
void AllocateUsingOtherVolume(VoxelVolume<TVoxelTarget, TIndex>* target_volume,
                              VoxelVolume<TVoxelSource, TIndex>* source_volume,
                              MemoryDeviceType memory_device_type){
	switch (memory_device_type) {
		case MEMORYDEVICE_CPU:
			internal::AllocateUsingOtherVolume<MEMORYDEVICE_CPU>(target_volume, source_volume);
			break;
		case MEMORYDEVICE_CUDA:
			internal::AllocateUsingOtherVolume<MEMORYDEVICE_CUDA>(target_volume, source_volume);
			break;
		default:
			DIEWITHEXCEPTION_REPORTLOCATION("Unsupported device type.");
	}
}

template<typename TVoxelTarget, typename TVoxelSource, typename TIndex>
void AllocateUsingOtherVolume_Bounded(VoxelVolume<TVoxelTarget, TIndex>* target_volume,
                              VoxelVolume<TVoxelSource, TIndex>* source_volume,
                              MemoryDeviceType memory_device_type){
	switch (memory_device_type) {
		case MEMORYDEVICE_CPU:
			internal::AllocateUsingOtherVolume_Bounded<MEMORYDEVICE_CPU>(target_volume, source_volume);
			break;
		case MEMORYDEVICE_CUDA:
			internal::AllocateUsingOtherVolume_Bounded<MEMORYDEVICE_CUDA>(target_volume, source_volume);
			break;
		default:
			DIEWITHEXCEPTION_REPORTLOCATION("Unsupported device type.");
	}
}

template<typename TVoxelTarget, typename TVoxelSource, typename TIndex>
void AllocateUsingOtherVolume_OffsetAndBounded(VoxelVolume<TVoxelTarget, TIndex>* target_volume,
                                      VoxelVolume<TVoxelSource, TIndex>* source_volume,
                                      MemoryDeviceType memory_device_type){
	switch (memory_device_type) {
		case MEMORYDEVICE_CPU:
			internal::AllocateUsingOtherVolume_OffsetAndBounded<MEMORYDEVICE_CPU>(target_volume, source_volume);
			break;
		case MEMORYDEVICE_CUDA:
			internal::AllocateUsingOtherVolume_OffsetAndBounded<MEMORYDEVICE_CUDA>(target_volume, source_volume);
			break;
		default:
			DIEWITHEXCEPTION_REPORTLOCATION("Unsupported device type.");
	}
}

}//namespace ITMLib

