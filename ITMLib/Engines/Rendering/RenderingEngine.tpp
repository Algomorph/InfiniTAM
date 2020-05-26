//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 5/25/20.
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

#include "Interface/RenderingEngine.h"
#include "../../Utils/Geometry/CheckBlockVisibility.h"

using namespace ITMLib;

namespace ITMLib{
namespace internal{

	template<class TVoxel, MemoryDeviceType TMemoryDeviceType>
	struct RenderingEngine_IndexSpecialized<TVoxel, PlainVoxelArray, TMemoryDeviceType>{
		void FindVisibleBlocks(VoxelVolume<TVoxel,PlainVoxelArray> *volume, const ORUtils::SE3Pose *pose, const Intrinsics *intrinsics,
		                       RenderState *render_state) const {

		}
	};

	template<class TVoxel, MemoryDeviceType TMemoryDeviceType>
	struct RenderingEngine_IndexSpecialized<TVoxel, VoxelBlockHash, TMemoryDeviceType>{
		void FindVisibleBlocks(VoxelVolume<TVoxel,VoxelBlockHash> *volume, const ORUtils::SE3Pose *pose, const Intrinsics *intrinsics,
		                       RenderState *render_state) const {
			const HashEntry* hash_table = volume->index.GetEntries();
			int hash_entry_count = volume->index.hash_entry_count;
			float voxel_size = volume->GetParameters().voxel_size;
			Vector2i image_size = render_state->renderingRangeImage->dimensions;

			Matrix4f M = pose->GetM();
			Vector4f projParams = intrinsics->projectionParamsSimple.all;

			int visible_block_count = 0;
			int* visible_block_hash_codes = volume->index.GetVisibleBlockHashCodes();

			//build visible list
			for (int hash_code = 0; hash_code < hash_entry_count; hash_code++) {
				unsigned char block_visibility_type = 0;// = blockVisibilityTypes[targetIdx];
				const HashEntry& hash_entry = hash_table[hash_code];

				if (hash_entry.ptr >= 0) {
					bool is_visible, is_visible_enlarged;
					CheckVoxelHashBlockVisibility<false>(is_visible, is_visible_enlarged, hash_entry.pos, M, projParams,
					                                     voxel_size,image_size);
					block_visibility_type = is_visible;
				}

				if (block_visibility_type > 0) {
					visible_block_hash_codes[visible_block_count] = hash_code;
					visible_block_count++;
				}
			}
			volume->index.SetVisibleBlockCount(visible_block_count);
		}

	};
} // namespace internal
} // namespace ITMLib
template<class TVoxel, class TIndex, MemoryDeviceType TMemoryDeviceType>
void RenderingEngine<TVoxel, TIndex, TMemoryDeviceType>::FindVisibleBlocks(VoxelVolume<TVoxel, TIndex>* volume, const ORUtils::SE3Pose* pose,
                                                                           const Intrinsics* intrinsics, RenderState* render_state) const {
	index_specialized_engine.FindVisibleBlocks(volume, pose, intrinsics, render_state);
}

template<class TVoxel, class TIndex, MemoryDeviceType TMemoryDeviceType>
int RenderingEngine<TVoxel, TIndex, TMemoryDeviceType>::CountVisibleBlocks(
		const VoxelVolume<TVoxel, TIndex>* volume, const RenderState* render_state, int min_block_index, int max_block_index) const {
	return 0;
}


template<class TVoxel, class TIndex, MemoryDeviceType TMemoryDeviceType>
void RenderingEngine<TVoxel, TIndex, TMemoryDeviceType>::CreateExpectedDepths(
		const VoxelVolume<TVoxel, TIndex>* volume, const ORUtils::SE3Pose* pose, const Intrinsics* intrinsics, RenderState* render_state) {

}

template<class TVoxel, class TIndex, MemoryDeviceType TMemoryDeviceType>
void RenderingEngine<TVoxel, TIndex, TMemoryDeviceType>::RenderImage(
		VoxelVolume<TVoxel, TIndex>* volume, const ORUtils::SE3Pose* pose, const Intrinsics* intrinsics, const RenderState* render_state,
		UChar4Image* output_image, IRenderingEngine::RenderImageType type, IRenderingEngine::RenderRaycastSelection raycast_type) {

}

template<class TVoxel, class TIndex, MemoryDeviceType TMemoryDeviceType>
void
RenderingEngine<TVoxel, TIndex, TMemoryDeviceType>::FindSurface(
		VoxelVolume<TVoxel, TIndex>* volume, const ORUtils::SE3Pose* pose, const Intrinsics* intrinsics, const RenderState* render_state) {

}

template<class TVoxel, class TIndex, MemoryDeviceType TMemoryDeviceType>
void RenderingEngine<TVoxel, TIndex, TMemoryDeviceType>::CreatePointCloud(
		VoxelVolume<TVoxel, TIndex>* volume, const View* view, CameraTrackingState* camera_tracking_state, RenderState* render_state, bool skipPoints) {

}

template<class TVoxel, class TIndex, MemoryDeviceType TMemoryDeviceType>
void RenderingEngine<TVoxel, TIndex, TMemoryDeviceType>::CreateICPMaps(
		VoxelVolume<TVoxel, TIndex>* volume, const View* view, CameraTrackingState* camera_tracking_state, RenderState* render_state) {

}

template<class TVoxel, class TIndex, MemoryDeviceType TMemoryDeviceType>
void RenderingEngine<TVoxel, TIndex, TMemoryDeviceType>::ForwardRender(
		const VoxelVolume<TVoxel, TIndex>* volume, const View* view, CameraTrackingState* camera_tracking_state, RenderState* render_state) {

}
