//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 5/29/20.
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


#include "RenderingEngine_Specialized.h"
#include "Shared/RenderingEngine_Shared.h"
#include "Shared/RenderingEngine_Functors.h"

using namespace ITMLib;
using namespace ITMLib::internal;


template<class TVoxel, MemoryDeviceType TMemoryDeviceType>
void RenderingEngine_Specialized<TVoxel, PlainVoxelArray, TMemoryDeviceType>::CreateExpectedDepths(
		const VoxelVolume<TVoxel, PlainVoxelArray>* volume, const ORUtils::SE3Pose* pose, const Intrinsics* intrinsics,
		RenderState* render_state) const {
	//TODO (from original InfiniTAM): "this could be improved a bit..." What they mean is that, for PlainVoxelArray, there isn't currently any
	// expected depth prediction, hence rendering / ray tracing isn't optimized.
	FillExpectedDepthsWithClippingDistancesFunctor<TMemoryDeviceType> functor(volume->GetParameters().near_clipping_distance,
	                                                                          volume->GetParameters().far_clipping_distance);
	ImageTraversalEngine<TMemoryDeviceType>::Traverse(render_state->renderingRangeImage, functor);
}

template<class TVoxel, MemoryDeviceType TMemoryDeviceType>
void RenderingEngine_Specialized<TVoxel, VoxelBlockHash, TMemoryDeviceType>::FindVisibleBlocks(
		VoxelVolume<TVoxel, VoxelBlockHash>* volume, const ORUtils::SE3Pose* pose, const Intrinsics* intrinsics, RenderState* render_state) const {
	FindVisibleBlocksFunctor<TMemoryDeviceType> functor(
			volume->index.GetVisibleBlockHashCodes(),
			volume->GetParameters().voxel_size,
			render_state->renderingRangeImage->dimensions,
			pose->GetM(),
			intrinsics->projectionParamsSimple.all
	);

	HashTableTraversalEngine<TMemoryDeviceType>::TraverseUtilizedWithHashCode(volume->index, functor);
	volume->index.SetVisibleBlockCount(functor.GetVisibleBlockCount());
}

template<class TVoxel, MemoryDeviceType TMemoryDeviceType>
int RenderingEngine_Specialized<TVoxel, VoxelBlockHash, TMemoryDeviceType>::CountVisibleBlocks(
		const VoxelVolume<TVoxel, VoxelBlockHash>* volume, int min_block_id, int max_block_id) const {
	CountVisibleBlocksInListIdRangeFunctor<TMemoryDeviceType> functor(min_block_id, max_block_id);
	HashTableTraversalEngine<TMemoryDeviceType>::TraverseVisibleWithHashCode(volume->index, functor);
	return functor.GetCurrentVisibleBlockInIDRangeCount();
}

template<class TVoxel, MemoryDeviceType TMemoryDeviceType>
void RenderingEngine_Specialized<TVoxel, VoxelBlockHash, TMemoryDeviceType>::CreateExpectedDepths(
		const VoxelVolume<TVoxel, VoxelBlockHash>* volume, const ORUtils::SE3Pose* pose, const Intrinsics* intrinsics,
		RenderState* render_state) const {

	Vector2f* ray_bound_data = render_state->renderingRangeImage->GetData(MEMORYDEVICE_CPU);

	// initialize with very large value for "min" and small value for "max" ray distance
	FillExpectedDepthsWithClippingDistancesFunctor<TMemoryDeviceType> fill_with_clipping_planes_functor(FAR_AWAY, VERY_CLOSE);
	ImageTraversalEngine<TMemoryDeviceType>::Traverse(render_state->renderingRangeImage, fill_with_clipping_planes_functor);

	DECLARE_ATOMIC(unsigned int, rendered_block_count);
	INITIALIZE_ATOMIC(unsigned int, rendered_block_count, 0u);
//	projectAndSplitBlocks_device <<<cuda_grid_size, cuda_block_size >>>(hash_entries, visible_block_hash_codes, visible_block_count, pose->GetM(),
//			intrinsics->projectionParamsSimple.all, imgSize, voxelSize, rendering_block_list_device, block_count_device);
	auto depth_image_size = render_state->renderingRangeImage->dimensions;
	float voxel_size = volume->GetParameters().voxel_size;

	ProjectAndSplitBlocksFunctor<TMemoryDeviceType> project_and_split_blocks_functor(pose->GetM(), intrinsics->projectionParamsSimple.all, depth_image_size, voxel_size);

	//TODO: (potential optimization) figure out whether using the "Visible" list instead of the "Utilized" list makes more sense here.
	// the visible list is only reliable if it's up-to-date. If you can ensure that it is, and keeping it up-to-date takes less resources than using the
	// bigger "utilized" list, change this back to "Visible" list like it was in the original InfiniTAM repo.

	HashTableTraversalEngine<TMemoryDeviceType>::TraverseUtilizedWithHashCode(volume->index, project_and_split_blocks_functor);

	//TODO
	DIEWITHEXCEPTION_REPORTLOCATION("Not finished");
	// go through rendering blocks
//	for (int i_rendered_block = 0; i_rendered_block < rendered_block_count; ++i_rendered_block) {
		// fill pixel ray bound data
//		const RenderingBlock& rendering_block = render_blocks[i_rendered_block];
//
//		for (int y = rendering_block.upper_left.y; y <= rendering_block.lower_right.y; ++y) {
//			for (int x = rendering_block.upper_left.x; x <= rendering_block.lower_right.x; ++x) {
//				Vector2f& pixel_ray_bound = ray_bound_data[x + y * depth_image_size.x];
//				if (pixel_ray_bound.x > rendering_block.z_range.x) pixel_ray_bound.x = rendering_block.z_range.x;
//				if (pixel_ray_bound.y < rendering_block.z_range.y) pixel_ray_bound.y = rendering_block.z_range.y;
//			}
//		}
//	}

	CLEAN_UP_ATOMIC(rendered_block_count);
}
