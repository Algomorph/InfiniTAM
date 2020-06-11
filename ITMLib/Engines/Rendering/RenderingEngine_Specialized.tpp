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

#include "../Traversal/Interface/Regular2DSubGridArrayTraversal.h"
#include "../../../ORUtils/CrossPlatformMacros.h"
#include "RenderingEngine_Specialized.h"
#include "Shared/RenderingEngine_Shared.h"
#include "Shared/RenderingEngine_Functors.h"
#include "../Traversal/Shared/JobCountPolicy.h"

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

	HashTableTraversalEngine<TMemoryDeviceType>::template TraverseUtilized_Padded(volume->index, functor);
	volume->index.SetVisibleBlockCount(functor.GetVisibleBlockCount());
}

template<class TVoxel, MemoryDeviceType TMemoryDeviceType>
int RenderingEngine_Specialized<TVoxel, VoxelBlockHash, TMemoryDeviceType>::CountVisibleBlocks(
		const VoxelVolume<TVoxel, VoxelBlockHash>* volume, int min_block_id, int max_block_id) const {
	CountVisibleBlocksInListIdRangeFunctor<TMemoryDeviceType> functor(min_block_id, max_block_id);
	HashTableTraversalEngine<TMemoryDeviceType>::template TraverseVisible(volume->index, functor);
	return functor.GetCurrentVisibleBlockInIDRangeCount();
}

template<class TVoxel, MemoryDeviceType TMemoryDeviceType>
void RenderingEngine_Specialized<TVoxel, VoxelBlockHash, TMemoryDeviceType>::CreateExpectedDepths(
		const VoxelVolume<TVoxel, VoxelBlockHash>* volume, const ORUtils::SE3Pose* pose, const Intrinsics* intrinsics,
		RenderState* render_state) const {

	// initialize with very large value for "min" and small value for "max" ray distance
	FillExpectedDepthsWithClippingDistancesFunctor<TMemoryDeviceType> fill_with_clipping_planes_functor(FAR_AWAY, VERY_CLOSE);
	ImageTraversalEngine<TMemoryDeviceType>::Traverse(render_state->renderingRangeImage, fill_with_clipping_planes_functor);

	auto depth_image_size = render_state->renderingRangeImage->dimensions;
	float voxel_size = volume->GetParameters().voxel_size;
	//TODO: optimize by making rendering_blocks a member variable to avoid repeated allocations & deallocations (requires converting it to raw pointer or removing const-ness from the function)
	ORUtils::MemoryBlock<RenderingBlock> rendering_blocks(ITMLib::MAX_RENDERING_BLOCKS, TMemoryDeviceType);
	ProjectAndSplitBlocksFunctor<TMemoryDeviceType> project_and_split_blocks_functor(
			rendering_blocks, pose->GetM(), intrinsics->projectionParamsSimple.all, depth_image_size, voxel_size);

	//TODO: (potential optimization) figure out whether using the "Visible" list instead of the "Utilized" list makes more sense here (and how to account for visible
	// list staying up-to-date in canonical.
	// the visible list is only reliable if it's up-to-date. If you can ensure that it is, and keeping it up-to-date takes less resources than using the
	// bigger "utilized" list, change this back to "Visible" list like it was in the original InfiniTAM repo.

	HashTableTraversalEngine<TMemoryDeviceType>::template TraverseUtilized_Padded(volume->index, project_and_split_blocks_functor);
	unsigned int final_rendering_block_count = project_and_split_blocks_functor.GetRenderingBlockCount();
	// this bounding is necessary due to prefix sums potentially used when parallelization is on (see comment in functor)
	if(final_rendering_block_count >= MAX_RENDERING_BLOCKS) final_rendering_block_count = MAX_RENDERING_BLOCKS;

	FillBlocksFunctor<TMemoryDeviceType> fill_blocks_functor(*render_state->renderingRangeImage);
	// go through rendering blocks
	Regular2DSubGridArrayTraversal<TMemoryDeviceType>::template Traverse<rendering_block_size_x, rendering_block_size_y>(
			rendering_blocks, final_rendering_block_count, fill_blocks_functor,
			CPU_AND_GPU_NONCAPTURE_LAMBDA()(const RenderingBlock& block, int& start_x, int& end_x, int& start_y, int& end_y) {
				start_x = block.upper_left.x, end_x = block.lower_right.x, start_y = block.upper_left.y, end_y = block.lower_right.y;
			});

}
