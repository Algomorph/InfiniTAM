//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 5/26/20.
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

#include "../../../../ORUtils/MemoryDeviceType.h"
#include "../../../../ORUtils/PlatformIndependentAtomics.h"
#include "../../../../ORUtils/CrossPlatformMacros.h"
#include "../../../Objects/Volume/VoxelBlockHash.h"
#include "../../../Utils/Geometry/CheckBlockVisibility.h"
#include "../../../../ORUtils/PlatformIndependedParallelSum.h"
#include "RenderingEngine_Shared.h"
#ifdef __CUDACC__
#include "../../../Utils/CUDAUtils.h"
#endif

namespace ITMLib {
//_DEBUG
//#define __CUDACC__
#ifdef __CUDACC__
constexpr int thread_group_size = 256;
template <typename T>
__device__ inline static int ComputePrefixSum_GS256_test(T element, T *sum, int thread_id)
{
	__shared__ unsigned int prefix_buffer[thread_group_size];
	__shared__ unsigned int prior_thread_group_sum; // "groupOffset" in legacy InfiniTAM/ORUtils code.

	prefix_buffer[thread_id] = element;
	__syncthreads();

	int s1, s2;

	//for localId == 0x00000001 (1), it. 0: prefix_buffer[1] += prefix_buffer[0] (final result: prefix_buffer[1] + prefix_buffer[0])
	//for localId == 0x00000011 (3), it. 0: prefix_buffer[3] += prefix_buffer[2]
	//                               it. 1: prefix_buffer[3] += prefix_buffer[1] (final result: sum of prefix_buffer from 0 to 3)
	//for localId == 0x00000101 (5), it. 0: prefix_buffer[5] += prefix_buffer[4] (final result: prefix_buffer[4] + prefix_buffer[5])
	//for localId == 0x00000111 (7), it. 0: prefix_buffer[7] += prefix_buffer[6]
	//                               it. 1: prefix_buffer[7] += prefix_buffer[5]
	//                               it. 2: prefix_buffer[7] += prefix_buffer[3] (final result: sum of prefix_buffer from 0 to 8)
	//for localId == 0x00001001 (9), it. 0: prefix_buffer[9] += prefix_buffer[8] (final result: prefix_buffer[8] + prefix_buffer[9])
	for (s1 = 1, s2 = 1; s1 < thread_group_size; s1 <<= 1)
	{
		s2 |= s1;
		if ((thread_id & s2) == s2) prefix_buffer[thread_id] += prefix_buffer[thread_id - s1];
		__syncthreads();
	}
	// at this point, we must have s1 == localSize >> 1 [s1 == 0x10000000 for localSize == 256],
	// and last s2 |= s1 means s2 is now a string of 1-bits up to localSize >> 1 [s2 == 0x11111111 for localSize == 256]
	// s1 >> 2 == 0x00100000
	// s2 >> 1 == 0x01111111
	// for next loop, when s1 == 0x00000001 (last iteration), s2 == 0x00000011
	// for localId == 0x00000011 (3), last iteration will do prefix_buffer[4] += prefix_buffer[3], final result for prefix_buffer[4]: sum of orig. prefix_buffer from 0 to 4
	// for localId == 0x00000111 (7), last iteration will do prefix_buffer[8] += prefix_buffer[7], final result for prefix_buffer[8]: sum of orig. prefix_buffer from 0 to 8
	//                                previous-to-last iteration will do prefix_buffer[9] += prefix_buffer[7], final result for prefix_buffer[9]: sum of orig. prefix_buffer from 0 to 9
	// for localId == 0x00001111 (15), last iteration will do prefix_buffer[16] += prefix_buffer[15], ... you get the idea.
	//                                 previous-to-last iteration will do prefix_buffer[17] += prefix_buffer[15],
	//                                 prev-prev-to-last iteration will do prefix_buffer[19] += prefix_buffer[15]
	for (s1 >>= 2, s2 >>= 1; s1 >= 1; s1 >>= 1, s2 >>= 1)
	{
		//for localSize = 256, if localId = 255, then localId + 1 goes outside the block
		// (and s2 of 0x1 would trigger that), hence, skip it
		if (thread_id != thread_group_size - 1 && (thread_id & s2) == s2) prefix_buffer[thread_id + s1] += prefix_buffer[thread_id];
		__syncthreads();
	}
	// at this point, prefix_buffer[localSize-1] has to contain the sum of the whole "group",
	// i.e. sum of elements from x localSize (256?) threads
	// then, groupOffset is the sum across the blocks of all elements before this "group", i.e. thread block
	if (thread_id == 0 && prefix_buffer[thread_group_size - 1] > 0) prior_thread_group_sum = atomicAdd(sum, prefix_buffer[thread_group_size - 1]);
	int test_thread = 255;
	if (thread_id == test_thread) printf("(thread_id %d, block_id %d) buffer val: %d, prior_sum: %d, last value in buffer: %d\n", test_thread, blockIdx.x, prefix_buffer[test_thread], prior_thread_group_sum, prefix_buffer[thread_group_size - 1]);
	__syncthreads();

	int current_sum;// = prior_thread_group_sum + prefix_buffer[localId] - 1;
	// the final "offset", i.e. sum up to this element, is the current "group offset"
	// (sum of elements from "prior" blocks/groups) plus the sum of elements up to and including
	// the thread within the current block.
	if (thread_id == 0) {
		if (prefix_buffer[thread_id] == 0) current_sum = -1;
		else current_sum = prior_thread_group_sum;
	} else {
		if (prefix_buffer[thread_id] == prefix_buffer[thread_id - 1]) current_sum = -1;
		else current_sum = prior_thread_group_sum + prefix_buffer[thread_id - 1];
	}

	return current_sum;
}
#endif

template<MemoryDeviceType TMemoryDeviceType>
struct FindVisibleBlocksFunctor {
private: // member variables
	int* visible_block_hash_codes;
	const float voxel_size;
	const Vector2i depth_image_size;
	const Matrix4f depth_camera_pose;
	const Vector4f depth_camera_projection_parameters;


	DECLARE_ATOMIC(int, visible_block_count);
public: // member functions
	FindVisibleBlocksFunctor(int* visible_block_hash_codes, float voxel_size, const Vector2i& depth_image_size,
	                         const Matrix4f& depth_camera_pose, const Vector4f depth_camera_projection_parameters)
			: visible_block_hash_codes(visible_block_hash_codes),
			  voxel_size(voxel_size),
			  depth_image_size(depth_image_size),
			  depth_camera_pose(depth_camera_pose),
			  depth_camera_projection_parameters(depth_camera_projection_parameters) {
		INITIALIZE_ATOMIC(int, visible_block_count, 0);
	}

	~FindVisibleBlocksFunctor() {
		CLEAN_UP_ATOMIC(visible_block_count);
	}

	_DEVICE_WHEN_AVAILABLE_
	void operator()(const HashEntry& hash_entry, const int hash_code) {
		bool is_visible, is_visible_enlarged;
		CheckVoxelHashBlockVisibility<false>(is_visible, is_visible_enlarged, hash_entry.pos, depth_camera_pose,
		                                     depth_camera_projection_parameters, voxel_size, depth_image_size);

		//_DEBUG
#ifdef __CUDACC__
		__syncthreads();
		int visible_block_index = ComputePrefixSum_GS256_test<int>(is_visible, visible_block_count, threadIdx.x);
#else
		int visible_block_index = ORUtils::ParallelSum<TMemoryDeviceType>::template Add1D<int>(is_visible, visible_block_count);
#endif

		//_DEBUG alloc
//		if(visible_block_index == 1){
//#ifdef __CUDACC__
//			printf("hash code: %d\nthreadId.x: %d; blockId.x: %d\n", hash_code, threadIdx.x, blockIdx.x);
//#endif
//		}
		if(visible_block_index != -1) visible_block_hash_codes[visible_block_index] = hash_code;

	}

	int GetVisibleBlockCount() {
		return GET_ATOMIC_VALUE_CPU(visible_block_count);
	}
};

template<MemoryDeviceType TMemoryDeviceType>
struct CountVisibleBlocksInListIdRangeFunctor {
private: // member variables
	const Vector2i range;
	DECLARE_ATOMIC(int, visible_block_in_id_range_count);
public: // member functions
	explicit CountVisibleBlocksInListIdRangeFunctor(Vector2i list_id_range) : range(list_id_range) {
		assert(list_id_range.from <= list_id_range.to);
		INITIALIZE_ATOMIC(int, visible_block_in_id_range_count, 0);
	}

	CountVisibleBlocksInListIdRangeFunctor(int min_block_id, int max_block_id) :
			CountVisibleBlocksInListIdRangeFunctor(Vector2i(min_block_id, max_block_id)) {};

	~CountVisibleBlocksInListIdRangeFunctor() {
		CLEAN_UP_ATOMIC(visible_block_in_id_range_count);
	}

	_DEVICE_WHEN_AVAILABLE_
	void operator()(const HashEntry& hash_entry, int hash_code) {
		if (hash_entry.ptr >= range.from && hash_entry.ptr <= range.to) {
			ATOMIC_ADD(visible_block_in_id_range_count, 1);
		}
	}

	int GetCurrentVisibleBlockInIDRangeCount() {
		return GET_ATOMIC_VALUE_CPU(visible_block_in_id_range_count);
	}
};

template<MemoryDeviceType TMemoryDeviceType>
struct FillExpectedDepthsWithClippingDistancesFunctor {
private: // member variables
	const Vector2f clipping_bounds;
public: // member functions
	explicit FillExpectedDepthsWithClippingDistancesFunctor(const Vector2f& clipping_bounds) : clipping_bounds(clipping_bounds) {}

	FillExpectedDepthsWithClippingDistancesFunctor(float near_clipping_distance, float far_clipping_distance)
			: FillExpectedDepthsWithClippingDistancesFunctor(Vector2f(near_clipping_distance, far_clipping_distance)) {}

	_DEVICE_WHEN_AVAILABLE_
	void operator()(Vector2f& pixel_ray_bound) {
		pixel_ray_bound.from = clipping_bounds.from;
		pixel_ray_bound.to = clipping_bounds.to;
	}
};

template<MemoryDeviceType TMemoryDeviceType>
struct ProjectAndSplitBlocksFunctor {
private: // member variables
	const Matrix4f depth_camera_pose;
	const Vector4f depth_camera_projection_parameters;
	const Vector2i depth_image_size;
	const float voxel_size;

	DECLARE_ATOMIC(unsigned int, total_rendering_block_count);
	RenderingBlock* rendering_blocks;

public: // member functions
	ProjectAndSplitBlocksFunctor(
			ORUtils::MemoryBlock<RenderingBlock>& rendering_blocks,
			const Matrix4f& depth_camera_pose,
			const Vector4f& depth_camera_projection_parameters,
			const Vector2i& depth_image_size,
			const float voxel_size)
			: depth_camera_pose(depth_camera_pose),
			  depth_camera_projection_parameters(depth_camera_projection_parameters),
			  depth_image_size(depth_image_size),
			  voxel_size(voxel_size),
			  rendering_blocks(rendering_blocks.GetData(TMemoryDeviceType)){
		INITIALIZE_ATOMIC(unsigned int, total_rendering_block_count, 0u);
	}

	_DEVICE_WHEN_AVAILABLE_
	void operator()(const HashEntry& hash_entry, const int hash_code) {
		Vector2f z_range;
		Vector2i upper_left, lower_right;
		if (ProjectSingleBlock(hash_entry.pos, depth_camera_pose, depth_camera_projection_parameters, depth_image_size, voxel_size,
		                       upper_left, lower_right, z_range)) {
			Vector2i new_rednering_block_dimensions(ceil_of_integer_quotient(lower_right.x - upper_left.x + 1, rendering_block_size_x),
			                                        ceil_of_integer_quotient(lower_right.y - upper_left.y + 1, rendering_block_size_y));

			size_t new_rendering_block_count = new_rednering_block_dimensions.x * new_rednering_block_dimensions.y;
			int current_rendering_block_count = ORUtils::ParallelSum<TMemoryDeviceType>::template Add1D<unsigned int>(new_rendering_block_count,
			                                                                                                          total_rendering_block_count);
			CreateRenderingBlocks2(rendering_blocks, current_rendering_block_count, upper_left, lower_right, z_range);
		} else {
			ORUtils::ParallelSum<TMemoryDeviceType>::template Add1D<unsigned int>(0u, total_rendering_block_count);
		}
	}

	unsigned int GetRenderingBlockCount() {
		return GET_ATOMIC_VALUE_CPU(total_rendering_block_count);
	}

};

template<MemoryDeviceType TMemoryDeviceType>
struct FillBlocksFunctor {
private: // member variables
	Vector2f* pixel_ray_bound_data;
	const int image_width;
public: // member functions
	explicit FillBlocksFunctor(ORUtils::Image<Vector2f>& pixel_ray_bounds)
		: pixel_ray_bound_data(pixel_ray_bounds.GetData(TMemoryDeviceType)),
		  image_width(pixel_ray_bounds.dimensions.width){}
#ifdef __CUDACC__
	__device__ void operator()(const RenderingBlock& block, const int x, const int y){
		Vector2f& pixel = pixel_ray_bound_data[x + y*image_width];
		atomicMin(&pixel.from, block.z_range.from); atomicMax(&pixel.to, block.z_range.to);
	}
#else
	void operator()(const RenderingBlock& block, const int x, const int y){
		Vector2f& pixel = pixel_ray_bound_data[x + y*image_width];
#pragma omp critical
		{
//TODO: figure out a parallel way to set this or to do single-threaded traversal in all cases instead (see issue #234)
			if (pixel.from > block.z_range.x) pixel.from = block.z_range.from;
			if (pixel.to < block.z_range.y) pixel.to = block.z_range.to;
		};
	}
#endif
};


} // namespace ITMLib