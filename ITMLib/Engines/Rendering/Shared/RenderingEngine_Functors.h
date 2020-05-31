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

namespace ITMLib {

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
	void operator()(const HashEntry& hash_entry, int hash_code) {
		unsigned char block_visibility_type = 0;// = blockVisibilityTypes[targetIdx];

		bool is_visible, is_visible_enlarged;
		CheckVoxelHashBlockVisibility<false>(is_visible, is_visible_enlarged, hash_entry.pos, depth_camera_pose,
		                                     depth_camera_projection_parameters, voxel_size, depth_image_size);
		block_visibility_type = is_visible;


		if (block_visibility_type > 0) {
			int visible_block_index = ATOMIC_ADD(visible_block_count, 1);
			visible_block_hash_codes[visible_block_index] = hash_code;
		}
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
	void operator()(Vector2f& pixel_ray_bound){
		pixel_ray_bound.from = clipping_bounds.from;
		pixel_ray_bound.to = clipping_bounds.to;
	}
};

template<MemoryDeviceType TMemoryDeviceType>
struct ProjectAndSplitBlocksFunctor {
private: // member variables
public: // member functions
	ProjectAndSplitBlocksFunctor(){

	}
	_DEVICE_WHEN_AVAILABLE_
	void operator()(const HashEntry& hash_entry, const int hash_code){
//		Vector2i upperLeft, lowerRight;
//		Vector2f zRange;
//		bool validProjection = false;
//		if (in_offset < block_count)
//			if (blockData.ptr >= 0)
//				validProjection = ProjectSingleBlock(blockData.pos, depth_camera_pose, depth_camera_projection_parameters, depth_image_size, voxel_size,
//				                                     upperLeft, lowerRight, zRange);
//
//		Vector2i requiredRenderingBlocks(ceilf((float) (lowerRight.x - upperLeft.x + 1) / rendering_block_size_x),
//		                                 ceilf((float) (lowerRight.y - upperLeft.y + 1) / rendering_block_size_y));
//
//		size_t requiredNumBlocks = requiredRenderingBlocks.x * requiredRenderingBlocks.y;
//		if (!validProjection) requiredNumBlocks = 0;
//
//		int out_offset = ORUtils::ParallelSum<MEMORYDEVICE_CUDA>::Add1D<int>(requiredNumBlocks, rendering_block_count);
//		if (!validProjection) return;
//		if ((out_offset == -1) || (out_offset + requiredNumBlocks > MAX_RENDERING_BLOCKS)) return;
//
//		CreateRenderingBlocks(rendering_blocks, out_offset, upperLeft, lowerRight, zRange);
	}

};


} // namespace ITMLib