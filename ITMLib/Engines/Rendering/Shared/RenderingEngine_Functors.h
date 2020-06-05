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
	inline void operator()(const HashEntry* hash_entries, int i_hash_entry, bool padding_job) {
		bool is_visible = false, is_visible_enlarged;
		if (!padding_job) {
			CheckVoxelHashBlockVisibility<false>(is_visible, is_visible_enlarged, hash_entries[i_hash_entry].pos, depth_camera_pose,
			                                     depth_camera_projection_parameters, voxel_size, depth_image_size);
		}

		int visible_block_index = ORUtils::ParallelSum<TMemoryDeviceType>::template Add1D<int>(is_visible, visible_block_count);
		if (is_visible) visible_block_hash_codes[visible_block_index] = i_hash_entry;

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
	inline void operator()(const HashEntry& hash_entry) {
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
	inline void operator()(Vector2f& pixel_ray_bound) {
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
			  rendering_blocks(rendering_blocks.GetData(TMemoryDeviceType)) {
		INITIALIZE_ATOMIC(unsigned int, total_rendering_block_count, 0u);
	}

	_DEVICE_WHEN_AVAILABLE_
	inline void operator()(const HashEntry* hash_entries, int i_item, bool padding_job) {
		Vector2f z_range;
		Vector2i upper_left, lower_right;
		unsigned int new_rendering_block_count = 0;

		if (!padding_job &&
		    ProjectSingleBlock(hash_entries[i_item].pos, depth_camera_pose, depth_camera_projection_parameters, depth_image_size, voxel_size,
		                       upper_left, lower_right, z_range)) {
			Vector2i new_rednering_block_dimensions(ceil_of_integer_quotient(lower_right.x - upper_left.x + 1, rendering_block_size_x),
			                                        ceil_of_integer_quotient(lower_right.y - upper_left.y + 1, rendering_block_size_y));

			new_rendering_block_count = new_rednering_block_dimensions.x * new_rednering_block_dimensions.y;
		}
		int current_rendering_block_count = ORUtils::ParallelSum<TMemoryDeviceType>::template Add1D<unsigned int>(new_rendering_block_count,
		                                                                                                          total_rendering_block_count);
		if (new_rendering_block_count != 0) CreateRenderingBlocks2(rendering_blocks, current_rendering_block_count, upper_left, lower_right, z_range);
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
			  image_width(pixel_ray_bounds.dimensions.width) {}


	_DEVICE_WHEN_AVAILABLE_
	inline void operator()(const RenderingBlock& block, const int x, const int y) {
		Vector2f& pixel = pixel_ray_bound_data[x + y * image_width];
#ifdef __CUDACC__
		atomicMin(&pixel.from, block.z_range.from); atomicMax(&pixel.to, block.z_range.to);
#else
#ifdef WITH_OPENMP
#pragma omp critical
#endif
		{
//TODO: figure out a parallel way to set this or to do single-threaded traversal in all cases instead (see issue #234)
			if (pixel.from > block.z_range.x) pixel.from = block.z_range.from;
			if (pixel.to < block.z_range.y) pixel.to = block.z_range.to;
		};
#endif
	}
};

namespace internal {
template<typename TIndex>
static inline HashBlockVisibility* GetBlockVisibilityTypesIfAvailable(TIndex& index);

template<>
inline HashBlockVisibility* GetBlockVisibilityTypesIfAvailable<VoxelBlockHash>(VoxelBlockHash& index) {
	return index.GetBlockVisibilityTypes();
}

template<>
inline HashBlockVisibility* GetBlockVisibilityTypesIfAvailable<PlainVoxelArray>(PlainVoxelArray& index) {
	return nullptr;
}
} // namespace internal

#if !defined(WITH_OPENMP) && !defined(__CUDACC__)
#define SINGLE_THREADED
#endif

template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType, bool TModifyVisibilityInformation>
struct RaycastFunctor {
	const Vector4f inverted_depth_camera_projection_parameters;
	const Matrix4f inverted_depth_camera_matrix;
	const float truncation_distance; //"mu" / μ in many fusion-type-reconstruction articles
	const float voxel_size_reciprocal;
	const TVoxel* voxels;
	const typename TIndex::IndexData* index_data;
#ifdef SINGLE_THREADED
	typename TIndex::IndexCache cache;
#endif
	const Vector2f* ray_depth_range_image;
	const int ray_depth_image_width;
	HashBlockVisibility* block_visibility_types;

public: // member functions
	RaycastFunctor(VoxelVolume<TVoxel, TIndex>& volume, const ORUtils::Image<Vector2f>& ray_depth_range_image,
	               const Vector4f& inverted_depth_camera_projection_parameters, const Matrix4f inverted_depth_camera_matrix)
			: inverted_depth_camera_projection_parameters(inverted_depth_camera_projection_parameters),
			  inverted_depth_camera_matrix(inverted_depth_camera_matrix),
			  truncation_distance(volume.GetParameters().truncation_distance),
			  voxel_size_reciprocal(1.0f / volume.GetParameters().voxel_size),
			  voxels(volume.GetVoxels()),
			  index_data(volume.index.GetIndexData()),
			  ray_depth_range_image(ray_depth_range_image.GetData(TMemoryDeviceType)),
			  ray_depth_image_width(ray_depth_range_image.dimensions.width),
			  block_visibility_types(internal::GetBlockVisibilityTypesIfAvailable(volume.index)) {}

	_DEVICE_WHEN_AVAILABLE_
	inline void operator()(Vector4f& point, const int x, const int y) {
		const Vector2f& ray_depth_range = ray_depth_range_image[x / ray_depth_image_subsampling_factor + (y / ray_depth_image_subsampling_factor) * ray_depth_image_width ];
		const Vector4f& inverted_projection = inverted_depth_camera_projection_parameters;

		Vector4f point_in_camera_space;
		Vector3f march_start_world_space_voxels, march_end_world_space_voxels, march_vector, march_point_world_space_voxels;
		bool pt_found;
		int index_identifier;
		float sdf_value = 1.0f, confidence;
		float distance_along_ray_voxels, step_length_voxels;

		const float step_scale = truncation_distance * voxel_size_reciprocal;

		point_in_camera_space.z = ray_depth_range.from;
		point_in_camera_space.x = point_in_camera_space.z * ((float(x) + inverted_projection.cx) * inverted_projection.fx);
		point_in_camera_space.y = point_in_camera_space.z * ((float(y) + inverted_projection.cy) * inverted_projection.fy);
		point_in_camera_space.w = 1.0f;
		distance_along_ray_voxels = length(TO_VECTOR3(point_in_camera_space)) * voxel_size_reciprocal;
		march_start_world_space_voxels = TO_VECTOR3(inverted_depth_camera_matrix * point_in_camera_space) * voxel_size_reciprocal;

		point_in_camera_space.z = ray_depth_range.to;
		point_in_camera_space.x = point_in_camera_space.z * ((float(x) + inverted_projection.cx) * inverted_projection.fx);
		point_in_camera_space.y = point_in_camera_space.z * ((float(y) + inverted_projection.cy) * inverted_projection.fy);
		point_in_camera_space.w = 1.0f;
		const float distance_to_ray_end_length_voxels = length(TO_VECTOR3(point_in_camera_space)) * voxel_size_reciprocal;
		march_end_world_space_voxels = TO_VECTOR3(inverted_depth_camera_matrix * point_in_camera_space) * voxel_size_reciprocal;

		march_vector = march_end_world_space_voxels - march_start_world_space_voxels;
		float direction_norm = 1.0f / sqrt(march_vector.x * march_vector.x + march_vector.y * march_vector.y + march_vector.z * march_vector.z);
		march_vector *= direction_norm;

		march_point_world_space_voxels = march_start_world_space_voxels;

#ifndef SINGLE_THREADED
		typename TIndex::IndexCache cache;
#endif
		while (distance_along_ray_voxels < distance_to_ray_end_length_voxels) {
			sdf_value = readFromSDF_float_uninterpolated(voxels, index_data, march_point_world_space_voxels, index_identifier, cache);

			if (TModifyVisibilityInformation) {
				if (index_identifier) block_visibility_types[index_identifier - 1] = ITMLib::HashBlockVisibility::IN_MEMORY_AND_VISIBLE;
			}

			if (!index_identifier) {
				step_length_voxels = VOXEL_BLOCK_SIZE;
			} else {
				if ((sdf_value <= 0.1f) && (sdf_value >= -0.5f)) {
					sdf_value = readFromSDF_float_interpolated(voxels, index_data, march_point_world_space_voxels, index_identifier, cache);
				}
				if (sdf_value <= 0.0f) break;
				step_length_voxels = ORUTILS_MAX(sdf_value * step_scale, 1.0f);
			}

			march_point_world_space_voxels += step_length_voxels * march_vector;
			distance_along_ray_voxels += step_length_voxels;
		}

		if (sdf_value <= 0.0f) {
			step_length_voxels = sdf_value * step_scale;
			march_point_world_space_voxels += step_length_voxels * march_vector;

			sdf_value = ReadWithConfidenceFromSdfFloatInterpolated
					<TVoxel::hasWeightInformation,
							TVoxel::hasSemanticInformation,
							TVoxel, typename TIndex::IndexData, typename TIndex::IndexCache>
			::compute(confidence, voxels, index_data, march_point_world_space_voxels, index_identifier, cache);

			step_length_voxels = sdf_value * step_scale;
			march_point_world_space_voxels += step_length_voxels * march_vector;

			pt_found = true;
		} else pt_found = false;

		point.x = march_point_world_space_voxels.x;
		point.y = march_point_world_space_voxels.y;
		point.z = march_point_world_space_voxels.z;
		if (pt_found) point.w = confidence + 1.0f; else point.w = 0.0f;
	}
};


} // namespace ITMLib