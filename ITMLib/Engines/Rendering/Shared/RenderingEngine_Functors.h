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
			Vector2i new_rendering_block_dimensions(ceil_of_integer_quotient(lower_right.x - upper_left.x + 1, rendering_block_size_x),
			                                        ceil_of_integer_quotient(lower_right.y - upper_left.y + 1, rendering_block_size_y));

			new_rendering_block_count = new_rendering_block_dimensions.x * new_rendering_block_dimensions.y;
			if(GET_ATOMIC_VALUE(total_rendering_block_count) + new_rendering_block_count >= ITMLib::MAX_RENDERING_BLOCKS){
				//TODO: not sure if this check is worth the performance improvement from skipped work... test...
				new_rendering_block_count = 0;
			}
		}
		int current_rendering_block_count = ORUtils::ParallelSum<TMemoryDeviceType>::template Add1D<unsigned int>(new_rendering_block_count,
		                                                                                                          total_rendering_block_count);
		if (new_rendering_block_count != 0 && current_rendering_block_count + new_rendering_block_count <= ITMLib::MAX_RENDERING_BLOCKS){
			//counters need to be re-checked here (second condition above), because parallel sum will still work for all thread blocks
			// that reach it before the total exceeds the bound in previous check
			CreateRenderingBlocks2(rendering_blocks, current_rendering_block_count, upper_left, lower_right, z_range);
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

//#if !defined(WITH_OPENMP) && !defined(__CUDACC__)
//#define SINGLE_THREADED
//#endif

template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType, bool TModifyVisibilityInformation>
struct RaycastFunctor {
	HashBlockVisibility* block_visibility_types;
	const Vector4f inverted_camera_projection_parameters;
	const Matrix4f inverted_camera_pose;
	const float truncation_distance; //"mu" / μ in many fusion-type-reconstruction articles
	const float voxel_size_reciprocal;
	const TVoxel* voxels;
	const typename TIndex::IndexData* index_data;
//#ifdef SINGLE_THREADED
//	typename TIndex::IndexCache cache;
//#endif
	const Vector2f* ray_depth_range_image;
	const int ray_depth_image_width;


public: // member functions
	RaycastFunctor(VoxelVolume<TVoxel, TIndex>& volume, const ORUtils::Image<Vector2f>& ray_depth_range_image,
	               const Vector4f& inverted_camera_projection_parameters, const Matrix4f inverted_camera_pose)
			: inverted_camera_projection_parameters(inverted_camera_projection_parameters),
			  inverted_camera_pose(inverted_camera_pose),
			  truncation_distance(volume.GetParameters().truncation_distance),
			  voxel_size_reciprocal(1.0f / volume.GetParameters().voxel_size),
			  voxels(volume.GetVoxels()),
			  index_data(volume.index.GetIndexData()),
			  ray_depth_range_image(ray_depth_range_image.GetData(TMemoryDeviceType)),
			  ray_depth_image_width(ray_depth_range_image.dimensions.width),
			  block_visibility_types(internal::GetBlockVisibilityTypesIfAvailable(volume.index)) {}

	_DEVICE_WHEN_AVAILABLE_
	inline void operator()(Vector4f& point, const int x, const int y) {
		const Vector2f& ray_depth_range = ray_depth_range_image[x / ray_depth_image_subsampling_factor +
		                                                        (y / ray_depth_image_subsampling_factor) * ray_depth_image_width];
		CastRay<TVoxel, TIndex, TModifyVisibilityInformation>(
				point, block_visibility_types, x, y, voxels, index_data, inverted_camera_pose, inverted_camera_projection_parameters,
				voxel_size_reciprocal, truncation_distance, ray_depth_range
//#ifdef SINGLE_THREADED
//				, cache
//#endif
				);
	}
};

template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
struct RenderPointCloudFunctor {
private: // member variables
	const bool point_skipping_enabled;
	const TVoxel* voxels;
	const typename TIndex::IndexData* index_data;
	const float voxel_size;
	Vector4f* colors;
	Vector4f* locations;
	const Vector3f light_source;

	DECLARE_ATOMIC(unsigned int, point_count);

public: // member functions
	RenderPointCloudFunctor(PointCloud& point_cloud, const VoxelVolume<TVoxel, TIndex>& volume, Vector3f light_source, bool point_skipping_enabled)
			: point_skipping_enabled(point_skipping_enabled),
			  voxels(volume.GetVoxels()),
			  index_data(volume.index.GetIndexData()),
			  voxel_size(volume.GetParameters().voxel_size),
			  colors(point_cloud.colours->GetData(TMemoryDeviceType)),
			  locations(point_cloud.locations->GetData(TMemoryDeviceType)),
			  light_source(light_source) {
		INITIALIZE_ATOMIC(unsigned int, point_count, 0u);
	}

	~RenderPointCloudFunctor() { CLEAN_UP_ATOMIC(point_count); }

	_DEVICE_WHEN_AVAILABLE_
	inline void operator()(const Vector4f& raycast_point, const int x, const int y) {

		bool found_point = raycast_point.w > 0;
		Vector3f point = raycast_point.toVector3();
		Vector3f normal;
		float angle;

		computeNormalAndAngle<TVoxel, TIndex>(found_point, point, voxels, index_data, light_source, normal, angle);

		if (point_skipping_enabled && ((x % 2 == 0) || (y % 2 == 0))) found_point = false;

		int offset = ORUtils::ParallelSum<TMemoryDeviceType>::template Add2D<uint>(found_point, point_count);

		if (found_point && offset != -1) {
			Vector4f tmp;
			tmp = VoxelColorReader<TVoxel::hasColorInformation, TVoxel, TIndex>::interpolate(voxels, index_data, point);
			if (tmp.w > 0.0f) {
				tmp.x /= tmp.w;
				tmp.y /= tmp.w;
				tmp.z /= tmp.w;
				tmp.w = 1.0f;
			}
			colors[offset] = tmp;

			Vector4f pt_ray_out;
			pt_ray_out.x = point.x * voxel_size;
			pt_ray_out.y = point.y * voxel_size;
			pt_ray_out.z = point.z * voxel_size;
			pt_ray_out.w = 1.0f;
			locations[offset] = pt_ray_out;
		}
	}

	unsigned int GetPointCount() {
		return GET_ATOMIC_VALUE_CPU(point_count);
	}
};

template<MemoryDeviceType TMemoryDeviceType, bool TUseSmoothing, bool TFlipNormals>
struct ICPMapRenderFunctor {
private: // member variables
	Vector4f* locations;
	Vector4f* normals;
	const float voxel_size;
	const Vector2i map_dimensions;
	const Vector4f* raycast_points;
	const Vector3f light_source;

public: // member functions
	ICPMapRenderFunctor(CameraTrackingState& camera_tracking_state, const float voxel_size, const RenderState& render_state,
	                    const Vector3f light_source)
			: locations(camera_tracking_state.pointCloud->locations->GetData(TMemoryDeviceType)),
			  normals(camera_tracking_state.pointCloud->colours->GetData(TMemoryDeviceType)),
			  voxel_size(voxel_size),
			  map_dimensions(render_state.raycastResult->dimensions),
			  raycast_points(render_state.raycastResult->GetData(TMemoryDeviceType)),
			  light_source(light_source) {}

	_DEVICE_WHEN_AVAILABLE_
	inline void operator()(const int pixel_index, const int x, const int y) {
		Vector3f normal;
		float angle;

		Vector4f raycast_point = raycast_points[pixel_index];
		bool point_found = raycast_point.w > 0.0f;

		computeNormalAndAngle<TUseSmoothing, TFlipNormals>(
				point_found, x, y, raycast_points, light_source, voxel_size, map_dimensions, normal, angle);

		if (point_found) {
			locations[pixel_index] =
					Vector4f(raycast_point.x * voxel_size, raycast_point.y * voxel_size, raycast_point.z * voxel_size, raycast_point.w);
			normals[pixel_index] = Vector4f(normal.x, normal.y, normal.z, 0.0f);
		} else {
			Vector4f blank(0.0f, 0.0f, 0.0f, -1.0f);
			locations[pixel_index] = blank;
			normals[pixel_index] = blank;
		}
	}

};

template<MemoryDeviceType TMemoryDeviceType>
struct ForwardProjectFunctor {
private: // member variables

	Vector4f* forward_projection;
	const int projection_image_width;
	const float voxel_size;
	const Vector4f depth_camera_projection_parameters;
	const Matrix4f depth_camera_pose;

public: // member functions
	ForwardProjectFunctor(ORUtils::Image<Vector4f>& forward_projection, const float voxel_size, const Vector4f& depth_camera_projection_parameters,
	                      const Matrix4f& depth_camera_pose)
			: forward_projection(forward_projection.GetData(TMemoryDeviceType)),
			  projection_image_width(forward_projection.dimensions.x),
			  voxel_size(voxel_size),
			  depth_camera_projection_parameters(depth_camera_projection_parameters),
			  depth_camera_pose(depth_camera_pose) {}

	_DEVICE_WHEN_AVAILABLE_
	inline void operator()(const Vector4f& raycast_point) {
		Vector4f pixel = raycast_point * voxel_size;
		pixel.w = 1.0f;
		pixel = depth_camera_pose * pixel;

		Vector2f point_image_space;
		point_image_space.x = depth_camera_projection_parameters.x * pixel.x / pixel.z + depth_camera_projection_parameters.z;
		point_image_space.y = depth_camera_projection_parameters.y * pixel.y / pixel.z + depth_camera_projection_parameters.w;

		int new_index = static_cast<int>(point_image_space.x + 0.5f) + static_cast<int>(point_image_space.y + 0.5f) * projection_image_width;
		forward_projection[new_index] = pixel;
	}
};

template<MemoryDeviceType TMemoryDeviceType>
struct FindMissingProjectionPointsFunctor {
private: // member variables
	int* projection_missing_point_indices;
	const Vector4f* forward_projection;
	const Vector2f* pixel_ray_depth_range_data;
	const int ray_depth_range_image_width;
	const float* depth;

	DECLARE_ATOMIC(unsigned int, missing_point_count);
public: // member functions
	FindMissingProjectionPointsFunctor(ORUtils::Image<int>& projection_missing_point_indices,
	                                   const ORUtils::Image<Vector4f>& forward_projection,
	                                   const ORUtils::Image<Vector2f>& pixel_ray_depth_range_image)
			: projection_missing_point_indices(projection_missing_point_indices.GetData(TMemoryDeviceType)),
			  forward_projection(forward_projection.GetData(TMemoryDeviceType)),
			  pixel_ray_depth_range_data(pixel_ray_depth_range_image.GetData(TMemoryDeviceType)),
			  ray_depth_range_image_width(pixel_ray_depth_range_image.dimensions.x) {
		INITIALIZE_ATOMIC(unsigned int, missing_point_count, 0u);
	}

	~FindMissingProjectionPointsFunctor() {
		CLEAN_UP_ATOMIC(missing_point_count);
	}

	_DEVICE_WHEN_AVAILABLE_
	inline void operator()(const int pixel_index, const int x, const int y) {

		int ray_ranges_pixel_index =
				(int) floor((float) x / ray_depth_image_subsampling_factor) +
				(int) floor((float) y / ray_depth_image_subsampling_factor) * ray_depth_range_image_width;

		Vector4f forward_projected_point = forward_projection[pixel_index];
		Vector2f pixel_ray_depth_range = pixel_ray_depth_range_data[ray_ranges_pixel_index];
		float depth_at_pixel = depth[pixel_index];

		bool has_point = false;
#ifdef __CUDACC__
		__shared__
#endif
		bool should_prefix;
		should_prefix = false;
#ifdef __CUDACC__
		__syncthreads();
#endif

		if ((forward_projected_point.w <= 0) &&
		    ((forward_projected_point.x == 0 && forward_projected_point.y == 0 && forward_projected_point.z == 0) || (depth_at_pixel > 0)) &&
		    (pixel_ray_depth_range.x < pixel_ray_depth_range.y)) {
			should_prefix = true;
			has_point = true;
		}
#ifdef __CUDACC__
		__syncthreads();
#endif
		if (should_prefix) {
			int offset = ORUtils::ParallelSum<TMemoryDeviceType>::template Add2D<uint>(has_point, missing_point_count);
			if (offset != -1) projection_missing_point_indices[offset] = pixel_index;
		}
	}

	unsigned int GetMissingPointCount() {
		return GET_ATOMIC_VALUE_CPU(missing_point_count);
	}
};

template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
struct RaycastMissingPointsFunctor {
private: // member variables
	const Vector4f inverted_camera_projection_parameters;
	const Matrix4f inverted_camera_pose;
	const float truncation_distance; //"mu" / μ in many fusion-type-reconstruction articles
	const float voxel_size_reciprocal;
	const TVoxel* voxels;
	const typename TIndex::IndexData* index_data;
	const Vector2f* pixel_ray_depth_range_data;
	const int ray_depth_range_image_width;

public: // member functions
	RaycastMissingPointsFunctor(const VoxelVolume<TVoxel, TIndex>& volume,
	                            const Vector4f& inverted_camera_projection_parameters, const Matrix4f inverted_camera_pose,
	                            const ORUtils::Image<Vector2f>& pixel_ray_depth_range_image)
			: inverted_camera_projection_parameters(inverted_camera_projection_parameters),
			  inverted_camera_pose(inverted_camera_pose),
			  truncation_distance(volume.GetParameters().truncation_distance),
			  voxel_size_reciprocal(1.0f / volume.GetParameters().voxel_size),
			  voxels(volume.GetVoxels()),
			  index_data(volume.index.GetIndexData()),
			  pixel_ray_depth_range_data(pixel_ray_depth_range_image.GetData(TMemoryDeviceType)),
			  ray_depth_range_image_width(pixel_ray_depth_range_image.dimensions.x) {}

	_DEVICE_WHEN_AVAILABLE_
	inline void operator()(Vector4f& point, const int x, const int y) {
		int ray_ranges_pixel_index =
				(int) floor((float) x / ray_depth_image_subsampling_factor) +
				(int) floor((float) y / ray_depth_image_subsampling_factor) * ray_depth_range_image_width;
		CastRay<TVoxel, TIndex, false>(point, nullptr, x, y, voxels,
		                               index_data, inverted_camera_pose, inverted_camera_projection_parameters,
		                               voxel_size_reciprocal, truncation_distance,
		                               pixel_ray_depth_range_data[ray_ranges_pixel_index]);
	}
};

template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType, IRenderingEngine::RenderImageType TRenderImageType>
struct RenderFromVolumeFunctor;

template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
struct RenderFromVolumeFunctor_Base {
protected: // member variables
	Vector4u* pixels;
	const Vector4f* raycast_points;
	const TVoxel* voxels;
	const typename TIndex::IndexData* index_data;
	const Vector3f light_source;
public: // member functions
	RenderFromVolumeFunctor_Base(ORUtils::Image<Vector4u>& output_image, const ORUtils::Image<Vector4f>& raycast_points,
	                             const VoxelVolume<TVoxel, TIndex>& volume, const Vector3f light_source)
			: pixels(output_image.GetData(TMemoryDeviceType)), raycast_points(raycast_points.GetData(TMemoryDeviceType)),
			  voxels(volume.GetVoxels()), index_data(volume.index.GetIndexData()), light_source(light_source) {}
};

template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
struct RenderFromVolumeFunctor<TVoxel, TIndex, TMemoryDeviceType, IRenderingEngine::RENDER_SHADED_GREYSCALE>
		: public RenderFromVolumeFunctor_Base<TVoxel, TIndex, TMemoryDeviceType> {
	using RenderFromVolumeFunctor_Base<TVoxel, TIndex, TMemoryDeviceType>::RenderFromVolumeFunctor_Base;
	_DEVICE_WHEN_AVAILABLE_
	inline void operator()(const int pixel_index, const int x, const int y) {
		const Vector4f& raycast_point = this->raycast_points[pixel_index];
		processPixelGrey<TVoxel, TIndex>(this->pixels[pixel_index], raycast_point.toVector3(), raycast_point.w > 0, this->voxels, this->index_data,
		                                 this->light_source);
	}
};

template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
struct RenderFromVolumeFunctor<TVoxel, TIndex, TMemoryDeviceType, IRenderingEngine::RENDER_COLOUR_FROM_VOLUME>
		: public RenderFromVolumeFunctor_Base<TVoxel, TIndex, TMemoryDeviceType> {
	using RenderFromVolumeFunctor_Base<TVoxel, TIndex, TMemoryDeviceType>::RenderFromVolumeFunctor_Base;
	_DEVICE_WHEN_AVAILABLE_
	inline void operator()(const int pixel_index, const int x, const int y) {
		const Vector4f& raycast_point = this->raycast_points[pixel_index];
		processPixelColour<TVoxel, TIndex>(this->pixels[pixel_index], raycast_point.toVector3(), raycast_point.w > 0, this->voxels, this->index_data);
	}
};

template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
struct RenderFromVolumeFunctor<TVoxel, TIndex, TMemoryDeviceType, IRenderingEngine::RENDER_COLOUR_FROM_NORMAL>
		: public RenderFromVolumeFunctor_Base<TVoxel, TIndex, TMemoryDeviceType> {
	using RenderFromVolumeFunctor_Base<TVoxel, TIndex, TMemoryDeviceType>::RenderFromVolumeFunctor_Base;
	_DEVICE_WHEN_AVAILABLE_
	inline void operator()(const int pixel_index, const int x, const int y) {
		const Vector4f& raycast_point = this->raycast_points[pixel_index];
		processPixelNormal<TVoxel, TIndex>(this->pixels[pixel_index], raycast_point.toVector3(), raycast_point.w > 0, this->voxels, this->index_data,
		                                   this->light_source);
	}
};

template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
struct RenderFromVolumeFunctor<TVoxel, TIndex, TMemoryDeviceType, IRenderingEngine::RENDER_COLOUR_FROM_CONFIDENCE>
		: public RenderFromVolumeFunctor_Base<TVoxel, TIndex, TMemoryDeviceType> {
	using RenderFromVolumeFunctor_Base<TVoxel, TIndex, TMemoryDeviceType>::RenderFromVolumeFunctor_Base;
	_DEVICE_WHEN_AVAILABLE_
	inline void operator()(const int pixel_index, const int x, const int y) {
		const Vector4f& raycast_point = this->raycast_points[pixel_index];
		processPixelConfidence<TVoxel, TIndex>(this->pixels[pixel_index], raycast_point, raycast_point.w > 0, this->voxels, this->index_data,
		                                       this->light_source);
	}
};

template<MemoryDeviceType TMemoryDeviceType, bool TFlipNormals, IRenderingEngine::RenderImageType TRenderImageType>
struct RenderFromRaycastFunctor;

template<MemoryDeviceType TMemoryDeviceType, bool TFlipNormals>
struct RenderFromRaycastFunctor_Base {
protected: // member variables
	Vector4u* pixels;
	const Vector2i image_dimensions;
	const Vector4f* raycast_points;
	const float voxel_size;
	const Vector3f light_source;
public: // member functions
	RenderFromRaycastFunctor_Base(ORUtils::Image<Vector4u>& output_image, const ORUtils::Image<Vector4f>& raycast_points,
	                              float voxel_size, const Vector3f light_source)
			: pixels(output_image.GetData(TMemoryDeviceType)), image_dimensions(output_image.dimensions),
			  raycast_points(raycast_points.GetData(TMemoryDeviceType)),
			  voxel_size(voxel_size), light_source(light_source) {}
};

template<MemoryDeviceType TMemoryDeviceType, bool TFlipNormals>
struct RenderFromRaycastFunctor<TMemoryDeviceType, TFlipNormals, IRenderingEngine::RENDER_SHADED_GREYSCALE_IMAGENORMALS>
		: public RenderFromRaycastFunctor_Base<TMemoryDeviceType, TFlipNormals> {
	using RenderFromRaycastFunctor_Base<TMemoryDeviceType, TFlipNormals>::RenderFromRaycastFunctor_Base;
	_DEVICE_WHEN_AVAILABLE_
	inline void operator()(const int pixel_index, const int x, const int y) {
		processPixelGrey_ImageNormals<true, TFlipNormals>(this->pixels, this->raycast_points, this->image_dimensions, x, y, this->voxel_size, this->light_source);
	}
};

} // namespace ITMLib