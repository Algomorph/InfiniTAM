//  ================================================================
//  Created by Gregory Kramida on 1/30/20.
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

#include "../../../ORUtils/PlatformIndependence.h"
#include "../Math.h"
#include "../../Objects/Volume/VoxelBlockHash.h"


template<bool TUseSwapping>
_CPU_AND_GPU_CODE_ inline void CheckPointVisibility(THREADPTR(bool)& intersects_camera_ray_through_pixel,
                                                    THREADPTR(bool)& intersects_enlarged_camera_frustum,
                                                    const THREADPTR(Vector4f)& point_world_space,
                                                    const CONSTPTR(Matrix4f)& depth_camera_pose,
                                                    const CONSTPTR(Vector4f)& depth_camera_projection_parameters,
                                                    const CONSTPTR(Vector2i)& depth_image_size) {

	// initially, use var for camera space to avoid separate declaration (not sure if this makes sense -- might be optimized-out anyway).
	Vector4f point_image_space = depth_camera_pose * point_world_space;

	if (point_image_space.z < 1e-10f) return;

	point_image_space.x = depth_camera_projection_parameters.fx * point_image_space.x / point_image_space.z +
	                      depth_camera_projection_parameters.cx;
	point_image_space.y = depth_camera_projection_parameters.fy * point_image_space.y / point_image_space.z +
	                      depth_camera_projection_parameters.cy;

	if (point_image_space.x >= 0 && point_image_space.x < depth_image_size.x && point_image_space.y >= 0 &&
	    point_image_space.y < depth_image_size.y) {
		intersects_camera_ray_through_pixel = true;
		intersects_enlarged_camera_frustum = true;
	} else if (TUseSwapping) {
		Vector4i enlarged_image_limits;
		const int frustum_margin_factor = 8;
		enlarged_image_limits.min_x = -depth_image_size.x / frustum_margin_factor;
		enlarged_image_limits.max_x = depth_image_size.x + depth_image_size.x / frustum_margin_factor;
		enlarged_image_limits.min_y = -depth_image_size.y / frustum_margin_factor;
		enlarged_image_limits.max_y = depth_image_size.y + depth_image_size.y / frustum_margin_factor;

		if (point_image_space.x >= enlarged_image_limits.min_x && point_image_space.x < enlarged_image_limits.max_x &&
		    point_image_space.y >= enlarged_image_limits.min_y && point_image_space.y < enlarged_image_limits.max_y) {
			intersects_enlarged_camera_frustum = true;
		}
	}
}

template<bool TUseSwapping>
_CPU_AND_GPU_CODE_ inline void CheckVoxelHashBlockVisibility(THREADPTR(bool)& intersects_camera_ray_through_pixel,
                                                             THREADPTR(bool)& intersects_enlarged_camera_frustum,
                                                             const THREADPTR(Vector3s)& hash_block_location_blocks,
                                                             const CONSTPTR(Matrix4f)& depth_camera_pose,
                                                             const CONSTPTR(
		                                                             Vector4f)& depth_camera_projection_parameters,
                                                             const CONSTPTR(float)& voxel_size,
                                                             const CONSTPTR(Vector2i)& depth_image_size) {
	Vector4f block_corner_m;
	float block_size_m = (float) VOXEL_BLOCK_SIZE * voxel_size;

	intersects_camera_ray_through_pixel = false;
	intersects_enlarged_camera_frustum = false;

	// 0 0 0
	block_corner_m.x = (float) hash_block_location_blocks.x * block_size_m;
	block_corner_m.y = (float) hash_block_location_blocks.y * block_size_m;
	block_corner_m.z = (float) hash_block_location_blocks.z * block_size_m;
	block_corner_m.w = 1.0f;
	CheckPointVisibility<TUseSwapping>(intersects_camera_ray_through_pixel, intersects_enlarged_camera_frustum,
	                                   block_corner_m, depth_camera_pose,
	                                   depth_camera_projection_parameters,
	                                   depth_image_size);
	if (intersects_camera_ray_through_pixel) return;

	// 0 0 1
	block_corner_m.z += block_size_m;
	CheckPointVisibility<TUseSwapping>(intersects_camera_ray_through_pixel,
	                                   intersects_enlarged_camera_frustum,
	                                   block_corner_m, depth_camera_pose,
	                                   depth_camera_projection_parameters,
	                                   depth_image_size);
	if (intersects_camera_ray_through_pixel) return;

	// 0 1 1
	block_corner_m.y += block_size_m;
	CheckPointVisibility<TUseSwapping>(intersects_camera_ray_through_pixel,
	                                   intersects_enlarged_camera_frustum,
	                                   block_corner_m, depth_camera_pose,
	                                   depth_camera_projection_parameters,
	                                   depth_image_size);
	if (intersects_camera_ray_through_pixel) return;

	// 1 1 1
	block_corner_m.x += block_size_m;
	CheckPointVisibility<TUseSwapping>(intersects_camera_ray_through_pixel,
	                                   intersects_enlarged_camera_frustum,
	                                   block_corner_m, depth_camera_pose,
	                                   depth_camera_projection_parameters,
	                                   depth_image_size);
	if (intersects_camera_ray_through_pixel) return;

	// 1 1 0
	block_corner_m.z -= block_size_m;
	CheckPointVisibility<TUseSwapping>(intersects_camera_ray_through_pixel,
	                                   intersects_enlarged_camera_frustum,
	                                   block_corner_m, depth_camera_pose,
	                                   depth_camera_projection_parameters,
	                                   depth_image_size);
	if (intersects_camera_ray_through_pixel) return;

	// 1 0 0
	block_corner_m.y -= block_size_m;
	CheckPointVisibility<TUseSwapping>(intersects_camera_ray_through_pixel,
	                                   intersects_enlarged_camera_frustum,
	                                   block_corner_m, depth_camera_pose,
	                                   depth_camera_projection_parameters,
	                                   depth_image_size);
	if (intersects_camera_ray_through_pixel) return;

	// 0 1 0
	block_corner_m.x -= block_size_m;
	block_corner_m.y += block_size_m;
	CheckPointVisibility<TUseSwapping>(intersects_camera_ray_through_pixel,
	                                   intersects_enlarged_camera_frustum,
	                                   block_corner_m, depth_camera_pose,
	                                   depth_camera_projection_parameters,
	                                   depth_image_size);
	if (intersects_camera_ray_through_pixel) return;

	// 1 0 1
	block_corner_m.x += block_size_m;
	block_corner_m.y -= block_size_m;
	block_corner_m.z += block_size_m;
	CheckPointVisibility<TUseSwapping>(intersects_camera_ray_through_pixel,
	                                   intersects_enlarged_camera_frustum,
	                                   block_corner_m, depth_camera_pose,
	                                   depth_camera_projection_parameters,
	                                   depth_image_size);
	if (intersects_camera_ray_through_pixel) return;
}