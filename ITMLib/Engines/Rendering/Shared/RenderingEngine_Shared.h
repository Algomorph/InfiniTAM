// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../../Objects/Volume/RepresentationAccess.h"
#include "../../../Utils/Enums/HashBlockProperties.h"
#include "RenderingBlock.h"

namespace ITMLib {


static const CONSTPTR(int) ray_depth_image_subsampling_factor = 8;

#if !(defined __METALC__)

#ifndef FAR_AWAY
#define FAR_AWAY 999999.9f
#endif

#ifndef VERY_CLOSE
#define VERY_CLOSE 0.05f
#endif

_CPU_AND_GPU_CODE_ inline Vector4f InvertProjectionParams(const THREADPTR(Vector4f)& projParams) {
	return Vector4f(1.0f / projParams.x, 1.0f / projParams.y, -projParams.z, -projParams.w);
}

_CPU_AND_GPU_CODE_ inline bool
ProjectSingleBlock(const THREADPTR(Vector3s)& blockPos, const THREADPTR(Matrix4f)& pose, const THREADPTR(Vector4f)& intrinsics,
                   const THREADPTR(Vector2i)& imgSize, float voxelSize, THREADPTR(Vector2i)& upperLeft, THREADPTR(Vector2i)& lowerRight,
                   THREADPTR(Vector2f)& zRange) {
	upperLeft = imgSize / ray_depth_image_subsampling_factor;
	lowerRight = Vector2i(-1, -1);
	zRange = Vector2f(FAR_AWAY, VERY_CLOSE);
	for (int corner = 0; corner < 8; ++corner) {
		// project all 8 corners down to 2D image
		Vector3s tmp = blockPos;
		tmp.x += (corner & 1) ? 1 : 0;
		tmp.y += (corner & 2) ? 1 : 0;
		tmp.z += (corner & 4) ? 1 : 0;
		Vector4f pt3d(TO_FLOAT3(tmp) * (float) VOXEL_BLOCK_SIZE * voxelSize, 1.0f);
		pt3d = pose * pt3d;
		if (pt3d.z < 1e-6) continue;

		Vector2f pt2d;
		pt2d.x = (intrinsics.x * pt3d.x / pt3d.z + intrinsics.z) / ray_depth_image_subsampling_factor;
		pt2d.y = (intrinsics.y * pt3d.y / pt3d.z + intrinsics.w) / ray_depth_image_subsampling_factor;

		// remember bounding box, zmin and zmax
		if (upperLeft.x > floor(pt2d.x)) upperLeft.x = (int) floor(pt2d.x);
		if (lowerRight.x < ceil(pt2d.x)) lowerRight.x = (int) ceil(pt2d.x);
		if (upperLeft.y > floor(pt2d.y)) upperLeft.y = (int) floor(pt2d.y);
		if (lowerRight.y < ceil(pt2d.y)) lowerRight.y = (int) ceil(pt2d.y);
		if (zRange.x > pt3d.z) zRange.x = pt3d.z;
		if (zRange.y < pt3d.z) zRange.y = pt3d.z;
	}

	// do some sanity checks and respect image bounds
	if (upperLeft.x < 0) upperLeft.x = 0;
	if (upperLeft.y < 0) upperLeft.y = 0;
	if (lowerRight.x >= imgSize.x) lowerRight.x = imgSize.x - 1;
	if (lowerRight.y >= imgSize.y) lowerRight.y = imgSize.y - 1;
	if (upperLeft.x > lowerRight.x) return false;
	if (upperLeft.y > lowerRight.y) return false;
	//if (zRange.y <= VERY_CLOSE) return false; never seems to happen
	if (zRange.x < VERY_CLOSE) zRange.x = VERY_CLOSE;
	if (zRange.y < VERY_CLOSE) return false;

	return true;
}

_CPU_AND_GPU_CODE_ inline void CreateRenderingBlocks(DEVICEPTR(RenderingBlock)* rendering_block_list, int offset,
                                                     const THREADPTR(Vector2i)& upper_left, const THREADPTR(Vector2i)& lower_right,
                                                     const THREADPTR(Vector2f)& z_range) {
	// split bounding box into 16x16 pixel rendering blocks
	for (int by = 0; by < ceil((float) (1 + lower_right.y - upper_left.y) / rendering_block_size_y); ++by) {
		for (int bx = 0; bx < ceil((float) (1 + lower_right.x - upper_left.x) / rendering_block_size_x); ++bx) {
			if (offset >= MAX_RENDERING_BLOCKS) return;


			//for each rendering block: add it to the list
			DEVICEPTR(RenderingBlock)& b(rendering_block_list[offset++]);
			b.upper_left.x = upper_left.x + bx * rendering_block_size_x;
			b.upper_left.y = upper_left.y + by * rendering_block_size_y;
			b.lower_right.x = upper_left.x + (bx + 1) * rendering_block_size_x - 1;
			b.lower_right.y = upper_left.y + (by + 1) * rendering_block_size_y - 1;
			if (b.lower_right.x > lower_right.x) b.lower_right.x = lower_right.x;
			if (b.lower_right.y > lower_right.y) b.lower_right.y = lower_right.y;
			b.z_range = z_range;
		}
	}
}

_CPU_AND_GPU_CODE_ inline void CreateRenderingBlocks2(DEVICEPTR(RenderingBlock)* rendering_block_list, int offset,
                                                      const THREADPTR(Vector2i)& upper_left, const THREADPTR(Vector2i)& lower_right,
                                                      const THREADPTR(Vector2f)& z_range) {
	// split bounding box into 16x16 pixel rendering blocks
	for (int by = 0; by < ceil_of_integer_quotient(1 + lower_right.y - upper_left.y, rendering_block_size_y); ++by) {
		for (int bx = 0; bx < ceil_of_integer_quotient(1 + lower_right.x - upper_left.x, rendering_block_size_x); ++bx) {
			if (offset >= MAX_RENDERING_BLOCKS) return;
			//for each rendering block: add it to the list
			DEVICEPTR(RenderingBlock)& b(rendering_block_list[offset++]);
			b.upper_left.x = upper_left.x + bx * rendering_block_size_x;
			b.upper_left.y = upper_left.y + by * rendering_block_size_y;
			b.lower_right.x = upper_left.x + (bx + 1) * rendering_block_size_x;
			b.lower_right.y = upper_left.y + (by + 1) * rendering_block_size_y;
			if (b.lower_right.x > lower_right.x + 1) b.lower_right.x = lower_right.x + 1;
			if (b.lower_right.y > lower_right.y + 1) b.lower_right.y = lower_right.y + 1;
			b.z_range = z_range;
		}
	}
}

#endif //!(defined __METALC__)

template<bool hasWeightInformation, bool hasSemanticInformation, typename TVoxel, typename TIndex, typename TCache>
struct ReadWithConfidenceFromSdfFloatInterpolated;


template<typename TVoxel, typename TIndex, typename TCache>
struct ReadWithConfidenceFromSdfFloatInterpolated<true, false, TVoxel, TIndex, TCache> {
	_CPU_AND_GPU_CODE_ static float compute(THREADPTR(float)& confidence, const CONSTPTR(TVoxel)* voxelData,
	                                        const CONSTPTR(TIndex)* voxelIndex, Vector3f point, THREADPTR(int)& vmIndex, THREADPTR(TCache)& cache) {
		return readWithConfidenceFromSDF_float_interpolated(confidence, voxelData, voxelIndex, point, vmIndex, cache);
	}
};
template<typename TVoxel, typename TIndex, typename TCache>
struct ReadWithConfidenceFromSdfFloatInterpolated<true, true, TVoxel, TIndex, TCache> {
	_CPU_AND_GPU_CODE_ static float compute(THREADPTR(float)& confidence, const CONSTPTR(TVoxel)* voxelData,
	                                        const CONSTPTR(TIndex)* voxelIndex, Vector3f point, THREADPTR(int)& vmIndex, THREADPTR(TCache)& cache) {
		return readWithConfidenceFromSDF_float_interpolated(confidence, voxelData, voxelIndex, point, vmIndex, cache);
	}
};
template<typename TVoxel, typename TIndex, typename TCache>
struct ReadWithConfidenceFromSdfFloatInterpolated<false, true, TVoxel, TIndex, TCache> {
	_CPU_AND_GPU_CODE_ static float compute(THREADPTR(float)& confidence, const CONSTPTR(TVoxel)* voxelData,
	                                        const CONSTPTR(TIndex)* voxelIndex, Vector3f point, THREADPTR(int)& vmIndex, THREADPTR(TCache)& cache) {
		return readWithConfidenceFromSDF_float_interpolated_semantic(confidence, voxelData, voxelIndex, point, vmIndex, cache);
	}
};


//#if !defined(WITH_OPENMP) && !defined(__CUDACC__)
//#define SINGLE_THREADED
//#endif

template<typename TVoxel, typename TIndex,
		bool TModifyVisibilityInformation>
_CPU_AND_GPU_CODE_ inline bool CastRay(DEVICEPTR(Vector4f)& point, DEVICEPTR(ITMLib::HashBlockVisibility)* block_visibility_types,
                                       int x, int y, const CONSTPTR(TVoxel)* voxels, const CONSTPTR(typename TIndex::IndexData)* index_data,
                                       Matrix4f inverted_camera_matrix, Vector4f inverted_camera_projection_parameters,
                                       float voxel_size_reciprocal, float truncation_distance, const CONSTPTR(Vector2f)& ray_depth_range) {
	const Vector4f& inverted_projection = inverted_camera_projection_parameters;

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
	march_start_world_space_voxels = TO_VECTOR3(inverted_camera_matrix * point_in_camera_space) * voxel_size_reciprocal;

	point_in_camera_space.z = ray_depth_range.to;
	point_in_camera_space.x = point_in_camera_space.z * ((float(x) + inverted_projection.cx) * inverted_projection.fx);
	point_in_camera_space.y = point_in_camera_space.z * ((float(y) + inverted_projection.cy) * inverted_projection.fy);
	point_in_camera_space.w = 1.0f;
	const float distance_to_ray_end_length_voxels = length(TO_VECTOR3(point_in_camera_space)) * voxel_size_reciprocal;
	march_end_world_space_voxels = TO_VECTOR3(inverted_camera_matrix * point_in_camera_space) * voxel_size_reciprocal;

	march_vector = march_end_world_space_voxels - march_start_world_space_voxels;
	float direction_norm = 1.0f / sqrt(march_vector.x * march_vector.x + march_vector.y * march_vector.y + march_vector.z * march_vector.z);
	march_vector *= direction_norm;

	march_point_world_space_voxels = march_start_world_space_voxels;

//#ifndef SINGLE_THREADED
	typename TIndex::IndexCache cache;
//#endif
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

	return pt_found;
}


_CPU_AND_GPU_CODE_ inline int forwardProjectPixel(Vector4f pixel, const CONSTPTR(Matrix4f)& M, const CONSTPTR(Vector4f)& projParams,
                                                  const THREADPTR(Vector2i)& imgSize) {
	pixel.w = 1;
	pixel = M * pixel;

	Vector2f pt_image;
	pt_image.x = projParams.x * pixel.x / pixel.z + projParams.z;
	pt_image.y = projParams.y * pixel.y / pixel.z + projParams.w;

	if ((pt_image.x < 0) || (pt_image.x > imgSize.x - 1) || (pt_image.y < 0) || (pt_image.y > imgSize.y - 1)) return -1;

	return (int) (pt_image.x + 0.5f) + (int) (pt_image.y + 0.5f) * imgSize.x;
}

template<class TVoxel, class TIndex>
_CPU_AND_GPU_CODE_ inline void computeNormalAndAngle(THREADPTR(bool)& found_point, const THREADPTR(Vector3f)& point,
                                                     const CONSTPTR(TVoxel)* voxel_block_data, const CONSTPTR(typename TIndex::IndexData)* index_data,
                                                     const THREADPTR(Vector3f)& light_source, THREADPTR(Vector3f)& out_normal,
                                                     THREADPTR(float)& angle) {
	if (!found_point) return;

	out_normal = computeSingleNormalFromSDF(voxel_block_data, index_data, point);

	float norm_scale = 1.0f / sqrt(out_normal.x * out_normal.x + out_normal.y * out_normal.y + out_normal.z * out_normal.z);
	out_normal *= norm_scale;

	angle = out_normal.x * light_source.x + out_normal.y * light_source.y + out_normal.z * light_source.z;
	if (!(angle > 1e-7)) found_point = false;
}

template<bool useSmoothing, bool flipNormals>
_CPU_AND_GPU_CODE_ inline void computeNormalAndAngle(THREADPTR(bool)& found_point, const THREADPTR(int)& x, const THREADPTR(int)& y,
                                                     const CONSTPTR(Vector4f)* points_ray, const THREADPTR(Vector3f)& light_source,
                                                     const THREADPTR(float)& voxel_size, const THREADPTR(Vector2i)& image_size,
                                                     THREADPTR(Vector3f)& out_normal, THREADPTR(float)& angle) {
	if (!found_point) return;

	Vector4f xp1_y, xm1_y, x_yp1, x_ym1;

	if (useSmoothing) {
		if (y <= 2 || y >= image_size.y - 3 || x <= 2 || x >= image_size.x - 3) {
			found_point = false;
			return;
		}

		xp1_y = points_ray[(x + 2) + y * image_size.x], x_yp1 = points_ray[x + (y + 2) * image_size.x];
		xm1_y = points_ray[(x - 2) + y * image_size.x], x_ym1 = points_ray[x + (y - 2) * image_size.x];
	} else {
		if (y <= 1 || y >= image_size.y - 2 || x <= 1 || x >= image_size.x - 2) {
			found_point = false;
			return;
		}

		xp1_y = points_ray[(x + 1) + y * image_size.x], x_yp1 = points_ray[x + (y + 1) * image_size.x];
		xm1_y = points_ray[(x - 1) + y * image_size.x], x_ym1 = points_ray[x + (y - 1) * image_size.x];
	}

	Vector4f diff_x(0.0f, 0.0f, 0.0f, 0.0f), diff_y(0.0f, 0.0f, 0.0f, 0.0f);

	bool do_plus_1 = false;
	if (xp1_y.w <= 0 || x_yp1.w <= 0 || xm1_y.w <= 0 || x_ym1.w <= 0) do_plus_1 = true;
	else {
		diff_x = xp1_y - xm1_y, diff_y = x_yp1 - x_ym1;

		float length_diff = ORUTILS_MAX(diff_x.x * diff_x.x + diff_x.y * diff_x.y + diff_x.z * diff_x.z,
		                                diff_y.x * diff_y.x + diff_y.y * diff_y.y + diff_y.z * diff_y.z);

		if (length_diff * voxel_size * voxel_size > (0.15f * 0.15f)) do_plus_1 = true;
	}

	if (do_plus_1) {
		if (useSmoothing) {
			xp1_y = points_ray[(x + 1) + y * image_size.x];
			x_yp1 = points_ray[x + (y + 1) * image_size.x];
			xm1_y = points_ray[(x - 1) + y * image_size.x];
			x_ym1 = points_ray[x + (y - 1) * image_size.x];
			diff_x = xp1_y - xm1_y;
			diff_y = x_yp1 - x_ym1;
		}

		if (xp1_y.w <= 0 || x_yp1.w <= 0 || xm1_y.w <= 0 || x_ym1.w <= 0) {
			found_point = false;
			return;
		}
	}

	out_normal.x = -(diff_x.y * diff_y.z - diff_x.z * diff_y.y);
	out_normal.y = -(diff_x.z * diff_y.x - diff_x.x * diff_y.z);
	out_normal.z = -(diff_x.x * diff_y.y - diff_x.y * diff_y.x);

	if (flipNormals) out_normal = -out_normal;

	float normScale = 1.0f / sqrt(out_normal.x * out_normal.x + out_normal.y * out_normal.y + out_normal.z * out_normal.z);
	out_normal *= normScale;

	angle = out_normal.x * light_source.x + out_normal.y * light_source.y + out_normal.z * light_source.z;
	if (!(angle > 1e-7)) found_point = false;
}

_CPU_AND_GPU_CODE_ inline void drawPixelGrey(DEVICEPTR(Vector4u)& dest, const THREADPTR(float)& angle) {
	float outRes = (0.8f * angle + 0.2f) * 255.0f;
	dest = Vector4u((uchar) outRes);
}

_CPU_AND_GPU_CODE_ inline void drawPixelGreen(DEVICEPTR(Vector4u)& dest, const THREADPTR(float)& angle) {
	float outRes = (0.8f * angle + 0.2f) * 255.0f;
	const float factorRed = 0.5f;
	const float factorGreen = 1.0f;
	const float factorBlue = 0.7f;
	dest = Vector4u((uchar) (outRes * factorRed), (uchar) (outRes * factorGreen), (uchar) (outRes * factorBlue), 255);
}

_CPU_AND_GPU_CODE_ inline void drawPixelOverlay(DEVICEPTR(Vector4u)& dest, const THREADPTR(float)& angle) {
	float outRes = (0.8f * angle + 0.2f) * 255.0f;
	const float factorRed = 0.5f;
	const float factorGreen = 0.5f;
	const float factorBlue = 1.0f;
	const float opacityFactor = 0.5f;
	const float transparencyFactor = 1.0f - opacityFactor;
	dest = Vector4u(
			(uchar) ((float) dest.r * transparencyFactor + (outRes * factorRed) * opacityFactor),
			(uchar) ((float) dest.g * transparencyFactor + (outRes * factorGreen) * opacityFactor),
			(uchar) ((float) dest.b * transparencyFactor + (outRes * factorBlue) * opacityFactor),
			255);
}

_CPU_AND_GPU_CODE_ inline float interpolateCol(float val, float y0, float x0, float y1, float x1) {
	return (val - x0) * (y1 - y0) / (x1 - x0) + y0;
}

_CPU_AND_GPU_CODE_ inline float baseCol(float val) {
	if (val <= -0.75f) return 0.0f;
	else if (val <= -0.25f) return interpolateCol(val, 0.0f, -0.75f, 1.0f, -0.25f);
	else if (val <= 0.25f) return 1.0f;
	else if (val <= 0.75f) return interpolateCol(val, 1.0f, 0.25f, 0.0f, 0.75f);
	else return 0.0;
}

_CPU_AND_GPU_CODE_ inline void drawPixelConfidence(DEVICEPTR(Vector4u)& dest, const THREADPTR(float)& angle, const THREADPTR(float)& confidence) {
	//Vector4f color_red(255, 0, 0, 255), color_green(0, 255, 0, 255);
	float confidenceNorm = CLAMP(confidence, 0, 100.f) / 100.0f;

	Vector4f color;
	color.r = (uchar) (baseCol(confidenceNorm) * 255.0f);
	color.g = (uchar) (baseCol(confidenceNorm - 0.5f) * 255.0f);
	color.b = (uchar) (baseCol(confidenceNorm + 0.5f) * 255.0f);
	color.a = 255;

	Vector4f outRes = (0.8f * angle + 0.2f) * color;
	dest = TO_UCHAR4(outRes);
}

_CPU_AND_GPU_CODE_ inline void drawPixelNormal(DEVICEPTR(Vector4u)& dest, const THREADPTR(Vector3f)& normal_obj) {
	dest.r = (uchar) ((0.3f + (-normal_obj.r + 1.0f) * 0.35f) * 255.0f);
	dest.g = (uchar) ((0.3f + (-normal_obj.g + 1.0f) * 0.35f) * 255.0f);
	dest.b = (uchar) ((0.3f + (-normal_obj.b + 1.0f) * 0.35f) * 255.0f);
}

template<class TVoxel, class TIndex>
_CPU_AND_GPU_CODE_ inline void drawPixelColour(DEVICEPTR(Vector4u)& dest, const CONSTPTR(Vector3f)& point,
                                               const CONSTPTR(TVoxel)* voxelBlockData, const CONSTPTR(typename TIndex::IndexData)* indexData) {
	Vector4f clr = VoxelColorReader<TVoxel::hasColorInformation, TVoxel, TIndex>::interpolate(voxelBlockData, indexData, point);

	dest.x = (uchar) (clr.x * 255.0f);
	dest.y = (uchar) (clr.y * 255.0f);
	dest.z = (uchar) (clr.z * 255.0f);
	dest.w = 255;
}


template<class TVoxel, class TIndex>
_CPU_AND_GPU_CODE_ inline void processPixelICP(DEVICEPTR(Vector4f)& pointsMap, DEVICEPTR(Vector4f)& normalsMap,
                                               const THREADPTR(Vector3f)& point, bool foundPoint, const CONSTPTR(TVoxel)* voxelData,
                                               const CONSTPTR(typename TIndex::IndexData)* voxelIndex,
                                               float voxelSize, const THREADPTR(Vector3f)& lightSource) {
	Vector3f outNormal;
	float angle;

	computeNormalAndAngle<TVoxel, TIndex>(foundPoint, point, voxelData, voxelIndex, lightSource, outNormal, angle);

	if (foundPoint) {
		Vector4f outPoint4;
		outPoint4.x = point.x * voxelSize;
		outPoint4.y = point.y * voxelSize;
		outPoint4.z = point.z * voxelSize;
		outPoint4.w = 1.0f;
		pointsMap = outPoint4;

		Vector4f outNormal4;
		outNormal4.x = outNormal.x;
		outNormal4.y = outNormal.y;
		outNormal4.z = outNormal.z;
		outNormal4.w = 0.0f;
		normalsMap = outNormal4;
	} else {
		Vector4f out4;
		out4.x = 0.0f;
		out4.y = 0.0f;
		out4.z = 0.0f;
		out4.w = -1.0f;

		pointsMap = out4;
		normalsMap = out4;
	}
}

template<bool TUseSmoothing, bool TFlipNormals>
_CPU_AND_GPU_CODE_ inline void processPixelICP(DEVICEPTR(Vector4f)* locations, DEVICEPTR(Vector4f)* normals,
                                               const CONSTPTR(Vector4f)* raycast_points, const THREADPTR(Vector2i)& raycast_image_size,
                                               const THREADPTR(int)& x, const THREADPTR(int)& y, const float voxel_size,
                                               const THREADPTR(Vector3f)& light_source) {
	Vector3f normal;
	float angle;

	int locId = x + y * raycast_image_size.x;
	Vector4f point = raycast_points[locId];

	bool point_found = point.w > 0.0f;

	computeNormalAndAngle<TUseSmoothing, TFlipNormals>(point_found, x, y, raycast_points, light_source, voxel_size, raycast_image_size, normal,
	                                                   angle);

	if (point_found) {
		Vector4f outPoint4;
		outPoint4.x = point.x * voxel_size;
		outPoint4.y = point.y * voxel_size;
		outPoint4.z = point.z * voxel_size;
		outPoint4.w = point.w;//outPoint4.w = 1.0f;
		locations[locId] = outPoint4;

		Vector4f outNormal4;
		outNormal4.x = normal.x;
		outNormal4.y = normal.y;
		outNormal4.z = normal.z;
		outNormal4.w = 0.0f;
		normals[locId] = outNormal4;
	} else {
		Vector4f out4;
		out4.x = 0.0f;
		out4.y = 0.0f;
		out4.z = 0.0f;
		out4.w = -1.0f;

		locations[locId] = out4;
		normals[locId] = out4;
	}
}

template<bool useSmoothing, bool flipNormals>
_CPU_AND_GPU_CODE_ inline void processPixelGrey_ImageNormals(DEVICEPTR(Vector4u)* outRendering, const CONSTPTR(Vector4f)* pointsRay,
                                                             const THREADPTR(Vector2i)& imgSize, const THREADPTR(int)& x, const THREADPTR(int)& y,
                                                             const float voxelSize, const THREADPTR(Vector3f)& lightSource) {
	Vector3f outNormal;
	float angle;

	int locId = x + y * imgSize.x;
	Vector4f point = pointsRay[locId];

	bool foundPoint = point.w > 0.0f;
	computeNormalAndAngle<useSmoothing, flipNormals>(foundPoint, x, y, pointsRay, lightSource, voxelSize, imgSize, outNormal, angle);

	if (foundPoint) drawPixelGrey(outRendering[locId], angle);
	else outRendering[locId] = Vector4u((uchar) 0);
}

template<bool useSmoothing, bool flipNormals>
_CPU_AND_GPU_CODE_ inline void processPixelNormals_ImageNormals(DEVICEPTR(Vector4u)* outRendering, const CONSTPTR(Vector4f)* pointsRay,
                                                                const THREADPTR(Vector2i)& imgSize, const THREADPTR(int)& x, const THREADPTR(int)& y,
                                                                float voxelSize, Vector3f lightSource) {
	Vector3f outNormal;
	float angle;

	int locId = x + y * imgSize.x;
	Vector4f point = pointsRay[locId];

	bool foundPoint = point.w > 0.0f;
	computeNormalAndAngle<useSmoothing, flipNormals>(foundPoint, x, y, pointsRay, lightSource, voxelSize, imgSize, outNormal, angle);

	if (foundPoint) drawPixelNormal(outRendering[locId], outNormal);
	else outRendering[locId] = Vector4u((uchar) 0);
}

template<bool useSmoothing, bool flipNormals>
_CPU_AND_GPU_CODE_ inline void processPixelConfidence_ImageNormals(DEVICEPTR(Vector4u)* outRendering, const CONSTPTR(Vector4f)* pointsRay,
                                                                   const THREADPTR(Vector2i)& imgSize, const THREADPTR(int)& x,
                                                                   const THREADPTR(int)& y, float voxelSize, Vector3f lightSource) {
	Vector3f outNormal;
	float angle;

	int locId = x + y * imgSize.x;
	Vector4f point = pointsRay[locId];

	bool foundPoint = point.w > 0.0f;
	computeNormalAndAngle<useSmoothing, flipNormals>(foundPoint, x, y, pointsRay, lightSource, voxelSize, imgSize, outNormal, angle);

	if (foundPoint) drawPixelConfidence(outRendering[locId], angle, point.w - 1.0f);
	else outRendering[locId] = Vector4u((uchar) 0);
}

template<class TVoxel, class TIndex>
_CPU_AND_GPU_CODE_ inline void processPixelGrey(DEVICEPTR(Vector4u)& outRendering, const CONSTPTR(Vector3f)& point,
                                                bool foundPoint, const CONSTPTR(TVoxel)* voxelData,
                                                const CONSTPTR(typename TIndex::IndexData)* voxelIndex,
                                                Vector3f lightSource) {
	Vector3f outNormal;
	float angle;

	computeNormalAndAngle<TVoxel, TIndex>(foundPoint, point, voxelData, voxelIndex, lightSource, outNormal, angle);

	if (foundPoint) drawPixelGrey(outRendering, angle);
	else outRendering = Vector4u((uchar) 0);
}

template<class TVoxel, class TIndex>
_CPU_AND_GPU_CODE_ inline void processPixelGreen(DEVICEPTR(Vector4u)& outRendering, const CONSTPTR(Vector3f)& point,
                                                 bool foundPoint, const CONSTPTR(TVoxel)* voxelData,
                                                 const CONSTPTR(typename TIndex::IndexData)* voxelIndex,
                                                 Vector3f lightSource) {
	Vector3f outNormal;
	float angle;

	computeNormalAndAngle<TVoxel, TIndex>(foundPoint, point, voxelData, voxelIndex, lightSource, outNormal, angle);

	if (foundPoint) drawPixelGreen(outRendering, angle);
	else outRendering = Vector4u((uchar) 0);
}

template<class TVoxel, class TIndex>
_CPU_AND_GPU_CODE_ inline void processPixelOverlay(DEVICEPTR(Vector4u)& outRendering, const CONSTPTR(Vector3f)& point,
                                                   bool foundPoint, const CONSTPTR(TVoxel)* voxelData,
                                                   const CONSTPTR(typename TIndex::IndexData)* voxelIndex,
                                                   Vector3f lightSource) {
	Vector3f outNormal;
	float angle;

	computeNormalAndAngle<TVoxel, TIndex>(foundPoint, point, voxelData, voxelIndex, lightSource, outNormal, angle);

	if (foundPoint) drawPixelOverlay(outRendering, angle);
	else outRendering = Vector4u((uchar) 0);
}

template<class TVoxel, class TIndex>
_CPU_AND_GPU_CODE_ inline void processPixelColour(DEVICEPTR(Vector4u)& outRendering, const CONSTPTR(Vector3f)& point,
                                                  bool foundPoint, const CONSTPTR(TVoxel)* voxelData,
                                                  const CONSTPTR(typename TIndex::IndexData)* voxelIndex) {
	if (foundPoint) drawPixelColour<TVoxel, TIndex>(outRendering, point, voxelData, voxelIndex);
	else outRendering = Vector4u((uchar) 0);
}

template<class TVoxel, class TIndex>
_CPU_AND_GPU_CODE_ inline void processPixelNormal(DEVICEPTR(Vector4u)& outRendering, const CONSTPTR(Vector3f)& point,
                                                  bool foundPoint, const CONSTPTR(TVoxel)* voxelData,
                                                  const CONSTPTR(typename TIndex::IndexData)* voxelIndex,
                                                  Vector3f lightSource) {
	Vector3f outNormal;
	float angle;

	computeNormalAndAngle<TVoxel, TIndex>(foundPoint, point, voxelData, voxelIndex, lightSource, outNormal, angle);

	if (foundPoint) drawPixelNormal(outRendering, outNormal);
	else outRendering = Vector4u((uchar) 0);
}

template<class TVoxel, class TIndex>
_CPU_AND_GPU_CODE_ inline void processPixelConfidence(DEVICEPTR(Vector4u)& outRendering, const CONSTPTR(Vector4f)& point,
                                                      bool foundPoint, const CONSTPTR(TVoxel)* voxelData,
                                                      const CONSTPTR(typename TIndex::IndexData)* voxelIndex,
                                                      Vector3f lightSource) {
	Vector3f outNormal;
	float angle;

	computeNormalAndAngle<TVoxel, TIndex>(foundPoint, TO_VECTOR3(point), voxelData, voxelIndex, lightSource, outNormal, angle);

	if (foundPoint) drawPixelConfidence(outRendering, angle, point.w - 1.0f);
	else outRendering = Vector4u((uchar) 0);
}

} // namespace ITMLib