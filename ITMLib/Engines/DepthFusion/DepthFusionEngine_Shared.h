//  ================================================================
//  Created by Gregory Kramida on 5/25/18.
//  Copyright (c) 2018-2000 Gregory Kramida
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

#include "../../Objects/Volume/RepresentationAccess.h"
#include "../../Objects/Volume/TrilinearInterpolation.h"
#include "../../Utils/PixelUtils.h"
#include "../../Utils/Enums/VoxelFlags.h"


// region ============================== UPDATE SDF/COLOR IN VOXEL USING DEPTH PIXEL ===================================

template<class TVoxel>
_CPU_AND_GPU_CODE_ inline void UpdateSdfAndFlagsBasedOnDistanceSurfaceToVoxel(
		DEVICEPTR(TVoxel)& voxel, float signed_distance_surface_to_voxel_along_camera_ray, float truncation_distance,
		float final_distance_cutoff) {
	if (signed_distance_surface_to_voxel_along_camera_ray < -truncation_distance + 4e-07) {
		if (signed_distance_surface_to_voxel_along_camera_ray < -final_distance_cutoff) {
			//the voxel is beyond the narrow band, on the other side of the surface, but also really far away.
			//exclude from computation.
			voxel.sdf = TVoxel::floatToValue(-1.0);
			voxel.flags = ITMLib::VOXEL_UNKNOWN;
		} else {
			//the voxel is beyond the narrow band, on the other side of the surface. Set SDF to -1.0
			voxel.sdf = TVoxel::floatToValue(-1.0);
			voxel.flags = ITMLib::VOXEL_TRUNCATED;
		}
	} else if (signed_distance_surface_to_voxel_along_camera_ray > truncation_distance - 4e-07) {
		if (signed_distance_surface_to_voxel_along_camera_ray > final_distance_cutoff) {
			//the voxel is in front of the narrow band, between the surface and the camera, but also really far away.
			//exclude from computation.
			voxel.sdf = TVoxel::floatToValue(1.0);
			voxel.flags = ITMLib::VOXEL_UNKNOWN;
		} else {
			//the voxel is in front of the narrow band, between the surface and the camera. Set SDF to 1.0
			voxel.sdf = TVoxel::floatToValue(1.0);
			voxel.flags = ITMLib::VOXEL_TRUNCATED;
		}
	} else {
		// The voxel lies within the narrow band, between truncation boundaries.
		// Update SDF in proportion to the distance from surface.
		voxel.sdf = TVoxel::floatToValue(signed_distance_surface_to_voxel_along_camera_ray / truncation_distance);
		voxel.flags = ITMLib::VOXEL_NONTRUNCATED;
	}
}

/**
 * \brief Voxel update without confidence computation
 * \tparam TVoxel
 * \param voxel
 * \param voxel_in_scene_coordinates
 * \param depth_camera_pose
 * \param depth_camera_projection_parameters
 * \param truncation_distance
 * \param depth_image an array of float depths corresponding to the depth image
 * \param depth_image_size
 * \return -1 if voxel point is behind camera or depth value is invalid (0.0f),
 * distance between voxel point & measured surface depth along camera ray otherwise
 */
template<class TVoxel>
_CPU_AND_GPU_CODE_ inline float ComputeUpdatedLiveVoxelDepthInfo(
		DEVICEPTR(TVoxel)& voxel,
		const THREADPTR(Vector4f)& voxel_in_scene_coordinates,
		const CONSTPTR(Matrix4f)& depth_camera_pose,
		const CONSTPTR(Vector4f)& depth_camera_projection_parameters,
		float truncation_distance,
		const CONSTPTR(float)* depth_image,
		const CONSTPTR(Vector2i)& depth_image_size,
		float effective_range_cutoff = 0.08f) {

	// project point into image (voxel point in camera coordinates)
	Vector4f voxel_point_in_camera_coordinates = depth_camera_pose * voxel_in_scene_coordinates;
	// if point is behind the camera, don't modify any voxels
	if (voxel_point_in_camera_coordinates.z <= 0) {
		return -1.0f;
	}

	Vector2f voxel_point_projected_to_image;
	voxel_point_projected_to_image.x = depth_camera_projection_parameters.fx * voxel_point_in_camera_coordinates.x
	                                   / voxel_point_in_camera_coordinates.z + depth_camera_projection_parameters.cx;
	voxel_point_projected_to_image.y = depth_camera_projection_parameters.fy * voxel_point_in_camera_coordinates.y
	                                   / voxel_point_in_camera_coordinates.z + depth_camera_projection_parameters.cy;

	// point falls outside of the image bounds
	if ((voxel_point_projected_to_image.x < 1) || (voxel_point_projected_to_image.x > depth_image_size.width - 2)
	    || (voxel_point_projected_to_image.y < 1) || (voxel_point_projected_to_image.y > depth_image_size.height - 2)) {
		return -1.0f;
	}

	// get measured depth_image from image
	float depth_measure = depth_image[static_cast<int>(voxel_point_projected_to_image.x + 0.5f) +
	                                  static_cast<int>(voxel_point_projected_to_image.y + 0.5f) * depth_image_size.width];

	// if depth_image is "invalid", return "unknown"
	if (depth_measure <= 0.0f) {
		//keep voxel flags at ITMLib::VOXEL_UNKNOWN
		return -1.0f;
	}

	// signed_distance_surface_to_voxel_along_camera_ray (i.e. "eta" or 𝜂 in many publications) =
	// [distance from surface to camera, i.e. depth_image] - [distance from voxel to camera]
	// effectively, eta is the distance between measured surface & voxel point
	float signed_distance_surface_to_voxel_along_camera_ray = depth_measure - voxel_point_in_camera_coordinates.z;
	UpdateSdfAndFlagsBasedOnDistanceSurfaceToVoxel(voxel, signed_distance_surface_to_voxel_along_camera_ray,
	                                               truncation_distance, effective_range_cutoff);
	return signed_distance_surface_to_voxel_along_camera_ray;
}

/**
 * \brief Voxel update with confidence computation
 * \tparam TVoxel
 * \param voxel
 * \param voxel_in_scene_coordinates
 * \param depth_camera_scene_matrix
 * \param depth_camera_projection_parameters
 * \param truncation_distance
 * \param maxW
 * \param depth_image
 * \param confidences_at_pixels
 * \param image_size
 * \return -1 if voxel point is behind camera or depth value is invalid (0.0f),
 * distance between voxel point & measured surface depth along camera ray otherwise
 */
template<class TVoxel>
_CPU_AND_GPU_CODE_ inline float ComputeUpdatedLiveVoxelDepthInfo(
		DEVICEPTR(TVoxel)& voxel,
		const THREADPTR(Vector4f)& voxel_in_scene_coordinates,
		const CONSTPTR(Matrix4f)& depth_camera_scene_matrix,
		const CONSTPTR(Vector4f)& depth_camera_projection_parameters,
		float truncation_distance,
		const CONSTPTR(float)* depth_image,
		const CONSTPTR(float)* confidences_at_pixels,
		const CONSTPTR(Vector2i)& image_size,
		float effective_range_cutoff = 0.08f) {


	// project point into image
	Vector4f voxel_point_in_camera_coordinates = depth_camera_scene_matrix * voxel_in_scene_coordinates;
	// if the point is behind the camera, don't make any changes to SDF and return -1 to short-circuit further updates
	if (voxel_point_in_camera_coordinates.z <= 0) return -1;

	Vector2f voxel_point_projected_to_image;
	voxel_point_projected_to_image.x =
			depth_camera_projection_parameters.x * voxel_point_in_camera_coordinates.x /
			voxel_point_in_camera_coordinates.z + depth_camera_projection_parameters.z;
	voxel_point_projected_to_image.y =
			depth_camera_projection_parameters.y * voxel_point_in_camera_coordinates.y /
			voxel_point_in_camera_coordinates.z + depth_camera_projection_parameters.w;
	if ((voxel_point_projected_to_image.x < 1) || (voxel_point_projected_to_image.x > image_size.x - 2) ||
	    (voxel_point_projected_to_image.y < 1) || (voxel_point_projected_to_image.y > image_size.y - 2))
		return -1;

	// point falls outside of the image bounds
	int pixelIndex =
			static_cast<int>(voxel_point_projected_to_image.x + 0.5f) +
			static_cast<int>(voxel_point_projected_to_image.y + 0.5f) * image_size.x;

	// get measured depth from image
	float depth_measure = depth_image[pixelIndex];
	if (depth_measure <= 0.0) return -1;

	// signed_surface_to_voxel_along_camera_ray (i.e. eta) =
	// [distance from surface to camera, i.e. depth_image] - [distance from voxel to camera]
	// effectively, eta is the distance between measured surface & voxel point
	float signed_surface_to_voxel_along_camera_ray = depth_measure - voxel_point_in_camera_coordinates.z;
	voxel.confidence = TVoxel::floatToValue(confidences_at_pixels[pixelIndex]);

	UpdateSdfAndFlagsBasedOnDistanceSurfaceToVoxel(voxel, signed_surface_to_voxel_along_camera_ray, truncation_distance,
	                                               effective_range_cutoff);
	return signed_surface_to_voxel_along_camera_ray;
}

template<class TVoxel>
_CPU_AND_GPU_CODE_ inline void ComputeUpdatedLiveVoxelColorInfo(
		DEVICEPTR(TVoxel)& voxel,
		const THREADPTR(Vector4f)& voxel_in_scene_coordinates,
		const CONSTPTR(Matrix4f)& rgb_camera_scene_matrix,
		const CONSTPTR(Vector4f)& rgb_camera_projection_parameters,
		float truncation_distance, float signed_distance_surface_to_voxel,
		const CONSTPTR(Vector4u)* rgb_image,
		const CONSTPTR(Vector2i)& image_size) {
	Vector4f voxel_in_camera_coordinates;
	Vector2f voxel_point_projected_to_image;

	voxel_in_camera_coordinates = rgb_camera_scene_matrix * voxel_in_scene_coordinates;

	voxel_point_projected_to_image.x =
			rgb_camera_projection_parameters.x * voxel_in_camera_coordinates.x
			/ voxel_in_camera_coordinates.z + rgb_camera_projection_parameters.z;
	voxel_point_projected_to_image.y =
			rgb_camera_projection_parameters.y * voxel_in_camera_coordinates.y
			/ voxel_in_camera_coordinates.z + rgb_camera_projection_parameters.w;
//TODO: the magic value 0.25f used to determine the cutoff distance for color processing should be pre-defined as a parameter -Greg (GitHub:Algomorph)
	//cut off voxels that are too far from the surface
	if (std::abs(signed_distance_surface_to_voxel) < 0.25f * truncation_distance) return;

	if ((voxel_point_projected_to_image.x < 1) || (voxel_point_projected_to_image.x > image_size.x - 2) ||
	    (voxel_point_projected_to_image.y < 1) || (voxel_point_projected_to_image.y > image_size.y - 2))
		return;

	voxel.clr = TO_UCHAR3(TO_VECTOR3(interpolateBilinear(rgb_image, voxel_point_projected_to_image, image_size)));
}
// endregion ===========================================================================================================

template<bool hasColor, bool hasConfidence, bool hasSemanticInformation, typename TVoxel>
struct ComputeUpdatedLiveVoxelInfo;

// region ========= VOXEL UPDATES FOR VOXELS WITH NO SEMANTIC INFORMATION ==============================================
//arguments to the "compute_allocated" member function should always be the same
#define COMPUTE_VOXEL_UPDATE_PARAMETERS \
DEVICEPTR(TVoxel) & voxel, const THREADPTR(Vector4f) & pt_model,\
const CONSTPTR(Matrix4f) & M_d, const CONSTPTR(Vector4f) & projParams_d,\
const CONSTPTR(Matrix4f) & M_rgb, const CONSTPTR(Vector4f) & projParams_rgb,\
const float mu, const int maxW,\
const CONSTPTR(float) *depth, const CONSTPTR(float) *confidence, const CONSTPTR(Vector2i) & imgSize_d,\
const CONSTPTR(Vector4u) *rgb, const CONSTPTR(Vector2i) & imgSize_rgb


// no color, no confidence, no semantic info
template<class TVoxel>
struct ComputeUpdatedLiveVoxelInfo<false, false, false, TVoxel> {
	_CPU_AND_GPU_CODE_ static void compute(COMPUTE_VOXEL_UPDATE_PARAMETERS) {
		ComputeUpdatedLiveVoxelDepthInfo(voxel, pt_model, M_d, projParams_d, mu, depth, imgSize_d);
	}
};
// with color, no confidence, no semantic info
template<class TVoxel>
struct ComputeUpdatedLiveVoxelInfo<true, false, false, TVoxel> {
	_CPU_AND_GPU_CODE_ static void compute(COMPUTE_VOXEL_UPDATE_PARAMETERS) {
		float eta = ComputeUpdatedLiveVoxelDepthInfo(voxel, pt_model, M_d, projParams_d, mu, depth, imgSize_d);
		ComputeUpdatedLiveVoxelColorInfo(voxel, pt_model, M_rgb, projParams_rgb, mu, eta, rgb, imgSize_rgb);
	}
};
// no color, with confidence, no semantic info
template<class TVoxel>
struct ComputeUpdatedLiveVoxelInfo<false, true, false, TVoxel> {
	_CPU_AND_GPU_CODE_ static void compute(COMPUTE_VOXEL_UPDATE_PARAMETERS) {
		ComputeUpdatedLiveVoxelDepthInfo(voxel, pt_model, M_d, projParams_d, mu, depth, confidence, imgSize_d);
	}
};
// with color, with confidence, no semantic info
template<class TVoxel>
struct ComputeUpdatedLiveVoxelInfo<true, true, false, TVoxel> {
	_CPU_AND_GPU_CODE_ static void compute(COMPUTE_VOXEL_UPDATE_PARAMETERS) {
		float eta = ComputeUpdatedLiveVoxelDepthInfo(voxel, pt_model, M_d, projParams_d, mu, depth, confidence,
		                                             imgSize_d);
		ComputeUpdatedLiveVoxelColorInfo(voxel, pt_model, M_rgb, projParams_rgb, mu, eta, rgb, imgSize_rgb);
	}
};
// endregion ===========================================================================================================
// region ========= VOXEL UPDATES FOR VOXELS WITH SEMANTIC INFORMATION =================================================

template<class TVoxel>
struct ComputeUpdatedLiveVoxelInfo<false, false, true, TVoxel> {
	_CPU_AND_GPU_CODE_ static void compute(COMPUTE_VOXEL_UPDATE_PARAMETERS) {
		float eta = ComputeUpdatedLiveVoxelDepthInfo(voxel, pt_model, M_d, projParams_d, mu, depth, imgSize_d);
	}
};
// with color, no confidence, with semantic info
template<class TVoxel>
struct ComputeUpdatedLiveVoxelInfo<true, false, true, TVoxel> {
	_CPU_AND_GPU_CODE_ static void compute(COMPUTE_VOXEL_UPDATE_PARAMETERS) {
		float eta = ComputeUpdatedLiveVoxelDepthInfo(voxel, pt_model, M_d, projParams_d, mu, depth, imgSize_d);
		ComputeUpdatedLiveVoxelColorInfo(voxel, pt_model, M_rgb, projParams_rgb, mu, eta, rgb, imgSize_rgb);
	}
};
// no color, with confidence, with semantic info
template<class TVoxel>
struct ComputeUpdatedLiveVoxelInfo<false, true, true, TVoxel> {
	_CPU_AND_GPU_CODE_ static void compute(COMPUTE_VOXEL_UPDATE_PARAMETERS) {
		float eta = ComputeUpdatedLiveVoxelDepthInfo(voxel, pt_model, M_d, projParams_d, mu, depth, confidence,
		                                             imgSize_d);
	}
};
// with color, with confidence, with semantic info
template<class TVoxel>
struct ComputeUpdatedLiveVoxelInfo<true, true, true, TVoxel> {
	_CPU_AND_GPU_CODE_ static void compute(COMPUTE_VOXEL_UPDATE_PARAMETERS) {
		float eta = ComputeUpdatedLiveVoxelDepthInfo(voxel, pt_model, M_d, projParams_d, mu, depth, confidence,
		                                             imgSize_d);
		ComputeUpdatedLiveVoxelColorInfo(voxel, pt_model, M_rgb, projParams_rgb, mu, eta, rgb, imgSize_rgb);
	}
};

#undef COMPUTE_VOXEL_UPDATE_PARAMETERS
// endregion ===========================================================================================================
// region ======================================== VOXEL UPDATE FUNCTOR ================================================

template<typename TTSDFVoxel, MemoryDeviceType TMemoryDeviceType, bool TStopIntegrationAtMaxIntegrationWeight, bool TUseSurfaceThickness>
struct VoxelDepthIntegrationFunctor {
public:
	VoxelDepthIntegrationFunctor(
			const ITMLib::VoxelVolumeParameters& volume_parameters,
			const ITMLib::View* view, Matrix4f depth_camera_extrinsic_matrix,
			float surface_thickness) :
			depth_image_size(view->depth.dimensions),
			depth_camera_projection_parameters(view->calibration_information.intrinsics_d.projectionParamsSimple.all),
			depth_camera_extrinsic_matrix(depth_camera_extrinsic_matrix),
			rgb_image_size(view->rgb.dimensions),
			rgb_camera_projection_parameters(view->calibration_information.intrinsics_rgb.projectionParamsSimple.all),
			rgb_camera_extrinsic_matrix(TTSDFVoxel::hasColorInformation ? view->calibration_information.trafo_rgb_to_depth.calib_inv * depth_camera_extrinsic_matrix : Matrix4f()),

			surface_thickness(surface_thickness),
			truncation_distance(volume_parameters.truncation_distance),
			max_integration_weight(volume_parameters.max_integration_weight),
			voxel_size(volume_parameters.voxel_size),

			depth(view->depth.GetData(TMemoryDeviceType)),
			rgb(view->rgb.GetData(TMemoryDeviceType)),
			confidence(view->depth_confidence.GetData(TMemoryDeviceType))
			{}

	_CPU_AND_GPU_CODE_
	inline void operator()(TTSDFVoxel& voxel, const Vector3i& voxel_position) {
		if (TStopIntegrationAtMaxIntegrationWeight) if (voxel.w_depth == max_integration_weight) return;
		Vector4f pt_model;
		pt_model.x = static_cast<float>(voxel_position.x * voxel_size);
		pt_model.y = static_cast<float>(voxel_position.y * voxel_size);
		pt_model.z = static_cast<float>(voxel_position.z * voxel_size);
		pt_model.w = 1.0f;
		ComputeUpdatedLiveVoxelInfo<TTSDFVoxel::hasColorInformation, TTSDFVoxel::hasConfidenceInformation, TTSDFVoxel::hasSemanticInformation, TTSDFVoxel>::compute(
				voxel, pt_model, depth_camera_extrinsic_matrix,
				depth_camera_projection_parameters, rgb_camera_extrinsic_matrix, rgb_camera_projection_parameters, truncation_distance, max_integration_weight, depth, confidence,
				depth_image_size, rgb, rgb_image_size);
	}

private:
	const Vector2i depth_image_size;
	Vector4f depth_camera_projection_parameters;
	Matrix4f depth_camera_extrinsic_matrix;
	Vector2i rgb_image_size;
	Vector4f rgb_camera_projection_parameters;
	Matrix4f rgb_camera_extrinsic_matrix;

	const float surface_thickness;
	const float truncation_distance;
	const int max_integration_weight;
	const float voxel_size;

	const float* depth;
	const Vector4u* rgb;
	const float* confidence;


};

// endregion
