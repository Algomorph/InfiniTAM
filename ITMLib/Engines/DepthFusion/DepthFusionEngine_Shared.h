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
#include "../../Utils/Enums/ExecutionMode.h"

namespace ITMLib {

template<bool THasSemanticInformation, bool TUseSurfaceThickness>
struct VoxelTsdfSetter;

template<bool TUseSurfaceThickness>
struct VoxelTsdfSetter<true, TUseSurfaceThickness> {
	template<typename TVoxel>
	_CPU_AND_GPU_CODE_ static inline void SetSdf(
			DEVICEPTR(TVoxel)& voxel, float signed_distance_surface_to_voxel_along_camera_ray, float truncation_distance,
			float final_distance_cutoff, float surface_thickness) {
		// Note: the small constant (1e+6) nudge is necessary to avoid PVA/VBH discrepancies in voxels
		// marked as truncated. Without it, some voxels whose volumes are mostly outside of the
		// final_distance_cutoff may not get allocated in the VBH index due to limited floating point precision.
		if (signed_distance_surface_to_voxel_along_camera_ray < -truncation_distance) {
			if (signed_distance_surface_to_voxel_along_camera_ray < (TUseSurfaceThickness ? -surface_thickness : -final_distance_cutoff + 1e+6)) {
				//the voxel is beyond the narrow band, on the other side of the surface, but also really far away.
				//exclude from computation.
				voxel.sdf = TVoxel::floatToValue(-1.0);
				voxel.flags = ITMLib::VOXEL_UNKNOWN;
			} else {
				//the voxel is beyond the narrow band, on the other side of the surface. Set SDF to -1.0
				voxel.sdf = TVoxel::floatToValue(-1.0);
				voxel.flags = ITMLib::VOXEL_TRUNCATED;
			}
		} else if (signed_distance_surface_to_voxel_along_camera_ray > truncation_distance) {
			if (signed_distance_surface_to_voxel_along_camera_ray > final_distance_cutoff - 1e+6) {
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
};

template<bool TUseSurfaceThickness>
struct VoxelTsdfSetter<false, TUseSurfaceThickness> {
	template<typename TVoxel>
	_CPU_AND_GPU_CODE_ static inline void SetSdf(
			DEVICEPTR(TVoxel)& voxel, float signed_distance_surface_to_voxel_along_camera_ray, float truncation_distance,
			float final_distance_cutoff, float surface_thickness) {
		if (signed_distance_surface_to_voxel_along_camera_ray < (TUseSurfaceThickness ? -surface_thickness : -final_distance_cutoff)) {
			voxel.sdf = TVoxel::floatToValue(-1.0);
		} else if (signed_distance_surface_to_voxel_along_camera_ray > truncation_distance) {
			voxel.sdf = TVoxel::floatToValue(1.0);
		} else {
			// The voxel lies within the narrow band, between truncation boundaries.
			// Update SDF in proportion to the distance from surface.
			voxel.sdf = TVoxel::floatToValue(signed_distance_surface_to_voxel_along_camera_ray / truncation_distance);
		}
	}
};

#pragma clang diagnostic push
#pragma ide diagnostic ignored "bugprone-incorrect-roundings"

_CPU_AND_GPU_CODE_ inline bool ProjectVoxelToImage(THREADPTR(int)& pixel_index,
                                                   THREADPTR(Vector4f)& voxel_in_camera_coordinates,
                                                   THREADPTR(Vector2f)& voxel_projected_to_image,
                                                   const THREADPTR(Vector4f)& voxel_in_volume_coordinates,
                                                   const CONSTPTR(Matrix4f)& depth_camera_pose,
                                                   const CONSTPTR(Vector4f)& depth_camera_projection_parameters,
                                                   const CONSTPTR(Vector2i)& image_size) {

	voxel_in_camera_coordinates = depth_camera_pose * voxel_in_volume_coordinates;
	// if point is behind the camera, don't modify any voxels
	if (voxel_in_camera_coordinates.z <= 0) {
		// if point is behind the camera, projection failed
		return false;
	}

	voxel_projected_to_image.x = depth_camera_projection_parameters.fx * voxel_in_camera_coordinates.x
	                             / voxel_in_camera_coordinates.z + depth_camera_projection_parameters.cx;
	voxel_projected_to_image.y = depth_camera_projection_parameters.fy * voxel_in_camera_coordinates.y
	                             / voxel_in_camera_coordinates.z + depth_camera_projection_parameters.cy;

	// point falls outside of the image bounds
	if ((voxel_projected_to_image.x < 1) || (static_cast<float>(voxel_projected_to_image.x) > image_size.width - 2.0f)
	    || (voxel_projected_to_image.y < 1) || (static_cast<float>(voxel_projected_to_image.y) > image_size.height - 2.0f)) {
		return false;
	}

	// get measured depth_image from image
	pixel_index =
			static_cast<int>(voxel_projected_to_image.x + 0.5f) +
			static_cast<int>(voxel_projected_to_image.y + 0.5f) * image_size.width;

	return true;
}

#pragma clang diagnostic pop

/**
 * \brief Voxel update without confidence computation
 * \tparam TVoxel
 * \param voxel
 * \param voxel_in_volume_coordinates
 * \param depth_camera_pose
 * \param depth_camera_projection_parameters
 * \param truncation_distance
 * \param depth_image an array of float depths corresponding to the depth image
 * \param depth_image_size
 * \return -1 if voxel point is behind camera or depth value is invalid (0.0f),
 * distance between voxel point & measured surface depth along camera ray otherwise
 */
template<bool THasSemanticInformation, bool TUseSurfaceThickness, ExecutionMode TExecutionMode, typename TVoxel>
_CPU_AND_GPU_CODE_ inline float FuseDepthIntoVoxel(
		DEVICEPTR(TVoxel)& voxel,
		const THREADPTR(Vector4f)& voxel_in_volume_coordinates,
		const CONSTPTR(Matrix4f)& depth_camera_pose,
		const CONSTPTR(Vector4f)& depth_camera_projection_parameters,
		float truncation_distance,
		const CONSTPTR(float)* depth_image,
		const CONSTPTR(Vector2i)& depth_image_size,
		float effective_range_cutoff,
		float surface_thickness,
		bool verbose) {

	int pixel_index;
	Vector4f voxel_in_camera_coordinates;
	Vector2f voxel_projected_to_image;
	bool projection_succeeded = ProjectVoxelToImage(pixel_index, voxel_in_camera_coordinates, voxel_projected_to_image,
	                                                voxel_in_volume_coordinates, depth_camera_pose, depth_camera_projection_parameters,
	                                                depth_image_size);
	if (TExecutionMode == DIAGNOSTIC && verbose) {
		printf("Focus spot/voxel projection: %s. Coordinates: %s%f, %f%s. Pixel index: %d.\n",
		       projection_succeeded ? "succeeded" : "failed", red, voxel_projected_to_image.x, voxel_projected_to_image.y, reset, pixel_index);
	}
	if (!projection_succeeded) {
		// if projection fails, don't modify and return special value
		return -1.0;
	}

	// get measured depth_image from image
	float depth_measure = depth_image[pixel_index];

	// if depth_image contains "invalid" magic value at this pixel, return magic value for "unknown"
	if (depth_measure <= 0.0f) {
		//keep sdf the same, voxel flags at ITMLib::VOXEL_UNKNOWN
		return -1.0f;
	}

	// signed_distance_surface_to_voxel_along_camera_ray (i.e. "eta" (𝜂) in many publications,
	// "delta" (δ) in Slavcheva's publications) =
	// [distance from surface to camera, i.e. depth_image] - [distance from voxel to camera]
	// effectively, eta is the distance between measured surface & voxel point
	float signed_distance_surface_to_voxel_along_camera_ray = depth_measure - voxel_in_camera_coordinates.z;

	if (TExecutionMode == DIAGNOSTIC && verbose) {
		printf("Projective signed distance from surface to voxel: %f, depth to surface: %f\n", signed_distance_surface_to_voxel_along_camera_ray, depth_measure);
	}

	VoxelTsdfSetter<THasSemanticInformation, TUseSurfaceThickness>::SetSdf(voxel, signed_distance_surface_to_voxel_along_camera_ray,
	                                                                       truncation_distance, effective_range_cutoff, surface_thickness);
	return signed_distance_surface_to_voxel_along_camera_ray;
}

/**
 * \brief Voxel update with confidence computation
 * \tparam TVoxel
 * \param voxel
 * \param voxel_in_volume_coordinates
 * \param depth_camera_pose
 * \param depth_camera_projection_parameters
 * \param truncation_distance
 * \param maxW
 * \param depth_image
 * \param confidences_at_pixels
 * \param depth_image_size
 * \return -1 if voxel point is behind camera or depth value is invalid (0.0f),
 * distance between voxel point & measured surface depth along camera ray otherwise
 */
template<bool THasSemanticInformation, bool TUseSurfaceThickness, ExecutionMode TExecutionMode, typename TVoxel>
_CPU_AND_GPU_CODE_ inline float FuseDepthIntoVoxel(
		DEVICEPTR(TVoxel)& voxel,
		const THREADPTR(Vector4f)& voxel_in_volume_coordinates,
		const CONSTPTR(Matrix4f)& depth_camera_pose,
		const CONSTPTR(Vector4f)& depth_camera_projection_parameters,
		float truncation_distance,
		const CONSTPTR(float)* depth_image,
		const CONSTPTR(float)* confidences_at_pixels,
		const CONSTPTR(Vector2i)& depth_image_size,
		float effective_range_cutoff,
		float surface_thickness,
		bool verbose) {


	int pixel_index;
	Vector4f voxel_in_camera_coordinates;
	Vector2f voxel_projected_to_image;
	bool projection_succeeded = ProjectVoxelToImage(pixel_index, voxel_in_camera_coordinates, voxel_projected_to_image,
	                                                voxel_in_volume_coordinates, depth_camera_pose, depth_camera_projection_parameters,
	                                                depth_image_size);
	if (!projection_succeeded) {
		// if projection fails, don't modify voxel and return special value
		return -1.0;
	}

	// get measured depth from image
	float depth_measure = depth_image[pixel_index];
	if (depth_measure <= 0.0) return -1;

	// signed_distance_surface_to_voxel_along_camera_ray (i.e. "eta" (𝜂) in many publications,
	// "delta" (δ) in Slavcheva's publications) =
	// [distance from surface to camera, i.e. depth_image] - [distance from voxel to camera]
	// effectively, eta is the distance between measured surface & voxel point
	float signed_surface_to_voxel_along_camera_ray = depth_measure - voxel_in_camera_coordinates.z;
	voxel.confidence = TVoxel::floatToValue(confidences_at_pixels[pixel_index]);
	VoxelTsdfSetter<THasSemanticInformation, TUseSurfaceThickness>::SetSdf(voxel, signed_surface_to_voxel_along_camera_ray, truncation_distance,
	                                                                       effective_range_cutoff, surface_thickness);
	return signed_surface_to_voxel_along_camera_ray;
}

template<class TVoxel>
_CPU_AND_GPU_CODE_ inline void FuseColorIntoVoxel(
		DEVICEPTR(TVoxel)& voxel,
		const THREADPTR(Vector4f)& voxel_in_volume_coordinates,
		const CONSTPTR(Matrix4f)& rgb_camera_pose,
		const CONSTPTR(Vector4f)& rgb_camera_projection_parameters,
		float truncation_distance, float signed_distance_surface_to_voxel,
		const CONSTPTR(Vector4u)* rgb_image,
		const CONSTPTR(Vector2i)& rgb_image_size) {

//TODO: the magic value 0.25f used to determine the cutoff distance for color processing should be pre-defined as a parameter -Greg (GitHub:Algomorph)
	//cut off voxels that are too far from the surface
	if (signed_distance_surface_to_voxel == -1.0 ||
	    std::abs(signed_distance_surface_to_voxel) < 0.25f * truncation_distance)
		return;

	int pixel_index;
	Vector4f voxel_in_camera_coordinates;
	Vector2f voxel_projected_to_image;
	if (!ProjectVoxelToImage(pixel_index, voxel_in_camera_coordinates, voxel_projected_to_image,
	                         voxel_in_volume_coordinates, rgb_camera_pose, rgb_camera_projection_parameters, rgb_image_size)) {
		// if projection fails, don't modify voxel and return
		return;
	}

	voxel.clr = TO_UCHAR3(TO_VECTOR3(interpolateBilinear(rgb_image, voxel_projected_to_image, rgb_image_size)));
}

// endregion ===========================================================================================================
template<bool THasColor, bool THasConfidence, bool THasSemanticInformation, bool TUseSurfaceThickness, typename TVoxel, ExecutionMode TExecutionMode>
struct ComputeUpdatedLiveVoxelInfo;
// region ========= VOXEL UPDATES FOR VOXELS WITH NO SEMANTIC INFORMATION ==============================================
//arguments to the "compute_allocated" member function should always be the same
#define COMPUTE_VOXEL_UPDATE_PARAMETERS \
DEVICEPTR(TVoxel) & voxel, const THREADPTR(Vector4f) & voxel_in_volume_coordinates,\
const CONSTPTR(Matrix4f) & depth_camera_pose, const CONSTPTR(Vector4f) & depth_camera_projection_parameters,\
const CONSTPTR(Matrix4f) & rgb_camera_pose, const CONSTPTR(Vector4f) & rgb_camera_projection_parameters,\
const float truncation_distance,\
const CONSTPTR(float)* depth_image, const CONSTPTR(float) *confidence, const CONSTPTR(Vector2i) &depth_image_size,\
const CONSTPTR(Vector4u)* rgb_image, const CONSTPTR(Vector2i) & rgb_image_size,  \
const CONSTPTR(float) effective_range_cutoff, const CONSTPTR(float) surface_thickness,                       \
bool verbose
// no color, no confidence
template<bool THasSemanticInformation, bool TUseSurfaceThickness, class TVoxel, ExecutionMode TExecutionMode>
struct ComputeUpdatedLiveVoxelInfo<false, false, THasSemanticInformation, TUseSurfaceThickness, TVoxel, TExecutionMode> {
	_CPU_AND_GPU_CODE_ static void compute(COMPUTE_VOXEL_UPDATE_PARAMETERS) {
		FuseDepthIntoVoxel<THasSemanticInformation, TUseSurfaceThickness, TExecutionMode>(
				voxel, voxel_in_volume_coordinates, depth_camera_pose, depth_camera_projection_parameters, truncation_distance, depth_image,
				depth_image_size, effective_range_cutoff, surface_thickness, verbose);
	}
};
// with color, no confidence
template<bool THasSemanticInformation, bool TUseSurfaceThickness, class TVoxel, ExecutionMode TExecutionMode>
struct ComputeUpdatedLiveVoxelInfo<true, false, THasSemanticInformation, TUseSurfaceThickness, TVoxel, TExecutionMode> {
	_CPU_AND_GPU_CODE_ static void compute(COMPUTE_VOXEL_UPDATE_PARAMETERS) {
		float eta = FuseDepthIntoVoxel<THasSemanticInformation, TUseSurfaceThickness, TExecutionMode>(
				voxel, voxel_in_volume_coordinates, depth_camera_pose, depth_camera_projection_parameters, truncation_distance, depth_image,
				depth_image_size, effective_range_cutoff, surface_thickness, verbose);
		FuseColorIntoVoxel(voxel, voxel_in_volume_coordinates, rgb_camera_pose, rgb_camera_projection_parameters, truncation_distance, eta,
		                   rgb_image, rgb_image_size);
	}
};
// no color, with confidence
template<bool THasSemanticInformation, bool TUseSurfaceThickness, class TVoxel, ExecutionMode TExecutionMode>
struct ComputeUpdatedLiveVoxelInfo<false, true, THasSemanticInformation, TUseSurfaceThickness, TVoxel, TExecutionMode> {
	_CPU_AND_GPU_CODE_ static void compute(COMPUTE_VOXEL_UPDATE_PARAMETERS) {
		FuseDepthIntoVoxel<THasSemanticInformation, TUseSurfaceThickness, TExecutionMode>(
				voxel, voxel_in_volume_coordinates, depth_camera_pose, depth_camera_projection_parameters, truncation_distance, depth_image,
				confidence, depth_image_size, effective_range_cutoff, surface_thickness, verbose);
	}
};
// with color, with confidence
template<bool THasSemanticInformation, bool TUseSurfaceThickness, class TVoxel, ExecutionMode TExecutionMode>
struct ComputeUpdatedLiveVoxelInfo<true, true, THasSemanticInformation, TUseSurfaceThickness, TVoxel, TExecutionMode> {
	_CPU_AND_GPU_CODE_ static void compute(COMPUTE_VOXEL_UPDATE_PARAMETERS) {
		float eta = FuseDepthIntoVoxel<THasSemanticInformation, TUseSurfaceThickness, TExecutionMode>(
				voxel, voxel_in_volume_coordinates, depth_camera_pose, depth_camera_projection_parameters, truncation_distance, depth_image,
				confidence, depth_image_size, effective_range_cutoff, surface_thickness, verbose);
		FuseColorIntoVoxel(voxel, voxel_in_volume_coordinates, rgb_camera_pose, rgb_camera_projection_parameters, truncation_distance, eta,
		                   rgb_image, rgb_image_size);
	}
};


#undef COMPUTE_VOXEL_UPDATE_PARAMETERS
// endregion ===========================================================================================================
// region ======================================== VOXEL UPDATE FUNCTOR ================================================

template<typename TVoxel, MemoryDeviceType TMemoryDeviceType, bool TStopIntegrationAtMaxIntegrationWeight,
		bool TUseSurfaceThickness, ExecutionMode TExecutionMode>
struct VoxelDepthIntegrationFunctor {
public:
	VoxelDepthIntegrationFunctor(
			const ITMLib::VoxelVolumeParameters& volume_parameters,
			const ITMLib::View* view, Matrix4f depth_camera_pose,
			float surface_thickness) :
			depth_image_size(view->depth.dimensions),
			depth_camera_projection_parameters(view->calibration_information.intrinsics_d.projectionParamsSimple.all),
			depth_camera_pose(depth_camera_pose),
			rgb_image_size(view->rgb.dimensions),
			rgb_camera_projection_parameters(view->calibration_information.intrinsics_rgb.projectionParamsSimple.all),
			rgb_camera_pose(
					TVoxel::hasColorInformation ? view->calibration_information.trafo_rgb_to_depth.calib_inv * depth_camera_pose
					                            : Matrix4f()),

			surface_thickness(surface_thickness),
			truncation_distance(volume_parameters.truncation_distance),
			max_integration_weight(volume_parameters.max_integration_weight),
			effective_range_cutoff(volume_parameters.truncation_distance * volume_parameters.block_allocation_band_factor),
			voxel_size(volume_parameters.voxel_size),

			depth(view->depth.GetData(TMemoryDeviceType)),
			rgb(view->rgb.GetData(TMemoryDeviceType)),
			confidence(view->depth_confidence.GetData(TMemoryDeviceType)),

			verbosity_level(configuration::Get().logging_settings.verbosity_level),
			focus_voxel(configuration::Get().focus_voxel) {}

	//TODO: after clangd EndlessLoop FP bug is solved , remove the diagnostic push/pop here
#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
	_CPU_AND_GPU_CODE_
	inline void operator()(TVoxel& voxel, const Vector3i& voxel_position) {

		if (TStopIntegrationAtMaxIntegrationWeight) if (voxel.w_depth == max_integration_weight) return;

		Vector4f voxel_volume_coordinates{static_cast<float>(voxel_position.x * voxel_size),
		                                  static_cast<float>(voxel_position.y * voxel_size),
		                                  static_cast<float>(voxel_position.z * voxel_size), 1.0f};
		bool verbose =
				TExecutionMode == DIAGNOSTIC && verbosity_level >= VerbosityLevel::VERBOSITY_FOCUS_SPOTS && focus_voxel == voxel_position;

		ComputeUpdatedLiveVoxelInfo<TVoxel::hasColorInformation, TVoxel::hasConfidenceInformation,
				TVoxel::hasSemanticInformation, TUseSurfaceThickness, TVoxel, TExecutionMode>::compute(
				voxel, voxel_volume_coordinates, depth_camera_pose,
				depth_camera_projection_parameters, rgb_camera_pose, rgb_camera_projection_parameters, truncation_distance,
				depth, confidence, depth_image_size, rgb, rgb_image_size, effective_range_cutoff, surface_thickness, verbose);
	}

#pragma clang diagnostic pop

private:
	const Vector2i depth_image_size;
	Vector4f depth_camera_projection_parameters;
	Matrix4f depth_camera_pose;
	Vector2i rgb_image_size;
	Vector4f rgb_camera_projection_parameters;
	Matrix4f rgb_camera_pose;

	const float surface_thickness;
	const float truncation_distance;
	const int max_integration_weight;
	const float voxel_size;
	const float effective_range_cutoff;

	const float* depth;
	const Vector4u* rgb;
	const float* confidence;

	const VerbosityLevel verbosity_level;
	const Vector3i focus_voxel;

};

// endregion
} // namespace ITMLib