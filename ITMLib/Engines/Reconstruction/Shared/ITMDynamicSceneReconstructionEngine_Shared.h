//  ================================================================
//  Created by Gregory Kramida on 5/25/18.
//  Copyright (c) 2018-2025 Gregory Kramida
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

#include "../../../Objects/Scene/ITMRepresentationAccess.h"
#include "../../../Objects/Scene/ITMTrilinearInterpolation.h"
#include "../../../Utils/ITMPixelUtils.h"
#include "../../../Utils/ITMVoxelFlags.h"


// region ============================== UPDATE SDF/COLOR IN VOXEL USING DEPTH PIXEL ===================================

template<class TVoxel>
_CPU_AND_GPU_CODE_ inline void updateSdfAndFlagsBasedOnDistanceSurfaceToVoxel(
		DEVICEPTR(TVoxel)& voxel, float signedDistanceSurfaceToVoxelAlongCameraRay, float narrowBandHalfWidth,
		float effectiveRangeCutoff) {
	if (signedDistanceSurfaceToVoxelAlongCameraRay < -narrowBandHalfWidth + 4e-07) {
		if (signedDistanceSurfaceToVoxelAlongCameraRay < -effectiveRangeCutoff) {
			//the voxel is beyond the narrow band, on the other side of the surface, but also really far away.
			//exclude from computation.
			voxel.sdf = TVoxel::floatToValue(-1.0);
			voxel.flags = ITMLib::VOXEL_UNKNOWN;
		} else {
			//the voxel is beyond the narrow band, on the other side of the surface. Set SDF to -1.0
			voxel.sdf = TVoxel::floatToValue(-1.0);
			voxel.flags = ITMLib::VOXEL_TRUNCATED;
		}
	} else if (signedDistanceSurfaceToVoxelAlongCameraRay > narrowBandHalfWidth - 4e-07) {
		if (signedDistanceSurfaceToVoxelAlongCameraRay > effectiveRangeCutoff) {
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
		voxel.sdf = TVoxel::floatToValue(signedDistanceSurfaceToVoxelAlongCameraRay / narrowBandHalfWidth);
		voxel.flags = ITMLib::VOXEL_NONTRUNCATED;
	}
}

/**
 * \brief Voxel update without confidence computation
 * \tparam TVoxel
 * \param voxel
 * \param voxelInSceneCoordinates
 * \param depthCameraSceneMatrix
 * \param depthCameraProjectionParameters
 * \param narrowBandHalfWidth
 * \param depthImage an array of float depths corresponding to the depth image
 * \param imageSize
 * \return -1 if voxel point is behind camera or depth value is invalid (0.0f),
 * distance between voxel point & measured surface depth along camera ray otherwise
 */
template<class TVoxel>
_CPU_AND_GPU_CODE_ inline float computeUpdatedLiveVoxelDepthInfo(
		DEVICEPTR(TVoxel)& voxel,
		const THREADPTR(Vector4f)& voxelInSceneCoordinates,
		const CONSTPTR(Matrix4f)& depthCameraSceneMatrix,
		const CONSTPTR(Vector4f)& depthCameraProjectionParameters,
		float narrowBandHalfWidth,
		const CONSTPTR(float)* depthImage,
		const CONSTPTR(Vector2i)& imageSize,
		float effectiveRangeCutoff = 0.08f) {

	// project point into image (voxel point in camera coordinates)
	Vector4f voxelPointInCameraCoordinates = depthCameraSceneMatrix * voxelInSceneCoordinates;
	// if point is behind the camera, don't modify any voxels
	if (voxelPointInCameraCoordinates.z <= 0) {
		return -1.0f;
	}

	Vector2f voxelPointProjectedToImage;
	voxelPointProjectedToImage.x = depthCameraProjectionParameters.x * voxelPointInCameraCoordinates.x
	                               / voxelPointInCameraCoordinates.z + depthCameraProjectionParameters.z;
	voxelPointProjectedToImage.y = depthCameraProjectionParameters.y * voxelPointInCameraCoordinates.y
	                               / voxelPointInCameraCoordinates.z + depthCameraProjectionParameters.w;

	// point falls outside of the image bounds
	if ((voxelPointProjectedToImage.x < 1) || (voxelPointProjectedToImage.x > imageSize.x - 2)
	    || (voxelPointProjectedToImage.y < 1) || (voxelPointProjectedToImage.y > imageSize.y - 2)) {
		return -1.0f;
	}

	// get measured depthImage from image
	float depthMeasure = depthImage[static_cast<int>(voxelPointProjectedToImage.x + 0.5f) +
	                                static_cast<int>(voxelPointProjectedToImage.y + 0.5f) * imageSize.x];

	// if depthImage is "invalid", return "unknown"
	if (depthMeasure <= 0.0f) {
		//keep voxel flags at ITMLib::VOXEL_UNKNOWN
		return -1.0f;
	}

	// signedDistanceSurfaceToVoxelAlongCameraRay (i.e. eta) =
	// [distance from surface to camera, i.e. depthImage] - [distance from voxel to camera]
	// effectively, eta is the distance between measured surface & voxel point
	float signedDistanceSurfaceToVoxelAlongCameraRay = depthMeasure - voxelPointInCameraCoordinates.z;
	updateSdfAndFlagsBasedOnDistanceSurfaceToVoxel(voxel, signedDistanceSurfaceToVoxelAlongCameraRay,
	                                               narrowBandHalfWidth, effectiveRangeCutoff);
	return signedDistanceSurfaceToVoxelAlongCameraRay;
}

/**
 * \brief Voxel update with confidence computation
 * \tparam TVoxel
 * \param voxel
 * \param voxelInSceneCoordinates
 * \param depthCameraSceneMatrix
 * \param depthCameraProjectionParameters
 * \param narrowBandHalfWidth
 * \param maxW
 * \param depthImage
 * \param confidencesAtPixels
 * \param imageSize
 * \return -1 if voxel point is behind camera or depth value is invalid (0.0f),
 * distance between voxel point & measured surface depth along camera ray otherwise
 */
template<class TVoxel>
_CPU_AND_GPU_CODE_ inline float computeUpdatedLiveVoxelDepthInfo(
		DEVICEPTR(TVoxel)& voxel,
		const THREADPTR(Vector4f)& voxelInSceneCoordinates,
		const CONSTPTR(Matrix4f)& depthCameraSceneMatrix,
		const CONSTPTR(Vector4f)& depthCameraProjectionParameters,
		float narrowBandHalfWidth,
		const CONSTPTR(float)* depthImage,
		const CONSTPTR(float)* confidencesAtPixels,
		const CONSTPTR(Vector2i)& imageSize,
		float effectiveRangeCutoff = 0.08f) {


	// project point into image
	Vector4f voxelPointInCameraCoordinates = depthCameraSceneMatrix * voxelInSceneCoordinates;
	// if the point is behind the camera, don't make any changes to SDF and return -1 to short-circuit further updates
	if (voxelPointInCameraCoordinates.z <= 0) return -1;

	Vector2f voxelPointProjectedToImage;
	voxelPointProjectedToImage.x =
			depthCameraProjectionParameters.x * voxelPointInCameraCoordinates.x /
			voxelPointInCameraCoordinates.z + depthCameraProjectionParameters.z;
	voxelPointProjectedToImage.y =
			depthCameraProjectionParameters.y * voxelPointInCameraCoordinates.y /
			voxelPointInCameraCoordinates.z + depthCameraProjectionParameters.w;
	if ((voxelPointProjectedToImage.x < 1) || (voxelPointProjectedToImage.x > imageSize.x - 2) ||
	    (voxelPointProjectedToImage.y < 1) || (voxelPointProjectedToImage.y > imageSize.y - 2))
		return -1;

	// point falls outside of the image bounds
	int pixelIndex =
			static_cast<int>(voxelPointProjectedToImage.x + 0.5f) +
			static_cast<int>(voxelPointProjectedToImage.y + 0.5f) * imageSize.x;

	// get measured depth from image
	float depthMeasure = depthImage[pixelIndex];
	if (depthMeasure <= 0.0) return -1;

	// signedSurfaceToVoxelAlongCameraRay (i.e. eta) =
	// [distance from surface to camera, i.e. depthImage] - [distance from voxel to camera]
	// effectively, eta is the distance between measured surface & voxel point
	float signedSurfaceToVoxelAlongCameraRay = depthMeasure - voxelPointInCameraCoordinates.z;
	voxel.confidence = TVoxel::floatToValue(confidencesAtPixels[pixelIndex]);

	updateSdfAndFlagsBasedOnDistanceSurfaceToVoxel(voxel, signedSurfaceToVoxelAlongCameraRay, narrowBandHalfWidth,
	                                               effectiveRangeCutoff);
	return signedSurfaceToVoxelAlongCameraRay;
}

template<class TVoxel>
_CPU_AND_GPU_CODE_ inline void computeUpdatedLiveVoxelColorInfo(
		DEVICEPTR(TVoxel)& voxel,
		const THREADPTR(Vector4f)& voxelInSceneCoordinates,
		const CONSTPTR(Matrix4f)& rgbCameraSceneMatrix,
		const CONSTPTR(Vector4f)& rgbCameraProjectionParameters,
		float narrowBandHalfWidth, float signedDistanceSurfaceToVoxel,
		const CONSTPTR(Vector4u)* rgbImage,
		const CONSTPTR(Vector2i)& imageSize) {
	Vector4f voxelInCameraCoordinates;
	Vector2f voxelPointProjectedToImage;

	voxelInCameraCoordinates = rgbCameraSceneMatrix * voxelInSceneCoordinates;

	voxelPointProjectedToImage.x =
			rgbCameraProjectionParameters.x * voxelInCameraCoordinates.x
			/ voxelInCameraCoordinates.z + rgbCameraProjectionParameters.z;
	voxelPointProjectedToImage.y =
			rgbCameraProjectionParameters.y * voxelInCameraCoordinates.y
			/ voxelInCameraCoordinates.z + rgbCameraProjectionParameters.w;
//TODO: the magic value 0.25f used to determine the cutoff distance for color processing should be pre-defined as a parameter -Greg (GitHub:Algomorph)
	//cut off voxels that are too far from the surface
	if (std::abs(signedDistanceSurfaceToVoxel) < 0.25f * narrowBandHalfWidth) return;

	if ((voxelPointProjectedToImage.x < 1) || (voxelPointProjectedToImage.x > imageSize.x - 2) ||
	    (voxelPointProjectedToImage.y < 1) || (voxelPointProjectedToImage.y > imageSize.y - 2))
		return;

	voxel.clr = TO_UCHAR3(TO_VECTOR3(interpolateBilinear(rgbImage, voxelPointProjectedToImage, imageSize)));
}
// endregion ===========================================================================================================

template<bool hasColor, bool hasConfidence, bool hasSemanticInformation, typename TVoxel>
struct ComputeUpdatedLiveVoxelInfo;

// region ========= VOXEL UPDATES FOR VOXELS WITH NO SEMANTIC INFORMATION ==============================================
//arguments to the "compute" member function should always be the same
#define COMPUTE_VOXEL_UPDATE_PARAMETERS \
DEVICEPTR(TVoxel) & voxel, const THREADPTR(Vector4f) & pt_model,\
const CONSTPTR(Matrix4f) & M_d, const CONSTPTR(Vector4f) & projParams_d,\
const CONSTPTR(Matrix4f) & M_rgb, const CONSTPTR(Vector4f) & projParams_rgb,\
float mu, int maxW,\
const CONSTPTR(float) *depth, const CONSTPTR(float) *confidence, const CONSTPTR(Vector2i) & imgSize_d,\
const CONSTPTR(Vector4u) *rgb, const CONSTPTR(Vector2i) & imgSize_rgb


// no color, no confidence, no semantic info
template<class TVoxel>
struct ComputeUpdatedLiveVoxelInfo<false, false, false, TVoxel> {
	_CPU_AND_GPU_CODE_ static void compute(COMPUTE_VOXEL_UPDATE_PARAMETERS) {
		computeUpdatedLiveVoxelDepthInfo(voxel, pt_model, M_d, projParams_d, mu, depth, imgSize_d);
	}
};
// with color, no confidence, no semantic info
template<class TVoxel>
struct ComputeUpdatedLiveVoxelInfo<true, false, false, TVoxel> {
	_CPU_AND_GPU_CODE_ static void compute(COMPUTE_VOXEL_UPDATE_PARAMETERS) {
		float eta = computeUpdatedLiveVoxelDepthInfo(voxel, pt_model, M_d, projParams_d, mu, depth, imgSize_d);
		computeUpdatedLiveVoxelColorInfo(voxel, pt_model, M_rgb, projParams_rgb, mu, eta, rgb, imgSize_rgb);
	}
};
// no color, with confidence, no semantic info
template<class TVoxel>
struct ComputeUpdatedLiveVoxelInfo<false, true, false, TVoxel> {
	_CPU_AND_GPU_CODE_ static void compute(COMPUTE_VOXEL_UPDATE_PARAMETERS) {
		computeUpdatedLiveVoxelDepthInfo(voxel, pt_model, M_d, projParams_d, mu, depth, confidence, imgSize_d);
	}
};
// with color, with confidence, no semantic info
template<class TVoxel>
struct ComputeUpdatedLiveVoxelInfo<true, true, false, TVoxel> {
	_CPU_AND_GPU_CODE_ static void compute(COMPUTE_VOXEL_UPDATE_PARAMETERS) {
		float eta = computeUpdatedLiveVoxelDepthInfo(voxel, pt_model, M_d, projParams_d, mu, depth, confidence,
		                                             imgSize_d);
		computeUpdatedLiveVoxelColorInfo(voxel, pt_model, M_rgb, projParams_rgb, mu, eta, rgb, imgSize_rgb);
	}
};
// endregion ===========================================================================================================
// region ========= VOXEL UPDATES FOR VOXELS WITH SEMANTIC INFORMATION =================================================

template<class TVoxel>
struct ComputeUpdatedLiveVoxelInfo<false, false, true, TVoxel> {
	_CPU_AND_GPU_CODE_ static void compute(COMPUTE_VOXEL_UPDATE_PARAMETERS) {
		float eta = computeUpdatedLiveVoxelDepthInfo(voxel, pt_model, M_d, projParams_d, mu, depth, imgSize_d);
	}
};
// with color, no confidence, with semantic info
template<class TVoxel>
struct ComputeUpdatedLiveVoxelInfo<true, false, true, TVoxel> {
	_CPU_AND_GPU_CODE_ static void compute(COMPUTE_VOXEL_UPDATE_PARAMETERS) {
		float eta = computeUpdatedLiveVoxelDepthInfo(voxel, pt_model, M_d, projParams_d, mu, depth, imgSize_d);
		computeUpdatedLiveVoxelColorInfo(voxel, pt_model, M_rgb, projParams_rgb, mu, eta, rgb, imgSize_rgb);
	}
};
// no color, with confidence, with semantic info
template<class TVoxel>
struct ComputeUpdatedLiveVoxelInfo<false, true, true, TVoxel> {
	_CPU_AND_GPU_CODE_ static void compute(COMPUTE_VOXEL_UPDATE_PARAMETERS) {
		float eta = computeUpdatedLiveVoxelDepthInfo(voxel, pt_model, M_d, projParams_d, mu, depth, confidence,
		                                             imgSize_d);
	}
};
// with color, with confidence, with semantic info
template<class TVoxel>
struct ComputeUpdatedLiveVoxelInfo<true, true, true, TVoxel> {
	_CPU_AND_GPU_CODE_ static void compute(COMPUTE_VOXEL_UPDATE_PARAMETERS) {
		float eta = computeUpdatedLiveVoxelDepthInfo(voxel, pt_model, M_d, projParams_d, mu, depth, confidence,
		                                             imgSize_d);
		computeUpdatedLiveVoxelColorInfo(voxel, pt_model, M_rgb, projParams_rgb, mu, eta, rgb, imgSize_rgb);
	}
};

#undef COMPUTE_VOXEL_UPDATE_PARAMETERS
// endregion ===========================================================================================================
// region ===================================== SDF2SDF FUSION =========================================================

template<typename TVoxelLive, typename TVoxelCanonical>
_CPU_AND_GPU_CODE_ inline void fuseLiveVoxelIntoCanonical(const DEVICEPTR(TVoxelLive)& liveVoxel, int maximumWeight,
                                                          DEVICEPTR(TVoxelCanonical)& canonicalVoxel) {
	//_DEBUG

	//fusion condition "HARSH" -- yields results almost identical to "COMBINED"
//		if(canonicalVoxel.flags != VOXEL_NONTRUNCATED
//				   && liveVoxel.flag_values[liveSourceFieldIndex] != VOXEL_NONTRUNCATED) return;

	//fusion condition "COMBINED"
	if (liveVoxel.flags == ITMLib::VoxelFlags::VOXEL_UNKNOWN
	    || (canonicalVoxel.flags != ITMLib::VoxelFlags::VOXEL_NONTRUNCATED
	        && liveVoxel.flags != ITMLib::VoxelFlags::VOXEL_NONTRUNCATED))
		return;

	float liveSdf = TVoxelLive::valueToFloat(liveVoxel.sdf);

	// parameter eta from SobolevFusion, Sec. 3.1, divided by voxel size
	//(voxel size, m) / (narrow-band half-width eta, m) * -("2-3 voxels", we use 3)
	const float threshold = -0.3;

	//fusion condition "THRESHOLD"
	if (liveVoxel.flags == ITMLib::VoxelFlags::VOXEL_UNKNOWN
	    || (canonicalVoxel.flags != ITMLib::VoxelFlags::VOXEL_NONTRUNCATED
	        && liveVoxel.flags != ITMLib::VoxelFlags::VOXEL_NONTRUNCATED) || liveSdf < threshold)
		return;

	//fusion condition "LIVE_UNKNOWN"
//		if(liveVoxel.flags == VOXEL_UNKNOWN) return;



	int oldWDepth = canonicalVoxel.w_depth;
	float oldSdf = TVoxelCanonical::valueToFloat(canonicalVoxel.sdf);

	float newSdf = oldWDepth * oldSdf + liveSdf;
	float newWDepth = oldWDepth + 1.0f;
	newSdf /= newWDepth;
	newWDepth = ORUTILS_MIN(newWDepth, maximumWeight);

	canonicalVoxel.sdf = TVoxelCanonical::floatToValue(newSdf);
	canonicalVoxel.w_depth = (uchar) newWDepth;
	if (canonicalVoxel.flags != ITMLib::VoxelFlags::VOXEL_NONTRUNCATED) {
		canonicalVoxel.flags = liveVoxel.flags;
	} else if (1.0f - std::abs(newSdf) < 1e-5f) {
		canonicalVoxel.flags = ITMLib::VoxelFlags::VOXEL_TRUNCATED;
	}
};

// endregion ===========================================================================================================
