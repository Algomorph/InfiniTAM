// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../../Objects/Scene/RepresentationAccess.h"
#include "../../../Utils/ITMPixelUtils.h"


/**
 * \brief Voxel update without confidence computation
 * \tparam TVoxel
 * \param voxel
 * \param pt_model
 * \param M_d
 * \param projParams_d
 * \param mu
 * \param maxW
 * \param depth
 * \param imgSize
 * \return -1 if voxel point is behind camera or depth value is invalid (0.0f),
 * distance between voxel point & measured surface depth along camera ray otherwise
 */
template<typename TVoxel>
_CPU_AND_GPU_CODE_ inline float computeUpdatedVoxelDepthInfo(
		DEVICEPTR(TVoxel)& voxel,
		const THREADPTR(Vector4f)& pt_model,
		const CONSTPTR(Matrix4f)& M_d,
		const CONSTPTR(Vector4f)& projParams_d,
		float mu, int maxW,
		const CONSTPTR(float)* depth,
		const CONSTPTR(Vector2i)& imgSize) {
	Vector4f pt_camera;
	Vector2f pt_image;
	float depth_measure, eta, oldF, newF;
	int oldW, newW;

	// project point into image (voxel point in camera coordinates)
	pt_camera = M_d * pt_model;
	// if point is behind the camera, don't modify any voxels
	if (pt_camera.z <= 0) return -1;

	pt_image.x = projParams_d.x * pt_camera.x / pt_camera.z + projParams_d.z;
	pt_image.y = projParams_d.y * pt_camera.y / pt_camera.z + projParams_d.w;
	if ((pt_image.x < 1) || (pt_image.x > imgSize.x - 2) || (pt_image.y < 1) || (pt_image.y > imgSize.y - 2)) return -1;

	// get measured depth from image
	depth_measure = depth[(int) (pt_image.x + 0.5f) + (int) (pt_image.y + 0.5f) * imgSize.x];
	if (depth_measure <= 0.0f) return -1;

	// check whether voxel needs updating
	// eta = [depth at pixel corresp. to current ray] - [depth of voxel point along the ray]
	// effectively, eta is the distance between measured surface & voxel point
	eta = depth_measure - pt_camera.z;

	//the voxel is beyond the narrow band, on the other side of the surface. Don't make any updates to SDF.
	if (eta < -mu) return eta;

	// compute updated SDF value and reliability
	oldF = TVoxel::valueToFloat(voxel.sdf);
	oldW = voxel.w_depth;

	newF = ORUTILS_MIN(1.0f, eta / mu);
	newW = 1;

	newF = oldW * oldF + newW * newF;
	newW = oldW + newW;
	newF /= newW;
	newW = ORUTILS_MIN(newW, maxW);

	// write back
	voxel.sdf = TVoxel::floatToValue(newF);
	voxel.w_depth = newW;

	return eta;
}

/**
 * \brief Voxel update with confidence computation
 * \tparam TVoxel
 * \param voxel
 * \param pt_model
 * \param M_d
 * \param projParams_d
 * \param mu
 * \param maxW
 * \param depth
 * \param confidence
 * \param imgSize
 * \return -1 if voxel point is behind camera or depth value is invalid (0.0f),
 * distance between voxel point & measured surface depth along camera ray otherwise
 */
template<class TVoxel>
_CPU_AND_GPU_CODE_ inline float computeUpdatedVoxelDepthInfo(
		DEVICEPTR(TVoxel)& voxel,
		const THREADPTR(Vector4f)& pt_model,
		const CONSTPTR(Matrix4f)& M_d,
		const CONSTPTR(Vector4f)& projParams_d,
		float mu, int maxW,
		const CONSTPTR(float)* depth,
		const CONSTPTR(float)* confidence,
		const CONSTPTR(Vector2i)& imgSize) {
	Vector4f pt_camera;
	Vector2f pt_image;
	float depth_measure, eta, oldF, newF;
	int oldW, newW, locId;

	// project point into image
	pt_camera = M_d * pt_model;
	// if the point is behind the camera, don't make any changes to SDF and return -1 to short-circuit further updates
	if (pt_camera.z <= 0) return -1;

	pt_image.x = projParams_d.x * pt_camera.x / pt_camera.z + projParams_d.z;
	pt_image.y = projParams_d.y * pt_camera.y / pt_camera.z + projParams_d.w;
	if ((pt_image.x < 1) || (pt_image.x > imgSize.x - 2) || (pt_image.y < 1) || (pt_image.y > imgSize.y - 2)) return -1;

	locId = (int) (pt_image.x + 0.5f) + (int) (pt_image.y + 0.5f) * imgSize.x;
	// get measured depth from image
	depth_measure = depth[locId];
	if (depth_measure <= 0.0) return -1;

	// check whether voxel needs updating
	eta = depth_measure - pt_camera.z;

	//the voxel is beyond the narrow band, on the other side of the surface. Don't make any updates to SDF.
	if (eta < -mu) return eta;

	// compute updated SDF value and reliability
	oldF = TVoxel::valueToFloat(voxel.sdf);
	oldW = voxel.w_depth;
	newF = ORUTILS_MIN(1.0f, eta / mu);
	newW = 1;

	newF = oldW * oldF + newW * newF;
	newW = oldW + newW;
	newF /= newW;
	newW = ORUTILS_MIN(newW, maxW);

	// write back^
	voxel.sdf = TVoxel::floatToValue(newF);
	voxel.w_depth = newW;
	voxel.confidence += TVoxel::floatToValue(confidence[locId]);

	return eta;
}

template<class TVoxel>
_CPU_AND_GPU_CODE_ inline void computeUpdatedVoxelColorInfo(
		DEVICEPTR(TVoxel)& voxel,
		const THREADPTR(Vector4f)& pt_model,
		const CONSTPTR(Matrix4f)& M_rgb,
		const CONSTPTR(Vector4f)& projParams_rgb,
		float mu, uchar maxW, float eta,
		const CONSTPTR(Vector4u)* rgb,
		const CONSTPTR(Vector2i)& imgSize) {
	Vector4f pt_camera;
	Vector2f pt_image;
	Vector3f rgb_measure, oldC, newC;
	Vector3u buffV3u;
	float newW, oldW;

	buffV3u = voxel.clr;
	oldW = (float) voxel.w_color;

	oldC = TO_FLOAT3(buffV3u) / 255.0f;
	newC = oldC;

	pt_camera = M_rgb * pt_model;

	pt_image.x = projParams_rgb.x * pt_camera.x / pt_camera.z + projParams_rgb.z;
	pt_image.y = projParams_rgb.y * pt_camera.y / pt_camera.z + projParams_rgb.w;

	if ((pt_image.x < 1) || (pt_image.x > imgSize.x - 2) || (pt_image.y < 1) || (pt_image.y > imgSize.y - 2)) return;

	rgb_measure = TO_VECTOR3(interpolateBilinear(rgb, pt_image, imgSize)) / 255.0f;
	//rgb_measure = rgb[(int)(pt_image.x + 0.5f) + (int)(pt_image.y + 0.5f) * imgSize.x].toVector3().toFloat() / 255.0f;
	newW = 1;

	newC = oldC * oldW + rgb_measure * newW;
	newW = oldW + newW;
	newC /= newW;
	newW = ORUTILS_MIN(newW, maxW);

	voxel.clr = TO_UCHAR3(newC * 255.0f);
	voxel.w_color = (uchar) newW;
}

template<bool hasColor, bool hasConfidence, bool hasSemanticInformation, class TVoxel>
struct ComputeUpdatedVoxelInfo;

//================= VOXEL UPDATES FOR VOXELS WITH NO SEMANTIC INFORMATION ==============================================
//arguments to the "compute" member function should always be the same
#define COMPUTE_VOXEL_UPDATE_PARAMETERS \
DEVICEPTR(TVoxel) & voxel, const THREADPTR(Vector4f) & pt_model,\
const CONSTPTR(Matrix4f) & M_d, const CONSTPTR(Vector4f) & projParams_d,\
const CONSTPTR(Matrix4f) & M_rgb, const CONSTPTR(Vector4f) & projParams_rgb,\
float mu, int maxW,\
const CONSTPTR(float) *depth, const CONSTPTR(float) *confidence, const CONSTPTR(Vector2i) & imgSize_d,\
const CONSTPTR(Vector4u) *rgb, const CONSTPTR(Vector2i) & imgSize_rgb
//TODO: the magic value 0.25f used to determine the cutoff distance for color processing should be pre-defined either as a constant or a preprocessor define -Greg (GitHub:Algomorph)
#define COMPUTE_COLOR_CHECK if ((eta > mu) || (fabs(eta / mu) > 0.25f)) return;

// no color, no confidence, no semantic info
template<class TVoxel>
struct ComputeUpdatedVoxelInfo<false, false, false, TVoxel> {
	_CPU_AND_GPU_CODE_ static void compute(COMPUTE_VOXEL_UPDATE_PARAMETERS) {
		computeUpdatedVoxelDepthInfo(voxel, pt_model, M_d, projParams_d, mu, maxW, depth, imgSize_d);
	}
};
// with color, no confidence, no semantic info
template<class TVoxel>
struct ComputeUpdatedVoxelInfo<true, false, false, TVoxel> {
	_CPU_AND_GPU_CODE_ static void compute(COMPUTE_VOXEL_UPDATE_PARAMETERS) {
		float eta = computeUpdatedVoxelDepthInfo(voxel, pt_model, M_d, projParams_d, mu, maxW, depth, imgSize_d);
		computeUpdatedVoxelColorInfo(voxel, pt_model, M_rgb, projParams_rgb, mu, maxW, eta, rgb, imgSize_rgb);
	}
};
// no color, with confidence, no semantic info
template<class TVoxel>
struct ComputeUpdatedVoxelInfo<false, true, false, TVoxel> {
	_CPU_AND_GPU_CODE_ static void compute(COMPUTE_VOXEL_UPDATE_PARAMETERS) {
		computeUpdatedVoxelDepthInfo(voxel, pt_model, M_d, projParams_d, mu, maxW, depth, confidence, imgSize_d);
	}
};
// with color, with confidence, no semantic info
template<class TVoxel>
struct ComputeUpdatedVoxelInfo<true, true, false, TVoxel> {
	_CPU_AND_GPU_CODE_ static void compute(COMPUTE_VOXEL_UPDATE_PARAMETERS) {
		float eta = computeUpdatedVoxelDepthInfo(voxel, pt_model, M_d, projParams_d, mu, maxW, depth, confidence,
		                                         imgSize_d);
		computeUpdatedVoxelColorInfo(voxel, pt_model, M_rgb, projParams_rgb, mu, maxW, eta, rgb, imgSize_rgb);
	}
};
//================= VOXEL UPDATES FOR VOXELS WITH SEMANTIC INFORMATION =================================================
// no color, no confidence, with semantic info
template<class TVoxel>
struct ComputeUpdatedVoxelInfo<false, false, true, TVoxel> {
	_CPU_AND_GPU_CODE_ static void compute(COMPUTE_VOXEL_UPDATE_PARAMETERS) {
		float eta = computeUpdatedVoxelDepthInfo(voxel, pt_model, M_d, projParams_d, mu, maxW, depth, imgSize_d);
	}
};
// with color, no confidence, with semantic info
template<class TVoxel>
struct ComputeUpdatedVoxelInfo<true, false, true, TVoxel> {
	_CPU_AND_GPU_CODE_ static void compute(COMPUTE_VOXEL_UPDATE_PARAMETERS) {
		float eta = computeUpdatedVoxelDepthInfo(voxel, pt_model, M_d, projParams_d, mu, maxW, depth, imgSize_d);
		computeUpdatedVoxelColorInfo(voxel, pt_model, M_rgb, projParams_rgb, mu, maxW, eta, rgb, imgSize_rgb);
	}
};
// no color, with confidence, with semantic info
template<class TVoxel>
struct ComputeUpdatedVoxelInfo<false, true, true, TVoxel> {
	_CPU_AND_GPU_CODE_ static void compute(COMPUTE_VOXEL_UPDATE_PARAMETERS) {
		float eta = computeUpdatedVoxelDepthInfo(voxel, pt_model, M_d, projParams_d, mu, maxW, depth, confidence,
		                                         imgSize_d);
	}
};
// with color, with confidence, with semantic info
template<class TVoxel>
struct ComputeUpdatedVoxelInfo<true, true, true, TVoxel> {
	_CPU_AND_GPU_CODE_ static void compute(COMPUTE_VOXEL_UPDATE_PARAMETERS) {
		float eta = computeUpdatedVoxelDepthInfo(voxel, pt_model, M_d, projParams_d, mu, maxW, depth, confidence,
		                                         imgSize_d);
		computeUpdatedVoxelColorInfo(voxel, pt_model, M_rgb, projParams_rgb, mu, maxW, eta, rgb, imgSize_rgb);
	}
};

#undef COMPUTE_COLOR_CHECK
#undef FLAG_UPDATE_CHECK
#undef COMPUTE_VOXEL_UPDATE_PARAMETERS
//======================================================================================================================

