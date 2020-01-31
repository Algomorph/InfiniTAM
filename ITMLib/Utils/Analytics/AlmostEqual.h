//  ================================================================
//  Created by Gregory Kramida on 8/28/19.
//  Copyright (c) 2019 Gregory Kramida
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
//stdlib
#include <cmath>
#include <string>
#include <limits>

//local
#include "VoxelVolumeComparison/VoxelVolumeComparison_CPU.h"
#include "../../Objects/Volume/VoxelTypes.h"
#include "../Math.h"
#include "../../../ORUtils/PlatformIndependence.h"

//TODO: can this be possibly shortened by using overloads instead of templates for tolerance type?
// e.g. functions of the pattern almostEqual<SomeVoxelType,unsigned int> all have equivalent code...

namespace ITMLib {

//  region ==================== DECLARATIONS ===========================================================================

_CPU_AND_GPU_CODE_ inline bool almostEqual(float a, float b);
_CPU_AND_GPU_CODE_ inline bool almostEqual(double a, double b);
/**
 * \brief Determine whether the two values are within a given tolerance of each-other
 * \details The comparison is done in an absolute way, i.e. the relative value magnitudes don't matter. This is useful
 * for situations where there is a predetermined upper bound on the values, i.e. values are in range [0.0,1.0], and
 * small values don't really matter all that much.
 * \param a the first value
 * \param b the second value
 * \return true if the two values are within the provided tolerance, false otherwise.
 */
_CPU_AND_GPU_CODE_ inline bool almostEqual(float a, float b, float tolerance);
_CPU_AND_GPU_CODE_ inline bool almostEqual(float a, float b, double tolerance);
_CPU_AND_GPU_CODE_ inline bool almostEqual(double a, double b, double tolerance);
_CPU_AND_GPU_CODE_ inline bool almostEqual(float a, float b, unsigned int decimal_places);


template<typename ElementType, typename ToleranceType>
_CPU_AND_GPU_CODE_
inline
bool almostEqual(ORUtils::Vector2<ElementType> a, ORUtils::Vector2<ElementType> b, ToleranceType tolerance);

template<typename ElementType, typename ToleranceType>
_CPU_AND_GPU_CODE_
inline
bool almostEqual(ORUtils::Vector3<ElementType> a, ORUtils::Vector3<ElementType> b, ToleranceType tolerance);

template<typename ElementType, typename ToleranceType>
_CPU_AND_GPU_CODE_
inline
bool almostEqual(ORUtils::Matrix3<ElementType> a, ORUtils::Matrix3<ElementType> b, ToleranceType tolerance);

template<typename ElementType, typename ToleranceType>
_CPU_AND_GPU_CODE_
inline
bool almostEqual(ORUtils::Matrix4<ElementType> a, ORUtils::Matrix4<ElementType> b, ToleranceType tolerance);

template<typename TVoxel, typename ToleranceType>
_CPU_AND_GPU_CODE_
inline
bool almostEqual(const TVoxel& a, const TVoxel& b, ToleranceType tolerance);

template<typename TVoxel, typename ToleranceType>
_CPU_AND_GPU_CODE_
inline
bool almostEqualVerbose(const TVoxel& a, const TVoxel& b, ToleranceType tolerance);

template<typename TVoxel, typename ToleranceType>
_CPU_AND_GPU_CODE_
inline
bool almostEqualVerbose_Position(const TVoxel& a, const TVoxel& b, const Vector3i& position, ToleranceType tolerance);

// endregion

// region ==================== ABSOLUTE / RELATIVE REAL TYPE COMPARISONS ===============================================
template<typename TReal>
_CPU_AND_GPU_CODE_
inline
bool almostEqualRelative(TReal a, TReal b, TReal epsilon = 3e-6) {
	const TReal absA = std::abs(a);
	const TReal absB = std::abs(b);
	const TReal diff = std::abs(a - b);

	if (a == b) { // shortcut, handles infinities
		return true;
	} else if (a == 0 || b == 0 || diff < std::numeric_limits<TReal>::denorm_min()) {
		// a or b is zero or both are extremely close to it
		// relative error is less meaningful here
		return diff < (epsilon * std::numeric_limits<TReal>::denorm_min());
	} else { // use relative error
		return diff / std::min((absA + absB), std::numeric_limits<TReal>::max()) < epsilon;
	}
}

template<typename TReal>
_CPU_AND_GPU_CODE_
inline
bool almostEqualAbsolute(TReal a, TReal b, TReal epsilon = 3e-6) {
	return std::abs(a - b) < epsilon;
}
// endregion

// region ==================== SPECIFIC REAL TYPE COMPARISONS ==========================================================

_CPU_AND_GPU_CODE_
inline
bool almostEqual(float a, float b) {
	return almostEqualAbsolute(a, b);
}

_CPU_AND_GPU_CODE_
inline
bool almostEqual(double a, double b) {
	return almostEqualAbsolute(a, b);
}

_CPU_AND_GPU_CODE_
inline
bool almostEqual(float a, float b, float tolerance) {
	return almostEqualAbsolute(a, b, tolerance);
}

_CPU_AND_GPU_CODE_
inline
bool almostEqual(float a, float b, double tolerance) {
	return almostEqualAbsolute(a, b, static_cast<float>(tolerance));
}

_CPU_AND_GPU_CODE_
inline
bool almostEqual(double a, double b, double tolerance) {
	return almostEqualAbsolute(a, b, tolerance);
}

_CPU_AND_GPU_CODE_
inline
bool almostEqual(float a, float b, unsigned int decimal_places) {
	const float tolerance = 1.0f / (10.0f * static_cast<float>(decimal_places));
	return almostEqualAbsolute(a, b, tolerance);
}

//endregion

//region ==================== SPECIFIC REAL COLLECTION COMPARISONS =====================================================
// *********** float tolerance type ***********
template<>
_CPU_AND_GPU_CODE_
inline
bool almostEqual<float, float>(Vector2f a, Vector2f b, float tolerance) {
	return almostEqualAbsolute(a.x, b.x, tolerance) && almostEqualAbsolute(a.y, b.y, tolerance);
}

template<>
_CPU_AND_GPU_CODE_
inline
bool almostEqual<float, unsigned int>(Vector2f a, Vector2f b, unsigned int tolerance) {
	const float tolerance_float = 1.0f / (10.0f * static_cast<float>(tolerance));
	return almostEqualAbsolute(a.x, b.x, tolerance_float) && almostEqualAbsolute(a.y, b.y, tolerance_float);
}

template<>
_CPU_AND_GPU_CODE_
inline
bool almostEqual<float, float>(Vector3f a, Vector3f b, float tolerance) {
	return almostEqualAbsolute(a.x, b.x, tolerance)
	       && almostEqualAbsolute(a.y, b.y, tolerance)
	       && almostEqualAbsolute(a.z, b.z, tolerance);
}

//color comparison
template<>
_CPU_AND_GPU_CODE_
inline
bool almostEqual<uchar, float>(Vector3u a, Vector3u b, float tolerance) {
	auto a_float_normalized = TO_FLOAT3(a) / 255.0f;
	auto b_float_normalized = TO_FLOAT3(b) / 255.0f;
	return almostEqual<float, float>(a_float_normalized, b_float_normalized, tolerance);
}

template<>
_CPU_AND_GPU_CODE_
inline
bool almostEqual<float, float>(Matrix3f a, Matrix3f b, float tolerance) {
	for (int i_entry = 0; i_entry < 9; i_entry++) {
		if (!almostEqualAbsolute(a.m[i_entry], b.m[i_entry], tolerance)) return false;
	}
	return true;
}

template<>
_CPU_AND_GPU_CODE_
inline
bool almostEqual<float, float>(Matrix4f a, Matrix4f b, float tolerance) {
	for (int i_entry = 0; i_entry < 16; i_entry++) {
		if (!almostEqualAbsolute(a.m[i_entry], b.m[i_entry], tolerance)) return false;
	}
	return true;
}

// *********** double tolerance type ***********
template<>
_CPU_AND_GPU_CODE_
inline
bool almostEqual<float, double>(Vector2f a, Vector2f b, double tolerance) {
	auto tolerance_float = static_cast<float>(tolerance);
	return almostEqualAbsolute(a.x, b.x, tolerance_float) && almostEqualAbsolute(a.y, b.y, tolerance_float);
}

template<>
_CPU_AND_GPU_CODE_
inline
bool almostEqual<float, double>(Vector3f a, Vector3f b, double tolerance) {
	auto tolerance_float = static_cast<float>(tolerance);
	return almostEqualAbsolute(a.x, b.x, tolerance_float)
	       && almostEqualAbsolute(a.y, b.y, tolerance_float)
	       && almostEqualAbsolute(a.z, b.z, tolerance_float);
}

//color comparison
template<>
_CPU_AND_GPU_CODE_
inline
bool almostEqual<uchar, double>(Vector3u a, Vector3u b, double tolerance) {
	auto tolerance_float = static_cast<float>(tolerance);
	auto a_float_normalized = TO_FLOAT3(a) / 255.0f;
	auto b_float_normalized = TO_FLOAT3(b) / 255.0f;
	return almostEqual<float, float>(a_float_normalized, b_float_normalized, tolerance_float);
}

template<>
_CPU_AND_GPU_CODE_
inline
bool almostEqual<float, double>(Matrix3f a, Matrix3f b, double tolerance) {
	auto tolerance_float = static_cast<float>(tolerance);
	for (int i_entry = 0; i_entry < 9; i_entry++) {
		if (!almostEqualAbsolute(a.m[i_entry], b.m[i_entry], tolerance_float)) return false;
	}
	return true;
}

template<>
_CPU_AND_GPU_CODE_
inline
bool almostEqual<float, double>(Matrix4f a, Matrix4f b, double tolerance) {
	auto tolerance_float = static_cast<float>(tolerance);
	for (int i_entry = 0; i_entry < 9; i_entry++) {
		if (!almostEqualAbsolute(a.m[i_entry], b.m[i_entry], tolerance_float)) return false;
	}
	return true;
}

// *********** unsigned int tolerance type (decimal places) ***********
template<>
_CPU_AND_GPU_CODE_
inline
bool almostEqual<float, unsigned int>(Vector3f a, Vector3f b, unsigned int tolerance) {
	const float tolerance_float = 1.0f / (10.0f * static_cast<float>(tolerance));
	return almostEqualAbsolute(a.x, b.x, tolerance_float)
	       && almostEqualAbsolute(a.y, b.y, tolerance_float)
	       && almostEqualAbsolute(a.z, b.z, tolerance_float);
}

//color comparison
template<>
_CPU_AND_GPU_CODE_
inline
bool almostEqual<uchar, unsigned int>(Vector3u a, Vector3u b, unsigned int tolerance) {
	auto a_float_normalized = TO_FLOAT3(a) / 255.0f;
	auto b_float_normalized = TO_FLOAT3(b) / 255.0f;
	return almostEqual<float, unsigned int>(a_float_normalized, b_float_normalized, tolerance);
}

template<>
_CPU_AND_GPU_CODE_
inline
bool almostEqual<float, unsigned int>(Matrix3f a, Matrix3f b, unsigned int tolerance) {
	const float tolerance_float = 1.0f / (10.0f * static_cast<float>(tolerance));
	for (int i_entry = 0; i_entry < 9; i_entry++) {
		if (!almostEqualAbsolute(a.m[i_entry], b.m[i_entry], tolerance_float)) return false;
	}
	return true;
}


template<>
_CPU_AND_GPU_CODE_
inline
bool almostEqual<float, unsigned int>(Matrix4f a, Matrix4f b, unsigned int tolerance) {
	const float tolerance_float = 1.0f / (10.0f * static_cast<float>(tolerance));
	for (int i_entry = 0; i_entry < 16; i_entry++) {
		if (!almostEqualAbsolute(a.m[i_entry], b.m[i_entry], tolerance_float)) return false;
	}
	return true;
}

//endregion

// region ==================== VOXEL COMPARISONS =======================================================================
// *********** float tolerance type ***********
template<>
_CPU_AND_GPU_CODE_
inline
bool almostEqual<TSDFVoxel_f, float>(const TSDFVoxel_f& a, const TSDFVoxel_f& b, float tolerance) {
	return almostEqual(a.sdf, b.sdf, tolerance) &&
	       a.w_depth == b.w_depth;
}

template<>
_CPU_AND_GPU_CODE_
inline
bool almostEqual<TSDFVoxel_s, float>(const TSDFVoxel_s& a, const TSDFVoxel_s& b, float tolerance) {
	return almostEqual(TSDFVoxel_s_rgb::valueToFloat(a.sdf), TSDFVoxel_s_rgb::valueToFloat(b.sdf), tolerance) &&
	       a.w_depth == b.w_depth;
}

template<>
_CPU_AND_GPU_CODE_
inline
bool almostEqual<TSDFVoxel_f_rgb, float>(const TSDFVoxel_f_rgb& a, const TSDFVoxel_f_rgb& b, float tolerance) {
	return almostEqual(a.sdf, b.sdf, tolerance) &&
	       a.w_depth == b.w_depth &&
	       almostEqual(a.clr, b.clr, tolerance) &&
	       a.w_color == b.w_color;
}

template<>
_CPU_AND_GPU_CODE_
inline
bool almostEqual<TSDFVoxel_s_rgb, float>(const TSDFVoxel_s_rgb& a, const TSDFVoxel_s_rgb& b, float tolerance) {
	return almostEqual(TSDFVoxel_s_rgb::valueToFloat(a.sdf), TSDFVoxel_s_rgb::valueToFloat(b.sdf), tolerance) &&
	       a.w_depth == b.w_depth &&
	       almostEqual(a.clr, b.clr, tolerance) &&
	       a.w_color == b.w_color;
}

template<>
_CPU_AND_GPU_CODE_
inline
bool almostEqual<TSDFVoxel_f_conf, float>(const TSDFVoxel_f_conf& a, const TSDFVoxel_f_conf& b, float tolerance) {
	return almostEqual(a.sdf, b.sdf, tolerance) &&
	       a.w_depth == b.w_depth &&
	       almostEqual(a.confidence, b.confidence, tolerance);
}

template<>
_CPU_AND_GPU_CODE_
inline
bool
almostEqual<TSDFVoxel_s_rgb_conf, float>(const TSDFVoxel_s_rgb_conf& a, const TSDFVoxel_s_rgb_conf& b, float tolerance) {
	return almostEqual(TSDFVoxel_s_rgb::valueToFloat(a.sdf), TSDFVoxel_s_rgb::valueToFloat(b.sdf), tolerance) &&
	       a.w_depth == b.w_depth &&
	       a.clr == b.clr &&
	       almostEqual(a.clr, b.clr, tolerance) &&
	       almostEqual(a.confidence, b.confidence, tolerance);
}

template<>
_CPU_AND_GPU_CODE_
inline
bool almostEqual<WarpVoxel_f_uf, float>(const WarpVoxel_f_uf& a, const WarpVoxel_f_uf& b, float tolerance) {
	return almostEqual(a.framewise_warp, b.framewise_warp, tolerance)
	       && almostEqual(a.gradient0, b.gradient0, tolerance)
	       && almostEqual(a.gradient1, b.gradient1, tolerance)
	       ;
}

_CPU_AND_GPU_CODE_
inline
void getNonMatchingComponents(bool& xMismatch, bool& yMismatch, bool& zMismatch, const Vector3f& a, const Vector3f& b, float tolerance){
	xMismatch = !almostEqual(a.x, b.x, tolerance);
	yMismatch = !almostEqual(a.y, b.y, tolerance);
	zMismatch = !almostEqual(a.z, b.z, tolerance);
}

_CPU_AND_GPU_CODE_
inline
void printVector3fVoxelError(const Vector3f& a, const Vector3f& b, float tolerance, const char* description){
	bool xMismatch, yMismatch, zMismatch;
	getNonMatchingComponents(xMismatch,yMismatch,zMismatch, a, b, tolerance);
	printf("(Showing first error only) %s not within %E: (%E, %E, %E) vs (%E, %E, %E)\n", description, tolerance,
			a.x, a.y, a.z, b.x, b.y, b.z);
}

_CPU_AND_GPU_CODE_
inline
void printVector3fVoxelError_Position(const Vector3f& a, const Vector3f& b, float tolerance, const char* description, const Vector3i& position){
	bool xMismatch, yMismatch, zMismatch;
	getNonMatchingComponents(xMismatch,yMismatch,zMismatch, a, b, tolerance);
	printf("Position %d, %d, %d:(Showing first error only) %s not within %E: (%E, %E, %E) vs (%E, %E, %E)\n",
			position.x, position.y, position.z, description, tolerance,
	       a.x, a.y, a.z, b.x, b.y, b.z);
}

template<>
_CPU_AND_GPU_CODE_
inline
bool almostEqualVerbose<WarpVoxel_f_uf, float>(const WarpVoxel_f_uf& a, const WarpVoxel_f_uf& b, float tolerance) {
	if(!almostEqual(a.framewise_warp, b.framewise_warp, tolerance)){
		printVector3fVoxelError(a.framewise_warp,b.framewise_warp,tolerance,"framewise_warp");
		return false;
	}
	if(!almostEqual(a.gradient0, b.gradient0, tolerance)){
		printVector3fVoxelError(a.gradient0,b.gradient0,tolerance,"gradient0");
		return false;
	}
	if(!almostEqual(a.gradient1, b.gradient1, tolerance)){
		printVector3fVoxelError(a.gradient1,b.gradient1,tolerance,"gradient1");
		return false;
	}
	return true;
}


template<>
_CPU_AND_GPU_CODE_
inline
bool almostEqualVerbose_Position<WarpVoxel_f_uf, float>(const WarpVoxel_f_uf& a, const WarpVoxel_f_uf& b, const Vector3i& position, float tolerance) {

	if(!almostEqual(a.framewise_warp, b.framewise_warp, tolerance)){
		printVector3fVoxelError_Position(a.framewise_warp,b.framewise_warp,tolerance,"framewise_warp", position);
		return false;
	}
	if(!almostEqual(a.gradient0, b.gradient0, tolerance)){
		printVector3fVoxelError_Position(a.gradient0,b.gradient0,tolerance,"gradient0", position);
		return false;
	}
	if(!almostEqual(a.gradient1, b.gradient1, tolerance)){
		printVector3fVoxelError_Position(a.gradient1,b.gradient1,tolerance,"gradient1", position);
		return false;
	}
	return true;
}

template<>
_CPU_AND_GPU_CODE_
inline
bool almostEqual<TSDFVoxel_f_flags, float>(const TSDFVoxel_f_flags& a, const TSDFVoxel_f_flags& b, float tolerance) {
	return almostEqual(a.sdf, b.sdf, tolerance) &&
	       a.w_depth == b.w_depth &&
	       a.flags == b.flags;
}


template<>
_CPU_AND_GPU_CODE_
inline
bool almostEqualVerbose_Position<TSDFVoxel_f_flags, float>(const TSDFVoxel_f_flags& a, const TSDFVoxel_f_flags& b, const Vector3i& position, float tolerance) {

	if(!almostEqual(a.sdf, b.sdf, tolerance)){
		printf("Position %d, %d, %d: mismatch between voxel:{sdf: %E, w_depth: %d, flags: %d} and voxel:{sdf: %E, w_depth: %d, flags: %d}. SDF not within tolerance %E.\n",
		       position.x, position.y, position.z, a.sdf, a.w_depth, a.flags, b.sdf, b.w_depth, b.flags, tolerance);
		return false;
	}
	if(a.w_depth != b.w_depth){
		printf("Position %d, %d, %d: mismatch between voxel:{sdf: %E, w_depth: %d, flags: %d} and voxel:{sdf: %E, w_depth: %d, flags: %d}. The w_depth values are different.\n",
		       position.x, position.y, position.z, a.sdf, a.w_depth, a.flags, b.sdf, b.w_depth, b.flags);
		return false;
	}
	if(a.flags != b.flags) {
		printf("Position %d, %d, %d: mismatch between voxel:{sdf: %E, w_depth: %d, flags: %d} and voxel:{sdf: %E, w_depth: %d, flags: %d}. The flags are different.\n",
		       position.x, position.y, position.z, a.sdf, a.w_depth, a.flags, b.sdf, b.w_depth, b.flags);
		return false;
	}
	return true;
}

// *********** unsigned int tolerance type (decimal places) ***********
template<>
_CPU_AND_GPU_CODE_
inline
bool almostEqual<TSDFVoxel_f, unsigned int>(const TSDFVoxel_f& a, const TSDFVoxel_f& b, unsigned int tolerance) {
	const float tolerance_float = 1.0f / (10.0f * static_cast<float>(tolerance));
	return almostEqual(a, b, tolerance_float);
}

template<>
_CPU_AND_GPU_CODE_
inline
bool almostEqual<TSDFVoxel_s, unsigned int>(const TSDFVoxel_s& a, const TSDFVoxel_s& b, unsigned int tolerance) {
	const float tolerance_float = 1.0f / (10.0f * static_cast<float>(tolerance));
	return almostEqual(a, b, tolerance_float);
}

template<>
_CPU_AND_GPU_CODE_
inline
bool
almostEqual<TSDFVoxel_f_rgb, unsigned int>(const TSDFVoxel_f_rgb& a, const TSDFVoxel_f_rgb& b, unsigned int tolerance) {
	const float tolerance_float = 1.0f / (10.0f * static_cast<float>(tolerance));
	return almostEqual(a, b, tolerance_float);
}

template<>
_CPU_AND_GPU_CODE_
inline
bool
almostEqual<TSDFVoxel_s_rgb, unsigned int>(const TSDFVoxel_s_rgb& a, const TSDFVoxel_s_rgb& b, unsigned int tolerance) {
	const float tolerance_float = 1.0f / (10.0f * static_cast<float>(tolerance));
	return almostEqual(a, b, tolerance_float);
}

template<>
_CPU_AND_GPU_CODE_
inline
bool
almostEqual<TSDFVoxel_f_conf, unsigned int>(const TSDFVoxel_f_conf& a, const TSDFVoxel_f_conf& b, unsigned int tolerance) {
	const float tolerance_float = 1.0f / (10.0f * static_cast<float>(tolerance));
	return almostEqual(a, b, tolerance_float);
}

template<>
_CPU_AND_GPU_CODE_
inline
bool almostEqual<TSDFVoxel_s_rgb_conf, unsigned int>(const TSDFVoxel_s_rgb_conf& a, const TSDFVoxel_s_rgb_conf& b,
                                                     unsigned int tolerance) {
	const float tolerance_float = 1.0f / (10.0f * static_cast<float>(tolerance));
	return almostEqual(a, b, tolerance_float);
}

template<>
_CPU_AND_GPU_CODE_
inline
bool
almostEqual<WarpVoxel_f_uf, unsigned int>(const WarpVoxel_f_uf& a, const WarpVoxel_f_uf& b, unsigned int tolerance) {
	const float tolerance_float = 1.0f / (10.0f * static_cast<float>(tolerance));
	return almostEqual(a, b, tolerance_float);
}


template<>
_CPU_AND_GPU_CODE_
inline
bool almostEqual<TSDFVoxel_f_flags, unsigned int>(const TSDFVoxel_f_flags& a, const TSDFVoxel_f_flags& b,
                                                  unsigned int tolerance) {
	const float tolerance_float = 1.0f / (10.0f * static_cast<float>(tolerance));
	return almostEqual(a, b, tolerance_float);
}

//endregion

} // namespace ITMLib
