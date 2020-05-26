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
// e.g. functions of the pattern AlmostEqual<SomeVoxelType,unsigned int> all have equivalent code...

namespace ITMLib {

//  region ==================== DECLARATIONS ===========================================================================

/**
 * \brief Determine whether the two values are within a given tolerance of each-other
 * \details The comparison is done in an absolute way, i.e. the relative value magnitudes don't matter. This is useful
 * for situations where there is a predetermined upper bound on the values, i.e. values are in range [0.0,1.0], and
 * small values don't really matter all that much.
 * \param a the first value
 * \param b the second value
 * \param tolerance tolerance bound within which all of the elements or sub-elements in @p a and @p b should match.
 * \details For integral types or element types an integral value of the same type is treated as an admissible difference bound.
 * For floating-point or element types: if a floating-point value is given, it is treated as an admissible difference bound directly, but if an
 * integral value is given, it is treated as the order after the decimal point point for the error bound, i.e. the bound for argument "1" will be 0.1,
 * the bound for argument "2" will be 0.02, etc.
 * \return true if the two values are within the provided tolerance, false otherwise.
 */
template<typename ElementType, typename ToleranceType>
_CPU_AND_GPU_CODE_
inline
bool AlmostEqual(ORUtils::Vector2<ElementType> a, ORUtils::Vector2<ElementType> b, ToleranceType tolerance);

/** \overload */
template<typename ElementType, typename ToleranceType>
_CPU_AND_GPU_CODE_
inline
bool AlmostEqual(ORUtils::Vector3<ElementType> a, ORUtils::Vector3<ElementType> b, ToleranceType tolerance);

/** \overload */
template<typename ElementType, typename ToleranceType>
_CPU_AND_GPU_CODE_
inline
bool AlmostEqual(ORUtils::Vector4<ElementType> a, ORUtils::Vector4<ElementType> b, ToleranceType tolerance);

/** \overload */
template<typename ElementType, typename ToleranceType>
_CPU_AND_GPU_CODE_
inline
bool AlmostEqual(ORUtils::Vector6<ElementType> a, ORUtils::Vector6<ElementType> b, ToleranceType tolerance);

/** \overload */
template<typename ElementType, typename ToleranceType>
_CPU_AND_GPU_CODE_
inline
bool AlmostEqual(ORUtils::Matrix3<ElementType> a, ORUtils::Matrix3<ElementType> b, ToleranceType tolerance);

/** \overload */
template<typename ElementType, typename ToleranceType>
_CPU_AND_GPU_CODE_
inline
bool AlmostEqual(ORUtils::Matrix4<ElementType> a, ORUtils::Matrix4<ElementType> b, ToleranceType tolerance);

/** \overload */
template<typename TVoxel, typename ToleranceType>
_CPU_AND_GPU_CODE_
inline
bool AlmostEqual(const TVoxel& a, const TVoxel& b, ToleranceType tolerance);

template<typename TVoxel, typename ToleranceType>
_CPU_AND_GPU_CODE_
inline
bool AlmostEqualVerbose(const TVoxel& a, const TVoxel& b, ToleranceType tolerance);

template<typename TVoxel, typename ToleranceType>
_CPU_AND_GPU_CODE_
inline
bool AlmostEqualVerbose_Position(const TVoxel& a, const TVoxel& b, const Vector3i& position, ToleranceType tolerance);

// endregion

// region ==================== ABSOLUTE / RELATIVE REAL TYPE COMPARISONS ===============================================
template<typename TReal>
_CPU_AND_GPU_CODE_
inline
bool RealAlmostEqualRelative(const TReal a, const TReal b, const TReal epsilon = 3e-6) {
	const TReal absA = std::abs(a);
	const TReal absB = std::abs(b);
	const TReal diff = std::abs(a - b);

	if (a == b) { // shortcut, handles infinities
		return true;
	} else if (a == 0 || b == 0 || diff < std::numeric_limits<TReal>::denorm_min()) {
		// a or b is zero or both are extremely Close to it
		// relative error is less meaningful here
		return diff < (epsilon * std::numeric_limits<TReal>::denorm_min());
	} else { // use relative error
		return diff / std::min((absA + absB), std::numeric_limits<TReal>::max()) < epsilon;
	}
}

template<typename TFloatingPoint>
_CPU_AND_GPU_CODE_
inline
bool FloatingPointAlmostEqualAbsolute(const TFloatingPoint a, const TFloatingPoint b, const TFloatingPoint epsilon = 3e-6) {
	return std::abs(a - b) < epsilon;
}

template<typename TSignedIntegral>
_CPU_AND_GPU_CODE_
inline
bool SignedIntegralAlmostEqualAbsolute(const TSignedIntegral a, const TSignedIntegral b, const TSignedIntegral epsilon = 10) {
	return std::abs<TSignedIntegral>(a - b) < epsilon;
}

template<typename TUnsignedIntegral>
_CPU_AND_GPU_CODE_
inline
bool UnsignedIntegralAlmostEqualAbsolute(const TUnsignedIntegral a, const TUnsignedIntegral b, const TUnsignedIntegral epsilon = 10) {
	return a > b ? a - b < epsilon : b - a < epsilon;
}

template<typename T>
struct NumericTypeTraits;

template<>
struct NumericTypeTraits<float> {
	_CPU_AND_GPU_CODE_
	static inline bool AlmostEqual(float a, float b, float epsilon) {
		return FloatingPointAlmostEqualAbsolute(a, b, epsilon);
	}

	_CPU_AND_GPU_CODE_
	static inline bool AlmostEqual(float a, float b) {
		return FloatingPointAlmostEqualAbsolute(a, b);
	}
};
template<>
struct NumericTypeTraits<double> {
	_CPU_AND_GPU_CODE_
	static inline bool AlmostEqual(double a, double b, double epsilon) {
		return FloatingPointAlmostEqualAbsolute(a, b, epsilon);
	}

	_CPU_AND_GPU_CODE_
	static inline bool AlmostEqual(double a, double b) {
		return FloatingPointAlmostEqualAbsolute(a, b);
	}
};
template<>
struct NumericTypeTraits<unsigned char> {
	_CPU_AND_GPU_CODE_
	static inline bool AlmostEqual(unsigned char a, unsigned char b, unsigned char epsilon) {
		return UnsignedIntegralAlmostEqualAbsolute(a, b, epsilon);
	}

	_CPU_AND_GPU_CODE_
	static inline bool AlmostEqual(unsigned char a, unsigned char b) {
		return UnsignedIntegralAlmostEqualAbsolute(a, b);
	}
};
template<>
struct NumericTypeTraits<int> {
	_CPU_AND_GPU_CODE_
	static inline bool AlmostEqual(int a, int b, int epsilon) {
		return SignedIntegralAlmostEqualAbsolute(a, b, epsilon);
	}

	_CPU_AND_GPU_CODE_
	static inline bool AlmostEqual(int a, int b) {
		return a == b;
	}
};
template<>
struct NumericTypeTraits<unsigned int> {
	_CPU_AND_GPU_CODE_
	static inline bool AlmostEqual(unsigned int a, unsigned int b, unsigned int epsilon) {
		return UnsignedIntegralAlmostEqualAbsolute(a, b, epsilon);
	}

	_CPU_AND_GPU_CODE_
	static inline bool AlmostEqual(unsigned int a, unsigned int b) {
		return a == b;
	}
};
//endregion

// region ==================== SPECIFIC REAL TYPE COMPARISONS ==========================================================
_CPU_AND_GPU_CODE_
inline float DecimalPlacesToFloatTolerance(const unsigned int decimal_places) {
	return 1.0f / (10.0f * static_cast<float>(decimal_places));
}

/** \overload */
template<typename T>
_CPU_AND_GPU_CODE_
inline
bool AlmostEqual(T a, T b) {
	return NumericTypeTraits<T>::AlmostEqual(a, b);
}

/** \overload */
template<typename T>
_CPU_AND_GPU_CODE_
inline
bool AlmostEqual(T a, T b, T tolerance) {
	return NumericTypeTraits<T>::AlmostEqual(a, b, tolerance);
}

//endregion

//region ==================== SPECIFIC REAL COLLECTION COMPARISONS =====================================================

// generic Vector versions
template<typename VectorElementType>
_CPU_AND_GPU_CODE_
inline bool
Vector2AlmostEqual(const ORUtils::Vector2<VectorElementType>& a, const ORUtils::Vector2<VectorElementType>& b,
                   VectorElementType tolerance) {
	return AlmostEqual(a.x, b.x, tolerance) && AlmostEqual(a.y, b.y, tolerance);
}

template<typename VectorElementType>
_CPU_AND_GPU_CODE_
inline bool
Vector3AlmostEqual(const ORUtils::Vector3<VectorElementType>& a, const ORUtils::Vector3<VectorElementType>& b,
                   VectorElementType tolerance) {
	return AlmostEqual(a.x, b.x, tolerance)
	       && AlmostEqual(a.y, b.y, tolerance)
	       && AlmostEqual(a.z, b.z, tolerance);
}

template<typename VectorElementType>
_CPU_AND_GPU_CODE_
inline bool
Vector4AlmostEqual(const ORUtils::Vector4<VectorElementType>& a, const ORUtils::Vector4<VectorElementType>& b,
                   VectorElementType tolerance) {
	return AlmostEqual(a.x, b.x, tolerance)
	       && AlmostEqual(a.y, b.y, tolerance)
	       && AlmostEqual(a.z, b.z, tolerance)
	       && AlmostEqual(a.w, b.w, tolerance);
}

template<typename VectorElementType>
_CPU_AND_GPU_CODE_
inline bool
Vector6AlmostEqual(const ORUtils::Vector6<VectorElementType>& a, const ORUtils::Vector6<VectorElementType>& b,
                   VectorElementType tolerance) {
	return AlmostEqual(a.min_x, b.min_x, tolerance)
	       && AlmostEqual(a.min_y, b.min_y, tolerance)
	       && AlmostEqual(a.min_z, b.min_z, tolerance)
	       && AlmostEqual(a.max_y, b.max_y, tolerance)
	       && AlmostEqual(a.max_z, b.max_z, tolerance)
	       && AlmostEqual(a.max_z, b.max_z, tolerance);
}


// *********** float tolerance type ***********

// ***Vector***

// *uchar*
//color comparison
template<>
_CPU_AND_GPU_CODE_
inline
bool AlmostEqual<uchar, float>(Vector3u a, Vector3u b, float tolerance) {
	auto a_float_normalized = TO_FLOAT3(a) / 255.0f;
	auto b_float_normalized = TO_FLOAT3(b) / 255.0f;
	return Vector3AlmostEqual<float>(a_float_normalized, b_float_normalized, tolerance);
}

template<>
_CPU_AND_GPU_CODE_
inline
bool AlmostEqual<uchar, float>(Vector4u a, Vector4u b, float tolerance) {
	auto a_float_normalized = a.toFloat() / 255.0f;
	auto b_float_normalized = b.toFloat() / 255.0f;
	return Vector4AlmostEqual<float>(a_float_normalized, b_float_normalized, tolerance);
}

// *float*
template<>
_CPU_AND_GPU_CODE_
inline
bool AlmostEqual<float, float>(Vector2f a, Vector2f b, float tolerance) {
	return Vector2AlmostEqual<float>(a, b, tolerance);
}

template<>
_CPU_AND_GPU_CODE_
inline
bool AlmostEqual<float, float>(Vector3f a, Vector3f b, float tolerance) {
	return Vector3AlmostEqual<float>(a, b, tolerance);
}

template<>
_CPU_AND_GPU_CODE_
inline
bool AlmostEqual<float, float>(Vector4f a, Vector4f b, float tolerance) {
	return Vector4AlmostEqual<float>(a, b, tolerance);
}

template<>
_CPU_AND_GPU_CODE_
inline
bool AlmostEqual<float, float>(Vector6f a, Vector6f b, float tolerance) {
	return Vector6AlmostEqual<float>(a, b, tolerance);
}

// *double*
template<>
_CPU_AND_GPU_CODE_
inline
bool AlmostEqual<double, float>(Vector2d a, Vector2d b, float tolerance) {
	return Vector2AlmostEqual<double>(a, b, static_cast<double>(tolerance));
}

template<>
_CPU_AND_GPU_CODE_
inline
bool AlmostEqual<double, float>(Vector3d a, Vector3d b, float tolerance) {
	return Vector3AlmostEqual<double>(a, b, static_cast<double>(tolerance));
}

template<>
_CPU_AND_GPU_CODE_
inline
bool AlmostEqual<double, float>(Vector4d a, Vector4d b, float tolerance) {
	return Vector4AlmostEqual<double>(a, b, static_cast<double>(tolerance));
}

// *** Matrix ***
template<>
_CPU_AND_GPU_CODE_
inline
bool AlmostEqual<float, float>(Matrix3f a, Matrix3f b, float tolerance) {
	for (int i_entry = 0; i_entry < 9; i_entry++) {
		if (!FloatingPointAlmostEqualAbsolute(a.m[i_entry], b.m[i_entry], tolerance)) return false;
	}
	return true;
}

template<>
_CPU_AND_GPU_CODE_
inline
bool AlmostEqual<float, float>(Matrix4f a, Matrix4f b, float tolerance) {
	for (int i_entry = 0; i_entry < 16; i_entry++) {
		if (!FloatingPointAlmostEqualAbsolute(a.m[i_entry], b.m[i_entry], tolerance)) return false;
	}
	return true;
}

// *********** double tolerance type ***********

// *** Vector ***

// *uchar*
//color comparison
template<>
_CPU_AND_GPU_CODE_
inline
bool AlmostEqual<uchar, double>(Vector3u a, Vector3u b, double tolerance) {
	auto tolerance_float = static_cast<float>(tolerance);
	auto a_float_normalized = TO_FLOAT3(a) / 255.0f;
	auto b_float_normalized = TO_FLOAT3(b) / 255.0f;
	return AlmostEqual(a_float_normalized, b_float_normalized, tolerance_float);
}

template<>
_CPU_AND_GPU_CODE_
inline
bool AlmostEqual<uchar, double>(Vector4u a, Vector4u b, double tolerance) {
	auto tolerance_float = static_cast<float>(tolerance);
	auto a_float_normalized = a.toFloat() / 255.0f;
	auto b_float_normalized = b.toFloat() / 255.0f;
	return Vector4AlmostEqual<float>(a_float_normalized, b_float_normalized, tolerance_float);
}

// *float*
template<>
_CPU_AND_GPU_CODE_
inline
bool AlmostEqual<float, double>(Vector2f a, Vector2f b, double tolerance) {
	const float tolerance_float = static_cast<float>(tolerance);
	return Vector2AlmostEqual<float>(a, b, tolerance_float);
}

template<>
_CPU_AND_GPU_CODE_
inline
bool AlmostEqual<float, double>(Vector3f a, Vector3f b, double tolerance) {
	const float tolerance_float = static_cast<float>(tolerance);
	return Vector3AlmostEqual<float>(a, b, tolerance_float);
}

template<>
_CPU_AND_GPU_CODE_
inline
bool AlmostEqual<float, double>(Vector4f a, Vector4f b, double tolerance) {
	const float tolerance_float = static_cast<float>(tolerance);
	return Vector4AlmostEqual<float>(a, b, tolerance_float);
}

template<>
_CPU_AND_GPU_CODE_
inline
bool AlmostEqual<float, double>(Vector6f a, Vector6f b, double tolerance) {
	const float tolerance_float = static_cast<float>(tolerance);
	return Vector6AlmostEqual<float>(a, b, tolerance_float);
}

// *double*
template<>
_CPU_AND_GPU_CODE_
inline
bool AlmostEqual<double, double>(Vector2d a, Vector2d b, double tolerance) {
	return Vector2AlmostEqual<double>(a, b, tolerance);
}

template<>
_CPU_AND_GPU_CODE_
inline
bool AlmostEqual<double, double>(Vector3d a, Vector3d b, double tolerance) {
	return Vector3AlmostEqual<double>(a, b, tolerance);
}

template<>
_CPU_AND_GPU_CODE_
inline
bool AlmostEqual<double, double>(Vector4d a, Vector4d b, double tolerance) {
	return Vector4AlmostEqual<double>(a, b, tolerance);
}


// *** Matrix ***

template<>
_CPU_AND_GPU_CODE_
inline
bool AlmostEqual<float, double>(Matrix3f a, Matrix3f b, double tolerance) {
	auto tolerance_float = static_cast<float>(tolerance);
	for (int i_entry = 0; i_entry < 9; i_entry++) {
		if (!FloatingPointAlmostEqualAbsolute(a.m[i_entry], b.m[i_entry], tolerance_float)) return false;
	}
	return true;
}

template<>
_CPU_AND_GPU_CODE_
inline
bool AlmostEqual<float, double>(Matrix4f a, Matrix4f b, double tolerance) {
	auto tolerance_float = static_cast<float>(tolerance);
	for (int i_entry = 0; i_entry < 9; i_entry++) {
		if (!FloatingPointAlmostEqualAbsolute(a.m[i_entry], b.m[i_entry], tolerance_float)) return false;
	}
	return true;
}

// *********** unsigned int tolerance type (decimal places) ***********
template<>
_CPU_AND_GPU_CODE_
inline
bool AlmostEqual<float, unsigned int>(Vector2f a, Vector2f b, unsigned int tolerance) {
	const float tolerance_float = DecimalPlacesToFloatTolerance(tolerance);
	return Vector2AlmostEqual<float>(a, b, tolerance_float);
}

template<>
_CPU_AND_GPU_CODE_
inline
bool AlmostEqual<float, unsigned int>(Vector3f a, Vector3f b, unsigned int tolerance) {
	const float tolerance_float = DecimalPlacesToFloatTolerance(tolerance);
	return Vector3AlmostEqual<float>(a, b, tolerance_float);
}

template<>
_CPU_AND_GPU_CODE_
inline
bool AlmostEqual<float, unsigned int>(Vector4f a, Vector4f b, unsigned int tolerance) {
	const float tolerance_float = DecimalPlacesToFloatTolerance(tolerance);
	return Vector4AlmostEqual<float>(a, b, tolerance_float);
}

//color comparison
template<>
_CPU_AND_GPU_CODE_
inline
bool AlmostEqual<uchar, unsigned int>(Vector3u a, Vector3u b, unsigned int tolerance) {
	uchar tolerance_uchar = static_cast<uchar>(tolerance);
	return Vector3AlmostEqual<uchar>(a, b, tolerance_uchar);
}

template<>
_CPU_AND_GPU_CODE_
inline
bool AlmostEqual<uchar, unsigned int>(Vector4u a, Vector4u b, unsigned int tolerance) {
	uchar tolerance_uchar = static_cast<uchar>(tolerance);
	return Vector4AlmostEqual<uchar>(a, b, tolerance_uchar);
}

template<>
_CPU_AND_GPU_CODE_
inline
bool AlmostEqual<float, unsigned int>(Matrix3f a, Matrix3f b, unsigned int tolerance) {
	const float tolerance_float = 1.0f / (10.0f * static_cast<float>(tolerance));
	for (int i_entry = 0; i_entry < 9; i_entry++) {
		if (!FloatingPointAlmostEqualAbsolute(a.m[i_entry], b.m[i_entry], tolerance_float)) return false;
	}
	return true;
}


template<>
_CPU_AND_GPU_CODE_
inline
bool AlmostEqual<float, unsigned int>(Matrix4f a, Matrix4f b, unsigned int tolerance) {
	const float tolerance_float = 1.0f / (10.0f * static_cast<float>(tolerance));
	for (int i_entry = 0; i_entry < 16; i_entry++) {
		if (!FloatingPointAlmostEqualAbsolute(a.m[i_entry], b.m[i_entry], tolerance_float)) return false;
	}
	return true;
}

//endregion

// region ==================== VOXEL COMPARISONS =======================================================================
// *********** float tolerance type ***********
template<>
_CPU_AND_GPU_CODE_
inline
bool AlmostEqual<TSDFVoxel_f, float>(const TSDFVoxel_f& a, const TSDFVoxel_f& b, float tolerance) {
	return AlmostEqual(a.sdf, b.sdf, tolerance) &&
	       a.w_depth == b.w_depth;
}

template<>
_CPU_AND_GPU_CODE_
inline
bool AlmostEqual<TSDFVoxel_s, float>(const TSDFVoxel_s& a, const TSDFVoxel_s& b, float tolerance) {
	return AlmostEqual(TSDFVoxel_s_rgb::valueToFloat(a.sdf), TSDFVoxel_s_rgb::valueToFloat(b.sdf), tolerance) &&
	       a.w_depth == b.w_depth;
}

template<>
_CPU_AND_GPU_CODE_
inline
bool AlmostEqual<TSDFVoxel_f_rgb, float>(const TSDFVoxel_f_rgb& a, const TSDFVoxel_f_rgb& b, float tolerance) {
	return AlmostEqual(a.sdf, b.sdf, tolerance) &&
	       a.w_depth == b.w_depth &&
	       AlmostEqual(a.clr, b.clr, tolerance) &&
	       a.w_color == b.w_color;
}

template<>
_CPU_AND_GPU_CODE_
inline
bool AlmostEqual<TSDFVoxel_s_rgb, float>(const TSDFVoxel_s_rgb& a, const TSDFVoxel_s_rgb& b, float tolerance) {
	return AlmostEqual(TSDFVoxel_s_rgb::valueToFloat(a.sdf), TSDFVoxel_s_rgb::valueToFloat(b.sdf), tolerance) &&
	       a.w_depth == b.w_depth &&
	       AlmostEqual(a.clr, b.clr, tolerance) &&
	       a.w_color == b.w_color;
}

template<>
_CPU_AND_GPU_CODE_
inline
bool AlmostEqual<TSDFVoxel_f_conf, float>(const TSDFVoxel_f_conf& a, const TSDFVoxel_f_conf& b, float tolerance) {
	return AlmostEqual(a.sdf, b.sdf, tolerance) &&
	       a.w_depth == b.w_depth &&
	       AlmostEqual(a.confidence, b.confidence, tolerance);
}

template<>
_CPU_AND_GPU_CODE_
inline
bool
AlmostEqual<TSDFVoxel_s_rgb_conf, float>(const TSDFVoxel_s_rgb_conf& a, const TSDFVoxel_s_rgb_conf& b,
                                         float tolerance) {
	return AlmostEqual(TSDFVoxel_s_rgb::valueToFloat(a.sdf), TSDFVoxel_s_rgb::valueToFloat(b.sdf), tolerance) &&
	       a.w_depth == b.w_depth &&
	       a.clr == b.clr &&
	       AlmostEqual(a.clr, b.clr, tolerance) &&
	       AlmostEqual(a.confidence, b.confidence, tolerance);
}

template<>
_CPU_AND_GPU_CODE_
inline
bool AlmostEqual<WarpVoxel_f_update, float>(const WarpVoxel_f_update& a, const WarpVoxel_f_update& b, float tolerance) {
	return AlmostEqual(a.warp_update, b.warp_update, tolerance)
	       && AlmostEqual(a.gradient0, b.gradient0, tolerance)
	       && AlmostEqual(a.gradient1, b.gradient1, tolerance);
}

_CPU_AND_GPU_CODE_
inline
void getNonMatchingComponents(bool& xMismatch, bool& yMismatch, bool& zMismatch, const Vector3f& a, const Vector3f& b,
                              float tolerance) {
	xMismatch = !AlmostEqual(a.x, b.x, tolerance);
	yMismatch = !AlmostEqual(a.y, b.y, tolerance);
	zMismatch = !AlmostEqual(a.z, b.z, tolerance);
}

_CPU_AND_GPU_CODE_
inline
void printVector3fVoxelError(const Vector3f& a, const Vector3f& b, float tolerance, const char* description) {
	bool xMismatch, yMismatch, zMismatch;
	getNonMatchingComponents(xMismatch, yMismatch, zMismatch, a, b, tolerance);
	printf("(Showing first error only) %s not within %E: (%E, %E, %E) vs (%E, %E, %E)\n", description, tolerance,
	       a.x, a.y, a.z, b.x, b.y, b.z);
}

_CPU_AND_GPU_CODE_
inline
void printVector3fVoxelError_Position(const Vector3f& a, const Vector3f& b, float tolerance, const char* description,
                                      const Vector3i& position) {
	bool xMismatch, yMismatch, zMismatch;
	getNonMatchingComponents(xMismatch, yMismatch, zMismatch, a, b, tolerance);
	printf("Position %d, %d, %d:(Showing first error only) %s not within %E: (%E, %E, %E) vs (%E, %E, %E)\n",
	       position.x, position.y, position.z, description, tolerance,
	       a.x, a.y, a.z, b.x, b.y, b.z);
}

template<>
_CPU_AND_GPU_CODE_
inline
bool AlmostEqualVerbose<WarpVoxel_f_update, float>(const WarpVoxel_f_update& a, const WarpVoxel_f_update& b,
                                                   float tolerance) {
	if (!AlmostEqual(a.warp_update, b.warp_update, tolerance)) {
		printVector3fVoxelError(a.warp_update, b.warp_update, tolerance, "warp_update");
		return false;
	}
	if (!AlmostEqual(a.gradient0, b.gradient0, tolerance)) {
		printVector3fVoxelError(a.gradient0, b.gradient0, tolerance, "gradient0");
		return false;
	}
	if (!AlmostEqual(a.gradient1, b.gradient1, tolerance)) {
		printVector3fVoxelError(a.gradient1, b.gradient1, tolerance, "gradient1");
		return false;
	}
	return true;
}


template<>
_CPU_AND_GPU_CODE_
inline
bool AlmostEqualVerbose_Position<WarpVoxel_f_update, float>(const WarpVoxel_f_update& a, const WarpVoxel_f_update& b,
                                                            const Vector3i& position, float tolerance) {

	if (!AlmostEqual(a.warp_update, b.warp_update, tolerance)) {
		printVector3fVoxelError_Position(a.warp_update, b.warp_update, tolerance, "warp_update", position);
		return false;
	}
	if (!AlmostEqual(a.gradient0, b.gradient0, tolerance)) {
		printVector3fVoxelError_Position(a.gradient0, b.gradient0, tolerance, "gradient0", position);
		return false;
	}
	if (!AlmostEqual(a.gradient1, b.gradient1, tolerance)) {
		printVector3fVoxelError_Position(a.gradient1, b.gradient1, tolerance, "gradient1", position);
		return false;
	}
	return true;
}

template<>
_CPU_AND_GPU_CODE_
inline
bool AlmostEqual<TSDFVoxel_f_flags, float>(const TSDFVoxel_f_flags& a, const TSDFVoxel_f_flags& b, float tolerance) {
	return AlmostEqual(a.sdf, b.sdf, tolerance) &&
	       a.w_depth == b.w_depth &&
	       a.flags == b.flags;
}


template<>
_CPU_AND_GPU_CODE_
inline
bool AlmostEqualVerbose_Position<TSDFVoxel_f_flags, float>(const TSDFVoxel_f_flags& a, const TSDFVoxel_f_flags& b,
                                                           const Vector3i& position, float tolerance) {
	if (!(a.flags == VoxelFlags::VOXEL_UNKNOWN && b.flags == VoxelFlags::VOXEL_UNKNOWN)) {
		if (!AlmostEqual(a.sdf, b.sdf, tolerance)) {
			printf("Position %d, %d, %d: mismatch between voxel:{sdf: %E, w_depth: %d, flags: %d} and voxel:{sdf: %E, w_depth: %d, flags: %d}. SDF not within tolerance %E.\n",
			       position.x, position.y, position.z, a.sdf, a.w_depth, a.flags, b.sdf, b.w_depth, b.flags, tolerance);
			return false;
		}
		if (a.w_depth != b.w_depth) {
			printf("Position %d, %d, %d: mismatch between voxel:{sdf: %E, w_depth: %d, flags: %d} and voxel:{sdf: %E, w_depth: %d, flags: %d}. The w_depth values are different.\n",
			       position.x, position.y, position.z, a.sdf, a.w_depth, a.flags, b.sdf, b.w_depth, b.flags);
			return false;
		}
		if (a.flags != b.flags) {
			printf("Position %d, %d, %d: mismatch between voxel:{sdf: %E, w_depth: %d, flags: %d} and voxel:{sdf: %E, w_depth: %d, flags: %d}. The flags are different.\n",
			       position.x, position.y, position.z, a.sdf, a.w_depth, a.flags, b.sdf, b.w_depth, b.flags);
			return false;
		}
	}
	return true;
}

// *********** unsigned int tolerance type (decimal places) ***********
template<>
_CPU_AND_GPU_CODE_
inline
bool AlmostEqual<TSDFVoxel_f, unsigned int>(const TSDFVoxel_f& a, const TSDFVoxel_f& b, unsigned int tolerance) {
	const float tolerance_float = 1.0f / (10.0f * static_cast<float>(tolerance));
	return AlmostEqual(a, b, tolerance_float);
}

template<>
_CPU_AND_GPU_CODE_
inline
bool AlmostEqual<TSDFVoxel_s, unsigned int>(const TSDFVoxel_s& a, const TSDFVoxel_s& b, unsigned int tolerance) {
	const float tolerance_float = 1.0f / (10.0f * static_cast<float>(tolerance));
	return AlmostEqual(a, b, tolerance_float);
}

template<>
_CPU_AND_GPU_CODE_
inline
bool
AlmostEqual<TSDFVoxel_f_rgb, unsigned int>(const TSDFVoxel_f_rgb& a, const TSDFVoxel_f_rgb& b, unsigned int tolerance) {
	const float tolerance_float = 1.0f / (10.0f * static_cast<float>(tolerance));
	return AlmostEqual(a, b, tolerance_float);
}

template<>
_CPU_AND_GPU_CODE_
inline
bool
AlmostEqual<TSDFVoxel_s_rgb, unsigned int>(const TSDFVoxel_s_rgb& a, const TSDFVoxel_s_rgb& b, unsigned int tolerance) {
	const float tolerance_float = 1.0f / (10.0f * static_cast<float>(tolerance));
	return AlmostEqual(a, b, tolerance_float);
}

template<>
_CPU_AND_GPU_CODE_
inline
bool
AlmostEqual<TSDFVoxel_f_conf, unsigned int>(const TSDFVoxel_f_conf& a, const TSDFVoxel_f_conf& b,
                                            unsigned int tolerance) {
	const float tolerance_float = 1.0f / (10.0f * static_cast<float>(tolerance));
	return AlmostEqual(a, b, tolerance_float);
}

template<>
_CPU_AND_GPU_CODE_
inline
bool AlmostEqual<TSDFVoxel_s_rgb_conf, unsigned int>(const TSDFVoxel_s_rgb_conf& a, const TSDFVoxel_s_rgb_conf& b,
                                                     unsigned int tolerance) {
	const float tolerance_float = 1.0f / (10.0f * static_cast<float>(tolerance));
	return AlmostEqual(a, b, tolerance_float);
}

template<>
_CPU_AND_GPU_CODE_
inline
bool
AlmostEqual<WarpVoxel_f_update, unsigned int>(const WarpVoxel_f_update& a, const WarpVoxel_f_update& b,
                                              unsigned int tolerance) {
	const float tolerance_float = 1.0f / (10.0f * static_cast<float>(tolerance));
	return AlmostEqual(a, b, tolerance_float);
}


template<>
_CPU_AND_GPU_CODE_
inline
bool AlmostEqual<TSDFVoxel_f_flags, unsigned int>(const TSDFVoxel_f_flags& a, const TSDFVoxel_f_flags& b,
                                                  unsigned int tolerance) {
	const float tolerance_float = 1.0f / (10.0f * static_cast<float>(tolerance));
	return AlmostEqual(a, b, tolerance_float);
}

//endregion

} // namespace ITMLib
