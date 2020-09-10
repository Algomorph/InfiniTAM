//  ================================================================
//  Created by Gregory Kramida on 4/26/18.
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

#include "../../../Utils/Enums/VoxelFlags.h"
#include "../../../Objects/Volume/RepresentationAccess.h"
#include "../../../../ORUtils/PlatformIndependence.h"
#include "../../../Utils/Logging/ConsolePrintColors.h"
#include "WarpHessian.h"


namespace ITMLib {


#define TRACKING_CONDITION_LIVE_KNOWN
//#define TRACKING_CONDITION_LIVE_NONTRUNCATED



template<typename TVoxel>
_CPU_AND_GPU_CODE_ inline
bool VoxelIsConsideredForTracking(const TVoxel& voxel_canonical, const TVoxel& voxel_live) {
#if defined(TRACKING_CONDITION_LIVE_KNOWN)
	return voxel_live.flags != ITMLib::VOXEL_UNKNOWN;
#elif defined(TRACKING_CONDITION_LIVE_NONTRUNCATED)
	return voxel_live.flags == ITMLib::VOXEL_NONTRUNCATED;
#endif
};


//#define DATA_CONDITION_IGNORE_ANY_UNKNOWN
#define DATA_CONDITION_LIVE_NONTRUNCATED_CANONICAL_KNOWN

template<typename TVoxel>
_CPU_AND_GPU_CODE_ inline
bool VoxelIsConsideredForDataTerm(const TVoxel& canonical_voxel, const TVoxel& live_voxel) {
//_DEBUG preprocessor options
#if defined(DATA_CONDITION_ALWAYS)
	return true;
#elif defined(DATA_CONDITION_IGNORE_ANY_UNKNOWN)
	return canonical_voxel.flags != ITMLib::VOXEL_UNKNOWN && live_voxel.flags != ITMLib::VOXEL_UNKNOWN;
#elif defined(DATA_CONDITION_LIVE_NONTRUNCATED_CANONICAL_KNOWN)
	return canonical_voxel.flags != ITMLib::VOXEL_UNKNOWN && live_voxel.flags == ITMLib::VOXEL_NONTRUNCATED;
#elif defined(DATA_CONDITION_ONLY_NONTRUNCATED)
	return live_voxel.flags == ITMLib::VOXEL_NONTRUNCATED
						 && canonical_voxel.flags == ITMLib::VOXEL_NONTRUNCATED;
#elif defined(DATA_CONDITION_IGNORE_BOTH_UNKNOWN)
	return canonical_voxel.flags != ITMLib::VOXEL_UNKNOWN || live_voxel.flags != ITMLib::VOXEL_UNKNOWN;
#elif defined(DATA_CONDITION_IGNORE_CANONICAL_UNKNOWN)
	return canonical_voxel.flags != ITMLib::VOXEL_UNKNOWN;
#else
	//Same as data condition DATA_CONDITION_ALWAYS
	return true;
#endif
};


// region =================================EXPLORATION OF NEIGHBORHOOD AROUND CANONICAL VOXEL===========================

/**
 * \brief Finds neighbor voxel's warps in the order specified below.
 *     0        1        2          3         4         5           6         7         8
 *	(-1,0,0) (0,-1,0) (0,0,-1)   (1, 0, 0) (0, 1, 0) (0, 0, 1)   (1, 1, 0) (0, 1, 1) (1, 0, 1)
 * \tparam TVoxel
 * \tparam TCache
 * \param[out] neighbor_warp_updates
 * \param[out] neighbor_known - current behavior is:
 * 1) record unallocated voxels as non-found
 * 2) truncated voxels marked unknown or known as found
 * 3) everything else (non-truncated), of course, as found
 * \param[in] voxel_position exact position of voxel in the scene.
 * \param[in] warps
 * \param[in] indexData
 * \param[in] cache
 */
template<typename TVoxel, typename TWarp, typename TIndexData, typename TCache>
_CPU_AND_GPU_CODE_
inline void findPoint2ndDerivativeNeighborhoodFramewiseWarp(THREADPTR(Vector3f)* neighbor_warp_updates, //x9, out
                                                            THREADPTR(bool)* neighbor_known, //x9, out
                                                            THREADPTR(bool)* neighbor_truncated, //x9, out
                                                            THREADPTR(bool)* neighbor_allocated, //x9, out
                                                            const CONSTPTR(Vector3i)& voxel_position,
                                                            const CONSTPTR(TWarp)* warps,
                                                            const CONSTPTR(TIndexData)* warp_index_data,
                                                            THREADPTR(TCache)& warp_cache,
                                                            const CONSTPTR(TVoxel)* voxels,
                                                            const CONSTPTR(TIndexData)* voxel_index_data,
                                                            THREADPTR(TCache)& voxel_cache) {
	int vm_index = 0;

	TVoxel voxel;
	TWarp warp;

	auto process_voxel = [&](Vector3i location, int index) {
#if !defined(__CUDACC__) && !defined(WITH_OPENMP)
		warp = readVoxel(warps, warp_index_data, voxel_position + (location), vm_index, warp_cache);
		voxel = readVoxel(voxels, voxel_index_data, voxel_position + (location), vm_index, voxel_cache);
#else //don't use cache for multithreading
		warp = readVoxel(warps, warp_index_data, voxel_position + (location), vm_index);
		voxel = readVoxel(voxels, voxel_index_data, voxel_position + (location), vm_index);
#endif
		neighbor_warp_updates[index] = warp.warp_update;
		neighbor_allocated[index] = vm_index != 0;
		neighbor_known[index] = voxel.flags != ITMLib::VOXEL_UNKNOWN;
		neighbor_truncated[index] = voxel.flags == ITMLib::VOXEL_TRUNCATED;
	};
	//necessary for 2nd derivatives in same direction, e.g. xx and zz
	process_voxel(Vector3i(-1, 0, 0), 0);
	process_voxel(Vector3i(0, -1, 0), 1);
	process_voxel(Vector3i(0, 0, -1), 2);

	//necessary for 1st derivatives
	process_voxel(Vector3i(1, 0, 0), 3);
	process_voxel(Vector3i(0, 1, 0), 4);
	process_voxel(Vector3i(0, 0, 1), 5);

	//necessary for 2nd derivatives in mixed directions, e.g. xy and yz
	process_voxel(Vector3i(1, 1, 0), 6);//xy corner
	process_voxel(Vector3i(0, 1, 1), 7);//yz corner
	process_voxel(Vector3i(1, 0, 1), 8);//xz corner
}


template<typename TVoxel, typename TIndexData, typename TCache>
_CPU_AND_GPU_CODE_
inline void findPoint2ndDerivativeNeighborhoodFramewiseWarp_DEBUG(
		THREADPTR(Vector3f)* neighborFramewiseWarps, //x9, out
		THREADPTR(bool)* neighborKnown, //x9, out
		THREADPTR(bool)* neighborTruncated, //x9, out
		THREADPTR(bool)* neighborAllocated, //x9, out
		const CONSTPTR(Vector3i)& voxelPosition,
		const CONSTPTR(TVoxel)* voxels,
		const CONSTPTR(TIndexData)* indexData,
		THREADPTR(TCache)& cache) {
	int vmIndex = 0;

	TVoxel voxel;
	//TODO: define inline function instead of macro
	voxel = readVoxel(voxels, indexData, voxelPosition + Vector3i(0, 0, 0), vmIndex, cache);
	neighborFramewiseWarps[0] = voxel.framewise_warp;
	neighborAllocated[0] = vmIndex != 0;
	neighborKnown[0] = voxel.flags != ITMLib::VOXEL_UNKNOWN;
	neighborTruncated[0] = voxel.flags == ITMLib::VOXEL_TRUNCATED;
}


/**
 * \brief Finds neighbor voxel's warps in the order specified below.
 *     0        1        2          3         4         5           6         7         8
 *	(-1,0,0) (0,-1,0) (0,0,-1)   (1, 0, 0) (0, 1, 0) (0, 0, 1)   (1, 1, 0) (0, 1, 1) (1, 0, 1)
 * \tparam TVoxel
 * \tparam TCache
 * \param[out] neighborUpdates
 * \param[out] neighborKnown - current behavior is:
 * 1) record unallocated voxels as non-found
 * 2) truncated voxels marked unknown or known as found
 * 3) everything else (non-truncated), of course, as found
 * \param[in] voxelPosition exact position of voxel in the scene.
 * \param[in] voxels
 * \param[in] indexData
 * \param[in] cache
 */
template<typename TVoxel, typename TIndexData, typename TCache>
_CPU_AND_GPU_CODE_
inline void findPoint2ndDerivativeNeighborhoodPreviousUpdate(THREADPTR(Vector3f)* neighborUpdates, //x9, out
                                                             THREADPTR(bool)* neighborKnown, //x9, out
                                                             THREADPTR(bool)* neighborTruncated, //x9, out
                                                             THREADPTR(bool)* neighborAllocated, //x9, out
                                                             const CONSTPTR(Vector3i)& voxelPosition,
                                                             const CONSTPTR(TVoxel)* voxels,
                                                             const CONSTPTR(TIndexData)* indexData,
                                                             THREADPTR(TCache)& cache) {
	int vmIndex = 0;

	TVoxel voxel;
	//TODO: define inline function instead of macro
#define PROCESS_VOXEL(location, index)\
    voxel = readVoxel(voxels, indexData, voxelPosition + (location), vmIndex, cache);\
    neighborUpdates[index] = voxel.warp_update;\
    neighborAllocated[index] = vmIndex != 0;\
    neighborKnown[index] = voxel.flags != ITMLib::VOXEL_UNKNOWN;\
    neighborTruncated[index] = voxel.flags == ITMLib::VOXEL_TRUNCATED;

	//necessary for 2nd derivatives in same direction, e.g. xx and zz
	PROCESS_VOXEL(Vector3i(-1, 0, 0), 0);
	PROCESS_VOXEL(Vector3i(0, -1, 0), 1);
	PROCESS_VOXEL(Vector3i(0, 0, -1), 2);

	//necessary for 1st derivatives
	PROCESS_VOXEL(Vector3i(1, 0, 0), 3);
	PROCESS_VOXEL(Vector3i(0, 1, 0), 4);
	PROCESS_VOXEL(Vector3i(0, 0, 1), 5);

	//necessary for 2nd derivatives in mixed directions, e.g. xy and yz
	PROCESS_VOXEL(Vector3i(1, 1, 0), 6);//xy corner
	PROCESS_VOXEL(Vector3i(0, 1, 1), 7);//yz corner
	PROCESS_VOXEL(Vector3i(1, 0, 1), 8);//xz corner
#undef PROCESS_VOXEL
}


/**
 * \brief Finds neighbor voxel's warps in the order specified below.
 *     0        1        2          3         4         5           6         7         8
 *	(-1,0,0) (0,-1,0) (0,0,-1)   (1, 0, 0) (0, 1, 0) (0, 0, 1)   (1, 1, 0) (0, 1, 1) (1, 0, 1)
 * \tparam TVoxel
 * \tparam TCache
 * \param[out] neighborWarps
 * \param[out] neighborKnown - current behavior is:
 * 1) record unallocated voxels as non-found
 * 2) truncated voxels marked unknown or known as found
 * 3) everything else (non-truncated), of course, as found
 * \param[in] voxelPosition exact position of voxel in the scene.
 * \param[in] voxels
 * \param[in] indexData
 * \param[in] cache
 */
template<typename TVoxel, typename TIndexData, typename TCache>
_CPU_AND_GPU_CODE_
inline void findPoint2ndDerivativeNeighborhoodWarp(THREADPTR(Vector3f)* neighborWarps, //x9, out
                                                   THREADPTR(bool)* neighborKnown, //x9, out
                                                   THREADPTR(bool)* neighborTruncated, //x9, out
                                                   THREADPTR(bool)* neighborAllocated, //x9, out
                                                   const CONSTPTR(Vector3i)& voxelPosition,
                                                   const CONSTPTR(TVoxel)* voxels,
                                                   const CONSTPTR(TIndexData)* indexData,
                                                   THREADPTR(TCache)& cache) {
	int vmIndex = 0;

	TVoxel voxel;
	//TODO: define inline function instead of macro
#define PROCESS_VOXEL(location, index)\
    voxel = readVoxel(voxels, indexData, voxelPosition + (location), vmIndex, cache);\
    neighborWarps[index] = voxel.warp;\
    neighborAllocated[index] = vmIndex != 0;\
    neighborKnown[index] = voxel.flags != ITMLib::VOXEL_UNKNOWN;\
    neighborTruncated[index] = voxel.flags == ITMLib::VOXEL_TRUNCATED;

	//necessary for 2nd derivatives in same direction, e.g. xx and zz
	PROCESS_VOXEL(Vector3i(-1, 0, 0), 0);
	PROCESS_VOXEL(Vector3i(0, -1, 0), 1);
	PROCESS_VOXEL(Vector3i(0, 0, -1), 2);

	//necessary for 1st derivatives
	PROCESS_VOXEL(Vector3i(1, 0, 0), 3);
	PROCESS_VOXEL(Vector3i(0, 1, 0), 4);
	PROCESS_VOXEL(Vector3i(0, 0, 1), 5);

	//necessary for 2nd derivatives in mixed directions, e.g. xy and yz
	PROCESS_VOXEL(Vector3i(1, 1, 0), 6);//xy corner
	PROCESS_VOXEL(Vector3i(0, 1, 1), 7);//yz corner
	PROCESS_VOXEL(Vector3i(1, 0, 1), 8);//xz corner
#undef PROCESS_VOXEL
}

//endregion




//region ================================= SDF GRADIENT ================================================================

template<typename TVoxel, typename TIndexData, typename TCache>
_CPU_AND_GPU_CODE_
inline void ComputeGradient_ForwardDifferences(THREADPTR(Vector3f)& gradient,
                                               const CONSTPTR(Vector3i)& position,
                                               const CONSTPTR(float)& sdf_at_position,
                                               const CONSTPTR(TVoxel)* voxels,
                                               const CONSTPTR(TIndexData)* index_data,
                                               THREADPTR(TCache) cache) {
	int vm_index = 0;
	auto sdf_at = [&](Vector3i offset) {
#if !defined(__CUDACC__) && !defined(WITH_OPENMP)
		return TVoxel::valueToFloat(readVoxel(voxels, index_data, position + (offset), vm_index, cache).sdf);
#else
		return TVoxel::valueToFloat(readVoxel(voxels, index_data, position + (offset), vm_index).sdf);
#endif
	};

	float sdf_at_x_plus_one = sdf_at(Vector3i(1, 0, 0));
	float sdf_at_y_plus_one = sdf_at(Vector3i(0, 1, 0));
	float sdf_at_z_plus_one = sdf_at(Vector3i(0, 0, 1));

	gradient[0] = sdf_at_x_plus_one - sdf_at_position;
	gradient[1] = sdf_at_y_plus_one - sdf_at_position;
	gradient[2] = sdf_at_z_plus_one - sdf_at_position;
};

template<typename TVoxel, typename TIndexData, typename TCache>
_CPU_AND_GPU_CODE_
inline void ComputeGradient_ForwardDifferences(THREADPTR(Vector3f)& gradient,
                                               const CONSTPTR(Vector3i)& position,
                                               const CONSTPTR(TVoxel)* voxels,
                                               const CONSTPTR(TIndexData)* index_data,
                                               THREADPTR(TCache)& cache) {

	int vm_index = 0;
#if !defined(__CUDACC__) && !defined(WITH_OPENMP)
	float sdf_at_position = TVoxel::valueToFloat(readVoxel(voxels, index_data, position, vm_index, cache).sdf);
#else
	float sdf_at_position = TVoxel::valueToFloat(readVoxel(voxels, index_data, position, vm_index).sdf);
#endif
	ComputeGradient_ForwardDifferences(gradient, position, sdf_at_position, voxels, index_data, cache);

};

template<typename TVoxel, typename TIndexData, typename TCache>
_CPU_AND_GPU_CODE_
inline void ComputeGradient_CentralDifferences(THREADPTR(Vector3f)& gradient,
                                               const CONSTPTR(Vector3i)& voxel_position,
                                               const CONSTPTR(TVoxel)* voxels,
                                               const CONSTPTR(TIndexData)* index_data,
                                               THREADPTR(TCache)& cache) {


	int vmIndex = 0;
	auto sdf_at = [&](Vector3i offset) {
#if !defined(__CUDACC__) && !defined(WITH_OPENMP)
		return TVoxel::valueToFloat(readVoxel(voxels, index_data, voxel_position + (offset), vmIndex, cache).sdf);
#else
		return TVoxel::valueToFloat(readVoxel(voxels, index_data, voxel_position + (offset), vmIndex).sdf);
#endif
	};

	float sdf_at_position = sdf_at(Vector3i(0, 0, 0));

	float sdf_at_x_plus_one = sdf_at(Vector3i(1, 0, 0));
	float sdf_at_y_plus_one = sdf_at(Vector3i(0, 1, 0));
	float sdf_at_z_plus_one = sdf_at(Vector3i(0, 0, 1));
	float sdf_at_x_minus_one = sdf_at(Vector3i(-1, 0, 0));
	float sdf_at_y_minus_one = sdf_at(Vector3i(0, -1, 0));
	float sdf_at_z_minus_one = sdf_at(Vector3i(0, 0, -1));

	gradient[0] = 0.5f * (sdf_at_x_plus_one - sdf_at_x_minus_one);
	gradient[1] = 0.5f * (sdf_at_y_plus_one - sdf_at_y_minus_one);
	gradient[2] = 0.5f * (sdf_at_z_plus_one - sdf_at_z_minus_one);
};


template<typename TVoxel, typename TIndexData, typename TCache>
_CPU_AND_GPU_CODE_
inline void ComputeGradient_CentralDifferences_ZeroIfTruncated(THREADPTR(Vector3f)& gradient,
                                                               const CONSTPTR(Vector3i)& voxel_position,
                                                               const CONSTPTR(TVoxel)* voxels,
                                                               const CONSTPTR(TIndexData)* index_data,
                                                               THREADPTR(TCache)& cache) {


	int vmIndex = 0;
	auto sdf_at = [&](Vector3i offset, bool& nontruncated) {
#if !defined(__CUDACC__) && !defined(WITH_OPENMP)
		TVoxel voxel = readVoxel(voxels, index_data, voxel_position + (offset), vmIndex, cache);
		nontruncated &= voxel.flags == ITMLib::VOXEL_NONTRUNCATED;
		return TVoxel::valueToFloat(voxel.sdf);
#else
		TVoxel voxel = readVoxel(voxels, index_data, voxel_position + (offset), vmIndex);
		nontruncated &= voxel.flags == ITMLib::VOXEL_NONTRUNCATED;
		return TVoxel::valueToFloat(voxel.sdf);
#endif
	};

	bool both_x_nontruncated = true;
	float sdf_at_x_plus_one = sdf_at(Vector3i(1, 0, 0), both_x_nontruncated);
	float sdf_at_x_minus_one = sdf_at(Vector3i(-1, 0, 0), both_x_nontruncated);

	bool both_y_nontruncated = true;
	float sdf_at_y_plus_one = sdf_at(Vector3i(0, 1, 0), both_y_nontruncated);
	float sdf_at_y_minus_one = sdf_at(Vector3i(0, -1, 0), both_y_nontruncated);

	bool both_z_nontruncated = true;
	float sdf_at_z_plus_one = sdf_at(Vector3i(0, 0, 1), both_z_nontruncated);
	float sdf_at_z_minus_one = sdf_at(Vector3i(0, 0, -1), both_z_nontruncated);

	gradient[0] = both_x_nontruncated * 0.5f * (sdf_at_x_plus_one - sdf_at_x_minus_one);
	gradient[1] = both_y_nontruncated * 0.5f * (sdf_at_y_plus_one - sdf_at_y_minus_one);
	gradient[2] = both_z_nontruncated * 0.5f * (sdf_at_z_plus_one - sdf_at_z_minus_one);
};

template<typename TVoxel, typename TIndexData, typename TCache>
_CPU_AND_GPU_CODE_
inline void ComputeGradient_CentralDifferences_ZeroIfTruncated_Factors(THREADPTR(Vector3f)& gradient,
                                                                       const CONSTPTR(Vector3i)& voxel_position,
                                                                       const CONSTPTR(TVoxel)* voxels,
                                                                       const CONSTPTR(TIndexData)* index_data,
                                                                       THREADPTR(TCache)& cache) {


	int vmIndex = 0;
	auto sdf_at = [&](Vector3i offset, bool& nontruncated) {
#if !defined(__CUDACC__) && !defined(WITH_OPENMP)
		TVoxel voxel = readVoxel(voxels, index_data, voxel_position + (offset), vmIndex, cache);
		nontruncated &= voxel.flags == ITMLib::VOXEL_NONTRUNCATED;
		return TVoxel::valueToFloat(voxel.sdf);
#else
		TVoxel voxel = readVoxel(voxels, index_data, voxel_position + (offset), vmIndex);
		nontruncated &= voxel.flags == ITMLib::VOXEL_NONTRUNCATED;
		return TVoxel::valueToFloat(voxel.sdf);
#endif
	};

	bool both_x_nontruncated = true;
	float sdf_at_x_plus_one = sdf_at(Vector3i(1, 0, 0), both_x_nontruncated);
	float sdf_at_x_minus_one = sdf_at(Vector3i(-1, 0, 0), both_x_nontruncated);

	bool both_y_nontruncated = true;
	float sdf_at_y_plus_one = sdf_at(Vector3i(0, 1, 0), both_y_nontruncated);
	float sdf_at_y_minus_one = sdf_at(Vector3i(0, -1, 0), both_y_nontruncated);

	bool both_z_nontruncated = true;
	float sdf_at_z_plus_one = sdf_at(Vector3i(0, 0, 1), both_z_nontruncated);
	float sdf_at_z_minus_one = sdf_at(Vector3i(0, 0, -1), both_z_nontruncated);

	gradient[0] = both_x_nontruncated * 5.0f * (sdf_at_x_plus_one - sdf_at_x_minus_one);
	gradient[1] = both_y_nontruncated * 5.0f * (sdf_at_y_plus_one - sdf_at_y_minus_one);
	gradient[2] = both_z_nontruncated * 5.0f * (sdf_at_z_plus_one - sdf_at_z_minus_one);
};


template<typename TVoxel, typename TIndexData, typename TCache>
_CPU_AND_GPU_CODE_
inline void ComputeGradient_ChooseStrategyOnTruncation(THREADPTR(Vector3f)& gradient,
                                                       const CONSTPTR(Vector3i)& voxel_position,
                                                       const CONSTPTR(TVoxel)* voxels,
                                                       const CONSTPTR(TIndexData)* index_data,
                                                       THREADPTR(TCache)& cache) {


	int vmIndex = 0;
	auto sdf_at = [&](Vector3i offset, bool& nontruncated) {
#if !defined(__CUDACC__) && !defined(WITH_OPENMP)
		TVoxel voxel = readVoxel(voxels, index_data, voxel_position + (offset), vmIndex, cache);
		nontruncated = voxel.flags == ITMLib::VOXEL_NONTRUNCATED;
		return TVoxel::valueToFloat(voxel.sdf);
#else
		TVoxel voxel = readVoxel(voxels, index_data, voxel_position + (offset), vmIndex);
		nontruncated = voxel.flags == ITMLib::VOXEL_NONTRUNCATED;
		return TVoxel::valueToFloat(voxel.sdf);
#endif
	};

	bool dummy;
	float sdf_at_position = sdf_at(Vector3i(0, 0, 0), dummy);

	bool x_plus_one_nontruncated;
	float sdf_at_x_plus_one = sdf_at(Vector3i(1, 0, 0), x_plus_one_nontruncated);
	bool y_plus_one_nontruncated;
	float sdf_at_y_plus_one = sdf_at(Vector3i(0, 1, 0), y_plus_one_nontruncated);
	bool z_plus_one_nontruncated;
	float sdf_at_z_plus_one = sdf_at(Vector3i(0, 0, 1), z_plus_one_nontruncated);
	bool x_minus_one_nontruncated;
	float sdf_at_x_minus_one = sdf_at(Vector3i(-1, 0, 0), x_minus_one_nontruncated);
	bool y_minus_one_nontruncated;
	float sdf_at_y_minus_one = sdf_at(Vector3i(0, -1, 0), y_minus_one_nontruncated);
	bool z_minus_one_nontruncated;
	float sdf_at_z_minus_one = sdf_at(Vector3i(0, 0, -1), z_minus_one_nontruncated);

	if (x_plus_one_nontruncated) {
		if (x_minus_one_nontruncated) {
			gradient[0] = 0.5f * (sdf_at_x_plus_one - sdf_at_x_minus_one);
		} else {
			gradient[0] = sdf_at_x_plus_one - sdf_at_position;
		}
	} else {
		if (x_minus_one_nontruncated) {
			gradient[0] = sdf_at_position - sdf_at_x_minus_one;
		}
	}

	if (y_plus_one_nontruncated) {
		if (y_minus_one_nontruncated) {
			gradient[1] = 0.5f * (sdf_at_y_plus_one - sdf_at_y_minus_one);
		} else {
			gradient[1] = sdf_at_y_plus_one - sdf_at_position;
		}
	} else {
		if (y_minus_one_nontruncated) {
			gradient[1] = sdf_at_position - sdf_at_y_minus_one;
		}
	}

	if (z_plus_one_nontruncated) {
		if (z_minus_one_nontruncated) {
			gradient[2] = 0.5f * (sdf_at_z_plus_one - sdf_at_z_minus_one);
		} else {
			gradient[2] = sdf_at_z_plus_one - sdf_at_position;
		}
	} else {
		if (z_minus_one_nontruncated) {
			gradient[2] = sdf_at_position - sdf_at_z_minus_one;
		}
	}

//	bool both_x_nontruncated = x_plus_one_nontruncated && x_minus_one_nontruncated;
//	bool both_y_nontruncated = y_plus_one_nontruncated && y_minus_one_nontruncated;
//	bool both_z_nontruncated = z_plus_one_nontruncated && z_minus_one_nontruncated;

//	gradient[0] = both_x_nontruncated * 0.5f * (sdf_at_x_plus_one - sdf_at_x_minus_one)
//	              + (!both_x_nontruncated && x_plus_one_nontruncated) * (sdf_at_x_plus_one - sdf_at_position)
//	              + (!both_x_nontruncated && x_minus_one_nontruncated) * (sdf_at_position - sdf_at_x_minus_one);
//	gradient[1] = both_y_nontruncated * 0.5f * (sdf_at_y_plus_one - sdf_at_y_minus_one)
//	              + (!both_y_nontruncated && y_plus_one_nontruncated) * (sdf_at_y_plus_one - sdf_at_position)
//	              + (!both_y_nontruncated && y_minus_one_nontruncated) * (sdf_at_position - sdf_at_y_minus_one);
//	gradient[2] = both_z_nontruncated * 0.5f * (sdf_at_z_plus_one - sdf_at_z_minus_one)
//	              + (!both_z_nontruncated && z_plus_one_nontruncated) * (sdf_at_z_plus_one - sdf_at_position)
//	              + (!both_z_nontruncated && z_minus_one_nontruncated) * (sdf_at_position - sdf_at_z_minus_one);
};

// endregion

// region ================================= SDF 2ND DERIVATIVE ================================================================

template<typename TVoxel, typename TIndexData, typename TCache>
_CPU_AND_GPU_CODE_
inline void ComputeSdf2ndDerivative(THREADPTR(Matrix3f)& hessian,
                                    const CONSTPTR(Vector3i &) position,
                                    const CONSTPTR(float)& sdf_at_position,
		//const CONSTPTR(Vector3f&) jacobianAtPosition,
		                            const CONSTPTR(TVoxel)* voxels,
		                            const CONSTPTR(TIndexData)* index_data,
		                            THREADPTR(TCache)& cache) {
	int vm_index = 0;

	auto sdf_at = [&](Vector3i offset) {
#if !defined(__CUDACC__) && !defined(WITH_OPENMP)
		return TVoxel::valueToFloat(readVoxel(voxels, index_data, position + (offset), vm_index, cache).sdf);
#else //don't use cache when multithreading
		return TVoxel::valueToFloat(readVoxel(voxels, index_data, position + (offset), vm_index).sdf);
#endif
	};

	//for xx, yy, zz
	float sdf_at_x_plus_one = sdf_at(Vector3i(1, 0, 0));
	float sdf_at_y_plus_one = sdf_at(Vector3i(0, 1, 0));
	float sdf_at_z_plus_one = sdf_at(Vector3i(0, 0, 1));
	float sdf_at_x_minus_one = sdf_at(Vector3i(-1, 0, 0));
	float sdf_at_y_minus_one = sdf_at(Vector3i(0, -1, 0));
	float sdf_at_z_minus_one = sdf_at(Vector3i(0, 0, -1));

	//for xy, xz, yz
	//@formatter:off
	float sdf_at_x_plus_one_y_plus_one   = sdf_at(Vector3i( 1,  1,  0));
	float sdf_at_x_minus_one_y_minus_one = sdf_at(Vector3i(-1, -1,  0));
	float sdf_at_y_plus_one_z_plus_one   = sdf_at(Vector3i( 0,  1,  1));
	float sdf_at_y_minus_one_z_minus_one = sdf_at(Vector3i( 0, -1, -1));
	float sdf_at_x_plus_one_z_plus_one   = sdf_at(Vector3i( 1,  0,  1));
	float sdf_at_x_minus_one_z_minus_one = sdf_at(Vector3i(-1,  0, -1));
	//@formatter:on

	float delta_xx = sdf_at_x_plus_one - 2 * sdf_at_position + sdf_at_x_minus_one;
	float delta_yy = sdf_at_y_plus_one - 2 * sdf_at_position + sdf_at_y_minus_one;
	float delta_zz = sdf_at_z_plus_one - 2 * sdf_at_position + sdf_at_z_minus_one;

	// Alternative formula for 2nd-order derivatives in multiple variables.
	// See https://en.wikipedia.org/wiki/Finite_difference#Finite_difference_in_several_variables
	float delta_xy = 0.5f * (sdf_at_x_plus_one_y_plus_one - sdf_at_x_plus_one - sdf_at_y_plus_one
	                         + 2 * sdf_at_position
	                         - sdf_at_x_minus_one - sdf_at_y_minus_one + sdf_at_x_minus_one_y_minus_one);

	float delta_yz = 0.5f * (sdf_at_y_plus_one_z_plus_one - sdf_at_y_plus_one - sdf_at_z_plus_one
	                         + 2 * sdf_at_position
	                         - sdf_at_y_minus_one - sdf_at_z_minus_one + sdf_at_y_minus_one_z_minus_one);

	float delta_xz = 0.5f * (sdf_at_x_plus_one_z_plus_one - sdf_at_x_plus_one - sdf_at_z_plus_one
	                         + 2 * sdf_at_position
	                         - sdf_at_x_minus_one - sdf_at_z_minus_one + sdf_at_x_minus_one_z_minus_one);

	float vals[9] = {delta_xx, delta_xy, delta_xz,
	                 delta_xy, delta_yy, delta_yz,
	                 delta_xz, delta_yz, delta_zz};

	hessian.setValues(vals);
};


template<typename TVoxel, typename TIndexData, typename TCache>
_CPU_AND_GPU_CODE_
inline void ComputeSdf2ndDerivative_Factors(THREADPTR(Matrix3f)& hessian,
                                            const CONSTPTR(Vector3i &) position,
                                            const CONSTPTR(float)& sdf_at_position,
		//const CONSTPTR(Vector3f&) jacobianAtPosition,
		                                    const CONSTPTR(TVoxel)* voxels,
		                                    const CONSTPTR(TIndexData)* index_data,
		                                    THREADPTR(TCache)& cache) {
	int vm_index = 0;

	auto sdf_at = [&](Vector3i offset) {
#if !defined(__CUDACC__) && !defined(WITH_OPENMP)
		return TVoxel::valueToFloat(readVoxel(voxels, index_data, position + (offset), vm_index, cache).sdf);
#else //don't use cache when multithreading
		return TVoxel::valueToFloat(readVoxel(voxels, index_data, position + (offset), vm_index).sdf);
#endif
	};

	//for xx, yy, zz
	float sdf_at_x_plus_one = sdf_at(Vector3i(1, 0, 0));
	float sdf_at_y_plus_one = sdf_at(Vector3i(0, 1, 0));
	float sdf_at_z_plus_one = sdf_at(Vector3i(0, 0, 1));
	float sdf_at_x_minus_one = sdf_at(Vector3i(-1, 0, 0));
	float sdf_at_y_minus_one = sdf_at(Vector3i(0, -1, 0));
	float sdf_at_z_minus_one = sdf_at(Vector3i(0, 0, -1));

	//for xy, xz, yz
	//@formatter:off
	float sdf_at_x_plus_one_y_plus_one   = sdf_at(Vector3i( 1,  1,  0));
	float sdf_at_x_minus_one_y_minus_one = sdf_at(Vector3i(-1, -1,  0));
	float sdf_at_y_plus_one_z_plus_one   = sdf_at(Vector3i( 0,  1,  1));
	float sdf_at_y_minus_one_z_minus_one = sdf_at(Vector3i( 0, -1, -1));
	float sdf_at_x_plus_one_z_plus_one   = sdf_at(Vector3i( 1,  0,  1));
	float sdf_at_x_minus_one_z_minus_one = sdf_at(Vector3i(-1,  0, -1));
	//@formatter:on

	float delta_xx = 100.0f * (sdf_at_x_plus_one - 2 * sdf_at_position + sdf_at_x_minus_one);
	float delta_yy = 100.0f * (sdf_at_y_plus_one - 2 * sdf_at_position + sdf_at_y_minus_one);
	float delta_zz = 100.0f * (sdf_at_z_plus_one - 2 * sdf_at_position + sdf_at_z_minus_one);

	// Alternative formula for 2nd-order derivatives in multiple variables.
	// See https://en.wikipedia.org/wiki/Finite_difference#Finite_difference_in_several_variables
	float delta_xy = 50.0f * (sdf_at_x_plus_one_y_plus_one - sdf_at_x_plus_one - sdf_at_y_plus_one
	                          + 2 * sdf_at_position
	                          - sdf_at_x_minus_one - sdf_at_y_minus_one + sdf_at_x_minus_one_y_minus_one);

	float delta_yz = 50.0f * (sdf_at_y_plus_one_z_plus_one - sdf_at_y_plus_one - sdf_at_z_plus_one
	                          + 2 * sdf_at_position
	                          - sdf_at_y_minus_one - sdf_at_z_minus_one + sdf_at_y_minus_one_z_minus_one);

	float delta_xz = 50.0f * (sdf_at_x_plus_one_z_plus_one - sdf_at_x_plus_one - sdf_at_z_plus_one
	                          + 2 * sdf_at_position
	                          - sdf_at_x_minus_one - sdf_at_z_minus_one + sdf_at_x_minus_one_z_minus_one);

	float vals[9] = {delta_xx, delta_xy, delta_xz,
	                 delta_xy, delta_yy, delta_yz,
	                 delta_xz, delta_yz, delta_zz};

	hessian.setValues(vals);
};


template<typename TVoxel, typename TIndexData, typename TCache>
_CPU_AND_GPU_CODE_
inline void ComputeSdf2ndDerivative_ZeroIfTruncated(THREADPTR(Matrix3f)& hessian,
                                                    const CONSTPTR(Vector3i &) position,
                                                    const CONSTPTR(float)& sdf_at_position,
                                                    const CONSTPTR(TVoxel)* voxels,
                                                    const CONSTPTR(TIndexData)* index_data,
                                                    THREADPTR(TCache)& cache) {
	int vm_index = 0;

	auto sdf_at = [&](Vector3i offset) {
#if !defined(__CUDACC__) && !defined(WITH_OPENMP)
		return TVoxel::valueToFloat(readVoxel(voxels, index_data, position + (offset), vm_index, cache).sdf);
#else //don't use cache when multithreading
		return TVoxel::valueToFloat(readVoxel(voxels, index_data, position + (offset), vm_index).sdf);
#endif
	};

	//for xx, yy, zz
	float sdf_at_x_plus_one = sdf_at(Vector3i(1, 0, 0));
	float sdf_at_y_plus_one = sdf_at(Vector3i(0, 1, 0));
	float sdf_at_z_plus_one = sdf_at(Vector3i(0, 0, 1));
	float sdf_at_x_minus_one = sdf_at(Vector3i(-1, 0, 0));
	float sdf_at_y_minus_one = sdf_at(Vector3i(0, -1, 0));
	float sdf_at_z_minus_one = sdf_at(Vector3i(0, 0, -1));

	bool x_plus_one_nontruncated = 1.0 - abs(sdf_at_x_plus_one) > 1e-5;
	bool x_minus_one_nontruncated = 1.0 - abs(sdf_at_x_minus_one) > 1e-5;
	bool x_neighbors_nontruncated = x_plus_one_nontruncated && x_minus_one_nontruncated;
	bool y_plus_one_nontruncated = 1.0 - abs(sdf_at_y_plus_one) > 1e-5;
	bool y_minus_one_nontruncated = 1.0 - abs(sdf_at_y_minus_one) > 1e-5;
	bool y_neighbors_nontruncated = y_plus_one_nontruncated && y_minus_one_nontruncated;
	bool z_plus_one_nontruncated = 1.0 - abs(sdf_at_z_plus_one) > 1e-5;
	bool z_minus_one_nontruncated = 1.0 - abs(sdf_at_z_minus_one) > 1e-5;
	bool z_neighbors_nontruncated = z_plus_one_nontruncated && z_minus_one_nontruncated;

	//for xy, xz, yz
	//@formatter:off
	float sdf_at_x_plus_one_y_plus_one   = sdf_at(Vector3i( 1,  1,  0));
	float sdf_at_x_minus_one_y_minus_one = sdf_at(Vector3i(-1, -1,  0));
	float sdf_at_y_plus_one_z_plus_one   = sdf_at(Vector3i( 0,  1,  1));
	float sdf_at_y_minus_one_z_minus_one = sdf_at(Vector3i( 0, -1, -1));
	float sdf_at_x_plus_one_z_plus_one   = sdf_at(Vector3i( 1,  0,  1));
	float sdf_at_x_minus_one_z_minus_one = sdf_at(Vector3i(-1,  0, -1));
	//@formatter:on

	bool x_plus_one_y_plus_one_nontruncated = 1.0 - abs(sdf_at_x_plus_one_y_plus_one) > 1e-5;
	bool x_minus_one_y_minus_one_nontruncated = 1.0 - abs(sdf_at_x_minus_one_y_minus_one) > 1e-5;
	bool xy_neighbors_nontruncated = x_plus_one_y_plus_one_nontruncated && x_minus_one_y_minus_one_nontruncated;

	bool y_plus_one_z_plus_one_nontruncated = 1.0 - abs(sdf_at_y_plus_one_z_plus_one) > 1e-5;
	bool y_minus_one_z_minus_one_nontruncated = 1.0 - abs(sdf_at_y_minus_one_z_minus_one) > 1e-5;
	bool yz_neighbors_nontruncated = y_plus_one_z_plus_one_nontruncated && y_minus_one_z_minus_one_nontruncated;

	bool x_plus_one_z_plus_one_nontruncated = 1.0 - abs(sdf_at_x_plus_one_z_plus_one) > 1e-5;
	bool x_minus_one_z_minus_one_nontruncated = 1.0 - abs(sdf_at_x_minus_one_z_minus_one) > 1e-5;
	bool xz_neighbors_nontruncated = x_plus_one_z_plus_one_nontruncated && x_minus_one_z_minus_one_nontruncated;

	float delta_xx = x_neighbors_nontruncated * sdf_at_x_plus_one - 2 * sdf_at_position + sdf_at_x_minus_one;
	float delta_yy = y_neighbors_nontruncated * sdf_at_y_plus_one - 2 * sdf_at_position + sdf_at_y_minus_one;
	float delta_zz = z_neighbors_nontruncated * sdf_at_z_plus_one - 2 * sdf_at_position + sdf_at_z_minus_one;

	// Alternative formula for 2nd-order derivatives in multiple variables.
	// See https://en.wikipedia.org/wiki/Finite_difference#Finite_difference_in_several_variables
	float delta_xy = (x_neighbors_nontruncated && y_neighbors_nontruncated && xy_neighbors_nontruncated) *
	                 0.5f * (sdf_at_x_plus_one_y_plus_one - sdf_at_x_plus_one - sdf_at_y_plus_one
	                         + 2 * sdf_at_position
	                         - sdf_at_x_minus_one - sdf_at_y_minus_one + sdf_at_x_minus_one_y_minus_one);

	float delta_yz = (y_neighbors_nontruncated && z_neighbors_nontruncated && yz_neighbors_nontruncated) *
	                 0.5f * (sdf_at_y_plus_one_z_plus_one - sdf_at_y_plus_one - sdf_at_z_plus_one
	                         + 2 * sdf_at_position
	                         - sdf_at_y_minus_one - sdf_at_z_minus_one + sdf_at_y_minus_one_z_minus_one);

	float delta_xz = (x_neighbors_nontruncated && z_neighbors_nontruncated && xz_neighbors_nontruncated) *
	                 0.5f * (sdf_at_x_plus_one_z_plus_one - sdf_at_x_plus_one - sdf_at_z_plus_one
	                         + 2 * sdf_at_position
	                         - sdf_at_x_minus_one - sdf_at_z_minus_one + sdf_at_x_minus_one_z_minus_one);

	float vals[9] = {delta_xx, delta_xy, delta_xz,
	                 delta_xy, delta_yy, delta_yz,
	                 delta_xz, delta_yz, delta_zz};

	hessian.setValues(vals);
};


//endregion

// region ================================ WARP LAPLACIAN, JACOBIAN, DIVERGENCE (SMOOTHING/TIKHONOV/KILLING TERMS) ====================================
_CPU_AND_GPU_CODE_
inline void ComputeVectorLaplacian(THREADPTR(Vector3f)& laplacian,
                                   const CONSTPTR(Vector3f)& local_vector,
                                   const CONSTPTR(Vector3f*) neighbor_vecotrs) {//in, x6-9
	//    0        1        2          3         4         5
	//(-1,0,0) (0,-1,0) (0,0,-1)   (1, 0, 0) (0, 1, 0) (0, 0, 1)
	laplacian = Vector3f(0.0f);
	//3D discrete Laplacian filter based on https://en.wikipedia.org/wiki/Discrete_Laplace_operator
	for (int i_six_connected_neighbor = 0; i_six_connected_neighbor < 6; i_six_connected_neighbor++) {
		laplacian += neighbor_vecotrs[i_six_connected_neighbor];
	}
	laplacian -= 6 * local_vector;
}

_CPU_AND_GPU_CODE_
inline void ComputeVectorJacobian(THREADPTR(Matrix3f)& jacobian,
                                  const CONSTPTR(Vector3f*) neighbor_vectors //in, x6
) {
	const Vector3f& vector_at_x_minus_one = neighbor_vectors[0];
	const Vector3f& vector_at_y_minus_one = neighbor_vectors[1];
	const Vector3f& vector_at_z_minus_one = neighbor_vectors[2];

	const Vector3f& vector_at_x_plus_one = neighbor_vectors[3];
	const Vector3f& vector_at_y_plus_one = neighbor_vectors[4];
	const Vector3f& vector_at_z_plus_one = neighbor_vectors[5];

	jacobian.setColumn(0, 0.5 * (vector_at_x_plus_one - vector_at_x_minus_one));//1st derivative in x
	jacobian.setColumn(1, 0.5 * (vector_at_y_plus_one - vector_at_y_minus_one));//1st derivative in y
	jacobian.setColumn(2, 0.5 * (vector_at_z_plus_one - vector_at_z_minus_one));//1st derivative in z
}


/**
 * \brief Computes the jacobian and hessian approximation for the warp vectors themselves in a given neighborhood of a
 * specific (local) voxel
 * \details
 * Index in \p neighbor_vectors array on top / voxel coordinate relative to local voxel below:
 *     0        1        2          3         4         5           6         7         8            9          10          11
 * (-1,0,0) (0,-1,0) (0,0,-1)   (1, 0, 0) (0, 1, 0) (0, 0, 1)   (1, 1, 0) (0, 1, 1) (1, 0, 1)   (-1, -1, 0) (0, -1,- 1) (-1, 0,-1)
 *
 *  Spatial representation:
 *          *** XY plane ***                  ***   XZ plane ***                 ***   YZ plane ***
 *               4 (y+1)                            5 (z+1)                            5 (z+1)
 *      0 (x-1)  <local>   3 (x+1)        0 (x-1)   <local>   3 (x+1)        1 (y-1)   <local>   4 (y+1)
 *               1 (y-1)                            2 (z-1)                            2 (z-1)
 *
 * \param[out] jacobian - approximation of the warp's Jacobian matrix, i.e. gradient (row) estimates for each vector component (column)
 * \param[out] hessian - approximation of the warp's Hessian matrix, i.e. one 2D 3x3 matrix of second derivative estimates for each vector component.
 * \param[in] local_vector - vectors from local/current voxel
 * \param[in] neighbor_vectors - vectors from neighboring voxels (pointer to the first element of a consecutive in-memory array of size 12,
 * see detailed description.)
 */
_CPU_AND_GPU_CODE_
inline void ComputeVectorLaplacianAndDivergenceDerivative(THREADPTR(Vector3f)& divergence_derivative,
                                                          THREADPTR(Vector3f)& laplacian,
                                                          const CONSTPTR(Vector3f)& local_vector,
                                                          const CONSTPTR(Vector3f*) neighbor_vectors //in, x12
) {
	const Vector3f& vector_at_x_minus_one = neighbor_vectors[0];
	const Vector3f& vector_at_y_minus_one = neighbor_vectors[1];
	const Vector3f& vector_at_z_minus_one = neighbor_vectors[2];

	const Vector3f& vector_at_x_plus_one = neighbor_vectors[3];
	const Vector3f& vector_at_y_plus_one = neighbor_vectors[4];
	const Vector3f& vector_at_z_plus_one = neighbor_vectors[5];

	const Vector3f& vector_at_x_plus_one_y_plus_one = neighbor_vectors[6];
	const Vector3f& vector_at_y_plus_one_z_plus_one = neighbor_vectors[7];
	const Vector3f& vector_at_x_plus_one_z_plus_one = neighbor_vectors[8];

	const Vector3f& vector_at_x_minus_one_y_minus_one = neighbor_vectors[9];
	const Vector3f& vector_at_y_minus_one_z_minus_one = neighbor_vectors[10];
	const Vector3f& vector_at_x_minus_one_z_minus_one = neighbor_vectors[11];

	Vector3f xx = vector_at_x_plus_one - 2.0f * local_vector + vector_at_x_minus_one;
	Vector3f yy = vector_at_y_plus_one - 2.0f * local_vector + vector_at_y_minus_one;
	Vector3f zz = vector_at_z_plus_one - 2.0f * local_vector + vector_at_z_minus_one;

	//TODO: optimize
	Vector3f xy = vector_at_x_plus_one_y_plus_one - vector_at_x_plus_one - vector_at_y_plus_one +
	              2.0f * local_vector -
	              vector_at_x_minus_one - vector_at_y_minus_one + vector_at_x_minus_one_y_minus_one;
	Vector3f yz = vector_at_y_plus_one_z_plus_one - vector_at_y_plus_one - vector_at_z_plus_one +
	              2.0f * local_vector -
	              vector_at_y_minus_one - vector_at_z_minus_one + vector_at_y_minus_one_z_minus_one;
	Vector3f xz = vector_at_x_plus_one_z_plus_one - vector_at_x_plus_one - vector_at_z_plus_one +
	              2.0f * local_vector -
	              vector_at_x_minus_one - vector_at_z_minus_one + vector_at_x_minus_one_z_minus_one;

	laplacian = xx + yy + zz;
	divergence_derivative = {
			xx.u + xy.v + xz.w,
			xy.u + yy.v + yz.w,
			xz.u + yz.v + zz.w
	};

};

/**
 * \brief Computes the jacobian and hessian approximation for the warp vectors themselves in a given neighborhood of a
 * specific (local) voxel
 * \details
 * Index in \p neighbor_vectors array on top / voxel coordinate relative to local voxel below:
 *     0        1        2          3         4         5           6         7         8            9          10          11
 * (-1,0,0) (0,-1,0) (0,0,-1)   (1, 0, 0) (0, 1, 0) (0, 0, 1)   (1, 1, 0) (0, 1, 1) (1, 0, 1)   (-1, -1, 0) (0, -1,- 1) (-1, 0,-1)
 *
 *  Spatial representation:
 *          *** XY plane ***                  ***   XZ plane ***                 ***   YZ plane ***
 *               4 (y+1)                            5 (z+1)                            5 (z+1)
 *      0 (x-1)  <local>   3 (x+1)        0 (x-1)   <local>   3 (x+1)        1 (y-1)   <local>   4 (y+1)
 *               1 (y-1)                            2 (z-1)                            2 (z-1)
 *
 * \param[out] jacobian - approximation of the warp's Jacobian matrix, i.e. gradient (row) estimates for each vector component (column)
 * \param[out] hessian - approximation of the warp's Hessian matrix, i.e. one 2D 3x3 matrix of second derivative estimates for each vector component.
 * \param[in] local_vector - vectors from local/current voxel
 * \param[in] neighbor_vectors - vectors from neighboring voxels (pointer to the first element of a consecutive in-memory array of size 12,
 * see detailed description.)
 */
_CPU_AND_GPU_CODE_
inline void ComputePerVoxelWarpJacobianAndHessian(THREADPTR(Matrix3f)& jacobian,
                                                  THREADPTR(WarpHessian)& hessian,
                                                  const CONSTPTR(Vector3f)& local_vector,
                                                  const CONSTPTR(Vector3f*) neighbor_vectors //in, x12
) {
	const Vector3f& vector_at_x_minus_one = neighbor_vectors[0];
	const Vector3f& vector_at_y_minus_one = neighbor_vectors[1];
	const Vector3f& vector_at_z_minus_one = neighbor_vectors[2];

	const Vector3f& vector_at_x_plus_one = neighbor_vectors[3];
	const Vector3f& vector_at_y_plus_one = neighbor_vectors[4];
	const Vector3f& vector_at_z_plus_one = neighbor_vectors[5];

	const Vector3f& vector_at_x_plus_one_y_plus_one = neighbor_vectors[6];
	const Vector3f& vector_at_y_plus_one_z_plus_one = neighbor_vectors[7];
	const Vector3f& vector_at_x_plus_one_z_plus_one = neighbor_vectors[8];

	const Vector3f& vector_at_x_minus_one_y_minus_one = neighbor_vectors[9];
	const Vector3f& vector_at_y_minus_one_z_minus_one = neighbor_vectors[10];
	const Vector3f& vector_at_x_minus_one_z_minus_one = neighbor_vectors[11];

	jacobian.setColumn(0, 0.5 * (vector_at_x_plus_one - vector_at_x_minus_one));//1st derivative in x
	jacobian.setColumn(1, 0.5 * (vector_at_y_plus_one - vector_at_y_minus_one));//1st derivative in y
	jacobian.setColumn(2, 0.5 * (vector_at_z_plus_one - vector_at_z_minus_one));//1st derivative in z

	hessian.xx = vector_at_x_plus_one - 2.0f * local_vector + vector_at_x_minus_one;
	hessian.yy = vector_at_y_plus_one - 2.0f * local_vector + vector_at_y_minus_one;
	hessian.zz = vector_at_z_plus_one - 2.0f * local_vector + vector_at_z_minus_one;

	hessian.xy = vector_at_x_plus_one_y_plus_one - vector_at_x_plus_one - vector_at_y_plus_one +
	             2.0f * local_vector -
	             vector_at_x_minus_one - vector_at_y_minus_one + vector_at_x_minus_one_y_minus_one;
	hessian.yz = vector_at_y_plus_one_z_plus_one - vector_at_y_plus_one - vector_at_z_plus_one +
	             2.0f * local_vector -
	             vector_at_y_minus_one - vector_at_z_minus_one + vector_at_y_minus_one_z_minus_one;
	hessian.xz = vector_at_x_plus_one_z_plus_one - vector_at_x_plus_one - vector_at_z_plus_one +
	             2.0f * local_vector -
	             vector_at_x_minus_one - vector_at_z_minus_one + vector_at_x_minus_one_z_minus_one;

};
//Computes the jacobian and hessian approximation for the warp vectors themselves in a given neighborhood
_CPU_AND_GPU_CODE_
inline void ComputePerVoxelWarpJacobianAndHessian_Old(const CONSTPTR(Vector3f)& voxel_warp,
                                                      const CONSTPTR(Vector3f*) neighbor_warps, //in, x9
                                                      THREADPTR(Matrix3f)& jacobian, //out
                                                      THREADPTR(Matrix3f)* hessian //out, x3
) {
	//    0        1        2          3         4         5           6         7         8
	//(-1,0,0) (0,-1,0) (0,0,-1)   (1, 0, 0) (0, 1, 0) (0, 0, 1)   (1, 1, 0) (0, 1, 1) (1, 0, 1)

	// |u_x, u_y, u_z|       |m00, m10, m20|
	// |v_x, v_y, v_z|       |m01, m11, m21|
	// |w_x, w_y, w_z|       |m02, m12, m22|
	jacobian.setColumn(0, neighbor_warps[3] - voxel_warp);//1st derivative in x
	jacobian.setColumn(1, neighbor_warps[4] - voxel_warp);//1st derivative in y
	jacobian.setColumn(2, neighbor_warps[5] - voxel_warp);//1st derivative in z

	Matrix3f backward_differences;
	// |u_x, u_y, u_z|
	// |v_x, v_y, v_z|
	// |w_x, w_y, w_z|
	backward_differences.setColumn(0, voxel_warp - neighbor_warps[0]);//1st derivative in x
	backward_differences.setColumn(1, voxel_warp - neighbor_warps[1]);//1st derivative in y
	backward_differences.setColumn(2, voxel_warp - neighbor_warps[2]);//1st derivative in z

	//second derivatives in same direction
	// |u_xx, u_yy, u_zz|       |m00, m10, m20|
	// |v_xx, v_yy, v_zz|       |m01, m11, m21|
	// |w_xx, w_yy, w_zz|       |m02, m12, m22|
	Matrix3f dd_XX_YY_ZZ = jacobian - backward_differences;

	Matrix3f neighbor_differences;
	neighbor_differences.setColumn(0, neighbor_warps[6] - neighbor_warps[4]);//(0,1,0)->(1,1,0)
	neighbor_differences.setColumn(1, neighbor_warps[7] - neighbor_warps[5]);//(0,0,1)->(0,1,1)
	neighbor_differences.setColumn(2, neighbor_warps[8] - neighbor_warps[3]);//(1,0,0)->(1,0,1)

	//second derivatives in different directions
	// |u_xy, u_yz, u_zx|      |m00, m10, m20|
	// |v_xy, v_yz, v_zx|      |m01, m11, m21|
	// |w_xy, w_yz, w_zx|      |m02, m12, m22|
	Matrix3f dd_XY_YZ_ZX = neighbor_differences - jacobian;

	//NOTE: Hessian matrices are symmetric in this case
	// |0, 3, 6|     |m00, m10, m20|      |u_xx, u_xy, u_xz|
	// |1, 4, 7|     |m01, m11, m21|      |u_xy, u_yy, u_yz|
	// |2, 5, 8|     |m02, m12, m22|      |u_xz, u_yz, u_zz|
	float valsU[9] = {dd_XX_YY_ZZ.m00, dd_XY_YZ_ZX.m00, dd_XY_YZ_ZX.m20,
	                  dd_XY_YZ_ZX.m00, dd_XX_YY_ZZ.m10, dd_XY_YZ_ZX.m10,
	                  dd_XY_YZ_ZX.m20, dd_XY_YZ_ZX.m10, dd_XX_YY_ZZ.m20};
	hessian[0].setValues(valsU);

	// |0, 3, 6|     |m00, m10, m20|      |v_xx, v_xy, v_xz|
	// |1, 4, 7|     |m01, m11, m21|      |v_xy, v_yy, v_yz|
	// |2, 5, 8|     |m02, m12, m22|      |v_xz, v_yz, v_zz|
	float valsV[9] = {dd_XX_YY_ZZ.m01, dd_XY_YZ_ZX.m01, dd_XY_YZ_ZX.m21,
	                  dd_XY_YZ_ZX.m01, dd_XX_YY_ZZ.m11, dd_XY_YZ_ZX.m11,
	                  dd_XY_YZ_ZX.m21, dd_XY_YZ_ZX.m11, dd_XX_YY_ZZ.m21};
	hessian[1].setValues(valsV);

	// |0, 3, 6|     |m00, m10, m20|      |w_xx, w_xy, w_xz|
	// |1, 4, 7|     |m01, m11, m21|      |w_xy, w_yy, w_yz|
	// |2, 5, 8|     |m02, m12, m22|      |w_xz, w_yz, w_zz|
	float valsW[9] = {dd_XX_YY_ZZ.m02, dd_XY_YZ_ZX.m02, dd_XY_YZ_ZX.m22,
	                  dd_XY_YZ_ZX.m02, dd_XX_YY_ZZ.m12, dd_XY_YZ_ZX.m12,
	                  dd_XY_YZ_ZX.m22, dd_XY_YZ_ZX.m12, dd_XX_YY_ZZ.m22};
	hessian[2].setValues(valsW);

};

// endregion

} // namespace ITMLib