//  ================================================================
//  Created by Gregory Kramida on 9/3/19.
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

#include "../../../ORUtils/PlatformIndependence.h"
#include "../ITMVoxelFlags.h"
#include "../ITMMath.h"
#include "../Geometry/SpatialIndexConversions.h"

// region =========================== FUNCTIONS TO DETERMINE WHETHER A VOXEL HAS BEEN ALTERED FROM DEFAULT =============

template<bool hasSDFInformation,
		bool hasSemanticInformation,
		bool hasFramewiseWarpInformation,
		bool hasWarpUpdateInformation,
		typename TVoxel>
struct IsAlteredUtility;


template<typename TVoxel>
struct IsAlteredUtility<true, false, false, false, TVoxel> {
	_CPU_AND_GPU_CODE_
	static inline
	bool evaluate(const TVoxel& voxel) {
		return voxel.w_depth != 0;
	}
};

template<typename TVoxel>
struct IsAlteredUtility<true, true, false, false, TVoxel> {
	_CPU_AND_GPU_CODE_
	static inline
	bool evaluate(const TVoxel& voxel) {
		return voxel.flags != ITMLib::VOXEL_UNKNOWN;
	}
};

template<typename TVoxel>
struct IsAlteredUtility<false, true, false, false, TVoxel> {
	_CPU_AND_GPU_CODE_
	static inline
	bool evaluate(const TVoxel& voxel) {
		return voxel.flags != ITMLib::VOXEL_UNKNOWN;
	}
};

template<typename TVoxel>
struct IsAlteredUtility<false, false, true, true, TVoxel> {
	_CPU_AND_GPU_CODE_
	static inline
	bool evaluate(const TVoxel& voxel) {
		return voxel.framewise_warp != Vector3f(0.0f) || voxel.warp_update != Vector3f(0.0f);
	}
};
// endregion

/**
 * \brief Tries to determine whether the voxel been altered from default
 * \tparam TVoxel voxel type
 * \param voxel the voxel to evaluate
 * \return true if the voxel has been altered for certain, false if not (or voxel seems to have default value)
 **/
template<typename TVoxel>
_CPU_AND_GPU_CODE_
inline
bool isAltered(TVoxel& voxel) {
	return IsAlteredUtility<TVoxel::hasSDFInformation, TVoxel::hasSemanticInformation, TVoxel::hasFramewiseWarp,
			TVoxel::hasWarpUpdate, TVoxel>::evaluate(voxel);
}

/**
 * \brief Tries to determine whether the voxel been altered from default; if it seems to have been altered,
 * reports the position
 * \tparam TVoxel voxel type
 * \param voxel the voxel to evaluate
 * \param position the position (presumably, of the voxel that's passed in)
 * \return true if the voxel has been altered for certain, false if not (or voxel seems to have default value)
 **/
template<typename TVoxel>
_CPU_AND_GPU_CODE_
inline
bool isAltered_VerbosePosition(TVoxel& voxel, Vector3i position, const char* message = "") {

	bool altered = IsAlteredUtility<TVoxel::hasSDFInformation, TVoxel::hasSemanticInformation, TVoxel::hasFramewiseWarp,
			TVoxel::hasWarpUpdate, TVoxel>::evaluate(voxel);
	if (altered) {
		printf("%sVoxel altered at position %d, %d, %d.\n", message, position.x, position.y, position.z);
	}

	return altered;
}

/**
 * \brief Tries to determine whether the voxel been altered from default; if it seems to have been altered,
 * reports the position
 * \tparam TVoxel voxel type
 * \param voxel the voxel to evaluate
 * \param position the position (presumably, of the voxel that's passed in)
 * \return true if the voxel has been altered for certain, false if not (or voxel seems to have default value)
 **/
template<typename TVoxel>
_CPU_AND_GPU_CODE_
inline
bool isAltered_VerbosePositionHash(TVoxel& voxel, Vector3i position, int hashCode, Vector3s blockPosition,
                                   const char* message = "") {

	bool altered = IsAlteredUtility<TVoxel::hasSDFInformation, TVoxel::hasSemanticInformation, TVoxel::hasFramewiseWarp,
			TVoxel::hasWarpUpdate, TVoxel>::evaluate(voxel);

	if (altered) {
		printf("%sVoxel altered at position %d, %d, %d (hash %d at %d, %d, %d).\n", message,
		       position.x, position.y, position.z, hashCode, blockPosition.x, blockPosition.y, blockPosition.z);
	}

	return altered;
}

template<typename TVoxel>
struct IsAlteredFunctor {
	_CPU_AND_GPU_CODE_
	bool operator()(const TVoxel& voxel) {
		return isAltered(voxel);
	}

};

template<typename TVoxel>
struct IsAlteredPositionFunctor {
	_CPU_AND_GPU_CODE_
	bool operator()(const TVoxel& voxel, const Vector3i& position) {
		return isAltered_VerbosePosition(voxel, position);
	}
};

template<typename TVoxel>
struct IsAlteredPositionHashFunctor {
	_CPU_AND_GPU_CODE_
	bool operator()(const TVoxel& voxel, const Vector3i& position, int hashCode, Vector3s blockPosition) {
		return isAltered_VerbosePositionHash(voxel, position, hashCode, blockPosition);
	}
};


template<typename TVoxel>
inline static bool
isVoxelBlockAltered(TVoxel* voxelBlock, bool verbose = false,
                    std::string message = "",
                    Vector3s blockSpatialPosition = Vector3s((short) 0),
                    int hashCode = 0) {
	if(verbose){
		for (int linearIndexInBlock = 0; linearIndexInBlock < VOXEL_BLOCK_SIZE3; linearIndexInBlock++) {
			TVoxel& voxel = voxelBlock[linearIndexInBlock];
			Vector3i voxelPosition = ComputePositionVectorFromLinearIndex_VoxelBlockHash(blockSpatialPosition,
			                                                                             linearIndexInBlock);
			if (isAltered_VerbosePositionHash(voxel, voxelPosition, hashCode, blockSpatialPosition, message.c_str())) {
				return true;
			}
		}
	}else{
		for (int linearIndexInBlock = 0; linearIndexInBlock < VOXEL_BLOCK_SIZE3; linearIndexInBlock++) {
			TVoxel& voxel = voxelBlock[linearIndexInBlock];
			if (isAltered(voxel)) {
				return true;
			}
		}
	}

	return false;
}

template<typename TVoxel, typename TOneVoxelPredicate>
inline static bool
isVoxelBlockAlteredPredicate(TVoxel* voxelBlock,
                             TOneVoxelPredicate&& oneVoxelPredicate, bool verbose = false,
                             std::string message = "",
                             Vector3s blockSpatialPosition = Vector3s((short) 0),
                             int hashCode = 0) {
	if (verbose) {
		for (int linearIndexInBlock = 0; linearIndexInBlock < VOXEL_BLOCK_SIZE3; linearIndexInBlock++) {
			TVoxel& voxel = voxelBlock[linearIndexInBlock];
			Vector3i voxelPosition = ComputePositionVectorFromLinearIndex_VoxelBlockHash(blockSpatialPosition,
			                                                                             linearIndexInBlock);
			if (std::forward<TOneVoxelPredicate>(oneVoxelPredicate)(voxel) &&
			    isAltered_VerbosePositionHash(voxel, voxelPosition, hashCode, blockSpatialPosition, message.c_str())) {
				return true;
			}
		}
	} else {
		for (int linearIndexInBlock = 0; linearIndexInBlock < VOXEL_BLOCK_SIZE3; linearIndexInBlock++) {
			TVoxel& voxel = voxelBlock[linearIndexInBlock];
			if (std::forward<TOneVoxelPredicate>(oneVoxelPredicate)(voxel) && isAltered(voxel)) {
				return true;
			}
		}
	}
	return false;
}


