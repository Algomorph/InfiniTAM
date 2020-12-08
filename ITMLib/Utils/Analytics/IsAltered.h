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
#include "../Enums/VoxelFlags.h"
#include "../Math.h"
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
		return voxel.sdf != TVoxel::SDF_initialValue();
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


template<typename TVoxel>
struct IsAlteredUtility<false, false, false, true, TVoxel> {
	_CPU_AND_GPU_CODE_
	static inline
	bool evaluate(const TVoxel& voxel) {
		return voxel.warp_update != Vector3f(0.0f);
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
		printf("%sVoxel altered at position %d, %d, %d [Block, if relevant: %d, %d, %d].\n",
				message, position.x, position.y, position.z,
				position.x / VOXEL_BLOCK_SIZE, position.y / VOXEL_BLOCK_SIZE, position.z / VOXEL_BLOCK_SIZE);
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
bool isAltered_VerbosePositionHash(TVoxel& voxel, Vector3i position, int hash_code, Vector3s block_position,
                                   const char* message = "") {

	bool altered = IsAlteredUtility<TVoxel::hasSDFInformation, TVoxel::hasSemanticInformation, TVoxel::hasFramewiseWarp,
			TVoxel::hasWarpUpdate, TVoxel>::evaluate(voxel);

	if (altered) {
		printf("%sVoxel altered at position %d, %d, %d (hash %d at %d, %d, %d).\n", message,
		       position.x, position.y, position.z, hash_code, block_position.x, block_position.y, block_position.z);
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
struct IsAlteredStaticFunctor {
	_CPU_AND_GPU_CODE_
	static bool isSatisfiedBy(const TVoxel& voxel) {
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
	bool operator()(const TVoxel& voxel, const Vector3i& position, int hash_code, Vector3s block_position) {
		return isAltered_VerbosePositionHash(voxel, position, hash_code, block_position);
	}
};


template<typename TVoxel>
inline static bool
isVoxelBlockAltered(TVoxel* voxel_block, bool verbose = false,
                    std::string message = "",
                    Vector3s block_spatial_position = Vector3s((short) 0),
                    int hash_code = 0) {
	if(verbose){
		for (int linear_index_in_block = 0; linear_index_in_block < VOXEL_BLOCK_SIZE3; linear_index_in_block++) {
			TVoxel& voxel = voxel_block[linear_index_in_block];
			Vector3i voxel_position = ComputePositionVectorFromLinearIndex_VoxelBlockHash(block_spatial_position,
			                                                                              linear_index_in_block);
			if (isAltered_VerbosePositionHash(voxel, voxel_position, hash_code, block_spatial_position, message.c_str())) {
				return true;
			}
		}
	}else{
		for (int linear_index_in_block = 0; linear_index_in_block < VOXEL_BLOCK_SIZE3; linear_index_in_block++) {
			TVoxel& voxel = voxel_block[linear_index_in_block];
			if (isAltered(voxel)) {
				return true;
			}
		}
	}
	return false;
}

template<typename TVoxel, typename TOneVoxelPredicate>
inline static bool
isVoxelBlockAlteredPredicate(TVoxel* voxel_block,
                             TOneVoxelPredicate&& one_voxel_predicate, bool verbose = false,
                             std::string message = "",
                             Vector3s block_spatial_position = Vector3s((short) 0),
                             int hash_code = 0) {
	if (verbose) {
		for (int linear_index_in_block = 0; linear_index_in_block < VOXEL_BLOCK_SIZE3; linear_index_in_block++) {
			TVoxel& voxel = voxel_block[linear_index_in_block];
			Vector3i voxel_position = ComputePositionVectorFromLinearIndex_VoxelBlockHash(block_spatial_position,
			                                                                              linear_index_in_block);
			if (std::forward<TOneVoxelPredicate>(one_voxel_predicate)(voxel) &&
			    isAltered_VerbosePositionHash(voxel, voxel_position, hash_code, block_spatial_position, message.c_str())) {
				return true;
			}
		}
	} else {
		for (int linear_index_in_block = 0; linear_index_in_block < VOXEL_BLOCK_SIZE3; linear_index_in_block++) {
			TVoxel& voxel = voxel_block[linear_index_in_block];
			if (std::forward<TOneVoxelPredicate>(one_voxel_predicate)(voxel) && isAltered(voxel)) {
				return true;
			}
		}
	}
	return false;
}
