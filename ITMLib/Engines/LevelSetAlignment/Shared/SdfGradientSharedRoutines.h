//  ================================================================
//  Created by Gregory Kramida (https://github.com/Algomorph) on 9/10/20.
//  Copyright (c) 2020 Gregory Kramida
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
namespace ITMLib {
//region ================================= SDF GRADIENT (USED FOR BOTH DATA AND LEVEL SET ENERGY GRADIENTS) ==========================================
template<typename TVoxel, typename TIndexData, typename TCache>
_CPU_AND_GPU_CODE_
inline void ComputeSdfGradient_ForwardDifferences(THREADPTR(Vector3f)& gradient,
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
inline void ComputeSdfGradient_ForwardDifferences(THREADPTR(Vector3f)& gradient,
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
	ComputeSdfGradient_ForwardDifferences(gradient, position, sdf_at_position, voxels, index_data, cache);

};

template<typename TVoxel, typename TIndexData, typename TCache>
_CPU_AND_GPU_CODE_
inline void ComputeSdfGradient_CentralDifferences(THREADPTR(Vector3f)& gradient,
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
inline void ComputeSdfGradient_CentralDifferences_ZeroIfTruncated(THREADPTR(Vector3f)& gradient,
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
inline void ComputeSdfGradient_ChooseStrategyOnTruncation(THREADPTR(Vector3f)& gradient,
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
};

//// THE FUNCTION ACTUALLY USED EVERYWHERE
template<typename TVoxel, typename TIndexData, typename TCache>
_CPU_AND_GPU_CODE_
inline void ComputeSdfGradient(THREADPTR(Vector3f)& gradient,
                               const CONSTPTR(Vector3i)& voxel_position,
                               const CONSTPTR(float)& sdf_at_position,
                               const CONSTPTR(TVoxel)* voxels,
                               const CONSTPTR(TIndexData)* index_data,
                               THREADPTR(TCache)& cache) {
	//      Different strategies for computing gradient of live TSDF.
	//      Observation: the only strategy that currently yields a gradual shrinking of overall energy and gradient lengths is "ZERO_IF_TRUNCATED"
//#define LIVE_GRADIENT_STRATEGY_NAIVE
#define LIVE_GRADIENT_STRATEGY_ZERO_IF_TRUNCATED
//#define LIVE_GRADIENT_STRATEGY_CHOOSE_IF_TRUNCATED

#if defined(LIVE_GRADIENT_STRATEGY_NAIVE)
	ComputeSdfGradient_CentralDifferences(live_sdf_gradient, voxel_position, live_voxels, live_index_data, live_cache);
#elif defined(LIVE_GRADIENT_STRATEGY_ZERO_IF_TRUNCATED)
	ComputeSdfGradient_CentralDifferences_ZeroIfTruncated(gradient, voxel_position, voxels, index_data, cache);
#elif defined(LIVE_GRADIENT_STRATEGY_CHOOSE_IF_TRUNCATED)
	ComputeSdfGradient_ChooseStrategyOnTruncation(live_sdf_gradient, voxel_position, live_voxels, live_index_data, live_cache);
#endif
}

// endregion
} // namespace ITMLib