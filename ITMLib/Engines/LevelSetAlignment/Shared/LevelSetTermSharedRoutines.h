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


// region ================================= SDF 2ND DERIVATIVE / HESSIAN ==========================================================

template<typename TVoxel, typename TIndexData, typename TCache>
_CPU_AND_GPU_CODE_
inline void ComputeSdfHessian_Basic(THREADPTR(Matrix3f)& hessian,
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
inline void ComputeSdfHessian_Factors(THREADPTR(Matrix3f)& hessian,
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
inline void ComputeSdfHessian_ZeroIfTruncated(THREADPTR(Matrix3f)& hessian,
                                              const CONSTPTR(Vector3i &) position,
                                              const CONSTPTR(float)& sdf_at_position,
                                              const CONSTPTR(TVoxel)* voxels,
                                              const CONSTPTR(TIndexData)* index_data,
                                              THREADPTR(TCache)& cache) {
	int vm_index = 0;
	auto sdf_at = [&](Vector3i offset, bool& nontruncated) {
#if !defined(__CUDACC__) && !defined(WITH_OPENMP)
		TVoxel voxel = readVoxel(voxels, index_data, position + (offset), vm_index);
		nontruncated &= voxel.flags == ITMLib::VOXEL_NONTRUNCATED;
		return TVoxel::valueToFloat(voxel.sdf);
#else
		TVoxel voxel = readVoxel(voxels, index_data, position + (offset), vm_index);
		nontruncated &= voxel.flags == ITMLib::VOXEL_NONTRUNCATED;
		return TVoxel::valueToFloat(voxel.sdf);
#endif
	};

	bool x_neighbors_nontruncated = true;
	float sdf_at_x_plus_one = sdf_at(Vector3i(1, 0, 0), x_neighbors_nontruncated);
	float sdf_at_x_minus_one = sdf_at(Vector3i(-1, 0, 0), x_neighbors_nontruncated);

	bool y_neighbors_nontruncated = true;
	float sdf_at_y_plus_one = sdf_at(Vector3i(0, 1, 0), y_neighbors_nontruncated);
	float sdf_at_y_minus_one = sdf_at(Vector3i(0, -1, 0), y_neighbors_nontruncated);

	bool z_neighbors_nontruncated = true;
	float sdf_at_z_plus_one = sdf_at(Vector3i(0, 0, 1), z_neighbors_nontruncated);
	float sdf_at_z_minus_one = sdf_at(Vector3i(0, 0, -1), z_neighbors_nontruncated);

	//for xy, xz, yz
	//@formatter:off
	bool xy_neighbors_nontruncated = true;
	float sdf_at_x_plus_one_y_plus_one   = sdf_at(Vector3i( 1,  1,  0), xy_neighbors_nontruncated);
	float sdf_at_x_minus_one_y_minus_one = sdf_at(Vector3i(-1, -1,  0), xy_neighbors_nontruncated);
	bool yz_neighbors_nontruncated = true;
	float sdf_at_y_plus_one_z_plus_one   = sdf_at(Vector3i( 0,  1,  1), yz_neighbors_nontruncated);
	float sdf_at_y_minus_one_z_minus_one = sdf_at(Vector3i( 0, -1, -1), yz_neighbors_nontruncated);
	bool xz_neighbors_nontruncated = true;
	float sdf_at_x_plus_one_z_plus_one   = sdf_at(Vector3i( 1,  0,  1), xz_neighbors_nontruncated);
	float sdf_at_x_minus_one_z_minus_one = sdf_at(Vector3i(-1,  0, -1), xz_neighbors_nontruncated);
	//@formatter:on


	float delta_xx = x_neighbors_nontruncated * (sdf_at_x_plus_one - 2 * sdf_at_position + sdf_at_x_minus_one);
	float delta_yy = y_neighbors_nontruncated * (sdf_at_y_plus_one - 2 * sdf_at_position + sdf_at_y_minus_one);
	float delta_zz = z_neighbors_nontruncated * (sdf_at_z_plus_one - 2 * sdf_at_position + sdf_at_z_minus_one);

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

//// Actual function used everywhere:
template<typename TVoxel, typename TIndexData, typename TCache>
_CPU_AND_GPU_CODE_
inline void ComputeSdfHessian(THREADPTR(Matrix3f)& hessian,
                              const CONSTPTR(Vector3i &) position,
                              const CONSTPTR(float)& sdf_at_position,
                              const CONSTPTR(TVoxel)* voxels,
                              const CONSTPTR(TIndexData)* index_data,
                              THREADPTR(TCache)& cache) {
	ComputeSdfHessian_Basic(hessian, position, sdf_at_position, voxels, index_data, cache);
}

//endregion
// region ================================== LEVEL SET ENERGY GRADIENT ===================================
template<typename TVoxel, typename TIndexData, typename TCache>
_CPU_AND_GPU_CODE_
inline void ComputeLevelSetEnergyGradient(THREADPTR(Vector3f)& level_set_energy_gradient,
                                          THREADPTR(Matrix3f)& live_sdf_hessian,
                                          THREADPTR(float)& live_sdf_gradient_norm_minus_unity,
                                          const CONSTPTR(Vector3i &) position,
                                          const CONSTPTR(float)& live_sdf,
                                          const CONSTPTR(TVoxel)* live_voxels,
                                          const CONSTPTR(TIndexData)* live_index_data,
                                          THREADPTR(TCache)& live_cache,
                                          const CONSTPTR(Vector3f &) live_sdf_gradient,
                                          const CONSTPTR(float)& weight_level_set_term,
                                          const CONSTPTR(float)& sdf_unity,
                                          const CONSTPTR(float)& epsilon) {

	ComputeSdfHessian(live_sdf_hessian, position, live_sdf, live_voxels, live_index_data, live_cache);
	// |∇φ_{proj}(Ψ)|
	float live_sdf_gradient_norm = ORUtils::length(live_sdf_gradient);
	// ~ |∇φ_{proj}(Ψ)| - '1'
	// Note: unity constant is actually not 1, so '1' here means 1/[truncation distance in voxels]
	live_sdf_gradient_norm_minus_unity = live_sdf_gradient_norm - sdf_unity;
	//                        ┏                                                          ┓
	// (|∇φ_{proj}(Ψ)| - 1)   ┃  ∇_{xx}φ_{proj}(Ψ)  ∇_{xy}φ_{proj}(Ψ)  ∇_{xz}φ_{proj}(Ψ) ┃
	// --------------------   ┃  ∇_{yx}φ_{proj}(Ψ)  ∇_{yy}φ_{proj}(Ψ)  ∇_{yz}φ_{proj}(Ψ) ┃ ∇φ_{proj}(Ψ
	//  |∇φ_{proj}(Ψ)|_{E}    ┃  ∇_{zx}φ_{proj}(Ψ)  ∇_{zy}φ_{proj}(Ψ)  ∇_{zz}φ_{proj}(Ψ) ┃
	//                        ┗                                                          ┛
	level_set_energy_gradient =
			(weight_level_set_term * live_sdf_gradient_norm_minus_unity / (live_sdf_gradient_norm + epsilon))
			* (live_sdf_hessian * live_sdf_gradient);
}

template<typename TVoxel, typename TIndexData, typename TCache>
_CPU_AND_GPU_CODE_
inline void ComputeLevelSetEnergyGradient(THREADPTR(Vector3f)& level_set_energy_gradient,
                                          const CONSTPTR(Vector3i &) position,
                                          const CONSTPTR(float)& live_sdf,
                                          const CONSTPTR(TVoxel)* live_voxels,
                                          const CONSTPTR(TIndexData)* live_index_data,
                                          THREADPTR(TCache)& live_cache,
                                          const CONSTPTR(Vector3f &) live_sdf_gradient,
                                          const CONSTPTR(float)& weight_level_set_term,
                                          const CONSTPTR(float)& sdf_unity,
                                          const CONSTPTR(float)& epsilon) {
	Matrix3f live_sdf_hessian;
	float live_sdf_gradient_norm_minus_unity;
	ComputeLevelSetEnergyGradient(level_set_energy_gradient, live_sdf_hessian, live_sdf_gradient_norm_minus_unity, position, live_sdf,
	                              live_voxels, live_index_data, live_cache, live_sdf_gradient, weight_level_set_term, sdf_unity, epsilon);
}

// endregion

} // namespace ITMLib