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

// region =================================EXPLORATION OF NEIGHBORHOOD AROUND LOCAL VOXEL===========================

/**
 * \brief Finds neighbor voxel's warps and other information in the order specified below.
 * Neighbor index:       0         1       2           3         4         5
 * Neighbor position: (-1,0,0) (0,-1,0) (0,0,-1)   (1, 0, 0) (0, 1, 0) (0, 0, 1)
 * The neighbor indices pertain to all the output arrays.
 * \tparam TVoxel
 * \tparam TCache
 * \param[out] neighbor_warp_updates
 * \param[out] neighbor_known array of booleans set as follows:
 * false: unallocated voxels (see below), record voxels still marked as "unknown" (default);
 * true: truncated voxels, non-truncated voxels
 * \param[out] neighbor_truncated whether or not each of the neighbors is truncated
 * \param[out] neighbor_allocated array of booleans, set to false if, for VBH, the voxel hash block containing this neighbor is not allocated
 * or, for PVA, voxel is outside the plain voxel array bounds; otherwise, set to true true.
 * \param[in] voxel_position exact position of voxel in the scene.
 * \param[in] warps warp memory storage from the warp volume
 * \param[in] warp_index_data warp index data from the warp volume
 * \param[in] voxel_cache
 * \param[in] voxels voxel memory storage from the voxel volume associated with the warps
 * \param[in] warp_index_data warp index data from the warp volume
 * \param[in] voxel_cache
 */
template<typename TVoxel, typename TWarp, typename TIndexData, typename TCache>
_CPU_AND_GPU_CODE_
inline void FindLocal1stDerivativeNeighborhoodWarpUpdate(THREADPTR(Vector3f)* neighbor_warp_updates, //x6, out
                                                         THREADPTR(bool)* neighbor_known, //x6, out
                                                         THREADPTR(bool)* neighbor_truncated, //x6, out
                                                         THREADPTR(bool)* neighbor_allocated, //x6, out
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
	// necessary for 1st derivatives (d_x, d_y, d_z) and 2nd derivatives in the same direction (d_xx, d_yy, d_zz)
	process_voxel(Vector3i(-1, 0, 0), 0);
	process_voxel(Vector3i(0, -1, 0), 1);
	process_voxel(Vector3i(0, 0, -1), 2);

	process_voxel(Vector3i(1, 0, 0), 3);
	process_voxel(Vector3i(0, 1, 0), 4);
	process_voxel(Vector3i(0, 0, 1), 5);
}

/**
 * \brief Finds neighbor voxel's warps and other information in the order specified below.
 * Neighbor index:       0         1       2           3         4         5           6         7         8            9          10          11
 * Neighbor position: (-1,0,0) (0,-1,0) (0,0,-1)   (1, 0, 0) (0, 1, 0) (0, 0, 1)   (1, 1, 0) (0, 1, 1) (1, 0, 1)   (-1, -1, 0) (0, -1, -1) (-1, 0, -1)
 * The neighbor indices pertain to all the output arrays.
 * \tparam TVoxel
 * \tparam TCache
 * \param[out] neighbor_warp_updates
 * \param[out] neighbor_known array of booleans set as follows:
 * false: unallocated voxels (see below), record voxels still marked as "unknown" (default);
 * true: truncated voxels, non-truncated voxels
 * \param[out] neighbor_truncated whether or not each of the neighbors is truncated
 * \param[out] neighbor_allocated array of booleans, set to false if, for VBH, the voxel hash block containing this neighbor is not allocated
 * or, for PVA, voxel is outside the plain voxel array bounds; otherwise, set to true true.
 * \param[in] voxel_position exact position of voxel in the scene.
 * \param[in] warps warp memory storage from the warp volume
 * \param[in] warp_index_data warp index data from the warp volume
 * \param[in] voxel_cache
 * \param[in] voxels voxel memory storage from the voxel volume associated with the warps
 * \param[in] warp_index_data warp index data from the warp volume
 * \param[in] voxel_cache
 */
template<typename TVoxel, typename TWarp, typename TIndexData, typename TCache>
_CPU_AND_GPU_CODE_
inline void FindLocal2ndDerivativeNeighborhoodWarpUpdate(THREADPTR(Vector3f)* neighbor_warp_updates, //x12, out
                                                         THREADPTR(bool)* neighbor_known, //x12, out
                                                         THREADPTR(bool)* neighbor_truncated, //x12, out
                                                         THREADPTR(bool)* neighbor_allocated, //x12, out
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
	// necessary for 1st derivatives (d_x, d_y, d_z) and 2nd derivatives in the same direction (d_xx, d_yy, d_zz)
	process_voxel(Vector3i(-1, 0, 0), 0);
	process_voxel(Vector3i(0, -1, 0), 1);
	process_voxel(Vector3i(0, 0, -1), 2);

	process_voxel(Vector3i(1, 0, 0), 3);
	process_voxel(Vector3i(0, 1, 0), 4);
	process_voxel(Vector3i(0, 0, 1), 5);

	//necessary for 2nd derivatives in mixed directions, e.g. xy and yz
	process_voxel(Vector3i(1, 1, 0), 6); // x+y+ corner
	process_voxel(Vector3i(0, 1, 1), 7); // y+z+ corner
	process_voxel(Vector3i(1, 0, 1), 8); // x+z+ corner

	process_voxel(Vector3i(-1, -1, 0), 9); // x-y- corner
	process_voxel(Vector3i(0, -1, -1), 10); // y-z- corner
	process_voxel(Vector3i(-1, 0, -1), 11); // x-z- corner
}

template<int TNeighborhoodSize>
_CPU_AND_GPU_CODE_
inline void SetUnknownNeighborToCentralWarpUpdate(THREADPTR(Vector3f)* neighbor_warp_updates,
                                                  const CONSTPTR(bool)* neighbors_known,
                                                  const CONSTPTR(Vector3f)& central_warp_update) {
	for (int i_neighbor = 0; i_neighbor < TNeighborhoodSize; i_neighbor++) {
		if (!neighbors_known[i_neighbor]) {
			//assign current warp to neighbor warp if the neighbor is not known
			neighbor_warp_updates[i_neighbor] = central_warp_update;
		}
	}
}


// endregion



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

	//TODO: optimize? (only 2 values are used for each of xy, yz, and xz; see below)
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
// endregion
// region ================================ DATA ENERGY GRADIENT ==============================================
/** Computes the dampened approximately-Killing-vector-field (AKVF) energy term gradient. */
_CPU_AND_GPU_CODE_ inline
void ComputeDampened_AKVF_EnergyGradient(THREADPTR(Vector3f)& smoothing_energy_gradient,
                                         THREADPTR(Vector3f)& laplacian,
                                         THREADPTR(Vector3f)& divergence_derivative,
                                         const CONSTPTR(Vector3f)* neighbor_warp_updates,
                                         const CONSTPTR(Vector3f)& warp_update,
                                         const CONSTPTR(float)& Killing_dampening_factor,
                                         const CONSTPTR(float)& weight_smoothing_term) {

	ComputeVectorLaplacianAndDivergenceDerivative(divergence_derivative, laplacian, warp_update, neighbor_warp_updates);
	smoothing_energy_gradient = -2.0f * weight_smoothing_term * (laplacian - Killing_dampening_factor * divergence_derivative);
}
/** Computes the dampened approximately-Killing-vector-field (AKVF) energy term gradient. */
_CPU_AND_GPU_CODE_ inline
void ComputeDampened_AKVF_EnergyGradient(THREADPTR(Vector3f)& smoothing_energy_gradient,
                                         const CONSTPTR(Vector3f)* neighbor_warp_updates,
                                         const CONSTPTR(Vector3f)& warp_update,
                                         const CONSTPTR(float)& Killing_dampening_factor,
                                         const CONSTPTR(float)& weight_smoothing_term) {
	Vector3f laplacian, divergence_derivative;
	ComputeDampened_AKVF_EnergyGradient(smoothing_energy_gradient, laplacian, divergence_derivative,
	                                    neighbor_warp_updates, warp_update, Killing_dampening_factor, weight_smoothing_term);
}

_CPU_AND_GPU_CODE_ inline
void ComputeTikhonovEnergyGradient(THREADPTR(Vector3f)& smoothing_energy_gradient,
                                   THREADPTR(Vector3f)& warp_update_Laplacian,
                                   const CONSTPTR(Vector3f)* neighbor_warp_updates,
                                   const CONSTPTR(Vector3f)& warp_update,
                                   const CONSTPTR(float)& weight_smoothing_term) {
	ComputeVectorLaplacian(warp_update_Laplacian, warp_update, neighbor_warp_updates);

	//∇E_{reg}(Ψ) = −[∆U ∆V ∆W]^T
	smoothing_energy_gradient = -weight_smoothing_term * warp_update_Laplacian;
}

_CPU_AND_GPU_CODE_ inline
void ComputeTikhonovEnergyGradient(THREADPTR(Vector3f)& smoothing_energy_gradient,
                                   const CONSTPTR(Vector3f)* neighbor_warp_updates,
                                   const CONSTPTR(Vector3f)& warp_update,
                                   const CONSTPTR(float)& weight_smoothing_term) {
	Vector3f warp_update_Laplacian;
	ComputeTikhonovEnergyGradient(smoothing_energy_gradient, warp_update_Laplacian, neighbor_warp_updates, warp_update, weight_smoothing_term);
}


//endregion
} // namespace ITMLib