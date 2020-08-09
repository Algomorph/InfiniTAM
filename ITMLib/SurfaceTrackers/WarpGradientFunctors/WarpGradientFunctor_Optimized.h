//  ================================================================
//  Created by Gregory Kramida on 11/15/19.
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

#pragma once

//stdlib
#include <chrono>
#include <iomanip>

//local
#include "WarpGradientFunctor.h"
#include "../Interface/LevelSetEvolutionParameters.h"
#include "../../../ORUtils/PlatformIndependentAtomics.h"
#include "../../../ORUtils/CrossPlatformMacros.h"
#include "../Shared/SurfaceTrackerSharedRoutines.h"
#include "../Shared/SurfaceTrackerDiagnosticRoutines.h"
#include "../Shared/WarpGradientAggregates.h"
#include "../Shared/WarpGradientCommon.h"
#include "../../Utils/VoxelFlags.h"
#include "../../Utils/CPrintHelpers.h"
#include "../../Engines/EditAndCopy/Shared/EditAndCopyEngine_Shared.h"
#include "../../Objects/Volume/VoxelVolume.h"


namespace ITMLib {


template<typename TTSDFVoxel, typename TWarpVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
struct WarpGradientFunctor<TTSDFVoxel, TWarpVoxel, TIndex, TMemoryDeviceType, OPTIMIZED> {
private:


public:

	// region ========================================= CONSTRUCTOR ====================================================
	WarpGradientFunctor(LevelSetEvolutionWeights parameters,
	                    LevelSetEvolutionSwitches switches,
	                    VoxelVolume<TWarpVoxel, TIndex>* warp_field,
	                    VoxelVolume<TTSDFVoxel, TIndex>* canonical_volume,
	                    VoxelVolume<TTSDFVoxel, TIndex>* live_volume,
	                    float voxelSize, float narrowBandHalfWidth) :
			parameters(parameters), switches(switches),
			live_voxels(live_volume->GetVoxels()), live_index_data(live_volume->index.GetIndexData()),
			warp_voxels(warp_field->GetVoxels()), warp_index_data(warp_field->index.GetIndexData()),
			canonical_voxels(canonical_volume->GetVoxels()), canonical_index_data(canonical_volume->index.GetIndexData()),
			live_cache(), canonical_cache(),
			sdf_unity(voxelSize / narrowBandHalfWidth) {}

	// endregion =======================================================================================================

	_DEVICE_WHEN_AVAILABLE_
	void operator()(TWarpVoxel& warp_voxel, TTSDFVoxel& canonical_voxel, TTSDFVoxel& live_voxel, const Vector3i& voxel_position) {

		if (!VoxelIsConsideredForTracking(canonical_voxel, live_voxel)) return;
		bool compute_data_and_level_set_terms = VoxelIsConsideredForDataAndLS_Term(canonical_voxel, live_voxel);

		Vector3f& warp_update = warp_voxel.warp_update;
		float live_sdf = TTSDFVoxel::valueToFloat(live_voxel.sdf);
		float canonical_sdf = TTSDFVoxel::valueToFloat(canonical_voxel.sdf);

		// region =============================== DECLARATIONS & DEFAULTS FOR ALL TERMS ====================

		Vector3f local_smoothing_energy_gradient(0.0f), local_data_energy_gradient(0.0f), local_level_set_energy_gradient(0.0f);
		// endregion

		if (compute_data_and_level_set_terms) {
			// region =============================== DATA TERM ================================================

			Vector3f live_sdf_Jacobian;
			ComputeLiveJacobian_CentralDifferences(
					live_sdf_Jacobian, voxel_position, live_voxels, live_index_data, live_cache);
			if (switches.enable_data_term) {

				// Compute data term error / energy
				float sdf_difference_between_live_and_canonical = live_sdf - canonical_sdf;
				// (φ_n(Ψ)−φ_{global}) ∇φ_n(Ψ) - also denoted as - (φ_{proj}(Ψ)−φ_{model}) ∇φ_{proj}(Ψ)
				// φ_n(Ψ) = φ_n(x+u, y+v, z+w), where u = u(x,y,z), v = v(x,y,z), w = w(x,y,z)
				// φ_{global} = φ_{global}(x, y, z)
				local_data_energy_gradient =
						parameters.weight_data_term * sdf_difference_between_live_and_canonical * live_sdf_Jacobian;
			}

			// endregion =======================================================================================
			// region =============================== LEVEL SET TERM ===========================================

			if (switches.enable_level_set_term) {
				Matrix3f live_sdf_Hessian;
				ComputeSdfHessian(live_sdf_Hessian, voxel_position, live_sdf, live_voxels, live_index_data, live_cache);

				float sdf_Jacobian_norm = ORUtils::length(live_sdf_Jacobian);
				float sdf_Jacobian_norm_minus_unity = sdf_Jacobian_norm - sdf_unity;
				local_level_set_energy_gradient = parameters.weight_level_set_term * sdf_Jacobian_norm_minus_unity *
				                                  (live_sdf_Hessian * live_sdf_Jacobian) /
				                                  (sdf_Jacobian_norm + parameters.epsilon);
			}
			// endregion =======================================================================================
		}

		// region =============================== SMOOTHING TERM (TIKHONOV & KILLING) ======================

		if (switches.enable_smoothing_term) {
			// region ============================== RETRIEVE NEIGHBOR'S WARPS =========================================

			const int neighborhood_size = 9;
			Vector3f neighbor_warp_updates[neighborhood_size];
			bool neighbors_known[neighborhood_size], neighbors_truncated[neighborhood_size], neighbors_allocated[neighborhood_size];

			//    0        1        2          3         4         5           6         7         8
			//(-1,0,0) (0,-1,0) (0,0,-1)   (1, 0, 0) (0, 1, 0) (0, 0, 1)   (1, 1, 0) (0, 1, 1) (1, 0, 1)
			findPoint2ndDerivativeNeighborhoodFramewiseWarp(
					neighbor_warp_updates/*x9*/, neighbors_known, neighbors_truncated, neighbors_allocated, voxel_position,
					warp_voxels, warp_index_data, warp_cache, canonical_voxels, canonical_index_data, canonical_cache);

			for (int iNeighbor = 0; iNeighbor < neighborhood_size; iNeighbor++) {
				if (!neighbors_allocated[iNeighbor]) {
					//assign current warp to neighbor warp if the neighbor is not allocated
					neighbor_warp_updates[iNeighbor] = warp_update;
				}
			}
			//endregion=================================================================================================

			if (switches.enable_killing_rigidity_enforcement_term) {
				Matrix3f warp_updateJacobian(0.0f);
				Matrix3f warp_updateHessian[3] = {Matrix3f(0.0f), Matrix3f(0.0f), Matrix3f(0.0f)};
				ComputePerVoxelWarpJacobianAndHessian(warp_update, neighbor_warp_updates, warp_updateJacobian,
				                                      warp_updateHessian);

				float gamma = parameters.weight_killing_term;
				float onePlusGamma = 1.0f + gamma;
				// |0, 3, 6|     |m00, m10, m20|      |u_xx, u_xy, u_xz|
				// |1, 4, 7|     |m01, m11, m21|      |u_xy, u_yy, u_yz|
				// |2, 5, 8|     |m02, m12, m22|      |u_xz, u_yz, u_zz|
				Matrix3f& H_u = warp_updateHessian[0];
				Matrix3f& H_v = warp_updateHessian[1];
				Matrix3f& H_w = warp_updateHessian[2];


				float KillingDeltaEu = -2.0f *
				                       ((onePlusGamma) * H_u.xx + (H_u.yy) + (H_u.zz) + gamma * H_v.xy +
				                        gamma * H_w.xz);
				float KillingDeltaEv = -2.0f *
				                       ((onePlusGamma) * H_v.yy + (H_v.zz) + (H_v.xx) + gamma * H_u.xy +
				                        gamma * H_w.yz);
				float KillingDeltaEw = -2.0f *
				                       ((onePlusGamma) * H_w.zz + (H_w.xx) + (H_w.yy) + gamma * H_v.yz +
				                        gamma * H_u.xz);

				local_smoothing_energy_gradient =
						parameters.weight_smoothing_term * Vector3f(KillingDeltaEu, KillingDeltaEv, KillingDeltaEw);
			} else {
				Matrix3f warp_updateJacobian(0.0f);
				Vector3f warp_update_laplacian;
				ComputeWarpLaplacianAndJacobian(warp_update_laplacian, warp_updateJacobian, warp_update,
				                                neighbor_warp_updates);
				//∇E_{reg}(Ψ) = −[∆U ∆V ∆W]' ,
				local_smoothing_energy_gradient = -parameters.weight_smoothing_term * warp_update_laplacian;
			}
		}
		// endregion
		// region =============================== COMPUTE ENERGY GRADIENT ==================================
		Vector3f local_energy_gradient =
				local_data_energy_gradient +
				local_level_set_energy_gradient +
				local_smoothing_energy_gradient;

		warp_voxel.gradient0 = local_energy_gradient;
	}


	void PrintStatistics() {

	}


private:

	const float sdf_unity;

	// *** data structure accessors
	const TTSDFVoxel* live_voxels;
	const typename TIndex::IndexData* live_index_data;
	typename TIndex::IndexCache live_cache;

	const TTSDFVoxel* canonical_voxels;
	const typename TIndex::IndexData* canonical_index_data;
	typename TIndex::IndexCache canonical_cache;

	TWarpVoxel* warp_voxels;
	const typename TIndex::IndexData* warp_index_data;
	typename TIndex::IndexCache warp_cache;


	const LevelSetEvolutionWeights parameters;
	const LevelSetEvolutionSwitches switches;
};

}// namespace ITMLib

