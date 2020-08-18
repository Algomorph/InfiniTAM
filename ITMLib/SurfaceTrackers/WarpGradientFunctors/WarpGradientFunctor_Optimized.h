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
#include "../../Utils/Enums/VoxelFlags.h"
#include "../../Utils/Logging/ConsolePrintColors.h"
#include "../../Engines/EditAndCopy/Shared/EditAndCopyEngine_Shared.h"
#include "../../Objects/Volume/VoxelVolume.h"


namespace ITMLib {


template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TMemoryDeviceType>
struct WarpGradientFunctor<TVoxel, TWarp, TIndex, TMemoryDeviceType, OPTIMIZED> {
private:


public:

	// region ========================================= CONSTRUCTOR ====================================================
	WarpGradientFunctor(LevelSetEvolutionWeights parameters,
	                    LevelSetEvolutionSwitches switches,
	                    VoxelVolume<TWarp, TIndex>* warp_field,
	                    VoxelVolume<TVoxel, TIndex>* canonical_volume,
	                    VoxelVolume<TVoxel, TIndex>* live_volume,
	                    float voxelSize, float narrowBandHalfWidth, int iteration_index) :
			parameters(parameters), switches(switches),
			live_voxels(live_volume->GetVoxels()), live_index_data(live_volume->index.GetIndexData()),
			warp_voxels(warp_field->GetVoxels()), warp_index_data(warp_field->index.GetIndexData()),
			canonical_voxels(canonical_volume->GetVoxels()), canonical_index_data(canonical_volume->index.GetIndexData()),
			live_cache(), canonical_cache(),
			sdf_unity(voxelSize / narrowBandHalfWidth), iteration_index(iteration_index) {}

	// endregion =======================================================================================================

	_DEVICE_WHEN_AVAILABLE_
	void operator()(TWarp& warp_voxel, TVoxel& canonical_voxel, TVoxel& live_voxel, const Vector3i& voxel_position) {

		if (!VoxelIsConsideredForTracking(canonical_voxel, live_voxel)) return;

		Vector3f& warp_update = warp_voxel.warp_update;
		float live_sdf = TVoxel::valueToFloat(live_voxel.sdf);
		float canonical_sdf = TVoxel::valueToFloat(canonical_voxel.sdf);

		// region =============================== DECLARATIONS & DEFAULTS FOR ALL TERMS ====================

		Vector3f local_smoothing_energy_gradient(0.0f), local_data_energy_gradient(0.0f), local_level_set_energy_gradient(0.0f);
		// endregion

		Vector3f live_sdf_gradient;
		ComputeGradient_CentralDifferences_ZeroIfTruncated(live_sdf_gradient, voxel_position, live_voxels, live_index_data, live_cache);
		// region =============================== DATA TERM ================================================
		if (VoxelIsConsideredForDataTerm(canonical_voxel, live_voxel) && switches.enable_data_term) {
			// Compute data term error / energy
			float sdf_difference_between_live_and_canonical = live_sdf - canonical_sdf;
			// (φ_n(Ψ)−φ_{global}) ∇φ_n(Ψ) - also denoted as - (φ_{proj}(Ψ)−φ_{model}) ∇φ_{proj}(Ψ)
			// φ_n(Ψ) = φ_n(x+u, y+v, z+w), where u = u(x,y,z), v = v(x,y,z), w = w(x,y,z)
			// φ_{global} = φ_{global}(x, y, z)
			local_data_energy_gradient = parameters.weight_data_term * sdf_difference_between_live_and_canonical * live_sdf_gradient;
		}

		// endregion =======================================================================================
		// region =============================== LEVEL SET TERM ===========================================
		if (switches.enable_level_set_term && live_voxel.flags == VOXEL_NONTRUNCATED) {
			Matrix3f live_sdf_2nd_derivative;
			ComputeSdf2ndDerivative(live_sdf_2nd_derivative, voxel_position, live_sdf, live_voxels, live_index_data, live_cache);

			float live_sdf_gradient_norm = ORUtils::length(live_sdf_gradient);
			float live_sdf_gradient_norm_minus_unity = live_sdf_gradient_norm - sdf_unity;
//			local_level_set_energy_gradient = parameters.weight_level_set_term * live_sdf_gradient_norm_minus_unity *
//			                                  (live_sdf_2nd_derivative * live_sdf_gradient) /
//			                                  (live_sdf_gradient_norm + parameters.epsilon);
			local_level_set_energy_gradient =
					(parameters.weight_level_set_term * live_sdf_gradient_norm_minus_unity / (live_sdf_gradient_norm + parameters.epsilon))
					* (live_sdf_2nd_derivative * live_sdf_gradient);
		}
		// endregion =======================================================================================

		// region =============================== SMOOTHING TERM (TIKHONOV & KILLING) ======================

		if (switches.enable_smoothing_term) {
			// region ============================== RETRIEVE NEIGHBOR'S WARPS =========================================

			const int neighborhood_size = 9;
			Vector3f neighbor_warp_updates[neighborhood_size];
			bool neighbors_known[neighborhood_size], neighbors_truncated[neighborhood_size], neighbors_allocated[neighborhood_size];

			// neighbor index:       0        1        2          3         4         5           6         7         8
			// neighbor position: (-1,0,0) (0,-1,0) (0,0,-1)   (1, 0, 0) (0, 1, 0) (0, 0, 1)   (1, 1, 0) (0, 1, 1) (1, 0, 1)
			findPoint2ndDerivativeNeighborhoodFramewiseWarp(
					neighbor_warp_updates/*x9*/, neighbors_known, neighbors_truncated, neighbors_allocated, voxel_position,
					warp_voxels, warp_index_data, warp_cache, canonical_voxels, canonical_index_data, canonical_cache);

			for (int iNeighbor = 0; iNeighbor < neighborhood_size; iNeighbor++) {
				if (!neighbors_known[iNeighbor]) {
					//assign current warp to neighbor warp if the neighbor is not known
					neighbor_warp_updates[iNeighbor] = warp_update;
				}
			}
			//endregion=================================================================================================

			if (switches.enable_killing_rigidity_enforcement_term) {
				Matrix3f warp_update_Jacobian(0.0f);
				Matrix3f warp_update_Hessian[3] = {Matrix3f(0.0f), Matrix3f(0.0f), Matrix3f(0.0f)};
				ComputePerVoxelWarpJacobianAndHessian(warp_update, neighbor_warp_updates, warp_update_Jacobian, warp_update_Hessian);

				float gamma = parameters.weight_killing_term;
				float one_plus_gamma = 1.0f + gamma;
				// |0, 3, 6|     |m00, m10, m20|      |u_xx, u_xy, u_xz|
				// |1, 4, 7|     |m01, m11, m21|      |u_xy, u_yy, u_yz|
				// |2, 5, 8|     |m02, m12, m22|      |u_xz, u_yz, u_zz|
				Matrix3f& H_u = warp_update_Hessian[0];
				Matrix3f& H_v = warp_update_Hessian[1];
				Matrix3f& H_w = warp_update_Hessian[2];

				float Killing_delta_E_u = -2.0f * ((one_plus_gamma) * H_u.xx + (H_u.yy) + (H_u.zz) + gamma * H_v.xy + gamma * H_w.xz);
				float Killing_delta_E_v = -2.0f * ((one_plus_gamma) * H_v.yy + (H_v.zz) + (H_v.xx) + gamma * H_u.xy + gamma * H_w.yz);
				float Killing_delta_E_w = -2.0f * ((one_plus_gamma) * H_w.zz + (H_w.xx) + (H_w.yy) + gamma * H_v.yz + gamma * H_u.xz);

				local_smoothing_energy_gradient =
						parameters.weight_smoothing_term * Vector3f(Killing_delta_E_u, Killing_delta_E_v, Killing_delta_E_w);
			} else {
				Matrix3f warp_update_Jacobian(0.0f);
				Vector3f warp_update_Laplacian;
				ComputeWarpLaplacianAndJacobian(warp_update_Laplacian, warp_update_Jacobian, warp_update, neighbor_warp_updates);
				//∇E_{reg}(Ψ) = −[∆U ∆V ∆W]' ,
				local_smoothing_energy_gradient = -parameters.weight_smoothing_term * warp_update_Laplacian;
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

	void SaveStatistics() {

	}


private:

	const float sdf_unity;
	const int iteration_index;

	// *** data structure accessors
	const TVoxel* live_voxels;
	const typename TIndex::IndexData* live_index_data;
	typename TIndex::IndexCache live_cache;

	const TVoxel* canonical_voxels;
	const typename TIndex::IndexData* canonical_index_data;
	typename TIndex::IndexCache canonical_cache;

	TWarp* warp_voxels;
	const typename TIndex::IndexData* warp_index_data;
	typename TIndex::IndexCache warp_cache;


	const LevelSetEvolutionWeights parameters;
	const LevelSetEvolutionSwitches switches;
};

}// namespace ITMLib

