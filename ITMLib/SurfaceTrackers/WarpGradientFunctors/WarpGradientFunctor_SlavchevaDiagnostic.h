//  ================================================================
//  Created by Gregory Kramida on 12/20/17.
//  Copyright (c) 2017-2025 Gregory Kramida
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
#include <chrono>
#include <iomanip>

//local
#include "WarpGradientFunctor.h"
#include "../../../ORUtils/PlatformIndependentAtomics.h"
#include "../Shared/SurfaceTrackerSharedRoutines.h"
#include "../Shared/SurfaceTrackerDiagnosticRoutines.h"
#include "../Shared/WarpGradientAggregates.h"
#include "../Shared/WarpGradientCommon.h"
#include "../../Utils/VoxelFlags.h"
#include "../../Utils/CPrintHelpers.h"
#include "../../Utils/Configuration.h"
#include "../../Engines/EditAndCopy/Shared/EditAndCopyEngine_Shared.h"


namespace ITMLib {

// region ========================== CALCULATE WARP GRADIENT ===========================================================
template<typename TWarp, bool hasDebugInformation>
struct SetGradientFunctor;

template<typename TWarp>
struct SetGradientFunctor<TWarp, false> {
	_CPU_AND_GPU_CODE_
	inline static void SetGradient(TWarp& warp,
	                               const Vector3f& localDataEnergyGradient,
	                               const Vector3f& localLevelSetEnergyGradient,
	                               const Vector3f& localSmoothingEnergyGradient
	) {
		Vector3f localEnergyGradient =
				localDataEnergyGradient +
				localLevelSetEnergyGradient +
				localSmoothingEnergyGradient;
		warp.gradient0 = localEnergyGradient;
	}
};

template<typename TWarp>
struct SetGradientFunctor<TWarp, true> {
	_CPU_AND_GPU_CODE_
	inline static void SetGradient(TWarp& warp,
	                               const Vector3f& localDataEnergyGradient,
	                               const Vector3f& localLevelSetEnergyGradient,
	                               const Vector3f& localSmoothingEnergyGradient
	) {
		warp.data_term_gradient = localDataEnergyGradient;
		warp.smoothing_term_gradient = localSmoothingEnergyGradient;
		Vector3f localEnergyGradient =
				localDataEnergyGradient +
				localLevelSetEnergyGradient +
				localSmoothingEnergyGradient;
		warp.gradient0 = localEnergyGradient;
	}
};


template<typename TTSDFVoxel, typename TWarpVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
struct WarpGradientFunctor<TTSDFVoxel, TWarpVoxel, TIndex, TMemoryDeviceType, TRACKER_SLAVCHEVA_DIAGNOSTIC> {
private:

	_CPU_AND_GPU_CODE_
	void SetUpFocusVoxelPrinting(bool& print_voxel_result, const Vector3i& voxel_position,
	                             const Vector3f& warp_update, const TTSDFVoxel& canonical_voxel, const TTSDFVoxel& live_voxel,
	                             bool compute_data_term) {
		if (use_focus_coordinates && voxel_position == focus_coordinates) {
			int x = 0, y = 0, z = 0, vmIndex = 0, locId = 0;
			GetVoxelHashLocals(vmIndex, locId, x, y, z, live_index_data, live_cache, voxel_position);

			printf("\n%s *** Printing gradient computation data for voxel at (%d, %d, %d) ***%s\n", c_bright_cyan,
			       voxel_position.x, voxel_position.y, voxel_position.z, c_reset);
			printf("Computing data term: %s\n", (compute_data_term ? "true" : "false"));
			printf("Position within block (x,y,z): (%d, %d, %d)\n", x, y, z);
			printf("Canonical vs. Live: \n");
			printf("TSDF: %f vs %f.\n", canonical_voxel.sdf, live_voxel.sdf);
			printf("Flags: %s vs. %s\n", VoxelFlagsAsCString(static_cast<VoxelFlags>(canonical_voxel.flags)),
			       VoxelFlagsAsCString(static_cast<VoxelFlags>(live_voxel.flags)));
			printf("===================\n");
			printf("Warp: %s%f, %f, %f%s\n", c_green, warp_update.x, warp_update.y, warp_update.z, c_reset);
			printf("Warp length: %s%f%s\n", c_green, ORUtils::length(warp_update), c_reset);

			print_voxel_result = true;
		}
	}


public:

	// region ========================================= CONSTRUCTOR ====================================================

	WarpGradientFunctor(SlavchevaSurfaceTracker::Parameters parameters,
	                    SlavchevaSurfaceTracker::Switches switches,
	                    VoxelVolume<TWarpVoxel,TIndex>* warp_field,
	                    VoxelVolume<TTSDFVoxel,TIndex>* canonical_volume,
	                    VoxelVolume<TTSDFVoxel,TIndex>* live_volume,
	                    float voxel_size, float narrow_band_half_width) :
			parameters(parameters), switches(switches),
			live_voxels(live_volume->localVBA.GetVoxelBlocks()), live_index_data(live_volume->index.GetIndexData()),
			warp_voxels(warp_field->localVBA.GetVoxelBlocks()), warp_index_data(warp_field->index.GetIndexData()),
			canonical_voxels(canonical_volume->localVBA.GetVoxelBlocks()), canonical_index_data(canonical_volume->index.GetIndexData()),
			live_cache(), canonical_cache(),
			use_focus_coordinates(configuration::get().verbosity_level >= configuration::VERBOSITY_FOCUS_SPOTS),
			focus_coordinates(configuration::get().telemetry_settings.focus_coordinates),
			sdf_unity(voxel_size / narrow_band_half_width),
			verbosity_level(configuration::get().verbosity_level)
			{}

	// endregion =======================================================================================================

	_DEVICE_WHEN_AVAILABLE_
	void operator()(TWarpVoxel& warp_voxel, TTSDFVoxel& canonical_voxel, TTSDFVoxel& live_voxel, const Vector3i& voxel_position) {


		bool print_voxel_result = false;
		Vector3f& warp_update = warp_voxel.warp_update;
		bool computeDataAndLevelSetTerms = VoxelIsConsideredForDataTerm(canonical_voxel, live_voxel);
		this->SetUpFocusVoxelPrinting(print_voxel_result, voxel_position, warp_update, canonical_voxel, live_voxel, computeDataAndLevelSetTerms);
		if (print_voxel_result) {
			printf("%sLive 6-connected neighbor information:%s\n", c_blue, c_reset);
			print6ConnectedNeighborInfo(voxel_position, live_voxels, live_index_data, live_cache);
		}
		if (!VoxelIsConsideredForTracking(canonical_voxel, live_voxel)) return;

		float live_sdf = TTSDFVoxel::valueToFloat(live_voxel.sdf);
		float canonical_sdf = TTSDFVoxel::valueToFloat(canonical_voxel.sdf);

		// term gradient results are stored here before being added up
		Vector3f localSmoothingEnergyGradient(0.0f), localDataEnergyGradient(0.0f), localLevelSetEnergyGradient(0.0f);



		// region =============================== DATA TERM ================================================
		if (computeDataAndLevelSetTerms) {
			Vector3f liveSdfJacobian;
			ComputeLiveJacobian_CentralDifferences(
					liveSdfJacobian, voxel_position, live_voxels, live_index_data, live_cache);
			if (switches.enable_data_term) {

				// Compute data term error / energy
				float sdfDifferenceBetweenLiveAndCanonical = live_sdf - canonical_sdf;
				// (φ_n(Ψ)−φ_{global}) ∇φ_n(Ψ) - also denoted as - (φ_{proj}(Ψ)−φ_{model}) ∇φ_{proj}(Ψ)
				// φ_n(Ψ) = φ_n(x+u, y+v, z+w), where u = u(x,y,z), v = v(x,y,z), w = w(x,y,z)
				// φ_{global} = φ_{global}(x, y, z)
				localDataEnergyGradient =
						parameters.weight_data_term * sdfDifferenceBetweenLiveAndCanonical * liveSdfJacobian;

				ATOMIC_ADD(aggregates.dataVoxelCount, 1u);

				float localDataEnergy = parameters.weight_data_term * 0.5f *
				                        (sdfDifferenceBetweenLiveAndCanonical * sdfDifferenceBetweenLiveAndCanonical);


				ATOMIC_ADD(energies.totalDataEnergy, localDataEnergy);

				ATOMIC_ADD(aggregates.dataVoxelCount, 1u);
				ATOMIC_ADD(aggregates.cumulativeSdfDiff, sdfDifferenceBetweenLiveAndCanonical);

				if (print_voxel_result) {
					_DEBUG_PrintDataTermStuff(liveSdfJacobian);
				}
			}

			// endregion

			// region =============================== LEVEL SET TERM ===========================================

			if (switches.enable_level_set_term) {
				Matrix3f liveSdfHessian;
				ComputeSdfHessian(liveSdfHessian, voxel_position, live_sdf, live_voxels, live_index_data, live_cache);

				float sdfJacobianNorm = ORUtils::length(liveSdfJacobian);
				float sdfJacobianNormMinusUnity = sdfJacobianNorm - sdf_unity;
				localLevelSetEnergyGradient = parameters.weight_level_set_term * sdfJacobianNormMinusUnity *
				                              (liveSdfHessian * liveSdfJacobian) /
				                              (sdfJacobianNorm + parameters.epsilon);
				ATOMIC_ADD(aggregates.levelSetVoxelCount, 1u);
				float localLevelSetEnergy = parameters.weight_level_set_term *
						0.5f * (sdfJacobianNormMinusUnity * sdfJacobianNormMinusUnity);
				ATOMIC_ADD(energies.totalLevelSetEnergy, localLevelSetEnergy);
				if (print_voxel_result) {
					_DEBUG_PrintLevelSetTermStuff(liveSdfJacobian, liveSdfHessian, sdfJacobianNormMinusUnity);
				}
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
				if (!neighbors_known[iNeighbor]) {
					//assign current warp to neighbor warp if the neighbor is not known
					neighbor_warp_updates[iNeighbor] = warp_update;
				}
			}
			//endregion=================================================================================================

			if (switches.enable_killing_rigidity_enforcement_term) {
				Matrix3f warp_update_Jacobian(0.0f);
				Matrix3f warp_update_Hessian[3] = {Matrix3f(0.0f), Matrix3f(0.0f), Matrix3f(0.0f)};
				ComputePerVoxelWarpJacobianAndHessian(warp_update, neighbor_warp_updates, warp_update_Jacobian,
				                                      warp_update_Hessian);
//TODO rewrite function to be used within CUDA code, remove guards
#ifndef __CUDACC__
				if (print_voxel_result) {
					_DEBUG_PrintKillingTermStuff(neighbor_warp_updates, neighbors_known, neighbors_truncated,
					                             warp_update_Jacobian, warp_update_Hessian);
				}
#endif

				float gamma = parameters.rigidity_enforcement_factor;
				float onePlusGamma = 1.0f + gamma;
				// |0, 3, 6|     |m00, m10, m20|      |u_xx, u_xy, u_xz|
				// |1, 4, 7|     |m01, m11, m21|      |u_xy, u_yy, u_yz|
				// |2, 5, 8|     |m02, m12, m22|      |u_xz, u_yz, u_zz|
				Matrix3f& H_u = warp_update_Hessian[0];
				Matrix3f& H_v = warp_update_Hessian[1];
				Matrix3f& H_w = warp_update_Hessian[2];


				float KillingDeltaEu = -2.0f *
				                       ((onePlusGamma) * H_u.xx + (H_u.yy) + (H_u.zz) + gamma * H_v.xy +
				                        gamma * H_w.xz);
				float KillingDeltaEv = -2.0f *
				                       ((onePlusGamma) * H_v.yy + (H_v.zz) + (H_v.xx) + gamma * H_u.xy +
				                        gamma * H_w.yz);
				float KillingDeltaEw = -2.0f *
				                       ((onePlusGamma) * H_w.zz + (H_w.xx) + (H_w.yy) + gamma * H_v.yz +
				                        gamma * H_u.xz);

				localSmoothingEnergyGradient =
						parameters.weight_smoothing_term * Vector3f(KillingDeltaEu, KillingDeltaEv, KillingDeltaEw);
				//=================================== ENERGY ===============================================
				// KillingTerm Energy
				Matrix3f warpJacobianTranspose = warp_update_Jacobian.t();

				float local_tikhonov_energy = parameters.weight_smoothing_term *
				                              dot(warp_update_Jacobian.getColumn(0),
				                                warp_update_Jacobian.getColumn(0)) +
				                              dot(warp_update_Jacobian.getColumn(1),
				                                warp_update_Jacobian.getColumn(1)) +
				                              dot(warp_update_Jacobian.getColumn(2), warp_update_Jacobian.getColumn(2));

				float localRigidityEnergy = gamma * parameters.weight_smoothing_term *
				                            (dot(warpJacobianTranspose.getColumn(0),
				                                 warp_update_Jacobian.getColumn(0)) +
				                             dot(warpJacobianTranspose.getColumn(1),
				                                 warp_update_Jacobian.getColumn(1)) +
				                             dot(warpJacobianTranspose.getColumn(2),
				                                 warp_update_Jacobian.getColumn(2)));
				ATOMIC_ADD(energies.totalTikhonovEnergy, local_tikhonov_energy);
				ATOMIC_ADD(energies.totalRigidityEnergy, localRigidityEnergy);
			} else {
				Matrix3f warp_updateJacobian(0.0f);
				Vector3f warp_updateLaplacian;
				ComputeWarpLaplacianAndJacobian(warp_updateLaplacian, warp_updateJacobian, warp_update,
				                                neighbor_warp_updates);


				if (print_voxel_result) {
					_DEBUG_PrintTikhonovTermStuff(neighbor_warp_updates, warp_updateLaplacian);
				}

				//∇E_{reg}(Ψ) = −[∆U ∆V ∆W]' ,
				localSmoothingEnergyGradient = -parameters.weight_smoothing_term * warp_updateLaplacian;
				float localTikhonovEnergy = parameters.weight_smoothing_term *
						dot(warp_updateJacobian.getColumn(0), warp_updateJacobian.getColumn(0)) +
						dot(warp_updateJacobian.getColumn(1), warp_updateJacobian.getColumn(1)) +
						dot(warp_updateJacobian.getColumn(2), warp_updateJacobian.getColumn(2));
				ATOMIC_ADD(energies.totalTikhonovEnergy, localTikhonovEnergy);
			}
		}
		// endregion
		// region =============================== COMPUTE ENERGY GRADIENT ==================================
		SetGradientFunctor<TWarpVoxel, TWarpVoxel::hasDebugInformation>::SetGradient(
				warp_voxel, localDataEnergyGradient, localLevelSetEnergyGradient, localSmoothingEnergyGradient);

		// endregion
		// region =============================== AGGREGATE VOXEL STATISTICS ===============================


		float warpLength = ORUtils::length(warp_voxel.warp_update);

		ATOMIC_ADD(aggregates.cumulativeCanonicalSdf, canonical_sdf);
		ATOMIC_ADD(aggregates.cumulativeLiveSdf, live_sdf);
		ATOMIC_ADD(aggregates.cumulativeWarpDist, warpLength);
		ATOMIC_ADD(aggregates.consideredVoxelCount, 1u);
		// endregion

		// region ======================== FINALIZE RESULT PRINTING / RECORDING ========================================

		if (print_voxel_result) {
			float energyGradientLength = ORUtils::length(warp_voxel.gradient0);
			_DEBUG_printLocalEnergyGradients(localDataEnergyGradient, localLevelSetEnergyGradient,
			                                 localSmoothingEnergyGradient, warp_voxel.gradient0, energyGradientLength);
		}
		// endregion ===================================================================================================
	}


	void PrintStatistics() {
		if(verbosity_level < configuration::VERBOSITY_PER_ITERATION) return;
		std::cout << bright_cyan << "*** Non-rigid Alignment Iteration Statistics ***" << reset << std::endl;
		PrintEnergyStatistics(this->switches.enable_data_term, this->switches.enable_level_set_term,
		                      this->switches.enable_smoothing_term, this->switches.enable_killing_rigidity_enforcement_term,
		                      this->parameters.rigidity_enforcement_factor, energies);
		CalculateAndPrintAdditionalStatistics(
				this->switches.enable_data_term, this->switches.enable_level_set_term, aggregates);
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

	AdditionalGradientAggregates<TMemoryDeviceType> aggregates;
	ComponentEnergies<TMemoryDeviceType> energies;

	// *** debugging / analysis variables
	bool use_focus_coordinates{};
	Vector3i focus_coordinates;

	const SlavchevaSurfaceTracker::Parameters parameters;
	const SlavchevaSurfaceTracker::Switches switches;

	const configuration::VerbosityLevel verbosity_level;
};

}// namespace ITMLib
