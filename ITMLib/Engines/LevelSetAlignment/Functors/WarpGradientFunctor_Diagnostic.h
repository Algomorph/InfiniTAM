//  ================================================================
//  Created by Gregory Kramida on 12/20/17.
//  Copyright (c) 2017-2000 Gregory Kramida
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
#include "../../../../ORUtils/PlatformIndependentAtomics.h"
#include "../../../../ORUtils/CrossPlatformMacros.h"
#include "../Shared/LevelSetAlignmentSharedRoutines.h"
#include "../Shared/SdfGradientSharedRoutines.h"
#include "../Shared/DataTermSharedRoutines.h"
#include "../Shared/LevelSetTermSharedRoutines.h"
#include "../Shared/SmoothingTermSharedRoutines.h"
#include "../Shared/LevelSetAlignmentDiagnosticRoutines.h"
#include "../Shared/WarpGradientAggregates.h"
#include "../Shared/WarpGradientCommon.h"
#include "../../../Utils/Enums/VoxelFlags.h"
#include "../../../Utils/Logging/ConsolePrintColors.h"
#include "../../../Utils/Configuration/Configuration.h"
#include "../../EditAndCopy/Shared/EditAndCopyEngine_Shared.h"
#include "../../Telemetry/TelemetryRecorder.h"

namespace ITMLib {

// region ========================== CALCULATE WARP GRADIENT ===========================================================
template<typename TWarp, bool THasDebugInformation>
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


template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TMemoryDeviceType>
struct WarpGradientFunctor<TVoxel, TWarp, TIndex, TMemoryDeviceType, DIAGNOSTIC> {
private:

	_CPU_AND_GPU_CODE_
	void SetUpFocusVoxelPrinting(bool& print_voxel_result, const Vector3i& voxel_position,
	                             const Vector3f& warp_update, const TVoxel& canonical_voxel, const TVoxel& live_voxel,
	                             bool compute_data_term) {
		if (use_focus_voxel && voxel_position == focus_voxel) {
			int x = 0, y = 0, z = 0, vmIndex = 0, locId = 0;
			GetVoxelHashLocals(vmIndex, locId, x, y, z, live_index_data, live_cache, voxel_position);

			printf("\n%s *** Printing gradient computation data for voxel at (%d, %d, %d) ***%s\n", bright_cyan,
			       voxel_position.x, voxel_position.y, voxel_position.z, reset);
			printf("Computing data term: %s\n", (compute_data_term ? "true" : "false"));
			printf("Position within block (x,y,z): (%d, %d, %d)\n", x, y, z);
			printf("Canonical vs. Live: \n");
			printf("TSDF: %f vs %f.\n", canonical_voxel.sdf, live_voxel.sdf);
			printf("Flags: %s vs. %s\n", VoxelFlagsAsCString(static_cast<VoxelFlags>(canonical_voxel.flags)),
			       VoxelFlagsAsCString(static_cast<VoxelFlags>(live_voxel.flags)));
			printf("===================\n");
			printf("Warp: %s%f, %f, %f%s\n", green, warp_update.x, warp_update.y, warp_update.z, reset);
			printf("Warp length: %s%f%s\n", green, ORUtils::length(warp_update), reset);

			print_voxel_result = true;
		} else {
			print_voxel_result = false;
		}
	}

public:

	// region ========================================= CONSTRUCTOR ==================================================================================
	WarpGradientFunctor(LevelSetAlignmentWeights parameters,
	                    LevelSetAlignmentSwitches switches,
	                    VoxelVolume<TWarp, TIndex>* warp_field,
	                    VoxelVolume<TVoxel, TIndex>* canonical_volume,
	                    VoxelVolume<TVoxel, TIndex>* live_volume,
	                    float voxel_size, float truncation_distance, int iteration_index) :
			parameters(parameters), switches(switches),
			live_voxels(live_volume->GetVoxels()), live_index_data(live_volume->index.GetIndexData()),
			warp_voxels(warp_field->GetVoxels()), warp_index_data(warp_field->index.GetIndexData()),
			canonical_voxels(canonical_volume->GetVoxels()), canonical_index_data(canonical_volume->index.GetIndexData()),
			live_cache(), canonical_cache(),
			use_focus_voxel(configuration::Get().logging_settings.verbosity_level >= VERBOSITY_FOCUS_SPOTS),
			focus_voxel(configuration::Get().focus_voxel),
			sdf_unity(voxel_size / truncation_distance),
			verbosity_level(configuration::Get().logging_settings.verbosity_level),
			iteration_index(iteration_index) {}

	// endregion =====================================================================================================================================

	_DEVICE_WHEN_AVAILABLE_
	void operator()(TWarp& warp_voxel, TVoxel& canonical_voxel, TVoxel& live_voxel, const Vector3i& voxel_position) {
		Vector3f& warp_update = warp_voxel.warp_update;

		bool compute_data_term = VoxelIsConsideredForDataTerm(canonical_voxel, live_voxel);

		bool print_voxel_result;
		this->SetUpFocusVoxelPrinting(print_voxel_result, voxel_position, warp_update, canonical_voxel, live_voxel, compute_data_term);

		if (print_voxel_result) {
			printf("%sLive 6-connected neighbor information:%s\n", blue, reset);
			Print6ConnectedNeighborInfo(voxel_position, live_voxels, live_index_data, live_cache);
		}

		if (!VoxelIsConsideredForAlignment(canonical_voxel, live_voxel)) return;


		float live_sdf = TVoxel::valueToFloat(live_voxel.sdf);
		float canonical_sdf = TVoxel::valueToFloat(canonical_voxel.sdf);

		// term gradient results are stored here before being added up
		Vector3f local_smoothing_energy_gradient(0.0f), local_data_energy_gradient(0.0f), local_level_set_energy_gradient(0.0f);

		Vector3f live_sdf_gradient;
		ComputeSdfGradient(live_sdf_gradient, voxel_position, live_sdf, live_voxels, live_index_data, live_cache);

		// live sdf jacobian is denoted in the text as live gradient ∇φ_proj(Ψ)
		// region =============================== DATA TERM ==========================================================================================
		if (compute_data_term && switches.enable_data_term && iteration_index < iteration_bound) {
			float sdf_difference_between_live_and_canonical;
			ComputeDataEnergyGradient(local_data_energy_gradient, sdf_difference_between_live_and_canonical, live_sdf, canonical_sdf,
			                          parameters.weight_data_term, live_sdf_gradient);
			//=================================== ENERGY =============================================================================================
			float local_data_energy = parameters.weight_data_term * 0.5f *
			                          (sdf_difference_between_live_and_canonical * sdf_difference_between_live_and_canonical);
			ATOMIC_ADD(aggregates.data_voxel_count, 1u);
			ATOMIC_ADD(energies.total_data_energy, local_data_energy);
			ATOMIC_ADD(aggregates.data_voxel_count, 1u);
			ATOMIC_ADD(aggregates.cumulative_sdf_diff, sdf_difference_between_live_and_canonical);
			ATOMIC_ADD(energies.combined_data_length, ORUtils::length(local_data_energy_gradient));
			if (print_voxel_result) {
				PrintDataTermInformation(live_sdf_gradient, local_data_energy);
			}
		}
		// endregion
		// region =============================== LEVEL SET TERM =====================================================================================
		if (switches.enable_level_set_term && live_voxel.flags == VOXEL_NONTRUNCATED) {
			float live_sdf_gradient_norm_minus_unity;
			Matrix3f live_sdf_hessian;
			ComputeLevelSetEnergyGradient(local_level_set_energy_gradient, live_sdf_hessian, live_sdf_gradient_norm_minus_unity,
			                              voxel_position, live_sdf, live_voxels, live_index_data, live_cache, live_sdf_gradient,
			                              parameters.weight_level_set_term, sdf_unity, parameters.epsilon);
			//=================================== ENERGY =============================================================================================

			ATOMIC_ADD(aggregates.level_set_voxel_count, 1u);
			// E_{level_set}(Ψ) = 1/2 Σ_{Ψ} (|∇φ_{proj}(Ψ)| - 1)^2
			float local_level_set_energy = parameters.weight_level_set_term *
			                               0.5f * (live_sdf_gradient_norm_minus_unity * live_sdf_gradient_norm_minus_unity);
			ATOMIC_ADD(energies.total_level_set_energy, local_level_set_energy);
			if (print_voxel_result) {
				PrintLevelSetTermInformation(live_sdf_gradient, live_sdf_hessian, live_sdf_gradient_norm_minus_unity);
				printf("local level set energy: %s%E%s\n",
				       yellow, local_level_set_energy, reset);
			}
			ATOMIC_ADD(energies.combined_level_set_length, ORUtils::length(local_level_set_energy_gradient));
		}
		// endregion =================================================================================================================================

		// region =============================== SMOOTHING TERM (TIKHONOV & KILLING) ================================================================

		if (switches.enable_smoothing_term && iteration_index < iteration_bound) {
			if (switches.enable_Killing_field) {
				// region ============================== RETRIEVE & PROCESS NEIGHBOR'S WARPS =========================================================
				constexpr int neighborhood_size = 12;
				Vector3f neighbor_warp_updates[neighborhood_size];
				bool neighbors_known[neighborhood_size], neighbors_truncated[neighborhood_size], neighbors_allocated[neighborhood_size];

				// neighbor index:       0         1       2           3         4         5           6         7         8            9          10          11
				// neighbor position: (-1,0,0) (0,-1,0) (0,0,-1)   (1, 0, 0) (0, 1, 0) (0, 0, 1)   (1, 1, 0) (0, 1, 1) (1, 0, 1)   (-1, -1, 0) (0, -1, -1) (-1, 0, -1)
				FindLocal2ndDerivativeNeighborhoodWarpUpdate(
						neighbor_warp_updates/*x12*/, neighbors_known, neighbors_truncated, neighbors_allocated, voxel_position,
						warp_voxels, warp_index_data, warp_cache, canonical_voxels, canonical_index_data, canonical_cache);
				SetUnknownNeighborToCentralWarpUpdate<neighborhood_size>(neighbor_warp_updates, neighbors_known, warp_update);
				// endregion
				//================================= COMPUTE GRADIENT =================================================================================
				Vector3f laplacian, divergence_derivative;
				ComputeDampened_AKVF_EnergyGradient(local_smoothing_energy_gradient, laplacian, divergence_derivative, neighbor_warp_updates,
				                                    warp_update, parameters.Killing_dampening_factor, parameters.weight_smoothing_term);
				//=================================== ENERGY =========================================================================================
				// (dampened approximately-Killing energy)
				Matrix3f warp_update_Jacobian(0.0f);
				ComputeVectorJacobian(warp_update_Jacobian, neighbor_warp_updates);
				Matrix3f warp_Jacobian_transpose = warp_update_Jacobian.t();

				float local_Tikhonov_energy = parameters.weight_smoothing_term *
				                              dot(warp_update_Jacobian.getColumn(0),
				                                  warp_update_Jacobian.getColumn(0)) +
				                              dot(warp_update_Jacobian.getColumn(1),
				                                  warp_update_Jacobian.getColumn(1)) +
				                              dot(warp_update_Jacobian.getColumn(2),
				                                  warp_update_Jacobian.getColumn(2));

				float local_Killing_energy = parameters.Killing_dampening_factor * parameters.weight_smoothing_term *
				                             (dot(warp_Jacobian_transpose.getColumn(0),
				                                  warp_update_Jacobian.getColumn(0)) +
				                              dot(warp_Jacobian_transpose.getColumn(1),
				                                  warp_update_Jacobian.getColumn(1)) +
				                              dot(warp_Jacobian_transpose.getColumn(2),
				                                  warp_update_Jacobian.getColumn(2)));
				if (print_voxel_result) {
					PrintDampened_AKVF_TermInformation(neighbor_warp_updates, neighbors_known, neighbors_truncated, laplacian, divergence_derivative,
					                                   local_Tikhonov_energy, local_Killing_energy);
				}
				ATOMIC_ADD(energies.total_Tikhonov_energy, local_Tikhonov_energy);
				ATOMIC_ADD(energies.total_Killing_energy, local_Killing_energy);
				ATOMIC_ADD(energies.combined_smoothing_length, ORUtils::length(local_smoothing_energy_gradient));
			} else {
				// region ============================== RETRIEVE & PROCESS NEIGHBOR'S WARPS =========================================================
				constexpr int neighborhood_size = 6;
				Vector3f neighbor_warp_updates[neighborhood_size];
				bool neighbors_known[neighborhood_size], neighbors_truncated[neighborhood_size], neighbors_allocated[neighborhood_size];

				// neighbor index:        0        1       2           3         4         5           6         7         8             9         10          11
				// neighbor position: (-1,0,0) (0,-1,0) (0,0,-1)   (1, 0, 0) (0, 1, 0) (0, 0, 1)   (1, 1, 0) (0, 1, 1) (1, 0, 1)   (-1, -1, 0) (0, -1, -1) (-1, 0, -1)
				FindLocal1stDerivativeNeighborhoodWarpUpdate(
						neighbor_warp_updates/*x12*/, neighbors_known, neighbors_truncated, neighbors_allocated, voxel_position,
						warp_voxels, warp_index_data, warp_cache, canonical_voxels, canonical_index_data, canonical_cache);
				SetUnknownNeighborToCentralWarpUpdate<neighborhood_size>(neighbor_warp_updates, neighbors_known, warp_update);
				// endregion
				//================================= COMPUTE GRADIENT =================================================================================
				Vector3f warp_update_Laplacian;
				ComputeTikhonovEnergyGradient(local_smoothing_energy_gradient, warp_update_Laplacian, neighbor_warp_updates, warp_update,
				                              parameters.weight_smoothing_term);
				//=================================== ENERGY =========================================================================================
				Matrix3f warp_update_Jacobian(0.0f);
				ComputeVectorJacobian(warp_update_Jacobian, neighbor_warp_updates);
				if (print_voxel_result) {
					PrintTikhonovTermInformation(neighbor_warp_updates, warp_update_Laplacian);
				}

				float local_Tikhonov_energy = parameters.weight_smoothing_term *
				                              dot(warp_update_Jacobian.getColumn(0), warp_update_Jacobian.getColumn(0)) +
				                              dot(warp_update_Jacobian.getColumn(1), warp_update_Jacobian.getColumn(1)) +
				                              dot(warp_update_Jacobian.getColumn(2), warp_update_Jacobian.getColumn(2));
				ATOMIC_ADD(energies.total_Tikhonov_energy, local_Tikhonov_energy);
				ATOMIC_ADD(energies.combined_smoothing_length, ORUtils::length(local_smoothing_energy_gradient));
			}
		}
		// endregion
		// region =============================== COMPUTE ENERGY GRADIENT ============================================================================
		SetGradientFunctor<TWarp, TWarp::hasDebugInformation>::SetGradient(
				warp_voxel, local_data_energy_gradient, local_level_set_energy_gradient, local_smoothing_energy_gradient);

		// endregion
		// region =============================== AGGREGATE VOXEL STATISTICS =========================================================================

		float warp_length = ORUtils::length(warp_voxel.warp_update);

		ATOMIC_ADD(aggregates.cumulative_canonical_sdf, canonical_sdf);
		ATOMIC_ADD(aggregates.cumulative_live_sdf, live_sdf);
		ATOMIC_ADD(aggregates.cumulative_warp_dist, warp_length);
		ATOMIC_ADD(aggregates.considered_voxel_count, 1u);
		// endregion

		// region ======================== FINALIZE RESULT PRINTING / RECORDING ======================================================================

		if (print_voxel_result) {
			float energy_gradient_length = ORUtils::length(warp_voxel.gradient0);
			PrintLocalEnergyGradients(local_data_energy_gradient, local_level_set_energy_gradient,
			                          local_smoothing_energy_gradient, warp_voxel.gradient0, energy_gradient_length);
		}
		// endregion =================================================================================================================================
	}


	void PrintStatistics() {
		if (verbosity_level < VERBOSITY_PER_ITERATION) return;
		if (configuration::Get().logging_settings.log_surface_tracking_optimization_energies) {
			PrintEnergyStatistics(this->switches.enable_data_term, this->switches.enable_level_set_term,
			                      this->switches.enable_smoothing_term, this->switches.enable_Killing_field,
			                      this->parameters.Killing_dampening_factor, energies);
		}
		if (configuration::Get().logging_settings.log_additional_surface_tracking_stats) {
			CalculateAndPrintAdditionalStatistics(
					this->switches.enable_data_term, this->switches.enable_level_set_term, aggregates);
		}
	}

	void SaveStatistics() {
		auto& recorder = TelemetryRecorder<TVoxel, TWarp, TIndex, TMemoryDeviceType>::GetDefaultInstance();
		recorder.RecordSurfaceTrackingEnergies(energies, iteration_index);
		recorder.RecordSurfaceTrackingStatistics(aggregates, iteration_index);
	}


private:

	const float sdf_unity;
	// iteration_index and iteration_bound are used in the diagnostic-execution-mode warp gradient functor ONLY
	// iteration_index is used for logging per-iteration data, as well as, in combination with
	// iteration_bound to halt computation of certain terms for debugging purposes
	const int iteration_index;
	const int iteration_bound = 99999;

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

	AdditionalGradientAggregates<TMemoryDeviceType> aggregates;
	ComponentEnergies<TMemoryDeviceType> energies;

	// *** debugging / analysis variables
	bool use_focus_voxel{};
	Vector3i focus_voxel;

	const LevelSetAlignmentWeights parameters;
	const LevelSetAlignmentSwitches switches;

	const VerbosityLevel verbosity_level;
};

}// namespace ITMLib
