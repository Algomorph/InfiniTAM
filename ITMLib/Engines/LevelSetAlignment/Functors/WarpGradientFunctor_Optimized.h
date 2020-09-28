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
#include "../Interface/LevelSetAlignmentParameters.h"
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
#include "../../EditAndCopy/Shared/EditAndCopyEngine_Shared.h"
#include "../../../Objects/Volume/VoxelVolume.h"


namespace ITMLib {


template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TMemoryDeviceType>
struct WarpGradientFunctor<TVoxel, TWarp, TIndex, TMemoryDeviceType, OPTIMIZED> {
private:


public:
	// region ========================================= CONSTRUCTOR ====================================================
	WarpGradientFunctor(LevelSetAlignmentWeights parameters,
	                    LevelSetAlignmentSwitches switches,
	                    VoxelVolume<TWarp, TIndex>* warp_field,
	                    VoxelVolume<TVoxel, TIndex>* canonical_volume,
	                    VoxelVolume<TVoxel, TIndex>* live_volume,
	                    float voxelSize, float truncation_distance, int iteration_index) :
			parameters(parameters), switches(switches),
			live_voxels(live_volume->GetVoxels()), live_index_data(live_volume->index.GetIndexData()),
			warp_voxels(warp_field->GetVoxels()), warp_index_data(warp_field->index.GetIndexData()),
			canonical_voxels(canonical_volume->GetVoxels()), canonical_index_data(canonical_volume->index.GetIndexData()),
			live_cache(), canonical_cache(),
			sdf_unity(voxelSize / truncation_distance), iteration_index(iteration_index) {}

	// endregion =======================================================================================================

	_DEVICE_WHEN_AVAILABLE_
	void operator()(TWarp& warp_voxel, TVoxel& canonical_voxel, TVoxel& live_voxel, const Vector3i& voxel_position) {

		if (!VoxelIsConsideredForAlignment(canonical_voxel, live_voxel)) return;

		Vector3f& warp_update = warp_voxel.warp_update;
		float live_sdf = TVoxel::valueToFloat(live_voxel.sdf);
		float canonical_sdf = TVoxel::valueToFloat(canonical_voxel.sdf);

		Vector3f local_smoothing_energy_gradient(0.0f), local_data_energy_gradient(0.0f), local_level_set_energy_gradient(0.0f);

		Vector3f live_sdf_gradient;
		ComputeSdfGradient(live_sdf_gradient, voxel_position, live_sdf, live_voxels, live_index_data, live_cache);

		// region =============================== DATA TERM ==========================================================================================
		if (VoxelIsConsideredForDataTerm(canonical_voxel, live_voxel) && switches.enable_data_term) {
			ComputeDataEnergyGradient(local_data_energy_gradient, live_sdf, canonical_sdf, parameters.weight_data_term, live_sdf_gradient);
		}

		// endregion =================================================================================================================================
		// region =============================== LEVEL SET TERM =====================================================================================
		if (switches.enable_level_set_term && live_voxel.flags == VOXEL_NONTRUNCATED) {
			ComputeLevelSetEnergyGradient(local_level_set_energy_gradient, voxel_position, live_sdf, live_voxels, live_index_data, live_cache,
			                              live_sdf_gradient, parameters.weight_level_set_term, sdf_unity, parameters.epsilon);
		}
		// endregion =================================================================================================================================

		// region =============================== SMOOTHING TERM (TIKHONOV & KILLING) ================================================================

		if (switches.enable_smoothing_term) {
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
				ComputeDampened_AKVF_EnergyGradient(local_smoothing_energy_gradient, neighbor_warp_updates, warp_update,
				                                    parameters.Killing_dampening_factor, parameters.weight_smoothing_term);
			} else {
				// region ============================== RETRIEVE & PROCESS NEIGHBOR'S WARPS =========================================================
				constexpr int neighborhood_size = 6;
				Vector3f neighbor_warp_updates[neighborhood_size];
				bool neighbors_known[neighborhood_size], neighbors_truncated[neighborhood_size], neighbors_allocated[neighborhood_size];

				// neighbor index:       0         1       2           3         4         5
				// neighbor position: (-1,0,0) (0,-1,0) (0,0,-1)   (1, 0, 0) (0, 1, 0) (0, 0, 1)
				FindLocal1stDerivativeNeighborhoodWarpUpdate(
						neighbor_warp_updates/*x12*/, neighbors_known, neighbors_truncated, neighbors_allocated, voxel_position,
						warp_voxels, warp_index_data, warp_cache, canonical_voxels, canonical_index_data, canonical_cache);
				SetUnknownNeighborToCentralWarpUpdate<neighborhood_size>(neighbor_warp_updates, neighbors_known, warp_update);
				// endregion
				//================================= COMPUTE GRADIENT =================================================================================
				ComputeTikhonovEnergyGradient(local_smoothing_energy_gradient, neighbor_warp_updates, warp_update, parameters.weight_smoothing_term);
			}
		}
		// endregion
		// region =============================== COMPUTE ENERGY GRADIENT ============================================================================
		Vector3f local_energy_gradient = local_data_energy_gradient + local_level_set_energy_gradient + local_smoothing_energy_gradient;
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


	const LevelSetAlignmentWeights parameters;
	const LevelSetAlignmentSwitches switches;
};

}// namespace ITMLib

