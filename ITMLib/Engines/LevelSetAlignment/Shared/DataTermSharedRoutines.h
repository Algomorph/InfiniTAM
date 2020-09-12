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

#include "../../../../ORUtils/PlatformIndependence.h"
#include "../../../Utils/Enums/VoxelFlags.h"

namespace ITMLib {


//#define DATA_CONDITION_IGNORE_ANY_UNKNOWN
#define DATA_CONDITION_LIVE_NONTRUNCATED_CANONICAL_KNOWN

template<typename TVoxel>
_CPU_AND_GPU_CODE_ inline
bool VoxelIsConsideredForDataTerm(const TVoxel& canonical_voxel, const TVoxel& live_voxel) {
//_DEBUG preprocessor options
#if defined(DATA_CONDITION_ALWAYS)
	return true;
#elif defined(DATA_CONDITION_IGNORE_ANY_UNKNOWN)
	return canonical_voxel.flags != ITMLib::VOXEL_UNKNOWN && live_voxel.flags != ITMLib::VOXEL_UNKNOWN;
#elif defined(DATA_CONDITION_LIVE_NONTRUNCATED_CANONICAL_KNOWN)
	return canonical_voxel.flags != ITMLib::VOXEL_UNKNOWN && live_voxel.flags == ITMLib::VOXEL_NONTRUNCATED;
#elif defined(DATA_CONDITION_ONLY_NONTRUNCATED)
	return live_voxel.flags == ITMLib::VOXEL_NONTRUNCATED
						 && canonical_voxel.flags == ITMLib::VOXEL_NONTRUNCATED;
#elif defined(DATA_CONDITION_IGNORE_BOTH_UNKNOWN)
	return canonical_voxel.flags != ITMLib::VOXEL_UNKNOWN || live_voxel.flags != ITMLib::VOXEL_UNKNOWN;
#elif defined(DATA_CONDITION_IGNORE_CANONICAL_UNKNOWN)
	return canonical_voxel.flags != ITMLib::VOXEL_UNKNOWN;
#else
	//Same as data condition DATA_CONDITION_ALWAYS
	return true;
#endif
};

// region ================================ DATA ENERGY GRADIENT ==============================================

_CPU_AND_GPU_CODE_ inline
void ComputeDataEnergyGradient(
		THREADPTR(Vector3f)& data_energy_gradient,
		THREADPTR(float)& sdf_difference_between_live_and_canonical,
		const CONSTPTR(float)& live_sdf,
		const CONSTPTR(float)& canonical_sdf,
		const CONSTPTR(float)& weight_data_term,
		const CONSTPTR(Vector3f)& live_sdf_gradient) {
	sdf_difference_between_live_and_canonical = live_sdf - canonical_sdf;
	// (φ_n(Ψ)−φ_{global}) ∇φ_n(Ψ) - also denoted as - (φ_{proj}(Ψ)−φ_{model}) ∇φ_{proj}(Ψ)
	// φ_{proj}(Ψ) = φ_{proj}(x+u, y+v, z+w), where u = u(x,y,z), v = v(x,y,z), w = w(x,y,z)
	// φ_{global} = φ_{global}(x, y, z)
	data_energy_gradient = weight_data_term * sdf_difference_between_live_and_canonical * live_sdf_gradient;
}


_CPU_AND_GPU_CODE_ inline
void ComputeDataEnergyGradient(THREADPTR(Vector3f)& data_energy_gradient,
                               const CONSTPTR(float)& live_sdf,
                               const CONSTPTR(float)& canonical_sdf,
                               const CONSTPTR(float)& weight_data_term,
                               const CONSTPTR(Vector3f)& live_sdf_gradient) {

	float sdf_difference_between_live_and_canonical;
	return ComputeDataEnergyGradient(data_energy_gradient, sdf_difference_between_live_and_canonical,
	                                 live_sdf, canonical_sdf, weight_data_term, live_sdf_gradient);
}

// endregion


} // namespace ITMLib