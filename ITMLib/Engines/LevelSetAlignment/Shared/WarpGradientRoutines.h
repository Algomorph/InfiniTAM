//  ================================================================
//  Created by Gregory Kramida on 10/15/19.
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

#include "../../../Utils/Math.h"
#include "../../../../ORUtils/PlatformIndependence.h"
#include "../../../../ORUtils/PlatformIndependentAtomics.h"
#include "WarpGradientAggregates.h"

_CPU_AND_GPU_CODE_
template<typename TVoxel, typename TIndexData, typename TCache>
inline
Vector3f computeDataTerm(Vector3f& liveSdfJacobian, float& sdfDifferenceBetweenLiveAndCanonical, float liveSdf,
                         float canonicalSdf, float weight,
                         const Vector3i& voxelPosition, const TVoxel* liveVoxels, const TIndexData* liveIndexData,
                         TCache liveCache) {

	ComputeLiveJacobian_CentralDifferences(
			liveSdfJacobian, voxelPosition, liveVoxels, liveIndexData, liveCache);
	// Compute data term error / energy
	sdfDifferenceBetweenLiveAndCanonical = liveSdf - canonicalSdf;
	// (φ_n(Ψ)−φ_{global}) ∇φ_n(Ψ) - also denoted as - (φ_{proj}(Ψ)−φ_{model}) ∇φ_{proj}(Ψ)
	// φ_n(Ψ) = φ_n(x+u, y+v, z+w), where u = u(x,y,z), v = v(x,y,z), w = w(x,y,z)
	// φ_{global} = φ_{global}(x, y, z)
	Vector3f localDataEnergyGradient = weight * sdfDifferenceBetweenLiveAndCanonical * liveSdfJacobian;
	return localDataEnergyGradient;
}

_CPU_AND_GPU_CODE_
template<typename TVoxel, typename TIndexData, typename TCache>
inline
Vector3f computeDataTerm(float liveSdf, float canonicalSdf, float weight,
                         const Vector3i& voxelPosition, const TVoxel* liveVoxels, const TIndexData* liveIndexData,
                         TCache liveCache) {
	Vector3f liveSdfJacobian;
	float sdfDifferenceBetweenLiveAndCanonical;
	return computeDataTerm(liveSdfJacobian, sdfDifferenceBetweenLiveAndCanonical, liveSdf, canonicalSdf, weight,
	                       voxelPosition, liveVoxels, liveIndexData, liveCache);
}

_CPU_AND_GPU_CODE_
inline
float computeDataEnergy(float weight, float sdfDifferenceBetweenLiveAndCanonical) {
	return weight * 0.5f * (sdfDifferenceBetweenLiveAndCanonical * sdfDifferenceBetweenLiveAndCanonical);
}

_CPU_AND_GPU_CODE_
template<typename TVoxel, typename TIndexData, typename TCache>
inline
Vector3f computeDataTerm(float& localDataEnergy, float liveSdf, float canonicalSdf, float weight,
                         const Vector3i& voxelPosition, const TVoxel* liveVoxels, const TIndexData* liveIndexData,
                         TCache liveCache) {
	Vector3f liveSdfJacobian;
	float sdfDifferenceBetweenLiveAndCanonical;
	Vector3f dataTermGradient = computeDataTerm(liveSdfJacobian, sdfDifferenceBetweenLiveAndCanonical, liveSdf,
	                                            canonicalSdf, weight, voxelPosition, liveVoxels, liveIndexData,
	                                            liveCache);

	localDataEnergy = computeDataEnergy(weight, sdfDifferenceBetweenLiveAndCanonical);
	return dataTermGradient;
}

_CPU_AND_GPU_CODE_
template<typename TVoxel, typename TIndexData, typename TCache>
inline
Vector3f computeDataTerm(ComponentEnergies& energies, float liveSdf, float canonicalSdf, float weight,
                         const Vector3i& voxelPosition, const TVoxel* liveVoxels, const TIndexData* liveIndexData,
                         TCache liveCache) {
	Vector3f liveSdfJacobian;
	float sdfDifferenceBetweenLiveAndCanonical;
	Vector3f dataTermGradient = computeDataTerm(liveSdfJacobian, sdfDifferenceBetweenLiveAndCanonical, liveSdf,
	                                            canonicalSdf, weight, voxelPosition, liveVoxels, liveIndexData,
	                                            liveCache);

	float localDataEnergy = computeDataEnergy(weight, sdfDifferenceBetweenLiveAndCanonical);
	ATOMIC_ADD(energies.total_data_energy, localDataEnergy);
	return dataTermGradient;
}


_CPU_AND_GPU_CODE_
template<typename TVoxel, typename TIndexData, typename TCache>
inline Vector3f
computeDataTerm(ComponentEnergies& energies, AdditionalGradientAggregates& aggregates, float liveSdf,
                         float canonicalSdf, float weight, const Vector3i& voxelPosition, const TVoxel* liveVoxels,
                         const TIndexData* liveIndexData, TCache liveCache) {
	Vector3f liveSdfJacobian;
	float sdfDifferenceBetweenLiveAndCanonical;
	Vector3f dataTermGradient = computeDataTerm(liveSdfJacobian, sdfDifferenceBetweenLiveAndCanonical, liveSdf,
	                                            canonicalSdf, weight, voxelPosition, liveVoxels, liveIndexData,
	                                            liveCache);

	float localDataEnergy = computeDataEnergy(weight, sdfDifferenceBetweenLiveAndCanonical);
	ATOMIC_ADD(energies.total_data_energy, localDataEnergy);
	ATOMIC_ADD(aggregates.data_voxel_count, 1u);
	ATOMIC_ADD(aggregates.cumulative_sdf_diff, std::abs(sdfDifferenceBetweenLiveAndCanonical));
	return dataTermGradient;
}

_CPU_AND_GPU_CODE_
template<typename TVoxel, typename TIndexData, typename TCache>
inline Vector3f
computeDataTerm(ComponentEnergies& energies, AdditionalGradientAggregates& aggregates, Vector3f& liveSdfJacobian,
                float liveSdf, float canonicalSdf, float weight, const Vector3i& voxelPosition, const TVoxel* liveVoxels,
                const TIndexData* liveIndexData, TCache liveCache) {
	float sdfDifferenceBetweenLiveAndCanonical;
	Vector3f dataTermGradient = computeDataTerm(liveSdfJacobian, sdfDifferenceBetweenLiveAndCanonical, liveSdf,
	                                            canonicalSdf, weight, voxelPosition, liveVoxels, liveIndexData,
	                                            liveCache);

	float localDataEnergy = computeDataEnergy(weight, sdfDifferenceBetweenLiveAndCanonical);
	ATOMIC_ADD(energies.total_data_energy, localDataEnergy);
	ATOMIC_ADD(aggregates.data_voxel_count, 1u);
	ATOMIC_ADD(aggregates.cumulative_sdf_diff, std::abs(sdfDifferenceBetweenLiveAndCanonical));
	return dataTermGradient;
}

