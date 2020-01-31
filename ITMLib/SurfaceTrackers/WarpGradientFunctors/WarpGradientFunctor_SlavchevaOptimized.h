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
#include "../Interface/SlavchevaSufraceTracker.h"
#include "../../../ORUtils/PlatformIndependentAtomics.h"
#include "../Shared/SurfaceTrackerSharedRoutines.h"
#include "../Shared/SurfaceTrackerDiagnosticRoutines.h"
#include "../Shared/ITMWarpGradientAggregates.h"
#include "../Shared/ITMWarpGradientCommon.h"
#include "../../Utils/VoxelFlags.h"
#include "../../Utils/CPrintHelpers.h"
#include "../../Engines/EditAndCopy/Shared/EditAndCopyEngine_Shared.h"
#include "../../Objects/Volume/VoxelVolume.h"


namespace ITMLib {


template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TMemoryDeviceType>
struct WarpGradientFunctor<TVoxel, TWarp, TIndex, TMemoryDeviceType, TRACKER_SLAVCHEVA_OPTIMIZED>{
private:


public:

	// region ========================================= CONSTRUCTOR ====================================================
	WarpGradientFunctor(SlavchevaSurfaceTracker::Parameters parameters,
	                    SlavchevaSurfaceTracker::Switches switches,
	                    VoxelVolume<TVoxel,TIndex>* liveVolume,
	                    VoxelVolume<TVoxel,TIndex>* canonicalVolume,
	                    VoxelVolume<TWarp,TIndex>* warpField,
	                    float voxelSize, float narrowBandHalfWidth) :
			parameters(parameters), switches(switches),
			liveVoxels(liveVolume->localVBA.GetVoxelBlocks()), liveIndexData(liveVolume->index.GetIndexData()),
			warps(warpField->localVBA.GetVoxelBlocks()), warpIndexData(warpField->index.GetIndexData()),
			canonicalVoxels(canonicalVolume->localVBA.GetVoxelBlocks()), canonicalIndexData(canonicalVolume->index.GetIndexData()),
			liveCache(), canonicalCache(),
			sdfUnity(voxelSize/narrowBandHalfWidth){}

	// endregion =======================================================================================================

	_DEVICE_WHEN_AVAILABLE_
	void operator()(TVoxel& voxelLive, TVoxel& voxelCanonical, TWarp& warp, Vector3i voxelPosition) {

		if (!VoxelIsConsideredForTracking(voxelCanonical, voxelLive)) return;
		bool computeDataAndLevelSetTerms = VoxelIsConsideredForDataTerm(voxelCanonical, voxelLive);

		Vector3f& framewiseWarp = warp.framewise_warp;
		float liveSdf = TVoxel::valueToFloat(voxelLive.sdf);
		float canonicalSdf = TVoxel::valueToFloat(voxelCanonical.sdf);

		// region =============================== DECLARATIONS & DEFAULTS FOR ALL TERMS ====================

		Vector3f localSmoothingEnergyGradient(0.0f), localDataEnergyGradient(0.0f), localLevelSetEnergyGradient(0.0f);
		// endregion

		if (computeDataAndLevelSetTerms) {
			// region =============================== DATA TERM ================================================

			Vector3f liveSdfJacobian;
			ComputeLiveJacobian_CentralDifferences(
					liveSdfJacobian, voxelPosition, liveVoxels, liveIndexData, liveCache);
			if (switches.enable_data_term) {

				// Compute data term error / energy
				float sdfDifferenceBetweenLiveAndCanonical = liveSdf - canonicalSdf;
				// (φ_n(Ψ)−φ_{global}) ∇φ_n(Ψ) - also denoted as - (φ_{proj}(Ψ)−φ_{model}) ∇φ_{proj}(Ψ)
				// φ_n(Ψ) = φ_n(x+u, y+v, z+w), where u = u(x,y,z), v = v(x,y,z), w = w(x,y,z)
				// φ_{global} = φ_{global}(x, y, z)
				localDataEnergyGradient =
						parameters.weight_data_term * sdfDifferenceBetweenLiveAndCanonical * liveSdfJacobian;
			}

			// endregion =======================================================================================
			// region =============================== LEVEL SET TERM ===========================================

			if (switches.enable_level_set_term) {
				Matrix3f liveSdfHessian;
				ComputeSdfHessian(liveSdfHessian, voxelPosition, liveSdf, liveVoxels, liveIndexData, liveCache);

				float sdfJacobianNorm = ORUtils::length(liveSdfJacobian);
				float sdfJacobianNormMinusUnity = sdfJacobianNorm - sdfUnity;
				localLevelSetEnergyGradient = parameters.weight_level_set_term * sdfJacobianNormMinusUnity *
				                              (liveSdfHessian * liveSdfJacobian) /
				                              (sdfJacobianNorm + parameters.epsilon);
			}
			// endregion =======================================================================================
		}

		// region =============================== SMOOTHING TERM (TIKHONOV & KILLING) ======================

		if (switches.enable_smoothing_term) {
			// region ============================== RETRIEVE NEIGHBOR'S WARPS =========================================

			const int neighborhoodSize = 9;
			Vector3f neighborFramewiseWarps[neighborhoodSize];
			bool neighborKnown[neighborhoodSize], neighborTruncated[neighborhoodSize], neighborAllocated[neighborhoodSize];

			//    0        1        2          3         4         5           6         7         8
			//(-1,0,0) (0,-1,0) (0,0,-1)   (1, 0, 0) (0, 1, 0) (0, 0, 1)   (1, 1, 0) (0, 1, 1) (1, 0, 1)
			findPoint2ndDerivativeNeighborhoodFramewiseWarp(
					neighborFramewiseWarps/*x9*/, neighborKnown, neighborTruncated, neighborAllocated, voxelPosition,
					warps, warpIndexData, warpCache, canonicalVoxels, canonicalIndexData, canonicalCache);

			for (int iNeighbor = 0; iNeighbor < neighborhoodSize; iNeighbor++) {
				if (!neighborAllocated[iNeighbor]) {
					//assign current warp to neighbor warp if the neighbor is not allocated
					neighborFramewiseWarps[iNeighbor] = framewiseWarp;
				}
			}
			//endregion=================================================================================================

			if (switches.enable_killing_rigidity_enforcement_term) {
				Matrix3f framewiseWarpJacobian(0.0f);
				Matrix3f framewiseWarpHessian[3] = {Matrix3f(0.0f), Matrix3f(0.0f), Matrix3f(0.0f)};
				ComputePerVoxelWarpJacobianAndHessian(framewiseWarp, neighborFramewiseWarps, framewiseWarpJacobian,
				                                      framewiseWarpHessian);

				float gamma = parameters.rigidity_enforcement_factor;
				float onePlusGamma = 1.0f + gamma;
				// |0, 3, 6|     |m00, m10, m20|      |u_xx, u_xy, u_xz|
				// |1, 4, 7|     |m01, m11, m21|      |u_xy, u_yy, u_yz|
				// |2, 5, 8|     |m02, m12, m22|      |u_xz, u_yz, u_zz|
				Matrix3f& H_u = framewiseWarpHessian[0];
				Matrix3f& H_v = framewiseWarpHessian[1];
				Matrix3f& H_w = framewiseWarpHessian[2];


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
				Matrix3f warpJacobianTranspose = framewiseWarpJacobian.t();

				float localTikhonovEnergy = parameters.weight_smoothing_term *
				                            dot(framewiseWarpJacobian.getColumn(0),
				                                framewiseWarpJacobian.getColumn(0)) +
				                            dot(framewiseWarpJacobian.getColumn(1),
				                                framewiseWarpJacobian.getColumn(1)) +
				                            dot(framewiseWarpJacobian.getColumn(2), framewiseWarpJacobian.getColumn(2));

				float localRigidityEnergy = gamma * parameters.weight_smoothing_term *
				                            (dot(warpJacobianTranspose.getColumn(0),
				                                 framewiseWarpJacobian.getColumn(0)) +
				                             dot(warpJacobianTranspose.getColumn(1),
				                                 framewiseWarpJacobian.getColumn(1)) +
				                             dot(warpJacobianTranspose.getColumn(2),
				                                 framewiseWarpJacobian.getColumn(2)));
			} else {
				Matrix3f framewiseWarpJacobian(0.0f);
				Vector3f framewiseWarpLaplacian;
				ComputeWarpLaplacianAndJacobian(framewiseWarpLaplacian, framewiseWarpJacobian, framewiseWarp,
				                                neighborFramewiseWarps);
				//∇E_{reg}(Ψ) = −[∆U ∆V ∆W]' ,
				localSmoothingEnergyGradient = -parameters.weight_smoothing_term * framewiseWarpLaplacian;
			}
		}
		// endregion
		// region =============================== COMPUTE ENERGY GRADIENT ==================================
		Vector3f localEnergyGradient =
				localDataEnergyGradient +
				localLevelSetEnergyGradient +
				localSmoothingEnergyGradient;

		warp.gradient0 = localEnergyGradient;
	}


	void PrintStatistics() {

	}


private:

	const float sdfUnity;

	// *** data structure accessors
	const TVoxel* liveVoxels;
	const typename TIndex::IndexData* liveIndexData;
	typename TIndex::IndexCache liveCache;

	const TVoxel* canonicalVoxels;
	const typename TIndex::IndexData* canonicalIndexData;
	typename TIndex::IndexCache canonicalCache;

	TWarp* warps;
	const typename TIndex::IndexData* warpIndexData;
	typename TIndex::IndexCache warpCache;


	const SlavchevaSurfaceTracker::Parameters parameters;
	const SlavchevaSurfaceTracker::Switches switches;
};

}// namespace ITMLib

