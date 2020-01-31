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
#include "../Shared/ITMWarpGradientAggregates.h"
#include "../Shared/ITMWarpGradientCommon.h"
#include "../../Utils/ITMVoxelFlags.h"
#include "../../Utils/ITMCPrintHelpers.h"
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


template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TMemoryDeviceType>
struct WarpGradientFunctor<TVoxel, TWarp, TIndex, TMemoryDeviceType, TRACKER_SLAVCHEVA_DIAGNOSTIC> {
private:

	_CPU_AND_GPU_CODE_
	void SetUpFocusVoxelPrinting(bool& printVoxelResult, const Vector3i& voxelPosition,
	                             const Vector3f& voxelWarp, const TVoxel& canonicalVoxel, const TVoxel& liveVoxel,
	                             bool computeDataTerm) {
		if (useFocusCoordinates && voxelPosition == focusCoordinates) {
			int x = 0, y = 0, z = 0, vmIndex = 0, locId = 0;
			GetVoxelHashLocals(vmIndex, locId, x, y, z, liveIndexData, liveCache, voxelPosition);

			printf("\n%s *** Printing gradient computation data for voxel at (%d, %d, %d) ***%s\n", c_bright_cyan,
			       voxelPosition.x, voxelPosition.y, voxelPosition.z, c_reset);
			printf("Computing data term: %s\n", (computeDataTerm ? "true" : "false"));
			printf("Position within block (x,y,z): (%d, %d, %d)\n", x, y, z);
			printf("Canonical vs. Live: \n");
			printf("TSDF: %f vs %f.\n", canonicalVoxel.sdf, liveVoxel.sdf);
			printf("Flags: %s vs. %s\n", VoxelFlagsAsCString(static_cast<VoxelFlags>(canonicalVoxel.flags)),
			       VoxelFlagsAsCString(static_cast<VoxelFlags>(liveVoxel.flags)));
			printf("===================\n");
			printf("Warping: %s%f, %f, %f%s\n", c_green, voxelWarp.x, voxelWarp.y, voxelWarp.z, c_reset);
			printf("Warping length: %s%f%s\n", c_green, ORUtils::length(voxelWarp), c_reset);

			printVoxelResult = true;
		}
	}


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
			useFocusCoordinates(configuration::get().verbosity_level >= configuration::VERBOSITY_FOCUS_SPOTS),
			focusCoordinates(configuration::get().telemetry_settings.focus_coordinates),
			sdfUnity(voxelSize/narrowBandHalfWidth),
			verbosity_level(configuration::get().verbosity_level)
			{}

	// endregion =======================================================================================================

	_DEVICE_WHEN_AVAILABLE_
	void operator()(TVoxel& voxelLive, TVoxel& voxelCanonical, TWarp& warp, const Vector3i& voxelPosition) {


		bool printVoxelResult = false;
		Vector3f& framewiseWarp = warp.framewise_warp;
		bool computeDataAndLevelSetTerms = VoxelIsConsideredForDataTerm(voxelCanonical, voxelLive);
		this->SetUpFocusVoxelPrinting(printVoxelResult, voxelPosition, framewiseWarp, voxelCanonical, voxelLive, computeDataAndLevelSetTerms);
		if (printVoxelResult) {
			printf("%sLive 6-connected neighbor information:%s\n", c_blue, c_reset);
			print6ConnectedNeighborInfo(voxelPosition, liveVoxels, liveIndexData, liveCache);
		}
		if (!VoxelIsConsideredForTracking(voxelCanonical, voxelLive)) return;

		float liveSdf = TVoxel::valueToFloat(voxelLive.sdf);
		float canonicalSdf = TVoxel::valueToFloat(voxelCanonical.sdf);

		// term gradient results are stored here before being added up
		Vector3f localSmoothingEnergyGradient(0.0f), localDataEnergyGradient(0.0f), localLevelSetEnergyGradient(0.0f);



		// region =============================== DATA TERM ================================================
		if (computeDataAndLevelSetTerms) {
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

				ATOMIC_ADD(aggregates.dataVoxelCount, 1u);

				float localDataEnergy = parameters.weight_data_term * 0.5f *
				                        (sdfDifferenceBetweenLiveAndCanonical * sdfDifferenceBetweenLiveAndCanonical);


				ATOMIC_ADD(energies.totalDataEnergy, localDataEnergy);

				ATOMIC_ADD(aggregates.dataVoxelCount, 1u);
				ATOMIC_ADD(aggregates.cumulativeSdfDiff, sdfDifferenceBetweenLiveAndCanonical);

				if (printVoxelResult) {
					_DEBUG_PrintDataTermStuff(liveSdfJacobian);
				}
			}

			// endregion

			// region =============================== LEVEL SET TERM ===========================================

			if (switches.enable_level_set_term) {
				Matrix3f liveSdfHessian;
				ComputeSdfHessian(liveSdfHessian, voxelPosition, liveSdf, liveVoxels, liveIndexData, liveCache);

				float sdfJacobianNorm = ORUtils::length(liveSdfJacobian);
				float sdfJacobianNormMinusUnity = sdfJacobianNorm - sdfUnity;
				localLevelSetEnergyGradient = parameters.weight_level_set_term * sdfJacobianNormMinusUnity *
				                              (liveSdfHessian * liveSdfJacobian) /
				                              (sdfJacobianNorm + parameters.epsilon);
				ATOMIC_ADD(aggregates.levelSetVoxelCount, 1u);
				float localLevelSetEnergy = parameters.weight_level_set_term *
						0.5f * (sdfJacobianNormMinusUnity * sdfJacobianNormMinusUnity);
				ATOMIC_ADD(energies.totalLevelSetEnergy, localLevelSetEnergy);
				if (printVoxelResult) {
					_DEBUG_PrintLevelSetTermStuff(liveSdfJacobian, liveSdfHessian, sdfJacobianNormMinusUnity);
				}
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
				if (!neighborKnown[iNeighbor]) {
					//assign current warp to neighbor warp if the neighbor is not known
					neighborFramewiseWarps[iNeighbor] = framewiseWarp;
				}
			}
			//endregion=================================================================================================

			if (switches.enable_killing_rigidity_enforcement_term) {
				Matrix3f framewiseWarpJacobian(0.0f);
				Matrix3f framewiseWarpHessian[3] = {Matrix3f(0.0f), Matrix3f(0.0f), Matrix3f(0.0f)};
				ComputePerVoxelWarpJacobianAndHessian(framewiseWarp, neighborFramewiseWarps, framewiseWarpJacobian,
				                                      framewiseWarpHessian);
//TODO rewrite function to be used within CUDA code, remove guards
#ifndef __CUDACC__
				if (printVoxelResult) {
					_DEBUG_PrintKillingTermStuff(neighborFramewiseWarps, neighborKnown, neighborTruncated,
					                             framewiseWarpJacobian, framewiseWarpHessian);
				}
#endif

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
				ATOMIC_ADD(energies.totalTikhonovEnergy, localTikhonovEnergy);
				ATOMIC_ADD(energies.totalRigidityEnergy, localRigidityEnergy);
			} else {
				Matrix3f framewiseWarpJacobian(0.0f);
				Vector3f framewiseWarpLaplacian;
				ComputeWarpLaplacianAndJacobian(framewiseWarpLaplacian, framewiseWarpJacobian, framewiseWarp,
				                                neighborFramewiseWarps);


				if (printVoxelResult) {
					_DEBUG_PrintTikhonovTermStuff(neighborFramewiseWarps, framewiseWarpLaplacian);
				}

				//∇E_{reg}(Ψ) = −[∆U ∆V ∆W]' ,
				localSmoothingEnergyGradient = -parameters.weight_smoothing_term * framewiseWarpLaplacian;
				float localTikhonovEnergy = parameters.weight_smoothing_term *
						dot(framewiseWarpJacobian.getColumn(0), framewiseWarpJacobian.getColumn(0)) +
						dot(framewiseWarpJacobian.getColumn(1), framewiseWarpJacobian.getColumn(1)) +
						dot(framewiseWarpJacobian.getColumn(2), framewiseWarpJacobian.getColumn(2));
				ATOMIC_ADD(energies.totalTikhonovEnergy, localTikhonovEnergy);
			}
		}
		// endregion
		// region =============================== COMPUTE ENERGY GRADIENT ==================================
		SetGradientFunctor<TWarp, TWarp::hasDebugInformation>::SetGradient(
				warp, localDataEnergyGradient, localLevelSetEnergyGradient, localSmoothingEnergyGradient);

		// endregion
		// region =============================== AGGREGATE VOXEL STATISTICS ===============================


		float warpLength = ORUtils::length(warp.framewise_warp);

		ATOMIC_ADD(aggregates.cumulativeCanonicalSdf, canonicalSdf);
		ATOMIC_ADD(aggregates.cumulativeLiveSdf, liveSdf);
		ATOMIC_ADD(aggregates.cumulativeWarpDist, warpLength);
		ATOMIC_ADD(aggregates.consideredVoxelCount, 1u);
		// endregion

		// region ======================== FINALIZE RESULT PRINTING / RECORDING ========================================

		if (printVoxelResult) {
			float energyGradientLength = ORUtils::length(warp.gradient0);
			_DEBUG_printLocalEnergyGradients(localDataEnergyGradient, localLevelSetEnergyGradient,
			                                 localSmoothingEnergyGradient, warp.gradient0, energyGradientLength);
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

	AdditionalGradientAggregates<TMemoryDeviceType> aggregates;
	ComponentEnergies<TMemoryDeviceType> energies;

	// *** debugging / analysis variables
	bool useFocusCoordinates{};
	Vector3i focusCoordinates;

	const SlavchevaSurfaceTracker::Parameters parameters;
	const SlavchevaSurfaceTracker::Switches switches;

	const configuration::VerbosityLevel verbosity_level;
};

}// namespace ITMLib
