//  ================================================================
//  Created by Gregory Kramida on 1/29/20.
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

// local
#include "../../../ORUtils/MemoryDeviceType.h"
#include "../../../ORUtils/PlatformIndependence.h"
#include "../../Utils/ITMVoxelFlags.h"
#include "../Common/ITMWarpEnums.h"
#include "../../Objects/Scene/ITMVoxelVolume.h"
#include "../../Objects/Scene/ITMTrilinearInterpolation.h"
#include "../../Utils/Configuration.h"
#include "../../../ORUtils/PlatformIndependentAtomics.h"
#include "../Common/ITMCommonFunctors.h"

namespace ITMLib {

// region ====================== TRILINEAR INTERPOLATION BASED ON WARP VECTOR ==========================================
template<typename TVoxel, typename TWarp, typename TIndex, ITMLib::WarpType TWarpType>
_CPU_AND_GPU_CODE_
inline void interpolateTSDFVolume(TVoxel* sdfSourceVoxels,
                                  const typename TIndex::IndexData* sdfSourceIndexData,
                                  typename TIndex::IndexCache& sourceTSDFCache,
                                  const TWarp& warpVoxel,
                                  TVoxel& destinationVoxel,
                                  const Vector3i& warpAndDestinationVoxelPosition,
                                  bool printResult) {

	Vector3f warpVector = ITMLib::WarpVoxelStaticFunctor<TWarp, TWarpType>::GetWarp(warpVoxel);

	if (ORUtils::length(warpVector) < 1e-5f) {
		int vmIndex;
#if !defined(__CUDACC__) && !defined(WITH_OPENMP)
		const TVoxel& sourceTSDFVoxelAtSameLocation = readVoxel(sdfSourceVoxels, sdfSourceIndexData,
		                                                        warpAndDestinationVoxelPosition,
		                                                        vmIndex, sourceTSDFCache);
#else //don't use cache when multithreading!
		const TVoxel& sourceTSDFVoxelAtSameLocation = readVoxel(sdfSourceVoxels, sdfSourceIndexData,
		                                                        warpAndDestinationVoxelPosition,
		                                                        vmIndex);
#endif
		destinationVoxel.sdf = sourceTSDFVoxelAtSameLocation.sdf;
		destinationVoxel.flags = sourceTSDFVoxelAtSameLocation.flags;
		return;
	}
	Vector3f warpedPosition = TO_FLOAT3(warpAndDestinationVoxelPosition) + warpVector;
	bool struckKnown;

	float sdf = InterpolateTrilinearly_StruckKnown(
			sdfSourceVoxels, sdfSourceIndexData, warpedPosition, sourceTSDFCache, struckKnown
	);

	// Update flags
	if (struckKnown) {
		destinationVoxel.sdf = TVoxel::floatToValue(sdf);
		if (1.0f - std::abs(sdf) < 1e-5f) {
			destinationVoxel.flags = ITMLib::VOXEL_TRUNCATED;
		} else {
			destinationVoxel.flags = ITMLib::VOXEL_NONTRUNCATED;
		}
	}
}


// endregion ===========================================================================================================


// MemoryDeviceType template parameter needed to disambiguate linker symbols for which PlatformIndependence macros are
// defined differently
template<typename TVoxel, MemoryDeviceType TMemoryDeviceType>
struct FieldClearFunctor {
	FieldClearFunctor() {}

	_CPU_AND_GPU_CODE_
	void operator()(TVoxel& voxel) {
		voxel.flags = VOXEL_UNKNOWN;
		voxel.sdf = TVoxel::SDF_initialValue();
	}
};

// MemoryDeviceType template parameter needed to disambiguate linker symbols for which PlatformIndependence macros are
// defined differently
template<typename TVoxel, typename TWarp, typename TIndex, ITMLib::WarpType TWarpType, MemoryDeviceType TMemoryDeviceType>
struct TrilinearInterpolationFunctor {
	/**
	 * \brief Initialize to transfer data from source sdf scene to a target sdf scene using the warps in the warp source scene
	 * \details traverses
	 * \param sourceTSDF
	 * \param warpSourceScene
	 */
	TrilinearInterpolationFunctor(ITMVoxelVolume<TVoxel, TIndex>* sourceTSDF,
	                              ITMVoxelVolume<TWarp, TIndex>* warpField) :

			sdfSourceScene(sourceTSDF),
			sdfSourceVoxels(sourceTSDF->localVBA.GetVoxelBlocks()),
			sdfSourceIndexData(sourceTSDF->index.GetIndexData()),
			sdfSourceCache(),

			warpSourceScene(warpField),
			warpSourceVoxels(warpField->localVBA.GetVoxelBlocks()),
			warpSourceHashEntries(warpField->index.GetIndexData()),
			warpSourceCache(),

			useFocusCoordinates(ITMLib::configuration::get().verbosity_level >= ITMLib::configuration::VERBOSITY_FOCUS_SPOTS),
			focusCoordinates(ITMLib::configuration::get().telemetry_settings.focus_coordinates) {
	}


	_DEVICE_WHEN_AVAILABLE_
	void operator()(TVoxel& destinationVoxel, TWarp& warp,
	                Vector3i warpAndDestinationVoxelPosition) {

		bool printResult = useFocusCoordinates && warpAndDestinationVoxelPosition == focusCoordinates;

		interpolateTSDFVolume<TVoxel, TWarp, TIndex, TWarpType>(
				sdfSourceVoxels, sdfSourceIndexData, sdfSourceCache, warp, destinationVoxel,
				warpAndDestinationVoxelPosition, printResult);
	}

private:


	ITMLib::ITMVoxelVolume<TVoxel, TIndex>* sdfSourceScene;
	TVoxel* sdfSourceVoxels;
	typename TIndex::IndexData* sdfSourceIndexData;
	typename TIndex::IndexCache sdfSourceCache;

	ITMLib::ITMVoxelVolume<TWarp, TIndex>* warpSourceScene;
	TWarp* warpSourceVoxels;
	typename TIndex::IndexData* warpSourceHashEntries;
	typename TIndex::IndexCache warpSourceCache;


	//_DEBUG
	bool useFocusCoordinates;
	Vector3i focusCoordinates;
};

} // namespace ITMLib
