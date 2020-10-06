//  ================================================================
//  Created by Gregory Kramida on 10/18/17.
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

//local
#include "../../../Objects/Volume/VoxelVolume.h"
#include "../../../Utils/Logging/ConsolePrintColors.h"
#include "../../../Utils/Enums/VoxelFlags.h"
#include "LevelSetAlignmentParameters.h"
#include "../../Common/Configurable.h"

namespace ITMLib {
/**
 * \brief Class responsible for tracking motion of rigid or dynamic surfaces within the scene
 * \tparam TVoxel TSDF voxel type
 * \tparam TWarp Warping vector voxel type
 * \tparam TIndex Indexing structure type used for voxel volumes
 */
template<typename TVoxel, typename TWarp, typename TIndex>
class LevelSetAlignmentEngineInterface : public Configurable<LevelSetAlignmentParameters> {

public:
	using Configurable<LevelSetAlignmentParameters>::Configurable;

	virtual ~LevelSetAlignmentEngineInterface() = default;

	/**
	 * \brief Iteratively align TSDF values initially inside the volume live_volume_pair[0] with
	 *  values in the canonical_volume, estimating a vector field inside the warp_field and using
	 *  trilinear interpolation to update the live volume at each iteration.
	 *
	 *  \details Bounces back-and-forth between the volumes in the live_volume_pair, i.e.
	 *  the result of live volume from the previous iteration is used as the source live volume
	 *  in the next iteration.
	 *
	 *  Expects all volumes to be properly allocated.
	 *
	 * \param warp_field warp field used to store updates at each iteration
	 * \param live_volume_pair volume pair, with the original source volume being in the first index
	 * \param canonical_volume target volume to warp towards
	 * \return pointer to the volume in the live_volume_pair that contains the final (aligned) result.
	 */
	virtual VoxelVolume<TVoxel, TIndex>* Align(
			VoxelVolume<TWarp, TIndex>* warp_field,
			VoxelVolume<TVoxel, TIndex>** live_volume_pair,
			VoxelVolume<TVoxel, TIndex>* canonical_volume) = 0;

	/**
	 * \brief Clear out the warp_update field of all voxels in the provided warp_field.
	 * \param warp_field the volume whose warp updates to clear out.
	 */
	virtual void ClearOutWarpUpdates(VoxelVolume<TWarp, TIndex>* warp_field) const = 0;

};


}//namespace ITMLib



