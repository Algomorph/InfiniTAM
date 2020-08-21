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

#include "../../../ORUtils/MemoryDeviceType.h"
#include "../../Objects/Volume/VoxelVolume.h"
#include "../../Utils/Enums/WarpType.h"

namespace ITMLib{
template<typename TVoxel, typename TWarp, typename TIndex>
class WarpingEngineInterface{
public:
	virtual ~WarpingEngineInterface() = default;
	/**
	* \brief Uses trilinear interpolation of the target TSDF volume at [source TSDF voxel positions + warp vector]
	*  to generate a new live SDF grid in the target scene. Uses cumulative / multi-frame warps as warp vectors.
	* \details Assumes target TSDF is empty / has been reset (does not reset).
	* Does 3 things:
	* <ol>
	*  <li> Traverses allocated canonical hash blocks and voxels, checks raw frame at [current voxel position + warp vector],
	*       if there is a non-truncated voxel in the live frame, marks the block in target SDF volume at the current hash
	*       block position for allocation
	*  <li> Traverses target hash blocks, if one is marked for allocation, allocates it
	*  <li> Traverses allocated target hash blocks and voxels, retrieves canonical voxel at same location, if it is marked
	*       as "truncated", skips it, otherwise uses trilinear interpolation at [current voxel position + warp vector] to
	*       retrieve SDF value from live frame, then stores it into the current target voxel, and marks latter voxel as
	*       truncated, non-truncated, or unknown based on the lookup flags & resultant SDF value.
	* </ol>
	* \param warpField  volume, containing the warp vectors
	* \param sourceTSDF source (current iteration live) scene
	* \param targetTSDF target (next iteration live) scene
	*/
	virtual void WarpVolume_CumulativeWarps(VoxelVolume<TWarp, TIndex>* warpField,
	                                        VoxelVolume<TVoxel, TIndex>* sourceTSDF,
	                                        VoxelVolume<TVoxel, TIndex>* targetTSDF) = 0;
/**
	 * \brief Uses trilinear interpolation of the target TSDF volume at [source TSDF voxel positions + warp vector]
	 *  to generate a new live SDF grid in the target scene. Uses framewise warps as warp vectors.
	 * \details Assumes target TSDF is empty / has been reset (does not reset).
	 * Does 3 things:
	 * <ol>
	 *  <li> Traverses allocated canonical hash blocks and voxels, checks raw frame at [current voxel position + warp vector],
	 *       if there is a non-truncated voxel in the live frame, marks the block in target SDF volume at the current hash
	 *       block position for allocation
	 *  <li> Traverses target hash blocks, if one is marked for allocation, allocates it
	 *  <li> Traverses allocated target hash blocks and voxels, retrieves canonical voxel at same location, if it is marked
	 *       as "truncated", skips it, otherwise uses trilinear interpolation at [current voxel position + warp vector] to
	 *       retrieve SDF value from live frame, then stores it into the current target voxel, and marks latter voxel as
	 *       truncated, non-truncated, or unknown based on the lookup flags & resultant SDF value.
	 * </ol>
	 * \param warpField  volume, containing the warp vectors
	 * \param sourceTSDF source (current iteration live) scene
	 * \param targetTSDF target (next iteration live) scene
	 */
	virtual void WarpVolume_FramewiseWarps(VoxelVolume<TWarp, TIndex>* warpField,
	                                       VoxelVolume<TVoxel, TIndex>* sourceTSDF,
	                                       VoxelVolume<TVoxel, TIndex>* targetTSDF) = 0;
	/**
	 * \brief Uses trilinear interpolation to set all locations within target TSDF volume
	 *  to interpolated value of the source TSDF volume at [source TSDF voxel positions + warp vector]
	 *  to generate a new live SDF grid in the target scene. Uses non-cumulative warp updates as warp vectors.
	 * \details Assumes target TSDF is empty / has been reset (does not reset).
	 * Does 3 things:
	 * <ol>
	 *  <li> Traverses allocated canonical hash blocks and voxels, checks raw frame at [current voxel position + warp vector],
	 *       if there is a non-truncated voxel in the live frame, marks the block in target SDF volume at the current hash
	 *       block position for allocation
	 *  <li> Traverses target hash blocks, if one is marked for allocation, allocates it
	 *  <li> Traverses allocated target hash blocks and voxels, retrieves canonical voxel at same location, if it is marked
	 *       as "truncated", skips it, otherwise uses trilinear interpolation at [current voxel position + warp vector] to
	 *       retrieve SDF value from live frame, then stores it into the current target voxel, and marks latter voxel as
	 *       truncated, non-truncated, or unknown based on the lookup flags & resultant SDF value.
	 * </ol>
	 * \param warpField  volume, containing the warp vectors
	 * \param sourceTSDF source (current iteration live) scene
	 * \param targetTSDF target (next iteration live) scene
	 */
	virtual void WarpVolume_WarpUpdates(VoxelVolume<TWarp, TIndex>* warpField,
	                                    VoxelVolume<TVoxel, TIndex>* sourceTSDF,
	                                    VoxelVolume<TVoxel, TIndex>* targetTSDF) = 0;
};

template<typename TVoxel, typename TWarp, typename TIndex, MemoryDeviceType TMemoryDeviceType>
class WarpingEngine :
		public WarpingEngineInterface<TVoxel, TWarp, TIndex>{
public:
	void WarpVolume_CumulativeWarps(VoxelVolume<TWarp, TIndex>* warpField,
	                                VoxelVolume<TVoxel, TIndex>* sourceTSDF,
	                                VoxelVolume<TVoxel, TIndex>* targetTSDF) override;

	void WarpVolume_FramewiseWarps(VoxelVolume<TWarp, TIndex>* warpField,
	                               VoxelVolume<TVoxel, TIndex>* sourceTSDF,
	                               VoxelVolume<TVoxel, TIndex>* targetTSDF) override;

	void WarpVolume_WarpUpdates(VoxelVolume<TWarp, TIndex>* warpField,
	                            VoxelVolume<TVoxel, TIndex>* sourceTSDF,
	                            VoxelVolume<TVoxel, TIndex>* targetTSDF) override;
private:
	template<WarpType TWarpType>
	void WarpVolume(VoxelVolume<TWarp,  TIndex>* warp_field,
	                VoxelVolume<TVoxel, TIndex>* source_volume,
	                VoxelVolume<TVoxel, TIndex>* target_volume);
};
} // namespace ITMLib