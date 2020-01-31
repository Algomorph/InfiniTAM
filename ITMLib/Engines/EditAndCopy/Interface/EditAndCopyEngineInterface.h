//  ================================================================
//  Created by Gregory Kramida on 9/26/19.
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
#include "../../../Objects/Volume/VoxelVolume.h"

namespace ITMLib {

template<typename TVoxel, typename TIndex>
class EditAndCopyEngineInterface {
	/**
	 * \brief Interface to engines implementing basic scene manipulation & copying routines for voxel volumes.
	 */
public:
	EditAndCopyEngineInterface() = default;
	virtual ~EditAndCopyEngineInterface() = default;
	/**
	 * \brief Clear out volume and reset the index
	 * \param volume
	 */
	virtual void ResetVolume(VoxelVolume <TVoxel, TIndex>* volume) = 0;

	/**
	 * \brief Set a single voxel at the desired location to the desired value
	 * \param volume the target voxel volume
	 * \param at the target voxel coordinate
	 * \param voxel the target voxel value
	 * \return true on success, false otherwise
	 */
	virtual bool SetVoxel(VoxelVolume <TVoxel, TIndex>* volume, Vector3i at, TVoxel voxel) = 0;

	/**
	 * \brief Read voxel at the desired location
	 * \details Returns a default-value voxel (initialized using the basic constructor of the voxel class) if the location
	 * does not fall within the allocated memory of the voxel volume
	 * \param volume voxel volume to query
	 * \param at coordinate to query
	 * \return value of the voxel at desired location
	 */
	virtual TVoxel ReadVoxel(VoxelVolume <TVoxel, TIndex>* volume, Vector3i at) = 0;
	virtual TVoxel ReadVoxel(VoxelVolume <TVoxel, TIndex>* volume, Vector3i at,
	                         typename TIndex::IndexCache& cache) = 0;


	/**
	 * \brief offset warps by a fixed amount in each direction
	 * \param volume the scene to modify
	 * \param offset the offset vector to use
	 */
	virtual void OffsetWarps(VoxelVolume <TVoxel, TIndex>* volume, Vector3f offset) = 0;

	/**
	 * \brief Copies the slice (box-like window) specified by points extremum1 and extremum2 from the source scene into a
	 * destination scene. Clears the destination scene before copying.
	 * \tparam TVoxel type of voxel
	 * \tparam TIndex type of voxel index
	 * \param targetVolume destination voxel grid (can be uninitialized)
	 * \param sourceVolume source voxel grid
	 * \param bounds minimum point in the desired slice (inclusive), i.e. minimum x, y, and z coordinates
	 * \param maxPoint maximum point in the desired slice (inclusive), i.e. maximum x, y, and z coordinates
	 * \return true on success (destination scene contains the slice), false on failure (there are no allocated hash blocks
	 */
	virtual bool
	CopyVolumeSlice(VoxelVolume <TVoxel, TIndex>* targetVolume, VoxelVolume <TVoxel, TIndex>* sourceVolume,
	                Vector6i bounds, const Vector3i& offset = Vector3i(0)) = 0;
	virtual bool CopyVolume(VoxelVolume <TVoxel, TIndex>* targetVolume,
	                        VoxelVolume <TVoxel, TIndex>* sourceVolume,
	                        const Vector3i& offset = Vector3i(0)) = 0;

};

} // end namespace ITMLib

