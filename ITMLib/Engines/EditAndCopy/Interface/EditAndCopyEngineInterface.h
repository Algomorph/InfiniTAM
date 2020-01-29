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

#include "../../../Utils/ITMMath.h"
#include "../../../Objects/Scene/ITMVoxelVolume.h"

namespace ITMLib {

template<typename TVoxel, typename TIndex>
class EditAndCopyEngineInterface {
	/**
	 * \brief Interface to engines implementing basic scene manipulation routines for different scene types.
	 */
public:
	EditAndCopyEngineInterface() = default;
	virtual ~EditAndCopyEngineInterface() = default;
	// TODO: better to make ResetScene a function of Scene (and template Scene on device type as well)
	/**
	 * \brief Clear out scene and reset the index
	 * \param scene
	 */
	virtual void ResetScene(ITMVoxelVolume <TVoxel, TIndex>* scene) = 0;

	/**
	 * \brief Set a single voxel at the desired location to the desired value
	 * \param scene the target voxel volume
	 * \param at the target voxel coordinate
	 * \param voxel the target voxel value
	 * \return true on success, false otherwise
	 */
	virtual bool SetVoxel(ITMVoxelVolume <TVoxel, TIndex>* scene, Vector3i at, TVoxel voxel) = 0;

	/**
	 * \brief Read voxel at the desired location
	 * \details Returns a default-value voxel (initialized using the basic constructor of the voxel class) if the location
	 * does not fall within the allocated memory of the voxel volume
	 * \param scene voxel volume to query
	 * \param at coordinate to query
	 * \return value of the voxel at desired location
	 */
	virtual TVoxel ReadVoxel(ITMVoxelVolume <TVoxel, TIndex>* scene, Vector3i at) = 0;
	virtual TVoxel ReadVoxel(ITMVoxelVolume <TVoxel, TIndex>* scene, Vector3i at,
			typename TIndex::IndexCache& cache) = 0;


	/**
	 * \brief offset warps by a fixed amount in each direction
	 * \param scene the scene to modify
	 * \param offset the offset vector to use
	 */
	virtual void OffsetWarps(ITMVoxelVolume <TVoxel, TIndex>* scene, Vector3f offset) = 0;

	/**
	 * \brief Copies the slice (box-like window) specified by points extremum1 and extremum2 from the source scene into a
	 * destination scene. Clears the destination scene before copying.
	 * \tparam TVoxel type of voxel
	 * \tparam TIndex type of voxel index
	 * \param destination destination voxel grid (can be uninitialized)
	 * \param source source voxel grid
	 * \param bounds minimum point in the desired slice (inclusive), i.e. minimum x, y, and z coordinates
	 * \param maxPoint maximum point in the desired slice (inclusive), i.e. maximum x, y, and z coordinates
	 * \return true on success (destination scene contains the slice), false on failure (there are no allocated hash blocks
	 */
	virtual bool
	CopySceneSlice(ITMVoxelVolume <TVoxel, TIndex>* destination, ITMVoxelVolume <TVoxel, TIndex>* source,
	               Vector6i bounds, const Vector3i& offset = Vector3i(0)) = 0;
	virtual bool CopyScene(ITMVoxelVolume <TVoxel, TIndex>* destination,
	                       ITMVoxelVolume <TVoxel, TIndex>* source,
	                       const Vector3i& offset = Vector3i(0)) = 0;

};

} // end namespace ITMLib

