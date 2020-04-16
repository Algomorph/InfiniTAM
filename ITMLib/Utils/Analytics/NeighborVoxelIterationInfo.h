//  ================================================================
//  Created by Gregory Kramida on 2/5/18.
//  Copyright (c) 2018-2000 Gregory Kramida
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

#include <array>
#include "../Math.h"

namespace ITMLib{



/**
 * \brief Used for logging information about a neighbor of a specific voxel during the scene tracking optimization for dynamic fusion
 */
struct NeighborVoxelIterationInfo {
public:
	int hash;
	int localId;
	Vector3f warp;
	Vector3f warpGradient;
	float sdf;
	float liveSdf;//looked up using the warp
	bool struckKnownVoxels;
	bool struckNarrowBand;

	bool unknown = false;
	bool notAllocated = false;

	bool operator==(const NeighborVoxelIterationInfo& rhs) const;
	bool operator!=(const NeighborVoxelIterationInfo& rhs) const;

};

/**
 * \brief Used for logging information about a specific voxel during the scene tracking optimization for dynamic fusion
 * Records information influencing this voxel's warp, i.e. warp updates & energy term contributions, neighboring voxels,
 * etc.
 */
struct ITMHighlightIterationInfo{
	int hash;
	int localId;
	Vector3i position;
	Vector3f warp;
	float sdf;
	float liveSdf;//looked up using the warp

	Vector3f warpUpdate;
	Vector3f warpUpdateData;
	Vector3f warpUpdateLevelSet;
	Vector3f warpUpdateKilling;

	double voxelEnergy;
	double voxelEnergyData;
	double voxelEnergyLevelSet;
	double voxelEnergySmoothness;
	double voxelEnergyTikhonov;
	double voxelEnergyKilling;


	Vector3f liveJacobian;
	Vector3f warpedJacobian;
	Matrix3f warpedHessian;
	Matrix3f warpJacobian;
	Matrix3f warpHessianU;
	Matrix3f warpHessianV;
	Matrix3f warpHessianW;

	//    0        1        2          3         4         5           6         7         8
	//(-1,0,0) (0,-1,0) (0,0,-1)   (1, 0, 0) (0, 1, 0) (0, 0, 1)   (1, 1, 0) (0, 1, 1) (1, 0, 1)
	std::array<NeighborVoxelIterationInfo,9> neighbors;
	bool anomalyDetected;

	bool operator==(const ITMHighlightIterationInfo& rhs) const;
	bool operator!=(const ITMHighlightIterationInfo& rhs) const;

	friend std::ostream& operator<<(std::ostream& stream, const ITMHighlightIterationInfo& nestedMap3D);
};

}//namespace ITMLib


