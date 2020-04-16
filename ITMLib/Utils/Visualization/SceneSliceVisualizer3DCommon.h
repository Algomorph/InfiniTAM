//  ================================================================
//  Created by Gregory Kramida on 3/2/18.
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

//VTK
#include <vtkPoints.h>
#include <vtkFloatArray.h>
#include <vtkIntArray.h>

//local
#include "../Math.h"
#include "../VoxelFlags.h"
#include "../../Objects/Volume/VoxelBlockHash.h"

namespace ITMLib {

namespace Viz {
//================= STATIC CONSTANTS ============================
extern const std::array<double, 4> canonicalPositiveTruncatedVoxelColor;
extern const std::array<double, 4> canonicalPositiveNonTruncatedVoxelColor;
extern const std::array<double, 4> canonicalNegativeNonTruncatedVoxelColor;
extern const std::array<double, 4> canonicalNegativeTruncatedVoxelColor;
extern const std::array<double, 4> canonicalUnknownVoxelColor;

extern const std::array<double, 4> canonicalNegativeInterestVoxelColor;
extern const std::array<double, 4> canonicalPositiveInterestVoxelColor;
extern const std::array<double, 4> highlightVoxelColor;
extern const std::array<double, 3> canonicalHashBlockEdgeColor;

extern const std::array<double, 4> livePositiveTruncatedVoxelColor;
extern const std::array<double, 4> livePositiveNonTruncatedVoxelColor;
extern const std::array<double, 4> liveNegativeNonTruncatedVoxelColor;
extern const std::array<double, 4> liveNegativeTruncatedVoxelColor;
extern const std::array<double, 4> liveUnknownVoxelColor;

extern const std::array<double, 3> liveHashBlockEdgeColor;
}//namespace Viz




#define COMPUTE_VOXEL_SCALE_HIDE_UNKNOWNS(sdf, flags) (flags == ITMLib::VOXEL_UNKNOWN ? 0.0f : 1.0f - 0.9f * std::abs(sdf))
#define COMPUTE_VOXEL_SCALE(sdf) (1.0f - 0.9f * std::abs(sdf))

/**
 * \brief indexes to be used for coloring different categories of voxels (based on the semantic flags of each voxel)
 * \details Note: @refitem VoxelColorIndexAsCString has magic strings associated with it (static reflection didn't make
 * it yet into the C++17 standard, stay put for C++20), so you'll have to change that if you're going to change any of
 * these enum names here.
 */
enum VoxelColorIndex : int {
	POSITIVE_TRUNCATED_SDF_COLOR_INDEX = 0,
	POSITIVE_NON_TRUNCATED_SDF_COLOR_INDEX = 1,
	NEGATIVE_NON_TRUNCATED_SDF_COLOR_INDEX = 2,
	NEGATIVE_TRUNCATED_SDF_COLOR_INDEX = 3,
	UNKNOWN_SDF_COLOR_INDEX = 4,
	HIGHLIGHT_SDF_COLOR_INDEX = 5,
	COLOR_INDEX_COUNT = 6
};

enum VoxelScaleMode {
	VOXEL_SCALE_HIDE_UNKNOWNS = 0,
	VOXEL_SCALE_SHOW_UNKNOWNS = 1
};

inline
const char* VoxelColorIndexAsCString(const VoxelColorIndex& index) {
	switch (index) {
		case POSITIVE_TRUNCATED_SDF_COLOR_INDEX:
			return "POSITIVE_TRUNCATED_SDF_COLOR_INDEX";
		case POSITIVE_NON_TRUNCATED_SDF_COLOR_INDEX:
			return "POSITIVE_NON_TRUNCATED_SDF_COLOR_INDEX";
		case NEGATIVE_NON_TRUNCATED_SDF_COLOR_INDEX:
			return "NEGATIVE_NON_TRUNCATED_SDF_COLOR_INDEX";
		case NEGATIVE_TRUNCATED_SDF_COLOR_INDEX:
			return "NEGATIVE_TRUNCATED_SDF_COLOR_INDEX";
		case UNKNOWN_SDF_COLOR_INDEX:
			return "UNKNOWN_SDF_COLOR_INDEX";
		case HIGHLIGHT_SDF_COLOR_INDEX:
			return "HIGHLIGHT_SDF_COLOR_INDEX";
		default:
			std::cerr << "Unrecognized color index: " << index << std::endl;
			return "[UNRECOGNIZED COLOR INDEX]";
	}
}

template<typename TVoxel>
inline void
AddVoxelPoint(const Vector3i& currentBlockPositionVoxels, int x, int y, int z, const TVoxel* localVoxelBlock,
              vtkPoints* points, vtkFloatArray* scaleAttribute, vtkFloatArray* alternativeScaleAttribute,
              vtkIntArray* colorAttribute, const int& hash, Vector6i bounds) {

	Vector3i voxelPosition = currentBlockPositionVoxels + Vector3i(x, y, z);
	if (voxelPosition.x < bounds.min_x || voxelPosition.x >= bounds.max_x ||
	    voxelPosition.y < bounds.min_y || voxelPosition.y >= bounds.max_y ||
	    voxelPosition.z < bounds.min_z || voxelPosition.z >= bounds.max_z) {
		return;
	}
	int locId = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;
	const TVoxel& voxel = localVoxelBlock[locId];
	float sdf = TVoxel::valueToFloat(voxel.sdf);
	float voxelScale = COMPUTE_VOXEL_SCALE_HIDE_UNKNOWNS(sdf, voxel.flags);
	float alternativeVoxelScale = COMPUTE_VOXEL_SCALE(sdf);
	bool truncated = voxel.flags == ITMLib::VOXEL_TRUNCATED;
	float voxelColor;

	voxelColor = voxel.flags == ITMLib::VOXEL_UNKNOWN ? UNKNOWN_SDF_COLOR_INDEX :
	             sdf > 0 ?
	             (truncated ? POSITIVE_TRUNCATED_SDF_COLOR_INDEX : POSITIVE_NON_TRUNCATED_SDF_COLOR_INDEX) :
	             (truncated ? NEGATIVE_TRUNCATED_SDF_COLOR_INDEX : NEGATIVE_NON_TRUNCATED_SDF_COLOR_INDEX);


	points->InsertNextPoint(voxelPosition.x,
	                        voxelPosition.y,
	                        voxelPosition.z);

	scaleAttribute->InsertNextValue(voxelScale);
	alternativeScaleAttribute->InsertNextValue(alternativeVoxelScale);
	colorAttribute->InsertNextValue(voxelColor);
}

enum VisibilityMode{
	VISIBILITY_CANONICAL_WITH_UPDATES,
	VISIBILITY_LIVE,
	VISIBILITY_LIVE_AND_CANONICAL_WITH_UPDATES,
	VISIBILITY_FUSED_CANONICAL
};

}//namespace ITMLib