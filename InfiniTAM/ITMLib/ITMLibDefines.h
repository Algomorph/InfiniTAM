// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "Objects/Scene/ITMPlainVoxelArray.h"
#include "Objects/Scene/ITMSurfelTypes.h"
#include "Objects/Scene/ITMVoxelBlockHash.h"
#include "Objects/Scene/ITMVoxelTypes.h"
#include "Objects/Scene/ITMWarpFieldTypes.h"

/** This chooses the information stored at each surfel. At the moment, valid
    options are ITMSurfel_grey and ITMSurfel_rgb.
*/
typedef ITMLib::ITMSurfel_rgb ITMSurfelT;

/** This chooses the information stored at each voxel. At the moment, valid
    options are ITMVoxel_s, ITMVoxel_f, ITMVoxel_s_rgb, ITMVoxel_f_rgb, and ITMVoxel_f_killing.
*/
typedef ITMVoxel_f_killing ITMVoxel;

typedef ITMVoxel_warp_t ITMWarpField;

/** This chooses the way the voxels are addressed and indexed. At the moment,
    valid options are ITMVoxelBlockHash and ITMPlainVoxelArray.
*/
typedef ITMLib::ITMVoxelBlockHash ITMVoxelIndex;
//typedef ITMLib::ITMPlainVoxelArray ITMVoxelIndex;
