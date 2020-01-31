// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "Objects/Scene/PlainVoxelArray.h"
#include "Objects/Scene/SurfelTypes.h"
#include "Objects/Scene/VoxelBlockHash.h"
#include "Objects/Scene/VoxelTypes.h"

/** This chooses the information stored at each surfel. At the moment, valid
    options are ITMSurfel_grey and ITMSurfel_rgb.
*/
typedef ITMLib::ITMSurfel_rgb ITMSurfelT;

/** This chooses the information stored at each voxel. At the moment, valid
    options are ITMVoxel_s, ITMVoxel_f, ITMVoxel_s_rgb, ITMVoxel_f_rgb, and ITMVoxel_f_flags.
*/
//typedef ITMVoxel_s ITMVoxel;
typedef ITMVoxel_f_flags ITMVoxel;
typedef ITMVoxel_f_warp ITMWarp;


/** This chooses the way the voxels are addressed and indexed. At the moment,
    valid options are VoxelBlockHash and PlainVoxelArray.
*/
typedef ITMLib::VoxelBlockHash ITMVoxelIndex;
//typedef ITMLib::PlainVoxelArray ITMVoxelIndex;
