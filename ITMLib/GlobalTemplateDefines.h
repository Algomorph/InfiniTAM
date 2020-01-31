// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "Objects/Volume/PlainVoxelArray.h"
#include "Objects/Volume/SurfelTypes.h"
#include "Objects/Volume/VoxelBlockHash.h"
#include "Objects/Volume/VoxelTypes.h"

/** This chooses the information stored at each surfel. At the moment, valid
    options are Surfel_grey and Surfel_rgb.
*/
typedef ITMLib::Surfel_rgb SurfelT;

/** This chooses the information stored at each voxel. At the moment, valid
    options are TSDFVoxel_s, TSDFVoxel_f, TSDFVoxel_s_rgb, TSDFVoxel_f_rgb, and TSDFVoxel_f_flags.
*/
//typedef TSDFVoxel_s TSDFVoxel;
typedef TSDFVoxel_f_flags TSDFVoxel;
typedef WarpVoxel_f_uf WarpVoxel;


/** This chooses the way the voxels are addressed and indexed. At the moment,
    valid options are VoxelBlockHash and PlainVoxelArray.
*/
typedef ITMLib::VoxelBlockHash VoxelIndex;
//typedef ITMLib::PlainVoxelArray VoxelIndex;
