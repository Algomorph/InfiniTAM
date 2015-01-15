// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#include <metal_stdlib>

#include "../../DeviceAgnostic/ITMSceneReconstructionEngine.h"
#include "ITMSceneReconstructionEngine_Metal.h"

using namespace metal;

kernel void integrateIntoScene_vh_device(DEVICEPTR(ITMVoxel) *localVBA                          [[ buffer(0) ]],
                                         const DEVICEPTR(ITMHashEntry) *hashTable               [[ buffer(1) ]],
                                         DEVICEPTR(int) *liveEntryIDs                           [[ buffer(2) ]],
                                         const DEVICEPTR(Vector4u) *rgb                         [[ buffer(3) ]],
                                         const DEVICEPTR(float) *depth                          [[ buffer(4) ]],
                                         const CONSTANT(IntegrateIntoScene_VH_Params) *params   [[ buffer(5) ]],
                                         uint3 threadIdx                                        [[ thread_position_in_threadgroup ]],
                                         uint3 blockIdx                                         [[ threadgroup_position_in_grid ]],
                                         uint3 blockDim                                         [[ threads_per_threadgroup ]])
{
    Vector3i globalPos;
    int entryId = liveEntryIDs[blockIdx.x];

    const DEVICEPTR(ITMHashEntry) &currentHashEntry = hashTable[entryId];

    if (currentHashEntry.ptr < 0) return;

    globalPos = (int3)currentHashEntry.pos * SDF_BLOCK_SIZE;

    DEVICEPTR(ITMVoxel) *localVoxelBlock = &(localVBA[currentHashEntry.ptr * SDF_BLOCK_SIZE3]);
    
    int x = threadIdx.x, y = threadIdx.y, z = threadIdx.z;

    Vector4f pt_model; int locId;

    locId = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;

    pt_model.x = (float)(globalPos.x + x) * params->_voxelSize;
    pt_model.y = (float)(globalPos.y + y) * params->_voxelSize;
    pt_model.z = (float)(globalPos.z + z) * params->_voxelSize;
    pt_model.w = 1.0f;
    
    ComputeUpdatedVoxelInfo<ITMVoxel::hasColorInformation,ITMVoxel>::compute(localVoxelBlock[locId], pt_model,
                                                                         params->M_d, params->projParams_d, params->M_rgb,
                                                                         params->projParams_rgb, params->mu, params->maxW, depth,
                                                                         params->depthImgSize, rgb, params->rgbImgSize);
}