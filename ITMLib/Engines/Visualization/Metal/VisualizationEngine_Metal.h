// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#ifndef __METALC__

#include "../CPU/VisualizationEngine_CPU.h"

namespace ITMLib
{
    template<class TVoxel, class TIndex>
    class VisualizationEngine_Metal : public VisualizationEngine_CPU < TVoxel, TIndex >
    { };
    
    template<class TVoxel>
    class VisualizationEngine_Metal<TVoxel, ITMVoxelBlockHash> : public VisualizationEngine_CPU < TVoxel, ITMVoxelBlockHash >
    {
    public:
        void CreateICPMaps(const ITMScene<TVoxel,ITMVoxelBlockHash> *scene, const ITMView *view, ITMTrackingState *trackingState, ITMRenderState *renderState) const;
        void RenderImage(const ITMScene<TVoxel,ITMVoxelBlockHash> *scene, const ORUtils::SE3Pose *pose, const ITMIntrinsics *intrinsics, const ITMRenderState *renderState,
                         ITMUChar4Image *outputImage, IVisualizationEngine::RenderImageType type = IVisualizationEngine::RENDER_SHADED_GREYSCALE,
                         IVisualizationEngine::RenderRaycastSelection raycastType = IVisualizationEngine::RENDER_FROM_NEW_RAYCAST) const;
        
        VisualizationEngine_Metal();
    };
}

#endif

#if (defined __OBJC__) || (defined __METALC__)

struct CreateICPMaps_Params
{
    Matrix4f invM;
    Matrix4f M;
    Vector4f projParams;
    Vector4f invProjParams;
    Vector4f lightSource;
    Vector4i imgSize;
    Vector2f voxelSizes;
};

#endif