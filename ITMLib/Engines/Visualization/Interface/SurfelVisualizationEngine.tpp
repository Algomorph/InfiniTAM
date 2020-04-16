// InfiniTAM: Surffuse. Copyright (c) Torr Vision Group and the authors of InfiniTAM, 2016.

#include "SurfelVisualizationEngine.h"

namespace ITMLib
{

//#################### DESTRUCTOR ####################

template <typename TSurfel>
SurfelVisualizationEngine<TSurfel>::~SurfelVisualizationEngine()
{}

//#################### PUBLIC MEMBER FUNCTIONS ####################

template <typename TSurfel>
void SurfelVisualizationEngine<TSurfel>::FindSurface(const SurfelScene<TSurfel> *scene, const ORUtils::SE3Pose *pose, const Intrinsics *intrinsics,
                                                     bool useRadii, UnstableSurfelRenderingMode unstableSurfelRenderingMode, SurfelRenderState *renderState) const
{
  MemoryDeviceType memoryType = GetMemoryType();
  MakeIndexImage(
    scene,
    pose,
    intrinsics,
    renderState->GetIndexImage()->dimensions.x,
    renderState->GetIndexImage()->dimensions.y,
    1,
    renderState->GetIndexImage()->GetData(memoryType),
    useRadii,
    unstableSurfelRenderingMode,
    renderState->GetDepthBuffer()->GetData(memoryType)
  );
}

template <typename TSurfel>
void SurfelVisualizationEngine<TSurfel>::FindSurfaceSuper(const SurfelScene<TSurfel> *scene, const ORUtils::SE3Pose *pose, const Intrinsics *intrinsics,
                                                          UnstableSurfelRenderingMode unstableSurfelRenderingMode, SurfelRenderState *renderState) const
{
  MemoryDeviceType memoryType = GetMemoryType();
  MakeIndexImage(
    scene,
    pose,
    intrinsics,
    renderState->GetIndexImageSuper()->dimensions.x,
    renderState->GetIndexImageSuper()->dimensions.y,
    scene->GetParams().supersampling_factor,
    renderState->GetIndexImageSuper()->GetData(memoryType),
    false,
    unstableSurfelRenderingMode,
    renderState->GetDepthBufferSuper()->GetData(memoryType)
  );
}

}
