// InfiniTAM: Surffuse. Copyright (c) Torr Vision Group and the authors of InfiniTAM, 2016.

#pragma once

#include "../Interface/SurfelVisualizationEngine.h"

namespace ITMLib
{
  /**
   * \brief An instance of an instantiation of a class template deriving from this one can be used to render a surfel-based 3D scene using the CPU.
   */
  template <typename TSurfel>
  class SurfelVisualizationEngine_CPU : public SurfelVisualizationEngine<TSurfel>
  {
    //#################### TYPEDEFS & USINGS ####################
  private:
    typedef SurfelVisualizationEngine<TSurfel> Base;
    using typename Base::RenderImageType;

    //#################### PUBLIC MEMBER FUNCTIONS ####################
  public:
    /** Override */
    virtual void CopyCorrespondencesToBuffers(const SurfelScene<TSurfel> *scene, float *newPositions, float *oldPositions, float *correspondences) const;

    /** Override */
    virtual void CopySceneToBuffers(const SurfelScene<TSurfel> *scene, float *positions, unsigned char *normals, unsigned char *colours) const;

    /** Override */
    virtual void CreateICPMaps(const SurfelScene<TSurfel> *scene, const SurfelRenderState *renderState, ITMTrackingState *trackingState) const;

    /** Override */
    virtual void RenderDepthImage(const SurfelScene<TSurfel> *scene, const ORUtils::SE3Pose *pose, const SurfelRenderState *renderState,
                                  ITMFloatImage *outputImage) const;

    /** Override */
    virtual void RenderImage(const SurfelScene<TSurfel> *scene, const ORUtils::SE3Pose *pose, const SurfelRenderState *renderState,
                             ITMUChar4Image *outputImage, RenderImageType type) const;

    //#################### PRIVATE MEMBER FUNCTIONS ####################
  private:
    /** Override */
    virtual MemoryDeviceType GetMemoryType() const;

    /** Override */
    virtual void MakeIndexImage(const SurfelScene<TSurfel> *scene, const ORUtils::SE3Pose *pose, const Intrinsics *intrinsics,
                                int width, int height, int scaleFactor, unsigned int *surfelIndexImage, bool useRadii,
                                UnstableSurfelRenderingMode unstableSurfelRenderingMode, int *depthBuffer) const;
  };
}
