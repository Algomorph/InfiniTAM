// InfiniTAM: Surffuse. Copyright (c) Torr Vision Group and the authors of InfiniTAM, 2016.

#pragma once

#include "../Interface/SurfelSceneReconstructionEngine.h"

namespace ITMLib
{
  /**
   * \brief An instance of an instantiation of this class template can be used to make a surfel-based reconstruction of a 3D scene using the CPU.
   */
  template <typename TSurfel>
  class SurfelSceneReconstructionEngine_CPU : public SurfelSceneReconstructionEngine<TSurfel>
  {
    //#################### CONSTRUCTORS ####################
  public:
    /**
     * \brief Constructs a CPU-based surfel scene reconstruction engine.
     *
     * \param depthImageSize  The size of the depth images that are being fused into the scene.
     */
    explicit SurfelSceneReconstructionEngine_CPU(const Vector2i& depthImageSize);

    //#################### PRIVATE MEMBER FUNCTIONS ####################
  private:
    /** Override */
    virtual void AddNewSurfels(SurfelScene<TSurfel> *scene, const ITMView *view, const ITMTrackingState *trackingState) const;

    /** Override */
    virtual void FindCorrespondingSurfels(const SurfelScene<TSurfel> *scene, const ITMView *view, const ITMTrackingState *trackingState,
                                          const SurfelRenderState *renderState) const;

    /** Override */
    virtual void FuseMatchedPoints(SurfelScene<TSurfel> *scene, const ITMView *view, const ITMTrackingState *trackingState) const;

    /** Override */
    virtual void MarkBadSurfels(SurfelScene<TSurfel> *scene) const;

    /** Override */
    virtual void MergeSimilarSurfels(SurfelScene<TSurfel> *scene, const SurfelRenderState *renderState) const;

    /** Override */
    virtual void PreprocessDepthMap(const ITMView *view, const SurfelVolumeParameters& sceneParams) const;

    /** Override */
    virtual void RemoveMarkedSurfels(SurfelScene<TSurfel> *scene) const;
  };
}
