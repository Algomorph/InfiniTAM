// InfiniTAM: Surffuse. Copyright (c) Torr Vision Group and the authors of InfiniTAM, 2016.

#pragma once

#include "../Reconstruction/Interface/SurfelSceneReconstructionEngine.h"
#include "../../Objects/RenderStates/ITMSurfelRenderState.h"
#include "../../Utils/Configuration.h"

namespace ITMLib
{
  /**
   * \brief An instance of an instantiation of this class template can be used to produce a dense surfel scene from a sequence of depth and RGB images.
   */
  template <typename TSurfel>
  class ITMDenseSurfelMapper
  {
    //#################### PRIVATE VARIABLES ####################
  private:
    /** The surfel scene reconstruction engine. */
    SurfelSceneReconstructionEngine<TSurfel> *m_reconstructionEngine;

    //#################### CONSTRUCTORS ####################
  public:
    /**
     * \brief Constructs a dense surfel mapper.
     *
     * \param depthImageSize  The size of the depth images.
     * \param deviceType      The device on which the mapper should operate.
     */
    ITMDenseSurfelMapper(const Vector2i& depthImageSize, MemoryDeviceType deviceType);

    //#################### DESTRUCTOR ####################
  public:
    /**
     * \brief Destroys the mapper.
     */
    ~ITMDenseSurfelMapper();

    //#################### COPY CONSTRUCTOR & ASSIGNMENT OPERATOR ####################
  private:
    // Deliberately private and unimplemented.
    ITMDenseSurfelMapper(const ITMDenseSurfelMapper&);
    ITMDenseSurfelMapper& operator=(const ITMDenseSurfelMapper&);

    //#################### PUBLIC MEMBER FUNCTIONS ####################
  public:
    /**
     * \brief Processes a frame from the input sequence.
     *
     * \param view            The current view (containing the live input images from the current image source).
     * \param trackingState   The current tracking state.
     * \param scene           The surfel scene.
     * \param liveRenderState The render state for the live camera.
     */
    void ProcessFrame(const ITMView *view, const ITMTrackingState *trackingState, ITMSurfelScene<TSurfel> *scene, ITMSurfelRenderState *liveRenderState) const;
  };
}
