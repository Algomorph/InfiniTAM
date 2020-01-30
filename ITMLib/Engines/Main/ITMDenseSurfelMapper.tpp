// InfiniTAM: Surffuse. Copyright (c) Torr Vision Group and the authors of InfiniTAM, 2016.

#include "ITMDenseSurfelMapper.h"

#include "../Reconstruction/SurfelSceneReconstructionEngineFactory.h"

namespace ITMLib
{

//#################### CONSTRUCTORS ####################

template <typename TSurfel>
ITMDenseSurfelMapper<TSurfel>::ITMDenseSurfelMapper(const Vector2i& depthImageSize, MemoryDeviceType deviceType)
: m_reconstructionEngine(SurfelSceneReconstructionEngineFactory<TSurfel>::make_surfel_scene_reconstruction_engine(depthImageSize, deviceType))
{}

//#################### DESTRUCTOR ####################

template <typename TSurfel>
ITMDenseSurfelMapper<TSurfel>::~ITMDenseSurfelMapper()
{
  delete m_reconstructionEngine;
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

template <typename TSurfel>
void ITMDenseSurfelMapper<TSurfel>::ProcessFrame(const ITMView *view, const ITMTrackingState *trackingState, ITMSurfelScene<TSurfel> *scene, ITMSurfelRenderState *liveRenderState) const
{
  m_reconstructionEngine->IntegrateIntoScene(scene, view, trackingState, liveRenderState);
}

}
