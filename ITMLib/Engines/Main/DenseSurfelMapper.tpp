// InfiniTAM: Surffuse. Copyright (c) Torr Vision Group and the authors of InfiniTAM, 2016.

#include "DenseSurfelMapper.h"

#include "../Reconstruction/SurfelSceneReconstructionEngineFactory.h"

namespace ITMLib
{

//#################### CONSTRUCTORS ####################

template <typename TSurfel>
DenseSurfelMapper<TSurfel>::DenseSurfelMapper(const Vector2i& depthImageSize, MemoryDeviceType deviceType)
: m_reconstructionEngine(SurfelSceneReconstructionEngineFactory::Build<TSurfel>(depthImageSize, deviceType))
{}

//#################### DESTRUCTOR ####################

template <typename TSurfel>
DenseSurfelMapper<TSurfel>::~DenseSurfelMapper()
{
  delete m_reconstructionEngine;
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

template <typename TSurfel>
void DenseSurfelMapper<TSurfel>::ProcessFrame(const ITMView *view, const ITMTrackingState *trackingState, ITMSurfelScene<TSurfel> *scene, ITMSurfelRenderState *liveRenderState) const
{
  m_reconstructionEngine->IntegrateIntoScene(scene, view, trackingState, liveRenderState);
}

}
