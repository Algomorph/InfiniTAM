// InfiniTAM: Surffuse. Copyright (c) Torr Vision Group and the authors of InfiniTAM, 2016.

#include "SurfelRenderState.h"

namespace ITMLib
{

//#################### CONSTRUCTORS ####################

SurfelRenderState::SurfelRenderState(const Vector2i& indexImageSize, int supersamplingFactor)
{
  depthBuffer = new ORUtils::Image<int>(indexImageSize, true, true);
  surfelIndexImage = new ORUtils::Image<unsigned int>(indexImageSize, true, true);

  Vector2i indexImageSizeSuper = indexImageSize * supersamplingFactor;
  depthBufferSuper = new ORUtils::Image<int>(indexImageSizeSuper, true, true);
  surfelIndexImageSuper = new ORUtils::Image<unsigned int>(indexImageSizeSuper, true, true);
}

//#################### DESTRUCTOR ####################

SurfelRenderState::~SurfelRenderState()
{
  delete depthBuffer;
  delete depthBufferSuper;
  delete surfelIndexImage;
  delete surfelIndexImageSuper;
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

ORUtils::Image<int> *SurfelRenderState::GetDepthBuffer()
{
  return depthBuffer;
}

const ORUtils::Image<int> *SurfelRenderState::GetDepthBuffer() const
{
  return depthBuffer;
}

ORUtils::Image<int> *SurfelRenderState::GetDepthBufferSuper()
{
  return depthBufferSuper;
}

const ORUtils::Image<int> *SurfelRenderState::GetDepthBufferSuper() const
{
  return depthBufferSuper;
}

ORUtils::Image<unsigned int> *SurfelRenderState::GetIndexImage()
{
  return surfelIndexImage;
}

const ORUtils::Image<unsigned int> *SurfelRenderState::GetIndexImage() const
{
  return surfelIndexImage;
}

ORUtils::Image<unsigned int> *SurfelRenderState::GetIndexImageSuper()
{
  return surfelIndexImageSuper;
}

const ORUtils::Image<unsigned int> *SurfelRenderState::GetIndexImageSuper() const
{
  return surfelIndexImageSuper;
}

}
