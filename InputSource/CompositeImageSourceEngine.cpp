// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "CompositeImageSourceEngine.h"

namespace InputSource {

//#################### CONSTRUCTORS ####################

CompositeImageSourceEngine::CompositeImageSourceEngine()
: m_curSubengineIndex(0)
{}

//#################### DESTRUCTOR ####################

CompositeImageSourceEngine::~CompositeImageSourceEngine()
{
  for(size_t i = 0, size = m_subengines.size(); i < size; ++i)
  {
    delete m_subengines[i];
  }
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void CompositeImageSourceEngine::addSubengine(ImageSourceEngine *subengine)
{
  m_subengines.push_back(subengine);
}

ITMLib::RGBDCalib CompositeImageSourceEngine::getCalib(void) const
{
  // There is an assumption being made that the calibrations for all the sub-engines are the same,
  // although this is not currently being enforced.
  if(!m_subengines.empty()) return m_subengines[0]->getCalib();
  else throw std::runtime_error("Cannot get calibration parameters from an empty composite image source engine");
}

ImageSourceEngine* CompositeImageSourceEngine::getCurrentSubengine(void)
{
  return m_curSubengineIndex < m_subengines.size() ? m_subengines[m_curSubengineIndex] : nullptr;
}

Vector2i CompositeImageSourceEngine::GetDepthImageSize(void)
{
  // There is an assumption being made that the depth image sizes for all the sub-engines are the same,
  // although this is not currently being enforced.
  if(!m_subengines.empty()) return m_subengines[0]->GetDepthImageSize();
  else throw std::runtime_error("Cannot get the depth image size from an empty composite image source engine");
}

void CompositeImageSourceEngine::GetImages(ITMUChar4Image& rgb, ITMShortImage& rawDepth)
{
  if(AdvanceToNextImages()) m_subengines[m_curSubengineIndex]->GetImages(rgb, rawDepth);
}

Vector2i CompositeImageSourceEngine::GetRGBImageSize(void)
{
  // There is an assumption being made that the RGB image sizes for all the sub-engines are the same,
  // although this is not currently being enforced.
  if(!m_subengines.empty()) return m_subengines[0]->GetRGBImageSize();
  else throw std::runtime_error("Cannot get the RGB image size from an empty composite image source engine");
}

bool CompositeImageSourceEngine::HasImagesNow(void)
{
  ImageSourceEngine *curSubengine = AdvanceToNextImages();
  return curSubengine != nullptr && curSubengine->HasImagesNow();
}

bool CompositeImageSourceEngine::HasMoreImages(void)
{
  return AdvanceToNextImages() != nullptr;
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

ImageSourceEngine *CompositeImageSourceEngine::AdvanceToNextImages(void)
{
  ImageSourceEngine *curSubengine = getCurrentSubengine();
  while(curSubengine && !curSubengine->HasMoreImages())
  {
    ++m_curSubengineIndex;
    curSubengine = getCurrentSubengine();
  }
  return curSubengine;
}

}
