// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "ImageSourceEngine.h"

namespace InputSource {

class LibUVCEngine : public BaseImageSourceEngine
{
public:
	class PrivateData;
private:
	PrivateData *data;
	Vector2i imageSize_rgb, imageSize_d;
	bool colorAvailable, depthAvailable;
public:
	LibUVCEngine(const char *calibFilename, Vector2i imageSize_rgb = Vector2i(640, 480), Vector2i imageSize_d = Vector2i(640, 480));
	~LibUVCEngine();

	bool HasMoreImages() const override;
	void GetImages(UChar4Image& rgb_image, ShortImage& raw_depth_image);
	Vector2i GetDepthImageSize() const override;
	Vector2i GetRGBImageSize() const override;
};

}
