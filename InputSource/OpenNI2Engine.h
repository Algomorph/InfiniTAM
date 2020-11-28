// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "ImageSourceEngine.h"

#if (!defined USING_CMAKE) && (defined _MSC_VER)
#pragma comment(lib, "OpenNI2")
#endif

namespace InputSource {

class OpenNI2Engine : public BaseImageSourceEngine
{
private:
	class PrivateData;
	PrivateData *data;
	Vector2i imageSize_rgb, imageSize_d;
	bool color_available, depth_available;
public:
	OpenNI2Engine(const char *calibFilename, const char *deviceURI = nullptr, const bool useInternalCalibration = false,
	              Vector2i imageSize_rgb = Vector2i(640, 480), Vector2i imageSize_d = Vector2i(640, 480));
	~OpenNI2Engine();

	bool HasMoreImages() const override;
	void GetImages(UChar4Image& rgb, ShortImage& rawDepth);
	Vector2i GetDepthImageSize() const override;
	Vector2i GetRGBImageSize() const override;
};

}
