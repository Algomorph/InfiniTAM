// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "ImageSourceEngine.h"

namespace InputSource {

class FFMPEGReader : public BaseImageSourceEngine
{
	public:
	class PrivateData;

	FFMPEGReader(const char *calibFilename, const char *filename1, const char *filename2 = nullptr);
	~FFMPEGReader(void);

	bool HasMoreImages(void) override;
	void GetImages(ITMUChar4Image& rgb_image, ITMShortImage& depth_image) override;

	Vector2i GetDepthImageSize(void) override;
	Vector2i GetRGBImageSize(void) override;

	private:
	PrivateData *mData1;
	PrivateData *mData2;
	bool is_valid;
};

}
