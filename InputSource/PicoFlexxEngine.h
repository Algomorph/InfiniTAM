// Copyright 2017 Akos Maroy

#pragma once

#include "ImageSourceEngine.h"

namespace InputSource
{
	class PicoFlexxEngine : public BaseImageSourceEngine
	{
	private:
		class PrivateData;
		PrivateData *data;
		Vector2i imageSize_rgb, imageSize_d;
		bool colorAvailable, depthAvailable;

	public:
		explicit PicoFlexxEngine(const char *calibFilename, const char *deviceURI = NULL, const bool useInternalCalibration = false,
			Vector2i imageSize_rgb = Vector2i(224, 171), Vector2i imageSize_d = Vector2i(224, 171));
		~PicoFlexxEngine();

		bool HasMoreImages() const override;
		void GetImages(UChar4Image& rgb, ShortImage& rawDepth) override;
		Vector2i GetDepthImageSize() const override;
		Vector2i GetRGBImageSize() const override;
	};
}