// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../ITMLib/Utils/ImageTypes.h"

namespace InputSource {

class FFMPEGWriter
{
	public:
	class PrivateData;

	FFMPEGWriter();
	~FFMPEGWriter();

	bool open(const char *filename, int size_x, int size_y, bool isDepth, int fps);
	bool writeFrame(UChar4Image *rgbImage);
	bool writeFrame(ShortImage *depthImage);
	bool close();

	bool isOpen() const;

	private:
	PrivateData *mData;
	int counter;
};

}
