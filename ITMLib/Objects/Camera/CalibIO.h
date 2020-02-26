// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include <iosfwd>

#include "RGBDCalib.h"

namespace ITMLib
{
	bool readIntrinsics(std::istream & src, Intrinsics & dest);
	bool readIntrinsics(const char *fileName, Intrinsics & dest);
	bool readExtrinsics(std::istream & src, Extrinsics & dest);
	bool readExtrinsics(const char *fileName, Extrinsics & dest);
	bool readDisparityCalib(std::istream & src, DisparityCalib & dest);
	bool readDisparityCalib(const char *fileName, DisparityCalib & dest);
	bool readRGBDCalib(std::istream & src, RGBDCalib & dest);
	bool readRGBDCalib(const char *fileName, RGBDCalib & dest);
	bool readRGBDCalib(const char *rgbIntrinsicsFile, const char *depthIntrinsicsFile, const char *disparityCalibFile, const char *extrinsicsFile, RGBDCalib & dest);
	void writeIntrinsics(std::ostream & dest, const Intrinsics & src);
	void writeExtrinsics(std::ostream & dest, const Extrinsics & src);
	void writeDisparityCalib(std::ostream & dest, const DisparityCalib & src);
	void writeRGBDCalib(std::ostream & dest, const RGBDCalib & src);
	void writeRGBDCalib(const char *fileName, const RGBDCalib & src);
}
