// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include <iosfwd>

#include "RGBD_CalibrationInformation.h"

namespace ITMLib
{
	bool readIntrinsics(std::istream& src, Intrinsics & dest);
	bool readIntrinsics(const char *fileName, Intrinsics & dest);

	bool readExtrinsics(std::istream & src, Extrinsics & dest);
	bool readExtrinsics(const char *fileName, Extrinsics & dest);

	bool readDisparityCalib(std::istream & src, DisparityCalib & dest);
	bool readDisparityCalib(const char *fileName, DisparityCalib & dest);

	bool readRGBDCalib(std::istream & src, RGBD_CalibrationInformation & dest);
	bool readRGBDCalib(const char *fileName, RGBD_CalibrationInformation & dest);
	bool readRGBDCalib(const char *rgbIntrinsicsFile, const char *depthIntrinsicsFile, const char *disparityCalibFile, const char *extrinsicsFile, RGBD_CalibrationInformation & dest);

	void writeIntrinsics(std::ostream & dest, const Intrinsics & src);
	std::ostream& operator<<(std::ostream& dest, Intrinsics& src);
	void writeExtrinsics(std::ostream & dest, const Extrinsics & src);
	std::ostream& operator<<(std::ostream& dest, Extrinsics& src);
	void writeDisparityCalib(std::ostream & dest, const DisparityCalib & src);
	std::ostream& operator<<(std::ostream& dest, DisparityCalib& src);
	void writeRGBDCalib(std::ostream & dest, const RGBD_CalibrationInformation & src);
	std::ostream& operator<<(std::ostream& dest, RGBD_CalibrationInformation& src);
	void writeRGBDCalib(const char *fileName, const RGBD_CalibrationInformation & src);
}
