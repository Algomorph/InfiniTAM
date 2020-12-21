// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "CalibIO.h"

#include <fstream>
#include <sstream>
#include <stdexcept>

using namespace ITMLib;

bool ITMLib::readIntrinsics(std::istream & src, Intrinsics & dest)
{
	float sizeX, sizeY;
	float focalLength[2], centerPoint[2];

	src >> sizeX >> sizeY;
	src >> focalLength[0] >> focalLength[1];
	src >> centerPoint[0] >> centerPoint[1];
	if (src.fail()) return false;

	dest.SetFrom(focalLength[0], focalLength[1], centerPoint[0], centerPoint[1]);
	return true;
}


bool ITMLib::readIntrinsics(const char *fileName, Intrinsics & dest)
{
	std::ifstream f(fileName);
	return ITMLib::readIntrinsics(f, dest);
}

bool ITMLib::readExtrinsics(std::istream & src, Extrinsics & dest)
{
	Matrix4f calib;
	src >> calib.m00 >> calib.m10 >> calib.m20 >> calib.m30;
	src >> calib.m01 >> calib.m11 >> calib.m21 >> calib.m31;
	src >> calib.m02 >> calib.m12 >> calib.m22 >> calib.m32;
	calib.m03 = 0.0f; calib.m13 = 0.0f; calib.m23 = 0.0f; calib.m33 = 1.0f;
	if (src.fail()) return false;

	dest.SetFrom(calib);
	return true;
}


bool ITMLib::readExtrinsics(const char *fileName, Extrinsics & dest)
{
	std::ifstream f(fileName);
	return ITMLib::readExtrinsics(f, dest);
}

bool ITMLib::readDisparityCalib(std::istream & src, DisparityCalib & dest)
{
	std::string word;
	src >> word;
	if (src.fail()) return false;

	DisparityCalib::TrafoType type = DisparityCalib::TrafoType::TRAFO_KINECT;
	float a,b;

	if (word.compare("kinect") == 0) {
		type = DisparityCalib::TrafoType::TRAFO_KINECT;
		src >> a;
	} else if (word.compare("affine") == 0) {
		type = DisparityCalib::TrafoType::TRAFO_AFFINE;
		src >> a;
	} else {
		std::stringstream wordstream(word);
		wordstream >> a;
		if (wordstream.fail()) return false;
	}

	src >> b;
	if (src.fail()) return false;

	if ((a == 0.0f) && (b == 0.0f)) {
		type = DisparityCalib::TrafoType::TRAFO_AFFINE;
		a = 1.0f/1000.0f; b = 0.0f;
	}

	dest.SetFrom(a, b, type);
	return true;
}



bool ITMLib::readDisparityCalib(const char *fileName, DisparityCalib & dest)
{
	std::ifstream f(fileName);
	return ITMLib::readDisparityCalib(f, dest);
}

bool ITMLib::readRGBDCalib(std::istream & src, RGBD_CalibrationInformation & dest)
{
	if (!ITMLib::readIntrinsics(src, dest.intrinsics_rgb)) return false;
	if (!ITMLib::readIntrinsics(src, dest.intrinsics_d)) return false;
	if (!ITMLib::readExtrinsics(src, dest.trafo_rgb_to_depth)) return false;
	if (!ITMLib::readDisparityCalib(src, dest.disparity_calibration_coefficients)) return false;
	return true;
}

bool ITMLib::readRGBDCalib(const char *fileName, RGBD_CalibrationInformation & dest)
{
	std::ifstream f(fileName);
	return ITMLib::readRGBDCalib(f, dest);
}

bool ITMLib::readRGBDCalib(const char *rgbIntrinsicsFile, const char *depthIntrinsicsFile, const char *disparityCalibFile, const char *extrinsicsFile, RGBD_CalibrationInformation & dest)
{
	bool ret = true;
	ret &= ITMLib::readIntrinsics(rgbIntrinsicsFile, dest.intrinsics_rgb);
	ret &= ITMLib::readIntrinsics(depthIntrinsicsFile, dest.intrinsics_d);
	ret &= ITMLib::readExtrinsics(extrinsicsFile, dest.trafo_rgb_to_depth);
	ret &= ITMLib::readDisparityCalib(disparityCalibFile, dest.disparity_calibration_coefficients);
	return ret;
}

void ITMLib::writeIntrinsics(std::ostream & dest, const Intrinsics & src)
{
	// Note: The size parameters are no longer used, but we don't want to change the calibration file format.
	const float dummySizeX = 640, dummySizeY = 480;

	dest << dummySizeX << ' ' << dummySizeY << '\n';
	dest << src.projection_params_simple.fx << ' ' << src.projection_params_simple.fy << '\n';
	dest << src.projection_params_simple.cx << ' ' << src.projection_params_simple.cy << '\n';
}

void ITMLib::writeExtrinsics(std::ostream & dest, const Extrinsics & src)
{
	const Matrix4f& calib = src.calib;
	dest << calib.m00 << ' ' << calib.m10 << ' ' << calib.m20 << ' ' << calib.m30 << '\n';
	dest << calib.m01 << ' ' << calib.m11 << ' ' << calib.m21 << ' ' << calib.m31 << '\n';
	dest << calib.m02 << ' ' << calib.m12 << ' ' << calib.m22 << ' ' << calib.m32 << '\n';
}

void ITMLib::writeDisparityCalib(std::ostream & dest, const DisparityCalib & src)
{
	switch(src.GetType())
	{
		case DisparityCalib::TrafoType::TRAFO_AFFINE:
			dest << "affine " << src.GetParams().x << ' ' << src.GetParams().y << '\n';
			break;
		case DisparityCalib::TrafoType::TRAFO_KINECT:
			dest << src.GetParams().x << ' ' << src.GetParams().y << '\n';
			break;
		default:
			throw std::runtime_error("Error: Unknown disparity calibration type");
	}
}

void ITMLib::writeRGBDCalib(std::ostream & dest, const RGBD_CalibrationInformation & src)
{
	writeIntrinsics(dest, src.intrinsics_rgb);
	dest << '\n';
	writeIntrinsics(dest, src.intrinsics_d);
	dest << '\n';
	writeExtrinsics(dest, src.trafo_rgb_to_depth);
	dest << '\n';
	writeDisparityCalib(dest, src.disparity_calibration_coefficients);
}

void ITMLib::writeRGBDCalib(const char *fileName, const RGBD_CalibrationInformation & src)
{
	std::ofstream f(fileName);
	writeRGBDCalib(f, src);
}
