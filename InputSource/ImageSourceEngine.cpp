// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "ImageSourceEngine.h"

#include "../ITMLib/Objects/Camera/CalibIO.h"
#include "../ORUtils/FileUtils.h"

#include <stdexcept>
#include <stdio.h>

using namespace InputSource;
using namespace ITMLib;

BaseImageSourceEngine::BaseImageSourceEngine(const char *calibFilename)
{
	if(!calibFilename || strlen(calibFilename) == 0)
	{
		printf("Calibration filename not specified. Using default parameters.\n");
		return;
	}

	if(!readRGBDCalib(calibFilename, calib))
		DIEWITHEXCEPTION("error: path to the calibration file was specified but data could not be read");
}

ITMLib::RGBDCalib BaseImageSourceEngine::getCalib() const
{
  return calib;
}

ImageMaskPathGenerator::ImageMaskPathGenerator(const char* rgbImageMask_, const char* depthImageMask_, const char* maskImageMask_) :
		hasMaskImagePaths(maskImageMask_ != nullptr)
{
	strncpy(rgbImageMask, rgbImageMask_, BUF_SIZE);
	strncpy(depthImageMask, depthImageMask_, BUF_SIZE);
	if(hasMaskImagePaths){
		strncpy(maskImageMask, maskImageMask_, BUF_SIZE);
	}
}

std::string ImageMaskPathGenerator::getRgbImagePath(size_t currentFrameNo) const
{
	char str[BUF_SIZE];
	sprintf(str, rgbImageMask, currentFrameNo);
	return std::string(str);
}

std::string ImageMaskPathGenerator::getDepthImagePath(size_t currentFrameNo) const
{
	char str[BUF_SIZE];
	sprintf(str, depthImageMask, currentFrameNo);
	return std::string(str);
}

std::string ImageMaskPathGenerator::getMaskImagePath(size_t currentFrameNo) const {
	if(hasMaskImagePaths){
		char str[BUF_SIZE];
		sprintf(str, maskImageMask, currentFrameNo);
		return std::string(str);
	}
	return std::string();
}


ImageListPathGenerator::ImageListPathGenerator(const std::vector<std::string>& rgbImagePaths_,
                                               const std::vector<std::string>& depthImagePaths_,
                                               const std::vector<std::string>& maskImagePaths_)
	: depthImagePaths(depthImagePaths_),
	  rgbImagePaths(rgbImagePaths_),
	  maskImagePaths(maskImagePaths_),
	  hasMaskImagePaths(true)
{
	if(rgbImagePaths.size() != depthImagePaths.size()) DIEWITHEXCEPTION("error: the rgb and depth image path lists do not have the same size");
	if(rgbImagePaths.size() != maskImagePaths.size()) DIEWITHEXCEPTION("error: the rgb and mask image path lists do not have the same size");
}

std::string ImageListPathGenerator::getRgbImagePath(size_t currentFrameNo) const
{
	return currentFrameNo < imageCount() ? rgbImagePaths[currentFrameNo] : "";
}

std::string ImageListPathGenerator::getDepthImagePath(size_t currentFrameNo) const
{
	return currentFrameNo < imageCount() ? depthImagePaths[currentFrameNo] : "";
}

std::string ImageListPathGenerator::getMaskImagePath(size_t currentFrameNo) const
{
	return hasMaskImagePaths ? currentFrameNo < imageCount() ? maskImagePaths[currentFrameNo] : "" : "";
}

size_t ImageListPathGenerator::imageCount() const
{
	return rgbImagePaths.size();
}

ImageListPathGenerator::ImageListPathGenerator(const std::vector<std::string>& rgbImagePaths_,
                                               const std::vector<std::string>& depthImagePaths_) :
		depthImagePaths(depthImagePaths_),
		rgbImagePaths(rgbImagePaths_),
		hasMaskImagePaths(false)
{
	if(rgbImagePaths.size() != depthImagePaths.size()) DIEWITHEXCEPTION("error: the rgb and depth image path lists do not have the same size");
}

template <typename PathGenerator>
ImageFileReader<PathGenerator>::ImageFileReader(const char *calibration_file_name, const PathGenerator& path_generator, size_t initial_frame_number)
		: BaseImageSourceEngine(calibration_file_name),
		  pathGenerator(path_generator),
		  cached_rgb(true, false),
		  cached_depth(true, false),
		  cached_mask(true, false) {
	current_frame_number = initial_frame_number;
	cached_frame_number = -1;


	cache_is_valid = false;
}

template <typename PathGenerator>
void ImageFileReader<PathGenerator>::LoadIntoCache()
{
	if (current_frame_number == cached_frame_number) return;
	cached_frame_number = current_frame_number;

	cache_is_valid = true;

	std::string rgbPath = pathGenerator.getRgbImagePath(current_frame_number);
	if (!ReadImageFromFile(cached_rgb, rgbPath.c_str()))
	{
		if (cached_rgb.dimensions.x > 0) cache_is_valid = false;
		printf("error reading file '%s'\n", rgbPath.c_str());
	}

	std::string depthPath = pathGenerator.getDepthImagePath(current_frame_number);
	if (!ReadImageFromFile(cached_depth, depthPath.c_str()))
	{
		if (cached_depth.dimensions.x > 0) cache_is_valid = false;
		printf("error reading file '%s'\n", depthPath.c_str());
	}

	if ((cached_rgb.dimensions.x <= 0) && (cached_depth.dimensions.x <= 0)) cache_is_valid = false;

	if(pathGenerator.hasMaskImagePaths){
		std::string maskPath = pathGenerator.getMaskImagePath(current_frame_number);
		if (!ReadImageFromFile(cached_mask, maskPath.c_str()))
		{
			if (cached_mask.dimensions.x > 0) cache_is_valid = false;
			printf("error reading file '%s'\n", maskPath.c_str());
		}else{
			Vector4u blackRGB((unsigned char)0);
			cached_depth.ApplyMask(cached_mask, 0);
			cached_rgb.ApplyMask(cached_mask,blackRGB);
		}
	}
}

template <typename PathGenerator>
bool ImageFileReader<PathGenerator>::HasMoreImages()
{
	LoadIntoCache();
	return cache_is_valid;
}

template <typename PathGenerator>
void ImageFileReader<PathGenerator>::GetImages(ITMUChar4Image& rgb, ITMShortImage& raw_depth)
{
	LoadIntoCache();
	rgb.SetFrom(cached_rgb, MemoryCopyDirection::CPU_TO_CPU);
	raw_depth.SetFrom(cached_depth, MemoryCopyDirection::CPU_TO_CPU);

	++current_frame_number;
}

template <typename PathGenerator>
Vector2i ImageFileReader<PathGenerator>::GetDepthImageSize()
{
	LoadIntoCache();
	return cached_depth.dimensions;
}

template <typename PathGenerator>
Vector2i ImageFileReader<PathGenerator>::GetRGBImageSize()
{
	LoadIntoCache();
	return cached_rgb.dimensions;
}

template<typename PathGenerator>
bool ImageFileReader<PathGenerator>::HasMaskImages() const {
	return this->pathGenerator.hasMaskImagePaths;
}

CalibSource::CalibSource(const char *calibFilename, Vector2i setImageSize, float ratio)
	: BaseImageSourceEngine(calibFilename)
{
	this->image_size = setImageSize;
	this->ResizeIntrinsics(calib.intrinsics_d, ratio);
	this->ResizeIntrinsics(calib.intrinsics_rgb, ratio);
}

void CalibSource::ResizeIntrinsics(Intrinsics &intrinsics, float ratio)
{
	intrinsics.projectionParamsSimple.fx *= ratio;
	intrinsics.projectionParamsSimple.fy *= ratio;
	intrinsics.projectionParamsSimple.px *= ratio;
	intrinsics.projectionParamsSimple.py *= ratio;
	intrinsics.projectionParamsSimple.all *= ratio;
}

RawFileReader::RawFileReader(const char *calibration_file_name, const char *rgb_image_filename_mask, const char *depth_image_filename_mask, Vector2i image_size, float ratio)
	: BaseImageSourceEngine(calibration_file_name),
	cached_rgb(image_size, MEMORYDEVICE_CPU),
	cached_depth(image_size, MEMORYDEVICE_CPU)
{
	this->image_size = image_size;
	this->ResizeIntrinsics(calib.intrinsics_d, ratio);
	this->ResizeIntrinsics(calib.intrinsics_rgb, ratio);

	strncpy(this->rgb_image_mask, rgb_image_filename_mask, BUF_SIZE);
	strncpy(this->depthImageMask, depth_image_filename_mask, BUF_SIZE);

	current_frame_number = 0;
	cached_frame_number = -1;
	LoadIntoCache();
}

void RawFileReader::ResizeIntrinsics(Intrinsics &intrinsics, float ratio)
{
	intrinsics.projectionParamsSimple.fx *= ratio;
	intrinsics.projectionParamsSimple.fy *= ratio;
	intrinsics.projectionParamsSimple.px *= ratio;
	intrinsics.projectionParamsSimple.py *= ratio;
	intrinsics.projectionParamsSimple.all *= ratio;
}

bool RawFileReader::LoadIntoCache()
{
	if (current_frame_number == cached_frame_number) return true;
	cached_frame_number = current_frame_number;
    
	char str[2048]; FILE *f; bool success = false;

	sprintf(str, rgb_image_mask, current_frame_number);

	f = fopen(str, "rb");
	if (f)
	{
		size_t tmp = fread(cached_rgb.GetData(MEMORYDEVICE_CPU), sizeof(Vector4u), image_size.x * image_size.y, f);
		fclose(f);
		if (tmp == (size_t)image_size.x * image_size.y) success = true;
	}
	if (!success)
	{
		printf("error reading file '%s'\n", str);
	}

	sprintf(str, depthImageMask, current_frame_number); success = false;
	f = fopen(str, "rb");
	if (f)
	{
		size_t tmp = fread(cached_depth.GetData(MEMORYDEVICE_CPU), sizeof(short), image_size.x * image_size.y, f);
		fclose(f);
		if (tmp == (size_t)image_size.x * image_size.y) success = true;
	}
	if (!success)
	{
		printf("error reading file '%s'\n", str);
	}
	return success;
}


bool RawFileReader::HasMoreImages()
{
	return LoadIntoCache();
}

void RawFileReader::GetImages(ITMUChar4Image& rgb, ITMShortImage& rawDepth)
{
	rgb.SetFrom(cached_rgb, MemoryCopyDirection::CPU_TO_CPU);
	rawDepth.SetFrom(cached_depth, MemoryCopyDirection::CPU_TO_CPU);

	if(this->LoadIntoCache()){
		++current_frame_number;
	}
}

BlankImageGenerator::BlankImageGenerator(const char *calibFilename, Vector2i setImageSize) : BaseImageSourceEngine(calibFilename)
{
	this->imgSize = setImageSize;
}

bool BlankImageGenerator::HasMoreImages()
{
	return true;
}

void BlankImageGenerator::GetImages(ITMUChar4Image& rgb, ITMShortImage& raw_depth)
{
	rgb.Clear();
	raw_depth.Clear();
}

template class InputSource::ImageFileReader<ImageMaskPathGenerator>;
template class InputSource::ImageFileReader<ImageListPathGenerator>;
