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
		has_mask_image_paths(maskImageMask_ != nullptr)
{
	strncpy(rgb_image_filename_mask, rgbImageMask_, BUF_SIZE);
	strncpy(depth_image_filename_mask, depthImageMask_, BUF_SIZE);
	if(has_mask_image_paths){
		strncpy(mask_image_filename_mask, maskImageMask_, BUF_SIZE);
	}
}

std::string ImageMaskPathGenerator::GetRgbImagePath(size_t current_frame_number) const
{
	char str[BUF_SIZE];
	sprintf(str, rgb_image_filename_mask, current_frame_number);
	return std::string(str);
}

std::string ImageMaskPathGenerator::GetDepthImagePath(size_t current_frame_number) const
{
	char str[BUF_SIZE];
	sprintf(str, depth_image_filename_mask, current_frame_number);
	return std::string(str);
}

std::string ImageMaskPathGenerator::GetMaskImagePath(size_t current_frame_number) const {
	if(has_mask_image_paths){
		char str[BUF_SIZE];
		sprintf(str, mask_image_filename_mask, current_frame_number);
		return std::string(str);
	}
	return std::string();
}


ImageListPathGenerator::ImageListPathGenerator(const std::vector<std::string>& rgbImagePaths_,
                                               const std::vector<std::string>& depthImagePaths_,
                                               const std::vector<std::string>& maskImagePaths_)
	: depth_image_paths(depthImagePaths_),
	  rgb_image_paths(rgbImagePaths_),
	  mask_image_paths(maskImagePaths_),
	  has_mask_image_paths(true)
{
	if(rgb_image_paths.size() != depth_image_paths.size()) DIEWITHEXCEPTION("error: the rgb and depth image path lists do not have the same size");
	if(rgb_image_paths.size() != mask_image_paths.size()) DIEWITHEXCEPTION("error: the rgb and mask image path lists do not have the same size");
}

std::string ImageListPathGenerator::GetRgbImagePath(size_t current_frame_number) const
{
	return current_frame_number < ImageCount() ? rgb_image_paths[current_frame_number] : "";
}

std::string ImageListPathGenerator::GetDepthImagePath(size_t current_frame_number) const
{
	return current_frame_number < ImageCount() ? depth_image_paths[current_frame_number] : "";
}

std::string ImageListPathGenerator::GetMaskImagePath(size_t current_frame_number) const
{
	return has_mask_image_paths ? current_frame_number < ImageCount() ? mask_image_paths[current_frame_number] : "" : "";
}

size_t ImageListPathGenerator::ImageCount() const
{
	return rgb_image_paths.size();
}

ImageListPathGenerator::ImageListPathGenerator(const std::vector<std::string>& rgbImagePaths_,
                                               const std::vector<std::string>& depthImagePaths_) :
		depth_image_paths(depthImagePaths_),
		rgb_image_paths(rgbImagePaths_),
		has_mask_image_paths(false)
{
	if(rgb_image_paths.size() != depth_image_paths.size()) DIEWITHEXCEPTION("error: the rgb and depth image path lists do not have the same size");
}

template <typename PathGenerator>
ImageFileReader<PathGenerator>::ImageFileReader(const char *calibration_file_name, const PathGenerator& path_generator, size_t initial_frame_number)
		: BaseImageSourceEngine(calibration_file_name),
		  path_generator(path_generator),
		  cached_rgb(true, false),
		  cached_depth(true, false),
		  cached_mask(true, false) {
	current_frame_number = initial_frame_number;
	cached_frame_number = -1;
	cache_is_valid = false;
	LoadIntoCache();
}

template <typename PathGenerator>
void ImageFileReader<PathGenerator>::LoadIntoCache(){
	if (current_frame_number == cached_frame_number) return;
	cached_frame_number = current_frame_number;

	cache_is_valid = true;

	std::string rgb_path = path_generator.GetRgbImagePath(current_frame_number);
	if (!ReadImageFromFile(cached_rgb, rgb_path.c_str())){
		if (cached_rgb.dimensions.x > 0) cache_is_valid = false;
		printf("error reading file '%s'\n", rgb_path.c_str());
	}

	std::string depth_path = path_generator.GetDepthImagePath(current_frame_number);
	if (!ReadImageFromFile(cached_depth, depth_path.c_str())){
		if (cached_depth.dimensions.x > 0) cache_is_valid = false;
		printf("error reading file '%s'\n", depth_path.c_str());
	}

	if ((cached_rgb.dimensions.x <= 0) && (cached_depth.dimensions.x <= 0)) cache_is_valid = false;

	if(path_generator.has_mask_image_paths){
		std::string mask_path = path_generator.GetMaskImagePath(current_frame_number);
		if (!ReadImageFromFile(cached_mask, mask_path.c_str())){
			if (cached_mask.dimensions.x > 0) cache_is_valid = false;
			printf("error reading file '%s'\n", mask_path.c_str());
		}else{
			Vector4u blackRGB((unsigned char)0);
			cached_depth.ApplyMask(cached_mask, 0);
			cached_rgb.ApplyMask(cached_mask,blackRGB);
		}
	}
}

template <typename PathGenerator>
bool ImageFileReader<PathGenerator>::HasMoreImages() const
{
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
Vector2i ImageFileReader<PathGenerator>::GetDepthImageSize() const
{
	return cached_depth.dimensions;
}

template <typename PathGenerator>
Vector2i ImageFileReader<PathGenerator>::GetRGBImageSize() const
{
	return cached_rgb.dimensions;
}

template<typename PathGenerator>
bool ImageFileReader<PathGenerator>::HasMaskImages() const {
	return this->path_generator.has_mask_image_paths;
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
	cached_depth(image_size, MEMORYDEVICE_CPU),
	image_size(image_size),
	current_frame_number(0),
	cached_frame_number(-1)
{
	this->ResizeIntrinsics(calib.intrinsics_d, ratio);
	this->ResizeIntrinsics(calib.intrinsics_rgb, ratio);

	strncpy(this->rgb_image_filename_mask, rgb_image_filename_mask, BUF_SIZE);
	strncpy(this->depth_image_filename_mask, depth_image_filename_mask, BUF_SIZE);

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

	cache_is_valid = true;
    
	char str[2048]; FILE *f; bool success = false;

	sprintf(str, rgb_image_filename_mask, current_frame_number);

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

	sprintf(str, depth_image_filename_mask, current_frame_number); success = false;
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

	cache_is_valid = success;

	return success;
}


bool RawFileReader::HasMoreImages() const
{
	return cache_is_valid;
}

void RawFileReader::GetImages(ITMUChar4Image& rgb, ITMShortImage& rawDepth)
{
	rgb.SetFrom(cached_rgb, MemoryCopyDirection::CPU_TO_CPU);
	rawDepth.SetFrom(cached_depth, MemoryCopyDirection::CPU_TO_CPU);

	if(this->LoadIntoCache()){
		++current_frame_number;
	}
}

BlankImageGenerator::BlankImageGenerator(const char *calibFilename, Vector2i image_size) : BaseImageSourceEngine(calibFilename)
{
	this->image_size = image_size;
}

bool BlankImageGenerator::HasMoreImages() const
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
