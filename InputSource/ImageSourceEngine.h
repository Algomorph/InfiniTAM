// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include <vector>

#include "../ITMLib/Objects/Camera/RGBDCalib.h"
#include "../ITMLib/Utils/ImageTypes.h"

namespace InputSource {

	class ImageSourceEngine
	{
	public:
		virtual ~ImageSourceEngine() {}

		/**
		 * \brief Gets the calibration parameters associated with the next RGB-D image (if any).
		 *
		 * \pre     HasMoreImages()
		 * \return  The calibration parameters associated with the next RGB-D image (if any), or the default calibration parameters otherwise.
		 */
		virtual ITMLib::RGBDCalib getCalib() const = 0;

		/**
		 * \brief Gets the size of the next depth image (if any).
		 *
		 * \pre     HasMoreImages()
		 * \return  The size of the next depth image (if any), or Vector2i(0,0) otherwise.
		 */
		virtual Vector2i GetDepthImageSize() = 0;

		/**
		 * \brief Gets the next RGB and depth images (if any).
		 *
		 * \pre             HasMoreImages()
		 * \param rgb       An image into which to store the next RGB image.
		 * \param rawDepth  An image into which to store the next depth image.
		 */
		virtual void GetImages(ITMUChar4Image& rgb, ITMShortImage& rawDepth) = 0;

		/**
		 * \brief Gets the size of the next RGB image (if any).
		 *
		 * \pre     HasMoreImages()
		 * \return  The size of the next RGB image (if any), or Vector2i(0,0) otherwise.
		 */
		virtual Vector2i GetRGBImageSize() = 0;

		/**
		 * \brief Determines whether or not the image source engine has RGB-D images currently available.
		 *
		 * \return  true, if the image source engine has RGB-D images currently available, or false otherwise.
		 */
		virtual bool HasImagesNow()
		{
			// By default, this just returns the same as HasMoreImages(), but it can be overridden by image
			// source engines that need to support the case of being temporarily unable to yield images.
			return HasMoreImages();
		}

		/**
		 * \brief Determines whether or not the image source engine is able to yield more RGB-D images.
		 *
		 * \return  true, if the image source engine is able to yield more RGB-D images, or false otherwise.
		 */
		virtual bool HasMoreImages() = 0;
	};

	class BaseImageSourceEngine : public ImageSourceEngine
	{
	protected:
		ITMLib::RGBDCalib calib;

	public:
		explicit BaseImageSourceEngine(const char *calibFilename);

		/** Override */
		virtual ITMLib::RGBDCalib getCalib() const;
	};

	class ImageMaskPathGenerator
	{
	private:
		static const int BUF_SIZE = 2048;
		char rgbImageMask[BUF_SIZE];
		char depthImageMask[BUF_SIZE];
		char maskImageMask[BUF_SIZE];
	public:
		const bool hasMaskImagePaths;
		ImageMaskPathGenerator(const char* rgbImageMask, const char* depthImageMask, const char* maskImageMask = nullptr);
		std::string getRgbImagePath(size_t currentFrameNo) const;
		std::string getDepthImagePath(size_t currentFrameNo) const;
		std::string getMaskImagePath(size_t currentFrameNo) const;
	};

	class ImageListPathGenerator
	{
	private:
		std::vector<std::string> depthImagePaths;
		std::vector<std::string> rgbImagePaths;
		std::vector<std::string> maskImagePaths;

		size_t imageCount() const;

	public:
		const bool hasMaskImagePaths;
		ImageListPathGenerator(const std::vector<std::string>& rgbImagePaths_,
		                       const std::vector<std::string>& depthImagePaths_);
		ImageListPathGenerator(const std::vector<std::string>& rgbImagePaths_,
				                       const std::vector<std::string>& depthImagePaths_,
				                       const std::vector<std::string>& maskImagePaths_);
		std::string getRgbImagePath(size_t currentFrameNo) const;
		std::string getDepthImagePath(size_t currentFrameNo) const;

		std::string getMaskImagePath(size_t currentFrameNo) const;
	};

	template <typename PathGenerator>
	class ImageFileReader : public BaseImageSourceEngine
	{
	private:
		ITMUChar4Image cached_rgb;
		ITMShortImage cached_depth;
		ITMUCharImage cached_mask;

		void LoadIntoCache();
		mutable size_t cached_frame_number;
		size_t current_frame_number;
		mutable bool cache_is_valid;

		PathGenerator pathGenerator;
	public:

		ImageFileReader(const char *calibration_file_name, const PathGenerator& path_generator, size_t initial_frame_number = 0);

		bool HasMaskImages() const;
		bool HasMoreImages();
		void GetImages(ITMUChar4Image& rgb, ITMShortImage& raw_depth);
		Vector2i GetDepthImageSize();
		Vector2i GetRGBImageSize();
	};

	class CalibSource : public BaseImageSourceEngine
	{
	private:
		Vector2i image_size;
		void ResizeIntrinsics(ITMLib::Intrinsics &intrinsics, float ratio);

	public:
		CalibSource(const char *calibFilename, Vector2i setImageSize, float ratio);
		~CalibSource() { }

		bool HasMoreImages() const { return true; }
		void GetImages(ITMUChar4Image& rgb, ITMShortImage& rawDepth) { }
		Vector2i GetDepthImageSize() const { return image_size; }
		Vector2i GetRGBImageSize() const { return image_size; }
	};

	class RawFileReader : public BaseImageSourceEngine
	{
	private:
		static const int BUF_SIZE = 2048;
		char rgb_image_mask[BUF_SIZE];
		char depthImageMask[BUF_SIZE];

		mutable ITMUChar4Image cached_rgb;
		mutable ITMShortImage cached_depth;

		bool LoadIntoCache();
		mutable int cached_frame_number;
		int current_frame_number;

		Vector2i image_size;
		void ResizeIntrinsics(ITMLib::Intrinsics &intrinsics, float ratio);

	public:
		RawFileReader(const char *calibration_file_name, const char *rgb_image_filename_mask, const char *depth_image_filename_mask, Vector2i image_size, float ratio);
		~RawFileReader() { }

		bool HasMoreImages() override;
		void GetImages(ITMUChar4Image& rgb, ITMShortImage& rawDepth) override;

		Vector2i GetDepthImageSize() override { return image_size; }
		Vector2i GetRGBImageSize() override { return image_size; }
	};

	class BlankImageGenerator : public BaseImageSourceEngine
	{
	private:
		Vector2i imgSize;
	
	public:
		BlankImageGenerator(const char *calibFilename, Vector2i setImageSize);
		~BlankImageGenerator() { }

		bool HasMoreImages() override;
		void GetImages(ITMUChar4Image& rgb, ITMShortImage& raw_depth) override;

		Vector2i GetDepthImageSize() override { return imgSize; }
		Vector2i GetRGBImageSize() override { return imgSize; }
	};
}
