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
		virtual Vector2i GetDepthImageSize() const = 0;

		/**
		 * \brief Gets the next RGB and depth images (if any).
		 *
		 * \pre             HasMoreImages()
		 * \param rgb       An image into which to store the next RGB image.
		 * \param rawDepth  An image into which to store the next depth image.
		 */
		virtual void GetImages(UChar4Image& rgb, ShortImage& rawDepth) = 0;

		/**
		 * \brief Gets the size of the next RGB image (if any).
		 *
		 * \pre     HasMoreImages()
		 * \return  The size of the next RGB image (if any), or Vector2i(0,0) otherwise.
		 */
		virtual Vector2i GetRGBImageSize() const = 0;

		/**
		 * \brief Determines whether or not the image source engine has RGB-D images currently available.
		 *
		 * \return  true, if the image source engine has RGB-D images currently available, or false otherwise.
		 */
		virtual bool HasImagesNow() const{
			// By default, this just returns the same as HasMoreImages(), but it can be overridden by image
			// source engines that need to support the case of being temporarily unable to yield images.
			return HasMoreImages();
		}

		/**
		 * \brief Determines whether or not the image source engine is able to yield more RGB-D images.
		 *
		 * \return  true, if the image source engine is able to yield more RGB-D images, or false otherwise.
		 */
		virtual bool HasMoreImages() const = 0;
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
		char rgb_image_filename_mask[BUF_SIZE];
		char depth_image_filename_mask[BUF_SIZE];
		char mask_image_filename_mask[BUF_SIZE];
	public:
		const bool has_mask_image_paths;
		ImageMaskPathGenerator(const char* rgbImageMask, const char* depthImageMask, const char* maskImageMask = nullptr);
		std::string GetRgbImagePath(size_t currentFrameNo) const;
		std::string GetDepthImagePath(size_t current_frame_number) const;
		std::string GetMaskImagePath(size_t currentFrameNo) const;
	};

	class ImageListPathGenerator
	{
	private:
		std::vector<std::string> depth_image_paths;
		std::vector<std::string> rgb_image_paths;
		std::vector<std::string> mask_image_paths;

		size_t ImageCount() const;

	public:
		const bool has_mask_image_paths;
		ImageListPathGenerator(const std::vector<std::string>& rgbImagePaths_,
		                       const std::vector<std::string>& depthImagePaths_);
		ImageListPathGenerator(const std::vector<std::string>& rgbImagePaths_,
				                       const std::vector<std::string>& depthImagePaths_,
				                       const std::vector<std::string>& maskImagePaths_);
		std::string GetRgbImagePath(size_t currentFrameNo) const;
		std::string GetDepthImagePath(size_t currentFrameNo) const;

		std::string GetMaskImagePath(size_t currentFrameNo) const;
	};

	template <typename PathGenerator>
	class ImageFileReader : public BaseImageSourceEngine
	{
	private:
		UChar4Image cached_rgb;
		ShortImage cached_depth;
		UCharImage cached_mask;

		void LoadIntoCache();
		mutable size_t cached_frame_number;
		size_t current_frame_number;
		mutable bool cache_is_valid;

		PathGenerator path_generator;
	public:

		ImageFileReader(const char *calibration_file_name, const PathGenerator& path_generator, size_t initial_frame_number = 0);

		bool HasMaskImages() const;
		bool HasMoreImages() const override;
		void GetImages(UChar4Image& rgb, ShortImage& raw_depth);
		Vector2i GetDepthImageSize() const override;
		Vector2i GetRGBImageSize() const override;
	};

	class CalibSource : public BaseImageSourceEngine
	{
	private:
		Vector2i image_size;
		void ResizeIntrinsics(ITMLib::Intrinsics &intrinsics, float ratio);

	public:
		CalibSource(const char *calibFilename, Vector2i setImageSize, float ratio);
		~CalibSource() { }

		bool HasMoreImages() const override { return true; }
		void GetImages(UChar4Image& rgb, ShortImage& rawDepth) { }
		Vector2i GetDepthImageSize() const override { return image_size; }
		Vector2i GetRGBImageSize() const override { return image_size; }
	};

	class RawFileReader : public BaseImageSourceEngine
	{
	private:
		static const int BUF_SIZE = 2048;
		char rgb_image_filename_mask[BUF_SIZE];
		char depth_image_filename_mask[BUF_SIZE];

		mutable UChar4Image cached_rgb;
		mutable ShortImage cached_depth;

		mutable int cached_frame_number;
		int current_frame_number;

		Vector2i image_size;
		bool cache_is_valid = false;

		bool LoadIntoCache();
		void ResizeIntrinsics(ITMLib::Intrinsics &intrinsics, float ratio);

	public:
		RawFileReader(const char *calibration_file_name, const char *rgb_image_filename_mask, const char *depth_image_filename_mask, Vector2i image_size, float ratio);

		bool HasMoreImages() const override;
		void GetImages(UChar4Image& rgb, ShortImage& rawDepth) override;

		Vector2i GetDepthImageSize() const override { return image_size; }
		Vector2i GetRGBImageSize() const override { return image_size; }
	};

	class BlankImageGenerator : public BaseImageSourceEngine
	{
	private:
		Vector2i image_size;
	
	public:
		BlankImageGenerator(const char *calibFilename, Vector2i image_size);

		bool HasMoreImages() const override;
		void GetImages(UChar4Image& rgb, ShortImage& raw_depth) override;

		Vector2i GetDepthImageSize() const override { return image_size; }
		Vector2i GetRGBImageSize() const override { return image_size; }
	};
}
