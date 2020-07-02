#pragma once

#include "../Camera/CalibIO.h"
#include "../../Utils/ImageTypes.h"

namespace ITMLib{
	/**
	 * \brief
	 * Represents a single "view", i.e. RGB and depth images along
	 * with all intrinsic and relative calibration information
	 * */
	class View{
	public:
		/// Intrinsic calibration information for the view.
		const RGBDCalib calibration_information;

		ShortImage* short_raw_disparity_image = nullptr;
		FloatImage* float_raw_disparity_image = nullptr;

		/// RGB colour image for the current frame.
		UChar4Image* rgb;

		/// RGB colour image for the previous frame.
		UChar4Image* rgb_prev;

		/// Float valued depth image, if available according to @ref inputImageType.
		FloatImage* depth;

		/// surface normal of depth image
		// allocated when needed
		Float4Image* depth_normal;

		/// uncertainty (std) in each pixel of depth value based on sensor noise model
		/// allocated when needed
		FloatImage* depthUncertainty;

		// confidence based on distance from center
		FloatImage* depthConfidence;

		View(const RGBDCalib& calibration_information, Vector2i rgb_image_size, Vector2i depth_image_size, bool use_GPU)
		: calibration_information(calibration_information)
		{
			this->short_raw_disparity_image = nullptr;
			this->float_raw_disparity_image = nullptr;
			this->rgb = new UChar4Image(rgb_image_size, true, use_GPU);
			this->rgb_prev = nullptr;
			this->depth = new FloatImage(depth_image_size, true, use_GPU);
			this->depth_normal = nullptr;
			this->depthUncertainty = nullptr;
			this->depthConfidence = new FloatImage(depth_image_size, true, use_GPU);
		}

		virtual ~View()
		{
			delete short_raw_disparity_image;
			delete float_raw_disparity_image;

			delete rgb;
			delete rgb_prev;

			delete depth;
			delete depthConfidence;

			delete depth_normal;
			delete depthUncertainty;
		}

		// Suppress the default copy constructor and assignment operator
		View(const View&);
		View& operator=(const View&);
	};
}
