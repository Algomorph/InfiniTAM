#pragma once

#include "../Camera/CalibIO.h"
#include "../../Utils/ImageTypes.h"

namespace ITMLib {
/**
 * \brief
 * Represents a single "view", i.e. RGB and depth images along
 * with all intrinsic and relative calibration information
 * */
class View {
public:
	/// Intrinsic calibration information for the view.
	const RGBDCalib calibration_information;

	ShortImage short_raw_disparity_image;
	FloatImage float_raw_disparity_image;

	/// RGB colour image for the current frame.
	UChar4Image rgb;

	/// Float valued depth image, if available according to @ref inputImageType.
	FloatImage depth;

	// confidence based on distance from center
	FloatImage depth_confidence;

	/// uncertainty (std) in each pixel of depth value based on sensor noise model
	/// allocated when needed
	FloatImage* depth_uncertainty;

	/// RGB colour image for the previous frame.
	UChar4Image* rgb_prev;

	/// surface normal of depth image
	// allocated when needed
	Float4Image* depth_normal;

	View(const RGBDCalib& calibration_information, Vector2i rgb_image_size, Vector2i depth_image_size, bool use_GPU)
			: calibration_information(calibration_information),
			  short_raw_disparity_image(depth_image_size, true, use_GPU),
			  float_raw_disparity_image(depth_image_size, true, use_GPU),
			  rgb(rgb_image_size, true, use_GPU),
			  depth(depth_image_size, true, use_GPU),
			  depth_confidence(depth_image_size, true, use_GPU) {
		this->rgb_prev = nullptr;
		this->depth_normal = nullptr;
		this->depth_uncertainty = nullptr;
	}

	virtual ~View() {
		delete rgb_prev;
		delete depth_normal;
		delete depth_uncertainty;
	}

	//TODO: Unsuppress, provide rule of 4.5
	// Suppress the default copy constructor and assignment operator
	View(const View&);
	View& operator=(const View&);
};
}
