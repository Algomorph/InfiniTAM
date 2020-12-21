// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "LibUVCEngine.h"

using namespace InputSource;

#ifdef COMPILE_WITH_LibUVC

#include <libuvc/libuvc.h>

class LibUVCEngine::PrivateData {
	public:
	PrivateData()
	{
		ctx = nullptr;
		dev = nullptr; devh_d = nullptr; devh_rgb = nullptr;
		//ctrl_rgb = nullptr; ctrl_d = nullptr;
		framebuffer_rgb = nullptr; framebuffer_depth = nullptr;
		got_depth = got_color = 0;
	}

	uvc_context_t *ctx;
	uvc_device_t *dev;
	uvc_device_handle_t *devh_d;
	uvc_device_handle_t *devh_rgb;
	uvc_stream_ctrl_t ctrl_rgb;
	uvc_stream_ctrl_t ctrl_d;

	uvc_frame_t *framebuffer_rgb;
	uvc_frame_t *framebuffer_depth;

	bool hasColour;
	int got_depth, got_color;
};

void callback_rgb(uvc_frame_t *frame, void *_data)
{
	LibUVCEngine::PrivateData *data = (LibUVCEngine::PrivateData*)_data;

	if (data->framebuffer_rgb == nullptr) {
		data->framebuffer_rgb = uvc_allocate_frame(frame->width * frame->height * 3);
	}

	uvc_any2rgb(frame, data->framebuffer_rgb);
	data->got_color = 1;
}

void callback_depth(uvc_frame_t *frame, void *_data)
{
	LibUVCEngine::PrivateData *data = (LibUVCEngine::PrivateData*)_data;

	if (data->framebuffer_depth == nullptr) {
		data->framebuffer_depth = uvc_allocate_frame(frame->width * frame->height * 2);
	}

	uvc_duplicate_frame(frame, data->framebuffer_depth);
	data->framebuffer_depth->frame_format = UVC_FRAME_FORMAT_GRAY16;
	data->got_depth = 1;
}

struct FormatSpec {
	const uvc_frame_desc *frame_desc;
	uint32_t interval;
};

bool formatIdentical(const uint8_t *guidFormat, const char *charFormat)
{
	for (int i = 0; i < 16; ++i) {
		if (*charFormat == 0) return true;
		if (*charFormat != (char)*guidFormat) return false;
		++charFormat; ++guidFormat;
	}
	return false;
}

struct FormatSpec findSuitableFormat(uvc_device_handle_t *dev, int requiredResolutionX = -1, int requiredResolutionY = -1, const char *requiredFormat = nullptr)
{
	FormatSpec ret;
	ret.frame_desc = nullptr; ret.interval = 0;

	const uvc_format_desc_t *formats = uvc_get_format_descs(dev);

	if (requiredFormat != nullptr) {
		const uvc_format_desc_t *format_it = formats;
		while (format_it != nullptr) {
			//fprintf(stderr, "%c%c%c%c\n", format_it->guidFormat[0], format_it->guidFormat[1], format_it->guidFormat[2], format_it->guidFormat[3]);
			if (formatIdentical(format_it->guidFormat, requiredFormat)) {
				formats = format_it;
				break;
			} else {
				format_it = format_it->next;
			}
		}
	}

	for (struct uvc_frame_desc *frame_desc = formats->frame_descs; frame_desc != nullptr; frame_desc = frame_desc->next) {
		//fprintf(stderr, "%i %i\n", frame_desc->wWidth, frame_desc->wHeight);
		uint32_t mininterval = frame_desc->intervals[0];
		for (int j = 0; frame_desc->intervals[j] != 0; ++j) {
			if (mininterval > frame_desc->intervals[j]) mininterval = frame_desc->intervals[j];
		}

		bool acceptAsBest = false;
		if (ret.frame_desc == nullptr) acceptAsBest = true;
		else if ((frame_desc->wWidth == ret.frame_desc->wWidth) &&
			(frame_desc->wHeight == ret.frame_desc->wHeight) &&
			(mininterval < ret.interval)) {
			acceptAsBest = true;
		} else if ((requiredResolutionX <= 0)&&(requiredResolutionY <= 0)) {
			if (frame_desc->wWidth > ret.frame_desc->wWidth) {
				acceptAsBest = true;
			}
		} else {
			int diffX_cur = abs(frame_desc->wWidth - requiredResolutionX);
			int diffX_best = abs(ret.frame_desc->wWidth - requiredResolutionX);
			int diffY_cur = abs(frame_desc->wHeight - requiredResolutionY);
			int diffY_best = abs(ret.frame_desc->wHeight - requiredResolutionY);
			if (requiredResolutionX > 0) {
				if (diffX_cur < diffX_best) {
					acceptAsBest = true;
				}
				if ((requiredResolutionY > 0)&&(diffX_cur == diffX_best)&&(diffY_cur < diffY_best)) {
					acceptAsBest = true;
				}
			} else if (requiredResolutionY > 0) {
				if (diffY_cur < diffY_best) {
					acceptAsBest = true;
				}
			}
		}
		if (acceptAsBest) {
			ret.frame_desc = frame_desc;
			ret.interval = mininterval;
		}
	}

	//fprintf(stderr, "bestMode: %i %i %i\n", ret.frame_desc->wWidth, ret.frame_desc->wHeight, (int)(10000000.0f*(1.0f/(float)ret.interval)));
	return ret;
}

enum {
	CAMERA_UNKNOWN,
	CAMERA_DEPTH,
	CAMERA_RGB
};

int findTypeOfCamera(uvc_device_handle_t *dev, const char **formatID = nullptr)
{
	const uvc_format_desc_t *formats = uvc_get_format_descs(dev);
	const uvc_format_desc_t *format_it = formats;

	bool reportsColourMode = false;
	int numReportedFormats = 0;
	if (formatID!=nullptr) *formatID = nullptr;
	while (format_it != nullptr) {
		if (formatIdentical(format_it->guidFormat, "Z16 ")) {
			if (formatID!=nullptr) *formatID = "Z16 ";
			return CAMERA_DEPTH;
		} else if (formatIdentical(format_it->guidFormat, "INVI")) {
			if (formatID!=nullptr) *formatID = nullptr;
			return CAMERA_DEPTH;
		} else if (formatIdentical(format_it->guidFormat, "RW10")) {
			return CAMERA_RGB;
		} else if (formatIdentical(format_it->guidFormat, "YUY2")) {
			reportsColourMode = true;
		}
		numReportedFormats++;
		format_it = format_it->next;
	}
	if (reportsColourMode&&(numReportedFormats==1)) return CAMERA_RGB;
	return CAMERA_UNKNOWN;
}

struct DeviceID {
	int vid, pid;
	bool providesTwoStreams;
	float depthScaleFactor;
} knownDeviceIDs[] = {
	{ 0x8086, 0x0a66, true, 1.0f/10000.0f }, // Intel RealSense F200
	{ 0x8086, 0x0a80, false, 1.0f/1000.0f }  // Intel RealSense R200
	//{ 0, 0 }  // match anything, not recommended...
};

LibUVCEngine::LibUVCEngine(const char *calibFilename, Vector2i requested_imageSize_rgb, Vector2i requested_imageSize_d)
	: BaseImageSourceEngine(calibFilename)
{
	// default values to be returned if nothing else works
	this->calib.disparity_calibration_coefficients.SetStandard();
	this->imageSize_rgb = Vector2i(0,0);
	this->imageSize_d = Vector2i(0,0);

	data = new PrivateData();

	do {
		uvc_error_t res;
		res = uvc_init(&(data->ctx), nullptr);
		if (res < 0) {
			uvc_perror(res, "LibUVCEngine");
			break;
		}

		// Step 1: find device
		unsigned int deviceID = 0;
		for (; deviceID < sizeof(knownDeviceIDs)/sizeof(struct DeviceID); ++deviceID) {
			//fprintf(stderr, "scanning for devices %x %x\n", knownDeviceIDs[deviceID].vid, knownDeviceIDs[deviceID].pid);
			res = uvc_find_device(data->ctx, &(data->dev),
				knownDeviceIDs[deviceID].vid, knownDeviceIDs[deviceID].pid, nullptr); /* filter devices: vendor_id, product_id, "serial_num" */
			if (res == 0) break;
		}

		if (res < 0) {
			/* no devices found */
			uvc_perror(res, "uvc_find_device failed");
			break;
		}

		bool providesTwoStreams = knownDeviceIDs[deviceID].providesTwoStreams;
		this->calib.disparity_calibration_coefficients.SetFrom(knownDeviceIDs[deviceID].depthScaleFactor,
			0.0f, ITMLib::DisparityCalib::TRAFO_AFFINE);

		// Step 2: find depth and rgb sub devices
		const char *depthFormatID = nullptr;
		for (int cameraID = 0; ; cameraID++) {
			uvc_device_handle_t *devh_tmp = nullptr;
			res = uvc_open2(data->dev, &devh_tmp, cameraID);
			if (res < 0) break;

			const char *tmpFormatID;
			int type = findTypeOfCamera(devh_tmp, &tmpFormatID);
			switch (type) {
			case CAMERA_DEPTH:
				if (data->devh_d == nullptr) {
					data->devh_d = devh_tmp;
					depthFormatID = tmpFormatID;
				}
				break;
			case CAMERA_RGB:
				if (data->devh_rgb == nullptr) data->devh_rgb = devh_tmp;
				break;
			case CAMERA_UNKNOWN:
				//uvc_close(devh_tmp);
				break;
			}
		}

		if (data->devh_d == nullptr) {
			fprintf(stderr, "could not identify camera, trying hardcoded defaults\n");
			depthFormatID = nullptr;

			/* Try to open the device: requires exclusive access */
			res = uvc_open2(data->dev, &(data->devh_rgb), 0);
			if (res < 0) {
				uvc_perror(res, "uvc_open rgb");
				break;
			}

			res = uvc_open2(data->dev, &(data->devh_d), 1);
			if (res < 0) {
				uvc_perror(res, "uvc_open depth");
				break;
			}
		}

		// R200 appears to provide only either depth or colour
		if (!providesTwoStreams) {
			data->devh_rgb = nullptr;
			data->hasColour = false;
		}

		/* Print out a message containing all the information that
		 * libuvc knows about the device */
		//if (data->hasColour) uvc_print_diag(data->devh_rgb, stderr);
		//uvc_print_diag(data->devh_d, stderr);

		// Step 3: Find video formats
		FormatSpec format_rgb, format_d;
		if (data->hasColour) format_rgb = findSuitableFormat(data->devh_rgb, requested_imageSize_rgb.x, requested_imageSize_rgb.y);
		else format_rgb.frame_desc = nullptr;
		format_d = findSuitableFormat(data->devh_d, requested_imageSize_rgb.x, requested_imageSize_rgb.y, depthFormatID);

		if (data->hasColour) {
			/* Try to negotiate a colour stream */
			res = uvc_get_stream_ctrl_frame_desc(
				data->devh_rgb, &(data->ctrl_rgb),
				format_rgb.frame_desc,
				format_rgb.interval);
			if (res < 0) {
				uvc_perror(res, "get_mode rgb");
				break;
			}

		}

		/* Try to negotiate a depth stream */
		res = uvc_get_stream_ctrl_frame_desc(
			data->devh_d, &(data->ctrl_d),
			format_d.frame_desc,
			format_d.interval);

		if (res < 0) {
			uvc_perror(res, "get_mode depth");
			break;
		}

		/* Print out the result */
		//uvc_print_stream_ctrl(&(data->ctrl_d), stderr);

		if (data->hasColour) {
			res = uvc_start_streaming(data->devh_rgb, &(data->ctrl_rgb), callback_rgb, data, 0);
			if (res < 0) {
				uvc_perror(res, "start_streaming rgb");
			}
		}
		res = uvc_start_streaming(data->devh_d, &(data->ctrl_d), callback_depth, data, 0);
		if (res < 0) {
			uvc_perror(res, "start_streaming depth");
			break;
		}
//		uvc_print_diag(data->devh_rgb, nullptr);

		if (format_rgb.frame_desc != nullptr) {
			this->imageSize_rgb = Vector2i(format_rgb.frame_desc->wWidth, format_rgb.frame_desc->wHeight);
		}
		this->imageSize_d = Vector2i(format_d.frame_desc->wWidth, format_d.frame_desc->wHeight);
	} while (false);
}

LibUVCEngine::~LibUVCEngine()
{
	/* End the stream. Blocks until last callback is serviced */
	if (data->devh_rgb != nullptr) {
		uvc_stop_streaming(data->devh_rgb);
	}
	if (data->devh_d != nullptr) {
		uvc_stop_streaming(data->devh_d);
	}
	if (data->devh_rgb != nullptr) {
		uvc_close(data->devh_rgb);
	}
	if (data->devh_d != nullptr) {
		uvc_close(data->devh_d);
	}

	/* Release the device descriptor */
	if (data->dev != nullptr) uvc_unref_device(data->dev);
	if (data->ctx != nullptr) uvc_exit(data->ctx);
}

void LibUVCEngine::GetImages(UChar4Image& rgb_image, ShortImage& raw_depth_image)
{
	int counter = 0;
	while ((data->hasColour&&(data->got_color == 0)) || (data->got_depth == 0)) {
		struct timespec sleep_time{0, 10000000};
		nanosleep(&sleep_time, nullptr);
		if (counter++ > 100) {
			fprintf(stderr, "timeout waiting for images\n");
			return;
		}
	}

	Vector4u *rgb = rgb_image.GetData(MEMORYDEVICE_CPU);
	for (int i = 0; i < rgb_image.dimensions.x * rgb_image.dimensions.y; i++) {
		Vector4u newPix; char *oldPix = &(((char*)(data->framebuffer_rgb->data))[i*3]);
		newPix.x = oldPix[0]; newPix.y = oldPix[1]; newPix.z = oldPix[2]; newPix.w = 255;
		rgb[i] = newPix;
	}

	short *depth = raw_depth_image.GetData(MEMORYDEVICE_CPU);
	memcpy(depth, data->framebuffer_depth->data, raw_depth_image.size() * sizeof(short));

	data->got_color = data->got_depth = 0;
}

bool LibUVCEngine::HasMoreImages() const  { return true; }
Vector2i LibUVCEngine::GetDepthImageSize() const  { return imageSize_d; }
Vector2i LibUVCEngine::GetRGBImageSize() const { return imageSize_rgb; }

#else

LibUVCEngine::LibUVCEngine(const char* calibFilename, Vector2i requested_imageSize_rgb, Vector2i requested_imageSize_d)
		: BaseImageSourceEngine(calibFilename) {
	printf("compiled without LibUVC support\n");
}

LibUVCEngine::~LibUVCEngine() {}

void LibUVCEngine::GetImages(UChar4Image& rgbImage, ShortImage& rawDepthImage) { }

bool LibUVCEngine::HasMoreImages() const { return false; }

Vector2i LibUVCEngine::GetDepthImageSize() const { return Vector2i(0, 0); }

Vector2i LibUVCEngine::GetRGBImageSize() const { return Vector2i(0, 0); }

#endif

