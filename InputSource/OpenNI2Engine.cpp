// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "OpenNI2Engine.h"

#include "../ORUtils/FileUtils.h"

#include <cstdio>
#include <iostream>
#include <stdexcept>
#include <string>

#ifdef WITH_OPENNI2
#include <OpenNI.h>

using namespace InputSource;

class EOFListener : public openni::OpenNI::DeviceStateChangedListener
{
	public:
	EOFListener() : eofReached(false){}

	void onDeviceStateChanged(const openni::DeviceInfo*, openni::DeviceState newState)
	{
		if (newState == openni::DEVICE_STATE_EOF)
		{
			printf("OpenNI: Reached end of file\n");
			eofReached = true;
		}
	}

	bool reachedEOF() const { return eofReached; }

	private:
	bool eofReached;
};

class OpenNI2Engine::PrivateData {
	public:
	PrivateData() : streams(NULL) {}
	openni::Device device;
	openni::VideoStream depthStream, colorStream;

	openni::VideoFrameRef depthFrame;
	openni::VideoFrameRef colorFrame;
	openni::VideoStream **streams;

	EOFListener eofListener;
};

static openni::VideoMode findBestMode(const openni::SensorInfo *sensorInfo, int requiredResolutionX = -1, int requiredResolutionY = -1, openni::PixelFormat requiredPixelFormat = (openni::PixelFormat)-1)
{
	const openni::Array<openni::VideoMode> & modes = sensorInfo->getSupportedVideoModes();
	openni::VideoMode bestMode = modes[0];
	for (int m = 0; m < modes.getSize(); ++m) {
		//fprintf(stderr, "mode %i: %ix%i, %i %i\n", m, modes[m].getResolutionX(), modes[m].getResolutionY(), modes[m].getFps(), modes[m].getPixelFormat());
		const openni::VideoMode & curMode = modes[m];
		if ((requiredPixelFormat != (openni::PixelFormat)-1)&&(curMode.getPixelFormat() != requiredPixelFormat)) continue;

		bool acceptAsBest = false;
		if ((curMode.getResolutionX() == bestMode.getResolutionX())&&
			 (curMode.getFps() > bestMode.getFps())) {
			acceptAsBest = true;
		} else if ((requiredResolutionX <= 0)&&(requiredResolutionY <= 0)) {
			if (curMode.getResolutionX() > bestMode.getResolutionX()) {
				acceptAsBest = true;
			}
		} else {
			int diffX_cur = abs(curMode.getResolutionX()-requiredResolutionX);
			int diffX_best = abs(bestMode.getResolutionX()-requiredResolutionX);
			int diffY_cur = abs(curMode.getResolutionY()-requiredResolutionY);
			int diffY_best = abs(bestMode.getResolutionY()-requiredResolutionY);
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
		if (acceptAsBest) bestMode = curMode;
	}
	//fprintf(stderr, "=> best mode: %ix%i, %i %i\n", bestMode.getResolutionX(), bestMode.getResolutionY(), bestMode.getFps(), bestMode.getPixelFormat());
	return bestMode;
}

template <typename T>
void mirrorHorizontally(ORUtils::Image<T> *img)
{
	T *data = img->GetData(MEMORYDEVICE_CPU);
	const int width = img->dimensions.x, height = img->dimensions.y;
	const int halfWidth = width / 2;

	int rowOffset = 0;
	for (int y = 0; y < height; ++y)
	{
		for (int x = 0; x < halfWidth; ++x)
		{
			std::swap(data[rowOffset + x], data[rowOffset + width - 1 - x]);
		}
		rowOffset += width;
	}
}

OpenNI2Engine::OpenNI2Engine(const char *calibFilename, const char *deviceURI, const bool useInternalCalibration,
                             Vector2i requested_imageSize_rgb, Vector2i requested_imageSize_d)
	: BaseImageSourceEngine(calibFilename)
{
	// images from openni always come in millimeters...
	this->calib.disparityCalib.SetStandard();

	this->imageSize_d = Vector2i(0,0);
	this->imageSize_rgb = Vector2i(0,0);

	if (deviceURI==NULL) deviceURI = openni::ANY_DEVICE;

	data = new PrivateData();

	openni::Status rc = openni::STATUS_OK;

	rc = openni::OpenNI::initialize();
	printf("OpenNI: Initialization ... \n%s\n", openni::OpenNI::getExtendedError());

	rc = data->device.open(deviceURI);
	if (rc != openni::STATUS_OK)
	{
		std::string message("OpenNI: Device open failed!\n");
		message += openni::OpenNI::getExtendedError();
		openni::OpenNI::shutdown();
		delete data;
		data = NULL;
		std::cout << message;
		return;
	}

	// Add listener to handle the EOF event and prevent deadlocks while waiting for more frames.
	openni::OpenNI::addDeviceStateChangedListener(&data->eofListener);

	openni::PlaybackControl *control = data->device.getPlaybackControl();
	if (control != NULL) {
		// this is a file! make sure we get every frame
		control->setSpeed(-1.0f);
		control->setRepeatEnabled(false);
	}

	

	rc = data->depthStream.create(data->device, openni::SENSOR_DEPTH);
	if (rc == openni::STATUS_OK)
	{
		openni::VideoMode depthMode = findBestMode(data->device.getSensorInfo(openni::SENSOR_DEPTH), requested_imageSize_d.x, requested_imageSize_d.y, openni::PIXEL_FORMAT_DEPTH_1_MM);
		rc = data->depthStream.setVideoMode(depthMode);
		if (rc != openni::STATUS_OK)
		{
			printf("OpenNI: Failed to set depth mode\n");
		}
		data->depthStream.setMirroringEnabled(false);

		rc = data->depthStream.start();

		if (useInternalCalibration) data->device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
		if (rc != openni::STATUS_OK)
		{
			printf("OpenNI: Couldn't start depthStream stream:\n%s\n", openni::OpenNI::getExtendedError());
			data->depthStream.destroy();
		}

		imageSize_d.x = data->depthStream.getVideoMode().getResolutionX();
		imageSize_d.y = data->depthStream.getVideoMode().getResolutionY();

		printf("Initialised OpenNI depth camera with resolution: %d x %d\n", imageSize_d.x, imageSize_d.y);

		depth_available = true;
	}
	else
	{
		printf("OpenNI: Couldn't find depthStream stream:\n%s\n", openni::OpenNI::getExtendedError());
		depth_available = false;
	}

	rc = data->colorStream.create(data->device, openni::SENSOR_COLOR);
	if (rc == openni::STATUS_OK)
	{
		openni::VideoMode colourMode = findBestMode(data->device.getSensorInfo(openni::SENSOR_COLOR), requested_imageSize_rgb.x, requested_imageSize_rgb.y);
		rc = data->colorStream.setVideoMode(colourMode);
		if (rc != openni::STATUS_OK)
		{
			printf("OpenNI: Failed to set color mode\n");
		}
		data->colorStream.setMirroringEnabled(false);

		rc = data->colorStream.start();
		if (rc != openni::STATUS_OK)
		{
			printf("OpenNI: Couldn't start colorStream stream:\n%s\n", openni::OpenNI::getExtendedError());
			data->colorStream.destroy();
		}

		imageSize_rgb.x = data->colorStream.getVideoMode().getResolutionX();
		imageSize_rgb.y = data->colorStream.getVideoMode().getResolutionY();

		printf("Initialised OpenNI color camera with resolution: %d x %d\n", imageSize_rgb.x, imageSize_rgb.y);

		color_available = true;
	}
	else
	{
		printf("OpenNI: Couldn't find colorStream stream:\n%s\n", openni::OpenNI::getExtendedError());
		color_available = false;
	}

	if (!depth_available)
	{
		openni::OpenNI::shutdown();
		delete data;
		data = NULL;
		std::cout << "OpenNI: No valid streams. Exiting." << std::endl;
		return;
	}
	
	data->streams = new openni::VideoStream*[2];
	if (depth_available) data->streams[0] = &data->depthStream;
	if (color_available) data->streams[1] = &data->colorStream;

	if (useInternalCalibration) {
		this->calib.trafo_rgb_to_depth = ITMLib::Extrinsics();
		if (depth_available) {
			float h_fov = data->depthStream.getHorizontalFieldOfView();
			float v_fov = data->depthStream.getVerticalFieldOfView();
			this->calib.intrinsics_d.SetFrom(
				(float)imageSize_d.x / (2.0f * tan(h_fov/2.0f)),
				(float)imageSize_d.y / (2.0f * tan(v_fov/2.0f)),
				(float)imageSize_d.x / 2.0f,
				(float)imageSize_d.y / 2.0f);
		}
		if (color_available) {
			float h_fov = data->colorStream.getHorizontalFieldOfView();
			float v_fov = data->colorStream.getVerticalFieldOfView();
			this->calib.intrinsics_rgb.SetFrom(
				(float)imageSize_rgb.x / (2.0f * tan(h_fov/2.0f)),
				(float)imageSize_rgb.y / (2.0f * tan(v_fov/2.0f)),
				(float)imageSize_rgb.x / 2.0f,
				(float)imageSize_rgb.y / 2.0f);
		}
	}
}

OpenNI2Engine::~OpenNI2Engine()
{
	if (data)
	{
		if (depth_available)
		{
			data->depthStream.stop();
			data->depthStream.destroy();
		}
		if (color_available)
		{
			data->colorStream.stop();
			data->colorStream.destroy();
		}
		data->device.close();

		openni::OpenNI::removeDeviceStateChangedListener(&data->eofListener);

		delete[] data->streams;
		delete data;
	}

	openni::OpenNI::shutdown();
}

void OpenNI2Engine::GetImages(UChar4Image& rgbImage, ShortImage& rawDepthImage)
{
	int changedIndex, waitStreamCount;
	if (depth_available && color_available) waitStreamCount = 2;
	else waitStreamCount = 1;

	openni::Status rc = openni::OpenNI::waitForAnyStream(data->streams, waitStreamCount, &changedIndex);
	if (rc != openni::STATUS_OK) { printf("OpenNI: Wait failed\n"); return /*false*/; }

	if(depth_available) data->depthStream.readFrame(&data->depthFrame);
	if(color_available) data->colorStream.readFrame(&data->colorFrame);

	if (depth_available && !data->depthFrame.isValid()) return;
	if (color_available && !data->colorFrame.isValid()) return;

	Vector4u *rgb = rgbImage.GetData(MEMORYDEVICE_CPU);
	if (color_available){
		const auto* colorImagePix = (const openni::RGB888Pixel*)data->colorFrame.getData();
		for (int i = 0; i < rgbImage.dimensions.x * rgbImage.dimensions.y; i++){
			Vector4u newPix; openni::RGB888Pixel oldPix = colorImagePix[i];
			newPix.x = oldPix.r; newPix.y = oldPix.g; newPix.z = oldPix.b; newPix.w = 255;
			rgb[i] = newPix;
		}
	}
	else memset(rgb, 0, rgbImage.size() * sizeof(Vector4u));

	short *depth = rawDepthImage.GetData(MEMORYDEVICE_CPU);
	if (depth_available){
		const auto* depthImagePix = (const openni::DepthPixel*)data->depthFrame.getData();
		memcpy(depth, depthImagePix, rawDepthImage.size() * sizeof(short));
	}
	else memset(depth, 0, rawDepthImage.size() * sizeof(short));

	//TODO: provide a state variable in the class that allows to configure input mirroring
	// dynamically instead of using preprocessor defines (or CMake)
#if USE_INPUT_MIRRORING
	// Mirror the input images horizontally (this is sometimes required when using a Kinect).
	if (color_available) mirrorHorizontally(rgbImage);
	if (depth_available) mirrorHorizontally(rawDepthImage);
#endif
}

bool OpenNI2Engine::HasMoreImages() const { return data && !data->eofListener.reachedEOF(); }
Vector2i OpenNI2Engine::GetDepthImageSize() const { return data ? imageSize_d : Vector2i(0, 0); }
Vector2i OpenNI2Engine::GetRGBImageSize() const { return data ? imageSize_rgb : Vector2i(0, 0); }

#else

using namespace InputSource;

OpenNI2Engine::OpenNI2Engine(const char* calibFilename, const char* deviceURI, const bool useInternalCalibration,
                           Vector2i requested_imageSize_rgb, Vector2i requested_imageSize_d)
		: BaseImageSourceEngine(calibFilename) {
	printf("compiled without OpenNI support\n");
}

OpenNI2Engine::~OpenNI2Engine() {}

void OpenNI2Engine::GetImages(UChar4Image& rgbImage, ShortImage& rawDepthImage) { return; }

bool OpenNI2Engine::HasMoreImages() const { return false; }

Vector2i OpenNI2Engine::GetDepthImageSize() const { return Vector2i(0, 0); }

Vector2i OpenNI2Engine::GetRGBImageSize() const { return Vector2i(0, 0); }

#endif // #ifdef WITH_OPENNI

