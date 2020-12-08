#pragma once

//local
#include "../../Objects/Misc/IMUMeasurement.h"
#include "../../CameraTrackers/Interface/CameraTracker.h"
#include "../../Utils/Configuration/Configuration.h"
#include "../Common/Configurable.h"
#include "MainEngineSettings.h"

//FIXME main documentation page
/** \mainpage
    This is the API reference documentation for Reco.
    It is generally under construction.
*/

namespace ITMLib {
//FIXME documentation for "main engines" (algorithms?) when new architecture is finalized
/** \brief
	Main engine, that instantiates all the other engines and
	provides a simplified interface to them.

	This class is the main entry point to the ITMLib library
	and basically performs the whole KinectFusion algorithm.
	It stores the latest image internally, as well as the 3D
	world model and additionally it keeps track of the camera
	pose.

	The intended use is as follows:
	-# Create an ITMMainEngine specifying the internal settings,
	   camera parameters and image sizes
	-# Get the pointer to the internally stored images with
	   @ref GetView() and write new image information to that
	   memory
	-# Call the method @ref ProcessFrame() to track the camera
	   and integrate the new information into the world model
	-# Optionally access the rendered reconstruction or another
	   image for Rendering using @ref GetImage()
	-# Iterate the above three steps for each image in the
	   sequence

	To access the internal information, look at the member
	variables @ref trackingState and @ref scene.
*/
class FusionAlgorithm : public Configurable<MainEngineSettings> {
public:
	// FIXME: this needs to be restructured. Why get original input images from here if the user is inputting them?...
	enum GetImageType {
		InfiniTAM_IMAGE_ORIGINAL_RGB,
		InfiniTAM_IMAGE_ORIGINAL_DEPTH,
		InfiniTAM_IMAGE_SCENERAYCAST,
		InfiniTAM_IMAGE_COLOUR_FROM_VOLUME,
		InfiniTAM_IMAGE_COLOUR_FROM_NORMAL,
		InfiniTAM_IMAGE_COLOUR_FROM_CONFIDENCE,
		InfiniTAM_IMAGE_FREECAMERA_CANONICAL,
		InfiniTAM_IMAGE_FREECAMERA_SHADED,
		InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_VOLUME,
		InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_NORMAL,
		InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_CONFIDENCE,
		InfiniTAM_IMAGE_UNKNOWN
	};

	/// Gives access to the current input frame
	virtual View* GetView() = 0;

	/// Gives access to the current camera pose and additional tracking information
	virtual CameraTrackingState* GetTrackingState() = 0;

	/// Process a frame with rgb and depth images and optionally a corresponding imu measurement
	virtual CameraTrackingState::TrackingResult
	ProcessFrame(UChar4Image* rgbImage, ShortImage* rawDepthImage, IMUMeasurement* imuMeasurement = nullptr) = 0;

	/// Get a result image as output
	virtual Vector2i GetImageSize() const = 0;

	virtual void GetImage(UChar4Image* out, GetImageType getImageType, ORUtils::SE3Pose* pose = nullptr, Intrinsics* intrinsics = nullptr) = 0;

	/// Extracts a mesh from the current volume and saves it to the model file specified by the file name
	virtual void SaveVolumeToMesh(const std::string& path) {};

	/// save and load the full scene and relocaliser (if any) to/from file
	virtual void SaveToFile() {};

	virtual void SaveToFile(const std::string& path) {};

	virtual void LoadFromFile() {};

	virtual void LoadFromFile(const std::string& path) {};

	/// resets the scene and the tracker
	virtual void ResetAll() = 0;

	//FIXME too specific
	/// switch for turning tracking on/off
	virtual void TurnOnTracking() = 0;
	virtual void TurnOffTracking() = 0;

	/// switch for turning integration on/off
	virtual void TurnOnIntegration() = 0;
	virtual void TurnOffIntegration() = 0;

	/// switch for turning main processing on/off
	virtual void TurnOnMainProcessing() = 0;
	virtual void TurnOffMainProcessing() = 0;

	virtual bool GetMainProcessingOn() const = 0;


	virtual ~FusionAlgorithm() {}
};

} // namespace ITMLib
