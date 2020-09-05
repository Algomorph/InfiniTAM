//  ================================================================
//  Created by Gregory Kramida on 7/4/18.
//  Copyright (c) 2018-2000 Gregory Kramida
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at

//  http://www.apache.org/licenses/LICENSE-2.0

//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
//  ================================================================

#include "CreateDefaultImageSource.h"

void CreateDefaultImageSource(ImageSourceEngine*& imageSource, IMUSourceEngine*& imuSource,
		const ITMLib::configuration::Paths& inputPaths) {


	if (inputPaths.calibration_file_path == "viewer") {
		imageSource = new BlankImageGenerator("", Vector2i(640, 480));
		printf("starting in viewer mode: make sure to press n first to initialize the views ... \n");
		return;
	}

	const char* calibFile = inputPaths.calibration_file_path.empty() ? nullptr : inputPaths.calibration_file_path.c_str();

	if (calibFile) {
		printf("using calibration file: %s\n", inputPaths.calibration_file_path.c_str());
	} else {
		printf("using default calibration file.\n");
	}

	if (imageSource == nullptr && !inputPaths.rgb_image_path_mask.empty() && !inputPaths.depth_image_path_mask.empty()) {
		printf("using rgb images: %s\nusing depth images: %s\n", inputPaths.rgb_image_path_mask.c_str(), inputPaths.depth_image_path_mask.c_str());
		if (!inputPaths.mask_image_path_mask.empty() && inputPaths.imu_input_path.empty()) {
			printf("using mask images: %s\n", inputPaths.mask_image_path_mask.c_str());
		}
		if (inputPaths.imu_input_path.empty()) {
			ImageMaskPathGenerator pathGenerator(inputPaths.rgb_image_path_mask.c_str(), inputPaths.depth_image_path_mask.c_str(),
			                                     inputPaths.mask_image_path_mask.empty() ? nullptr : inputPaths.mask_image_path_mask.c_str());
			imageSource = new ImageFileReader<ImageMaskPathGenerator>(inputPaths.calibration_file_path.c_str(), pathGenerator);
		} else {
			printf("using imu data: %s\n", inputPaths.imu_input_path.c_str());
			imageSource = new RawFileReader(inputPaths.calibration_file_path.c_str(), inputPaths.rgb_image_path_mask.c_str(),
			                                inputPaths.depth_image_path_mask.c_str(), Vector2i(320, 240), 0.5f);
			imuSource = new IMUSourceEngine(inputPaths.imu_input_path.c_str());
		}
		int depthWidth = imageSource->GetDepthImageSize().x;
		if (depthWidth == 0) {
			delete imageSource;
			if (imuSource != nullptr) delete imuSource;
			imuSource = nullptr;
			imageSource = nullptr;
		}
	}
	if ((imageSource == nullptr) && (!inputPaths.rgb_video_file_path.empty() && !inputPaths.depth_video_file_path.empty()) &&
	    (inputPaths.imu_input_path.empty())) {
		imageSource = new InputSource::FFMPEGReader(inputPaths.calibration_file_path.c_str(), inputPaths.rgb_video_file_path.c_str(),
		                                            inputPaths.depth_video_file_path.c_str());
		if (imageSource->GetDepthImageSize().x == 0) {
			delete imageSource;
			imageSource = nullptr;
		}
	}

	if (imageSource == nullptr) {
		// If no calibration file specified, use the factory default calibration
		bool useInternalCalibration = !calibFile || strlen(calibFile) == 0;

		printf("trying OpenNI device: %s - calibration: %s\n",
		       inputPaths.openni_file_path.empty() ? "<OpenNI default device>" : inputPaths.openni_file_path.c_str(),
		       useInternalCalibration ? "internal" : "from file");
		imageSource = new OpenNI2Engine(calibFile, inputPaths.openni_file_path.empty() ? nullptr : inputPaths.openni_file_path.c_str(),
		                                useInternalCalibration);
		if (imageSource->GetDepthImageSize().x == 0) {
			delete imageSource;
			imageSource = nullptr;
		}
	}

	if (imageSource == nullptr) {
		printf("trying UVC device\n");
		imageSource = new LibUVCEngine(calibFile);
		if (imageSource->GetDepthImageSize().x == 0) {
			delete imageSource;
			imageSource = nullptr;
		}
	}

	if (imageSource == nullptr) {
		printf("trying RealSense device\n");
		imageSource = new RealSenseEngine(calibFile);
		if (imageSource->GetDepthImageSize().x == 0) {
			delete imageSource;
			imageSource = nullptr;
		}
	}

	if (imageSource == nullptr) {
		printf("trying MS Kinect 2 device\n");
		imageSource = new Kinect2Engine(calibFile);
		if (imageSource->GetDepthImageSize().x == 0) {
			delete imageSource;
			imageSource = nullptr;
		}
	}

	if (imageSource == nullptr) {
		printf("trying PMD PicoFlexx device\n");
		imageSource = new PicoFlexxEngine(calibFile);
		if (imageSource->GetDepthImageSize().x == 0) {
			delete imageSource;
			imageSource = nullptr;
		}
	}
}
