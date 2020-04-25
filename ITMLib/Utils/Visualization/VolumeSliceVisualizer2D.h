//  ================================================================
//  Created by Gregory Kramida on 12/5/17.
//  Copyright (c) 2017-2000 Gregory Kramida
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
#pragma once

#ifdef WITH_OPENCV
// OpenCV
#include <opencv2/core/core.hpp>

// local
#include "../Math.h"
#include "../../Objects/Volume/VoxelVolume.h"
#include "../Geometry/CardinalAxesAndPlanes.h"

namespace ITMLib {


template<typename TVoxel, typename TWarp, typename TIndex>
class VolumeSliceVisualizer2D {

	//TODO: positions of voxels to highlight / draw around should be defined extenally in the user code, not as class static members. Static functions should be probably changed to become member functions, with focus coordinates (testPos1, testPos2...) being set in user code during construction. -Greg (GitHub: Algomorph)
private: // member variables
	cv::Mat blank;
	cv::Mat liveImgTemplate;

	static const bool enable_abs_tsdf_filling_strategy;

	const int imageSizeVoxels;
	const int imageHalfSizeVoxels;

	const Vector6i bounds;


	const int imgXSlice;
	const int imgYSlice;
	const int imgZSlice;

	const int imgPixelRangeX;
	const int imgPixelRangeY;
	const int imgPixelRangeZ;

	Plane plane;
	const Vector3i focus_coordinates;

	const std::string outputDirectory;
	const float pixelsPerVoxel;

public: // member functions

	static float SdfToShadeValue(float sdf);

	explicit VolumeSliceVisualizer2D(Vector3i focus_coordinates, unsigned int imageSizeVoxels, float pixelsPerVoxel,
	                                 Plane plane);

	virtual ~VolumeSliceVisualizer2D() = default;

	Plane GetPlane() const;

	void SaveSceneSlicesAs2DImages_AllDirections(
			VoxelVolume <TVoxel, TIndex>* scene, std::string pathWithoutPostfix);
	void SaveSceneSlicesAs2DImages(VoxelVolume <TVoxel, TIndex>* scene,
	                               Axis axis, std::string path);

	cv::Mat DrawSceneImageAroundPoint(VoxelVolume <TVoxel, TIndex>* scene);
	cv::Mat DrawWarpedSceneImageAroundPoint(VoxelVolume <TVoxel, TIndex>* scene, VoxelVolume <TWarp, TIndex>* warpField);

	void MarkWarpedSceneImageAroundFocusPoint(VoxelVolume <TVoxel, TIndex>* scene,
			VoxelVolume <TWarp, TIndex>* warpField, cv::Mat& imageToMarkOn);
	void MarkWarpedSceneImageAroundPoint(VoxelVolume <TVoxel, TIndex>* scene,
			VoxelVolume <TWarp, TIndex>* warpField, cv::Mat& imageToMarkOn, Vector3i positionOfVoxelToMark);

	void RecordWarpUpdates();


private: // member functions
	void InitializeWarp2DSliceRecording(VoxelVolume<TVoxel, TIndex>* canonicalScene,
	                                    VoxelVolume<TVoxel, TIndex>* sourceLiveScene);

	void RenderSceneSlices(VoxelVolume <TVoxel, TIndex>* scene,
	                       Axis axis,
	                       const std::string& outputFolder,
	                       bool verbose = false);

	Vector2i GetVoxelImageCoordinates(Vector3i coordinates, Plane plane);
	Vector2i GetVoxelImageCoordinates(Vector3f coordinates, Plane plane);

	bool IsPointInImageRange(int x, int y, int z) const;
};


}//namespace ITMLib

#endif // #ifdef WITH_OPENCV
