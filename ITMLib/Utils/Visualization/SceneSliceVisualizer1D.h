//  ================================================================
//  Created by Gregory Kramida on 6/5/18.
//  Copyright (c) 2018-2025 Gregory Kramida
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


//local
#include "../Geometry/CardinalAxesAndPlanes.h"
#include "../Math.h"
#include "../../Objects/Volume/VoxelVolume.h"
#include "VisualizationWindowManager.h"

template<typename T>
class vtkSmartPointer;
class vtkChartXY;


namespace ITMLib{
class SceneSliceVisualizer1D {
public:
	SceneSliceVisualizer1D(Vector3i focus_coordinate, Axis axis, unsigned int voxel_range);
	~SceneSliceVisualizer1D() = default;


	template<typename TVoxel, typename TIndex>
	void Plot1DSceneSlice(VoxelVolume <TVoxel, TIndex>* scene, Vector4i color, double width);
	template<typename TVoxel, typename TWarp, typename TIndex>
	void Draw1DWarpUpdateVector(
			VoxelVolume <TVoxel, TIndex>* TSDF,
			VoxelVolume<TWarp, TIndex>* warp_field, Vector4i color);
	void SaveScreenshot(std::string path);


private:
	template<typename TVoxel, typename TIndex, typename TGetSDFFunctor>
	void Plot1DSceneSliceHelper(VoxelVolume <TVoxel, TIndex>* volume, Vector4i color, double width);

	ChartWindow* window;

	const Vector3i focus_coordinates;
	const Axis axis;
	const int rangeStartVoxelIndex;
	const int rangeEndVoxelIndex;
	const unsigned int voxelRange;
	float previous_point = 0.0;


};


}//namespace ITMLib


