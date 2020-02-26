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

//stdlib
#include <utility>
#include <vtkWindowToImageFilter.h>
#include <vtkPNGWriter.h>

//local
#include "SceneSliceVisualizer1D.h"
#include "SceneSliceVisualizer1D.tpp"
#include "../../Objects/Volume/VoxelVolume.h"
#include "../../Objects/Volume/RepresentationAccess.h"
#include "../CPPPrintHelpers.h"

#include "../../GlobalTemplateDefines.h"
#include "VisualizationWindowManager.h"


using namespace ITMLib;

// region ==================================== CONSTRUCTORS / DESTRUCTORS ==============================================

SceneSliceVisualizer1D::SceneSliceVisualizer1D(Vector3i focus_coordinate, Axis axis, unsigned int voxel_range) :
		focus_coordinates(focus_coordinate),
		axis(axis),
		voxelRange(voxel_range),
		rangeStartVoxelIndex(focus_coordinate[axis] - ((voxel_range + 1) / 2)),
		rangeEndVoxelIndex(focus_coordinate[axis] + (voxel_range / 2)),
		previous_point(focus_coordinate[axis]),
		window(VisualizationWindowManager::Instance().MakeOrGetChartWindow(
				"SceneSliceVisualizer1D_" + AxisToString(axis),
				"Volume 1D Slice Visualizer for " + AxisToString(axis) + " Axis")){}

//TODO: DRY violation -- same code as EnergyPlotter -- group into single class hierarchy with shared methods
void SceneSliceVisualizer1D::SaveScreenshot(std::string path) {
	vtkSmartPointer<vtkWindowToImageFilter> windowToImageFilter = vtkSmartPointer<vtkWindowToImageFilter>::New();
	vtkSmartPointer<vtkRenderWindow> renderWindow = window->GetRenderWindow();
	windowToImageFilter->SetInput(renderWindow);
	windowToImageFilter->SetScale(2);
	windowToImageFilter->SetInputBufferTypeToRGBA();
	windowToImageFilter->ReadFrontBufferOff();
	windowToImageFilter->Update();
	vtkSmartPointer<vtkPNGWriter> writer = vtkSmartPointer<vtkPNGWriter>::New();
	writer->SetFileName(path.c_str());
	writer->SetInputConnection(windowToImageFilter->GetOutputPort());
	writer->Write();
}

// endregion
// region ==================================== EXPLICIT INSTANTIATIONS =================================================

template void
SceneSliceVisualizer1D::Plot1DSceneSlice<TSDFVoxel, PlainVoxelArray>(
		VoxelVolume<TSDFVoxel, PlainVoxelArray>* scene, Vector4i color, double width);

template void
SceneSliceVisualizer1D::Draw1DWarpUpdateVector<TSDFVoxel, WarpVoxel, PlainVoxelArray>(
		VoxelVolume<TSDFVoxel, PlainVoxelArray>* TSDF,
		VoxelVolume<WarpVoxel, PlainVoxelArray>* warp_field,
		Vector4i color);


template void
SceneSliceVisualizer1D::Plot1DSceneSlice<TSDFVoxel, VoxelBlockHash>(
		VoxelVolume<TSDFVoxel, VoxelBlockHash>* scene, Vector4i color, double width);

template void
SceneSliceVisualizer1D::Draw1DWarpUpdateVector<TSDFVoxel, WarpVoxel, VoxelBlockHash>(
		VoxelVolume<TSDFVoxel, VoxelBlockHash>* TSDF,
		VoxelVolume<WarpVoxel, VoxelBlockHash>* warp_field,
		Vector4i color);

//======================================================================================================================
