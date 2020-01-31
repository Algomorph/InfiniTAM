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
#include "ITMSceneSliceVisualizer1D.h"
#include "ITMSceneSliceVisualizer1D.tpp"
#include "../../Objects/Scene/VoxelVolume.h"
#include "../../Objects/Scene/RepresentationAccess.h"
#include "../ITMPrintHelpers.h"

#include "../../ITMLibDefines.h"
#include "ITMVisualizationWindowManager.h"


using namespace ITMLib;

// region ==================================== CONSTRUCTORS / DESTRUCTORS ==============================================

ITMSceneSliceVisualizer1D::ITMSceneSliceVisualizer1D(Vector3i focusCoordinate, Axis axis, unsigned int voxelRange) :
		focusCoordinates(focusCoordinate),
		axis(axis),
		voxelRange(voxelRange),
		rangeStartVoxelIndex(focusCoordinate[axis] - ((voxelRange + 1) / 2)),
		rangeEndVoxelIndex(focusCoordinate[axis] + (voxelRange / 2)),
		window(ITMVisualizationWindowManager::Instance().MakeOrGetChartWindow(
				"SceneSliceVisualizer1D_" + AxisToString(axis),
				"Scene 1D Slice Visualizer for " + AxisToString(axis) + " Axis")){}

//TODO: DRY violation -- same code as EnergyPlotter -- group into single class hierarchy with shared methods
void ITMSceneSliceVisualizer1D::SaveScreenshot(std::string path) {
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
ITMSceneSliceVisualizer1D::Plot1DSceneSlice<ITMVoxel, PlainVoxelArray>(
		ITMVoxelVolume<ITMVoxel, PlainVoxelArray>* scene, Vector4i color, double width);

template void
ITMSceneSliceVisualizer1D::Draw1DWarpUpdateVector<ITMVoxel, ITMWarp, PlainVoxelArray>(
		ITMVoxelVolume<ITMVoxel, PlainVoxelArray>* TSDF,
		ITMVoxelVolume<ITMWarp, PlainVoxelArray>* warpField,
		Vector4i color);


template void
ITMSceneSliceVisualizer1D::Plot1DSceneSlice<ITMVoxel, VoxelBlockHash>(
		ITMVoxelVolume<ITMVoxel, VoxelBlockHash>* scene, Vector4i color, double width);

template void
ITMSceneSliceVisualizer1D::Draw1DWarpUpdateVector<ITMVoxel, ITMWarp, VoxelBlockHash>(
		ITMVoxelVolume<ITMVoxel, VoxelBlockHash>* TSDF,
		ITMVoxelVolume<ITMWarp, VoxelBlockHash>* warpField,
		Vector4i color);

//======================================================================================================================