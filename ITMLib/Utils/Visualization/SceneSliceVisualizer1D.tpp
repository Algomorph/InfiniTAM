//  ================================================================
//  Created by Gregory Kramida on 6/6/18.
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

//stdlib
#include <utility>

//VTK
#include <vtkContextView.h>
#include <vtkSmartPointer.h>
#include <vtkChartXY.h>
#include <vtkTable.h>
#include <vtkFloatArray.h>
#include <vtkRenderer.h>
#include <vtkContextScene.h>
#include <vtkPlot.h>
#include <vtkRenderWindow.h>
#include <vtkAxis.h>
#include <vtkPen.h>

//local
#include "SceneSliceVisualizer1D.h"
#include "../../Objects/Volume/VoxelVolume.h"
#include "../../Objects/Volume/RepresentationAccess.h"
#include "VisualizationWindowManager.h"
#include "../../Engines/EditAndCopy/CPU/EditAndCopyEngine_CPU.h"

using namespace ITMLib;

template<typename TVoxel>
struct TGetRegularSDFFunctor{
	static inline float GetSdf(TVoxel& voxel){
		return TVoxel::valueToFloat(voxel.sdf);
	}
};

template<typename TVoxel, int TFieldIndex>
struct TGetIndexedSDFFunctor{
	static inline float GetSdf(TVoxel& voxel){
		return TVoxel::valueToFloat(voxel.sdf_values[TFieldIndex]);
	}
};


template<typename TVoxel, typename TIndex, typename TGetSDFFunctor>
void SceneSliceVisualizer1D::Plot1DSceneSliceHelper(VoxelVolume<TVoxel, TIndex>* scene, Vector4i color, double width) {

	// set up table & columns
	vtkSmartPointer<vtkFloatArray> horizontalAxisPoints = vtkSmartPointer<vtkFloatArray>::New();
	horizontalAxisPoints->SetName((AxisToString(this->axis) + " Axis").c_str());
	vtkSmartPointer<vtkFloatArray> sdfValues = vtkSmartPointer<vtkFloatArray>::New();
	sdfValues->SetName("SDF Value");
	vtkSmartPointer<vtkTable> table = vtkSmartPointer<vtkTable>::New();
	table->AddColumn(horizontalAxisPoints);
	table->AddColumn(sdfValues);
	table->SetNumberOfRows(voxelRange);

	// scene access variables
	typename TIndex::IndexCache cache;

	Vector3i currentVoxelPosition = focusCoordinates;
	currentVoxelPosition[axis] = rangeStartVoxelIndex;

	// fill table
	for(int iValue = 0; iValue < voxelRange; iValue++,currentVoxelPosition[axis]++){
		int vmIndex = 0;
		TVoxel voxel = EditAndCopyEngine_CPU<TVoxel, TIndex>::Inst().ReadVoxel(scene, focusCoordinates, cache);
		table->SetValue(iValue, 0, currentVoxelPosition[axis]);
		table->SetValue(iValue, 1, TGetSDFFunctor::GetSdf(voxel));
	}

	vtkSmartPointer<vtkChartXY> chart = this->window->GetChart();
	chart->GetAxis(1)->SetTitle((AxisToString(this->axis) + " Axis").c_str());
	chart->ForceAxesToBoundsOff();
	chart->AutoAxesOff();
	chart->DrawAxesAtOriginOn();

	chart->GetAxis(0)->SetRange(-1.05,1.05);
	chart->GetAxis(0)->SetBehavior(vtkAxis::FIXED);
	chart->GetAxis(0)->SetTitle("SDF Value");


	vtkPlot* line = chart->AddPlot(vtkChart::LINE);
	line->SetInputData(table, 0, 1);
	line->SetColor(color.r, color.g, color.b, color.a);
	line->SetWidth(width);
	chart->Update();

	this->window->Update();
}


template<typename TVoxel, typename TIndex>
void SceneSliceVisualizer1D::Plot1DSceneSlice(VoxelVolume<TVoxel, TIndex>* scene, Vector4i color, double width) {
	Plot1DSceneSliceHelper<TVoxel,TIndex, TGetRegularSDFFunctor<TVoxel>>(scene, color, width);
}


template<typename TVoxel, typename TWarp, typename TIndex>
void SceneSliceVisualizer1D::Draw1DWarpUpdateVector(
		VoxelVolume<TVoxel, TIndex>* TSDF,
		VoxelVolume<TWarp, TIndex>* warpField,
		Vector4i color) {
	// scene access variables

	TWarp warp = EditAndCopyEngine_CPU<TWarp, TIndex>::Inst().ReadVoxel(warpField, focusCoordinates);
	TVoxel voxel = EditAndCopyEngine_CPU<TVoxel, TIndex>::Inst().ReadVoxel(TSDF, focusCoordinates);

	float warp1D = warp.framewise_warp[axis];
	float warpUpdate1D = warp.gradient0[axis];
	float startPoint = static_cast<float>(focusCoordinates[axis]) + warp1D - warpUpdate1D;
	float endPoint = static_cast<float>(focusCoordinates[axis]) + warp1D;
	float sdfValue = TVoxel::valueToFloat(voxel.sdf);

	vtkSmartPointer<vtkFloatArray> horizontalAxisPoints = vtkSmartPointer<vtkFloatArray>::New();
	horizontalAxisPoints->SetName("horVec");
	vtkSmartPointer<vtkFloatArray> verticalAxisPoints = vtkSmartPointer<vtkFloatArray>::New();
	verticalAxisPoints->SetName("verVec");
	vtkSmartPointer<vtkTable> table = vtkSmartPointer<vtkTable>::New();
	table->AddColumn(horizontalAxisPoints);
	table->AddColumn(verticalAxisPoints);
	table->SetNumberOfRows(2);
	table->SetValue(0,0,startPoint);
	table->SetValue(1,0,endPoint);
	table->SetValue(0,1,sdfValue);
	table->SetValue(1,1,sdfValue);


	vtkSmartPointer<vtkChartXY> chart = window->GetChart();
	vtkPlot* line = chart->AddPlot(vtkChart::LINE);
	line->SetInputData(table, 0, 1);
	line->SetColor(color.r, color.g, color.b, color.a);
	line->SetWidth(2.0);
	chart->Update();

	window->Update();
}
