//  ================================================================
//  Created by Gregory Kramida on 2/15/18.
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
#include "HighlightVisualization.h"

#include <vtkLine.h>
#include <vtkNamedColors.h>
#include <vtkUnsignedCharArray.h>
#include <vtkCellData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>


HighlightVisualization::HighlightVisualization():
	neighborsData(vtkSmartPointer<vtkPolyData>::New()),
	highlightPolyData(vtkSmartPointer<vtkPolyData>::New()),
	highlightActor(vtkSmartPointer<vtkActor>::New())
	{
	highlightActor->GetProperty()->SetLineWidth(4);
}

void HighlightVisualization::SetData(const Vector3d& highlightPosition,
                                     const ITMLib::ITMHighlightIterationInfo& highlightInfo,
                                     const std::vector<Vector3d>& neighborPositions, const Vector3d& cameraRight) {

	SetUpHighlightPolyData(highlightPosition, highlightInfo, cameraRight);
	vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputData(this->highlightPolyData);
	highlightActor->SetMapper(mapper);

	vtkSmartPointer<vtkPoints> neighborPoints = vtkSmartPointer<vtkPoints>::New();


}

vtkSmartPointer<vtkActor> HighlightVisualization::GetHighlightActor() {
	return this->highlightActor;
}

void

HighlightVisualization::SetUpHighlightPolyData(const Vector3d& highlightPosition,
                                               const ITMLib::ITMHighlightIterationInfo& highlightInfo,
                                               const Vector3d& cameraRight) {

	//======= define points for warp update lines ===============
	vtkSmartPointer<vtkPoints> highlightPoints = vtkSmartPointer<vtkPoints>::New();
	Vector3f warpUpdateComponentOffset;
	const double offsetLength = 0.02;
	warpUpdateComponentOffset = (cameraRight.normalised() * offsetLength).toFloat();
	Vector3f startPoint = highlightPosition.toFloat();
	Vector3f endPoint;

	//*** total warp ***
	endPoint = startPoint - highlightInfo.warpUpdate;
	highlightPoints->InsertNextPoint(startPoint.values);
	highlightPoints->InsertNextPoint(endPoint.values);
	//*** data term ***
	startPoint += warpUpdateComponentOffset;
	endPoint = startPoint - highlightInfo.warpUpdateData;
	highlightPoints->InsertNextPoint(startPoint.values);
	highlightPoints->InsertNextPoint(endPoint.values);
	//*** level set term ***
	startPoint += warpUpdateComponentOffset;
	endPoint = startPoint - highlightInfo.warpUpdateLevelSet;
	highlightPoints->InsertNextPoint(startPoint.values);
	highlightPoints->InsertNextPoint(endPoint.values);
	//*** Killing term ***
	startPoint += warpUpdateComponentOffset;
	endPoint = startPoint - highlightInfo.warpUpdateKilling;
	highlightPoints->InsertNextPoint(startPoint.values);
	highlightPoints->InsertNextPoint(endPoint.values);
	highlightPolyData->SetPoints(highlightPoints);

	//========= define warp update lines ===========================
	vtkSmartPointer<vtkCellArray> warpUpdateLines = vtkSmartPointer<vtkCellArray>::New();

	vtkSmartPointer<vtkLine> warpUpdateLine = vtkSmartPointer<vtkLine>::New();
	warpUpdateLine->GetPointIds()->SetId(0,0); warpUpdateLine->GetPointIds()->SetId(1,1);
	warpUpdateLines->InsertNextCell(warpUpdateLine);
	vtkSmartPointer<vtkLine> warpUpdateDataLine = vtkSmartPointer<vtkLine>::New();
	warpUpdateDataLine->GetPointIds()->SetId(0,2); warpUpdateDataLine->GetPointIds()->SetId(1,3);
	warpUpdateLines->InsertNextCell(warpUpdateDataLine);
	vtkSmartPointer<vtkLine> warpUpdateLevelSetLine = vtkSmartPointer<vtkLine>::New();
	warpUpdateLevelSetLine->GetPointIds()->SetId(0,4); warpUpdateLevelSetLine->GetPointIds()->SetId(1,5);
	warpUpdateLines->InsertNextCell(warpUpdateLevelSetLine);
	vtkSmartPointer<vtkLine> warpUpdateKillingLine = vtkSmartPointer<vtkLine>::New();
	warpUpdateKillingLine->GetPointIds()->SetId(0,6); warpUpdateKillingLine->GetPointIds()->SetId(1,7);
	warpUpdateLines->InsertNextCell(warpUpdateKillingLine);

	highlightPolyData->SetLines(warpUpdateLines);

	//========== define warp update colors ==========================
	vtkSmartPointer<vtkNamedColors> namedColors =
			vtkSmartPointer<vtkNamedColors>::New();

	// Create a vtkUnsignedCharArray container and store the colors in it
	vtkSmartPointer<vtkUnsignedCharArray> colors =
			vtkSmartPointer<vtkUnsignedCharArray>::New();
	colors->SetNumberOfComponents(3);
	colors->InsertNextTypedTuple(namedColors->GetColor3ub("Red").GetData());
	colors->InsertNextTypedTuple(namedColors->GetColor3ub("Green").GetData());
	colors->InsertNextTypedTuple(namedColors->GetColor3ub("Blue").GetData());
	colors->InsertNextTypedTuple(namedColors->GetColor3ub("Yellow").GetData());
	highlightPolyData->GetCellData()->SetScalars(colors);

}
