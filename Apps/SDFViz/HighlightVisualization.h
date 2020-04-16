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
#pragma once


#include <vtk-8.1/vtkPolyData.h>
#include <vtk-8.1/vtkActor.h>
#include <vtk-8.1/vtkCamera.h>
#include "../../ITMLib/Utils/Math.h"
#include "../../ITMLib/Utils/Analytics/NeighborVoxelIterationInfo.h"

class HighlightVisualization {
public:
	HighlightVisualization();
	void SetData(const Vector3d& highlightPosition, const ITMLib::ITMHighlightIterationInfo& highlightInfo,
	             const std::vector<Vector3d>& neighborPositions, const Vector3d& cameraRight);

	vtkSmartPointer<vtkActor> GetHighlightActor();


private:
	void SetUpHighlightPolyData(const Vector3d& highlightPosition,
	                            const ITMLib::ITMHighlightIterationInfo& highlightInfo,
	                            const Vector3d& cameraRight);
	vtkSmartPointer<vtkPolyData> neighborsData;
	vtkSmartPointer<vtkPolyData> highlightPolyData;
	vtkSmartPointer<vtkActor> highlightActor;


};



