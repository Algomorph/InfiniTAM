//  ================================================================
//  Created by Gregory Kramida on 3/2/18.
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

#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkPointPicker.h>
#include <vtkSmartPointer.h>
#include <vtk-8.1/vtkCellPicker.h>

class SDFViz;


/**
 * \brief A standard VTK trackball interactor style with added functionality for
 * some keyboard keys & selecting points
 */
class SDFVizInteractorStyle : public vtkInteractorStyleTrackballCamera {

public:
	static SDFVizInteractorStyle* New();

	enum Mode{
		VIEW, // default/standard
		VOXEL_SELECT,
		SLICE_SELECT,
	};

	vtkTypeMacro(SDFVizInteractorStyle, vtkInteractorStyleTrackballCamera);
	SDFVizInteractorStyle();

	SDFViz* parent;

	void OnKeyPress() override;
	void OnLeftButtonDown() override;
	void OnLeftButtonUp() override;

private:
	//============================= MEMBER FUNCTIONS ===================================================================
	void TurnOnSliceSelectionMode();
	void TurnOffSliceSelectionMode();

	//============================= MEMBER VARIABLES ===================================================================
	Mode mode = VIEW;
	Mode previousMode = VIEW;
	bool keySymbolPrinting = false;
	bool previouslyWarpWasEnabled = false;
	bool sliceSelected = true;
	vtkSmartPointer<vtkPointPicker> pointPicker;
	vtkIdType selectedPointId = -1;


	void MakeSlice();
	void ClearSliceSelection();
};
