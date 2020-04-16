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
//stdlib
#include <regex>

//VTK
#include <vtkObjectFactory.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRendererCollection.h>

//local
#include "SDFVizInteractorStyle.h"
#include "SDFViz.h"

vtkStandardNewMacro(SDFVizInteractorStyle);


SDFVizInteractorStyle::SDFVizInteractorStyle() :
		mode(VIEW),
		pointPicker(vtkSmartPointer<vtkPointPicker>::New()),
		selectedPointId(-1) {
}

void SDFVizInteractorStyle::OnKeyPress() {

	// Get the keypress
	vtkRenderWindowInteractor* rwi = this->Interactor;
	std::string key = rwi->GetKeySym();
	std::regex digit("\\d");


	if (parent != nullptr) {
		if (key == "c") {
			//record camera position
			double x, y, z;
			double xFocalPoint, yFocalPoint, zFocalPoint;
			double xUpVector, yUpVector, zUpVector;
			double dNear, dFar;
			parent->sdfRenderer->GetActiveCamera()->GetPosition(x, y, z);
			parent->sdfRenderer->GetActiveCamera()->GetFocalPoint(xFocalPoint, yFocalPoint, zFocalPoint);
			parent->sdfRenderer->GetActiveCamera()->GetViewUp(xUpVector, yUpVector, zUpVector);
			parent->sdfRenderer->GetActiveCamera()->GetClippingRange(dNear, dFar);
			std::cout << "Camera:" << std::endl;
			std::cout << "  Current position: " << x << ", " << y << ", " << z << std::endl;
			std::cout << "  Current focal point: " << xFocalPoint << ", " << yFocalPoint << ", " << zFocalPoint
			          << std::endl;
			std::cout << "  Current up-vector: " << xUpVector << ", " << yUpVector << ", " << zUpVector
			          << std::endl;
			std::cout << "  Current clipping range: " << dNear << ", " << dFar << std::endl;
			std::cout.flush();
		} else if (key == "v") {
			//toggle voxel blocks visibility
			if (rwi->GetAltKey()) {
				parent->ToggleCanonicalVoxelVisibility();
			} else {
				parent->ToggleLiveVoxelVisibility();
			}
		} else if (key == "h") {
			//toggle hash blocks visibility
			if (rwi->GetAltKey()) {
				parent->ToggleCanonicalHashBlockVisibility();
			} else {
				parent->ToggleLiveHashBlockVisibility();
			}
		} else if (key == "i") {
			//toggle interest region visibility
			parent->ToggleInterestVoxelVisibility();
		} else if (key == "minus" || key == "KP_Subtract") {
			if (rwi->GetAltKey()) {
				parent->DecreaseCanonicalVoxelOpacity();
			} else {
				parent->DecreaseLiveVoxelOpacity();
			}
		} else if (key == "equal" || key == "KP_Add") {
			if (rwi->GetAltKey()) {
				parent->IncreaseCanonicalVoxelOpacity();
			} else {
				parent->IncreaseLiveVoxelOpacity();
			}
		} else if (key == "bracketright") {
			parent->AdvanceIteration();
		} else if (key == "bracketleft") {
			parent->RetreatIteration();
		} else if (key == "period") {
			parent->AdvanceFrame();
		} else if (key == "comma") {
			parent->RetreatFrame();
		} else if (key == "Prior") {
			if (parent->hasHighlightInfo && !rwi->GetAltKey()) {
				parent->MoveFocusToPreviousHighlight();
			} else {
				parent->sdfRenderer->GetActiveCamera()->SetViewUp(0.0, 1.0, 0.0);
				parent->renderWindow->Render();
			}
		} else if (key == "Next") {
			if (parent->hasHighlightInfo && !rwi->GetAltKey()) {
				parent->MoveFocusToNextHighlight();
			} else {
				parent->sdfRenderer->GetActiveCamera()->SetViewUp(0.0, -1.0, 0.0);
				parent->renderWindow->Render();
			}
		} else if (key == "Escape") {
			if (sliceSelected) {
				ClearSliceSelection();
			} else {
				rwi->TerminateApp();
				std::cout << "Exiting application..." << std::endl;
			}
		} else if (key == "Home") {
			if (parent->hasHighlightInfo) {
				parent->RefocusAtCurrentHighlight();
			} else {
				if(rwi->GetAltKey()){
					parent->sdfRenderer->ResetCamera();
					parent->renderWindow->Render();
				}else{
					parent->MoveFocusToSelectedVoxel();
				}
			}
		} else if (key == "b") {
			if (rwi->GetAltKey()) {
				parent->PreviousBackgroundColor();
			} else {
				parent->NextBackgroundColor();
			}
		} else if (key == "t") {
			if (rwi->GetAltKey()) {
				parent->ToggleCanonicalUnknownVoxelVisibility();
				std::cout << "Canonical voxel unknown visibility: "
				          << (this->parent->canonicalUnknownVoxelsVisible ? "ON" : "OFF") << std::endl;
			} else {
				parent->ToggleLiveUnknownVoxelVisibility();
				std::cout << "Live voxel unknown visibility: "
				          << (this->parent->liveUnknownVoxelsVisible ? "ON" : "OFF") << std::endl;
			}
		} else if (key == "s") {
			if (rwi->GetAltKey()) {
				// slice selection mode
				if (mode == SLICE_SELECT) {
					this->TurnOffSliceSelectionMode();
				} else {
					ClearSliceSelection();
					TurnOnSliceSelectionMode();
				}
			} else if (mode == VIEW || mode == VOXEL_SELECT) {
				// toggle between view & voxel selection mode
				previousMode = mode;
				this->mode = this->mode == VOXEL_SELECT ? VIEW : VOXEL_SELECT;
				std::cout << "Selection mode " << (this->mode == VOXEL_SELECT ? "ON" : "OFF") << std::endl;
			}
		} else if (key == "grave") {
			this->keySymbolPrinting = !this->keySymbolPrinting;
			std::cout << "Key symbol & code printing: " << (keySymbolPrinting ? "ON" : "OFF") << std::endl;
		} else if (key == "KP_Multiply" && mode == VIEW) {
			parent->canonicalScenePipe.ToggleWarpEnabled();
			std::cout << "Warping display: " << (parent->canonicalScenePipe.GetWarpEnabled() ? "ON" : "OFF") << std::endl;
			parent->renderWindow->Render();
		} else if (key == "Return" || key == "KP_Enter") {
			if (sliceSelected) {
				MakeSlice();
			}
		} else if (std::regex_match(key, digit)){
			int numberOfTheKey = std::stoi(key);
			if(numberOfTheKey < 10 && numberOfTheKey > 0){
				parent->ToggleSliceMode(static_cast<unsigned int>(numberOfTheKey - 1));
			}else if(numberOfTheKey == 0){
				parent->SwitchToFullScene();
			}
		}
	}
	if (keySymbolPrinting) {
		std::cout << "Key symbol: " << key << std::endl;
		std::cout << "Key code: " << static_cast<int>(rwi->GetKeyCode()) << std::endl;
	}


	// Forward events
	vtkInteractorStyleTrackballCamera::OnKeyPress();

}

void SDFVizInteractorStyle::OnLeftButtonUp() {
	switch (mode) {
		case VIEW:
			vtkInteractorStyleTrackballCamera::OnLeftButtonUp();
			break;
		case VOXEL_SELECT: {
			int* pos = this->Interactor->GetEventPosition();
			this->pointPicker->Pick(pos[0], pos[1], 0,
			                        this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer());

			vtkIdType newSelectedPointId = this->pointPicker->GetPointId();
			vtkActor* selectedActor = this->pointPicker->GetActor();
			if (selectedActor == parent->canonicalScenePipe.GetVoxelActor() && newSelectedPointId >= 0) {
				selectedPointId = newSelectedPointId;
				parent->canonicalScenePipe.SelectOrDeselectVoxel(selectedPointId, true,
				                                                 parent->sceneLogger->GetActiveWarpScene());
				parent->renderWindow->Render();
			}
		}
			break;
		case SLICE_SELECT: {
			int* pos = this->Interactor->GetEventPosition();
			this->pointPicker->Pick(pos[0], pos[1], 0,
			                        this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer());
			vtkIdType newSelectedPointId = this->pointPicker->GetPointId();
			vtkActor* selectedActor = this->pointPicker->GetActor();
			if (selectedActor == parent->canonicalScenePipe.GetVoxelActor() && newSelectedPointId >= 0) {
				bool continueSliceSelection = true;
				parent->canonicalScenePipe.SetSliceSelection(newSelectedPointId, continueSliceSelection,
				                                             parent->sceneLogger->GetActiveWarpScene());
				if (!continueSliceSelection) {
					TurnOffSliceSelectionMode();
					sliceSelected = true;
					parent->UpdateMessageBar("Enter: make slice (overwrite), Esc: cancel and clear slice selection.");
				} else {
					parent->renderWindow->Render();
				}
			}
		}
			break;
	}
}


void SDFVizInteractorStyle::OnLeftButtonDown() {
	if (mode == VIEW) {
		vtkInteractorStyleTrackballCamera::OnLeftButtonDown();
	}
}

void SDFVizInteractorStyle::TurnOnSliceSelectionMode() {
	previousMode = mode;
	mode = SLICE_SELECT;
	previouslyWarpWasEnabled = parent->canonicalScenePipe.GetWarpEnabled();
	if (previouslyWarpWasEnabled) {
		//disable warp during slice selection
		parent->canonicalScenePipe.ToggleWarpEnabled();
	}
	parent->renderWindow->Render();
	std::cout << "Slice selection mode: ON" << std::endl;
}

void SDFVizInteractorStyle::TurnOffSliceSelectionMode() {
	mode = previousMode; //reset to previous mode
	previousMode = SLICE_SELECT;
	if (previouslyWarpWasEnabled) {
		//reset to using warp again
		parent->canonicalScenePipe.ToggleWarpEnabled();
	}
	parent->renderWindow->Render();
	std::cout << "Slice selection mode: OFF" << std::endl;
}

void SDFVizInteractorStyle::ClearSliceSelection(){
	parent->canonicalScenePipe.ClearSliceSelection();
	parent->ClearMessageBar();//rerenders
	sliceSelected = false;
}

void SDFVizInteractorStyle::MakeSlice() {
	parent->UpdateMessageBar("Making slice...");
	if(parent->MakeSlice()){
		ClearSliceSelection();
		parent->UpdateMessageBar("Slice completed. Switching to slice...");
		parent->SwitchToSlice(static_cast<unsigned int>(parent->sliceIdentifiers.size() - 1));
		parent->UpdateMessageBar("Now in Slice mode.");
	}else{
		ClearSliceSelection();
		std::cerr << "Slicing procedure failed, potentially because a duplicate slice was attempted.";
		parent->UpdateMessageBar("Slice failed.");
	}

}

