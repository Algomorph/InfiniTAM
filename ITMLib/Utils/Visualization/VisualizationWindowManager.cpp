//  ================================================================
//  Created by Gregory Kramida on 6/7/18.
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
#include <memory>

//VTK
#include <vtkContextScene.h>
#include <vtkRenderer.h>
#include <vtkCamera.h>
#include <vtkRenderWindow.h>
#include <vtkRendererCollection.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkNamedColors.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkAxesActor.h>
#include <thread>


//local
#include "VisualizationWindowManager.h"
#include "../Math.h"
#include "../CPPPrintHelpers.h"


using namespace ITMLib;





ITMChartWindow::ITMChartWindow(const std::string& name, const std::string& title, int width, int height) :
		view(vtkSmartPointer<vtkContextView>::New()),
		chart(vtkSmartPointer<vtkChartXY>::New()),
		name(name) {
	renderWindow = view->GetRenderWindow();
	renderWindow->SetWindowName(title.c_str());
	renderWindow->SetSize(width, height);
	view->GetRenderer()->SetBackground(1.0, 1.0, 1.0);
	view->GetScene()->AddItem(chart);
	interactor = view->GetInteractor();
	interactor->Initialize();
}


vtkSmartPointer<vtkChartXY> ITMChartWindow::GetChart() {
	return chart;
}

void ITMChartWindow::Update() {
	renderWindow->Render();
}

ITMChartWindow::~ITMChartWindow() {
	interactor->TerminateApp();
}

vtkSmartPointer<vtkRenderWindow> ITMChartWindow::GetRenderWindow() {
	return this->renderWindow;
}

ITMChartWindow*
ITMVisualizationWindowManager::MakeOrGetChartWindow(const std::string& name, const std::string& title, int width,
                                                    int height) {
	auto it = chartWindows.find(name);
	if (it == chartWindows.end()) {
		ITMChartWindow window(name, title, width, height);
		chartWindows.emplace(name, window);
		it = chartWindows.find(name);
	}
	return it == chartWindows.end() ? nullptr : &(chartWindows.find(name)->second);
}

ITM3DWindow*
ITMVisualizationWindowManager::MakeOrGet3DWindow(const std::string& name, const std::string& title, int width,
                                                 int height) {
	auto it = _3dWindows.find(name);
	if (it == _3dWindows.end()) {
		ITM3DWindow window(name, title, width, height);
		_3dWindows.emplace(name, window);
		it = _3dWindows.find(name);
	}
	return it == _3dWindows.end() ? nullptr : &(_3dWindows.find(name)->second);
}

void ITMVisualizationWindowManager::CloseAndDelete3DWindow(const std::string& name) {
	_3dWindows.erase(name);
}


ITM3DWindow::ITM3DWindow(const std::string& name, const std::string& title, int width, int height) :
		renderWindow(vtkSmartPointer<vtkRenderWindow>::New()),
		interactor(vtkSmartPointer<vtkRenderWindowInteractor>::New()) {
	renderWindow->SetWindowName(title.c_str());
	if (width == -1 || height == -1) {
		renderWindow->SetSize(renderWindow->GetScreenSize());
	} else {
		renderWindow->SetSize(width, height);
	}
	AddLayer(Vector4d(1.0, 1.0, 1.0, 1.0));

	vtkSmartPointer<vtkInteractorStyleTrackballCamera> interactorStyle = vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
	interactorStyle->SetMouseWheelMotionFactor(0.05);
	interactor->SetInteractorStyle(interactorStyle);
	interactor->SetRenderWindow(renderWindow);

	//make axis gizmo
	vtkSmartPointer<vtkNamedColors> colors =
			vtkSmartPointer<vtkNamedColors>::New();

	vtkSmartPointer<vtkAxesActor> axes =
			vtkSmartPointer<vtkAxesActor>::New();

	orientationWidget = vtkSmartPointer<vtkOrientationMarkerWidget>::New();
	double rgba[4]{0.0, 0.0, 0.0, 0.0};
	colors->GetColor("Carrot", rgba);

	orientationWidget->SetOutlineColor(rgba[0], rgba[1], rgba[2]);
	orientationWidget->SetOrientationMarker(axes);
	orientationWidget->SetInteractor(interactor);
	orientationWidget->SetViewport(0.0, 0.0, 0.4, 0.4);
	orientationWidget->SetEnabled(1);
	orientationWidget->InteractiveOn();

	interactor->Initialize();
}

ITM3DWindow::~ITM3DWindow() {
	this->interactor->TerminateApp();
}

void ITM3DWindow::Update() {
	renderWindow->Render();
}


void ITM3DWindow::AddLayer(const Vector4d& backgroundColor) {
	vtkSmartPointer<vtkRenderer> newRenderer = vtkSmartPointer<vtkRenderer>::New();
	newRenderer->SetBackground(backgroundColor.r, backgroundColor.g, backgroundColor.b);
	newRenderer->SetBackgroundAlpha(backgroundColor.a);
	newRenderer->SetLayer(static_cast<int>(layerRenderers.size()));
	if (!layerRenderers.empty()) {
		newRenderer->SetActiveCamera(layerRenderers[0]->GetActiveCamera());
	}
	renderWindow->SetNumberOfLayers(static_cast<int>(layerRenderers.size() + 1));
	renderWindow->AddRenderer(newRenderer);
	layerRenderers.push_back(newRenderer);
}


void ITM3DWindow::AddActorToLayer(vtkSmartPointer<vtkActor> actor, int layer) {
	if (layer < 0 || layer >= renderWindow->GetNumberOfLayers()) {
		DIEWITHEXCEPTION_REPORTLOCATION("Layer " + std::to_string(layer) + " out of bounds.");
	}
	layerRenderers[layer]->AddActor(actor);
}

void ITM3DWindow::AddActor2DToLayer(vtkSmartPointer<vtkActor2D> actor, int layer) {
	if (layer < 0 || layer >= renderWindow->GetNumberOfLayers()) {
		DIEWITHEXCEPTION_REPORTLOCATION("Layer " + std::to_string(layer) + " out of bounds.");
	}
	layerRenderers[layer]->AddActor2D(actor);
}

void ITM3DWindow::AddActorToFirstLayer(vtkSmartPointer<vtkActor> actor) {
	renderWindow->GetRenderers()->GetFirstRenderer()->AddActor(actor);
}

vtkSmartPointer<vtkRenderWindow> ITM3DWindow::GetRenderWindow() {
	return this->renderWindow;
}

void ITM3DWindow::ResetCamera() {
	vtkRenderer* renderer = renderWindow->GetRenderers()->GetFirstRenderer();
	renderer->ResetCamera();
	vtkCamera* camera = renderer->GetActiveCamera();
	camera->SetViewUp(0,-1,0);
	Vector3d cameraPosition, focalPoint;
	camera->GetPosition(cameraPosition.values);
	camera->GetFocalPoint(focalPoint.values);
	Vector3d focalToCamera = cameraPosition-focalPoint;
	Vector3d newCameraPos = focalPoint;
	newCameraPos.z-= focalToCamera.z;
	camera->SetPosition(newCameraPos.values);
	camera->SetClippingRange(1.0,1500.0);
}

void ITM3DWindow::RunInteractor() {
	interactor->Start();
}

void ITM3DWindow::AddLoopCallback(vtkSmartPointer<vtkCommand> callback) {
	interactor->CreateRepeatingTimer(1);
	interactor->AddObserver(vtkCommand::TimerEvent,callback);
}

void ITM3DWindow::SetInteractorStyle(vtkSmartPointer<vtkInteractorStyle> style) {
	interactor->SetInteractorStyle(style);
}

void ITM3DWindow::HideLayer(int layer) {
	layerRenderers[layer]->Clear();
	layerRenderers[layer]->DrawOff();
	renderWindow->Render();
}

void ITM3DWindow::ShowLayer(int layer) {
	layerRenderers[layer]->DrawOn();
	renderWindow->Render();
}

int ITM3DWindow::GetLayerCount() const {
	return static_cast<int>(this->layerRenderers.size());
}

std::string ITM3DWindow::GetName() const {
	return this->name;
}

