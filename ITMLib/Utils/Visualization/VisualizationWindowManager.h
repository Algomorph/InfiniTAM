//  ================================================================
//  Created by Gregory Kramida on 6/7/18.
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

//stdlib
#include <unordered_map>
#include <vector>

#ifdef WITH_VTK
//vtk
#include <vtkRenderWindowInteractor.h>
#include <vtkContextView.h>
#include <vtkChartXY.h>
#include <vtkActor.h>
#include <vtkActor2D.h>
#include <vtkOrientationMarkerWidget.h>
#endif

//local
#include "../Math.h"



namespace ITMLib {

class ChartWindow {
public:
	ChartWindow(const std::string& name, const std::string& title, int width, int height);
	~ChartWindow();
	vtkSmartPointer<vtkChartXY> GetChart();
	vtkSmartPointer<vtkRenderWindow> GetRenderWindow();
	void Update();

private:
	std::string name;
	vtkSmartPointer<vtkRenderWindowInteractor> interactor;
	vtkSmartPointer<vtkContextView> view;
	vtkSmartPointer<vtkRenderWindow> renderWindow;
	vtkSmartPointer<vtkChartXY> chart;

};

class Window3D {
public:
	Window3D(const std::string& name, const std::string& title, int width, int height);
	~Window3D();
	void Update();
	vtkSmartPointer<vtkRenderWindow> GetRenderWindow();
	void ResetCamera();
	void AddLoopCallback(vtkSmartPointer<vtkCommand> callback);
	void AddLayer(const Vector4d& backgroundColor = Vector4d(1.0));

	void AddActorToLayer(vtkSmartPointer<vtkActor> actor, int layer);
	void AddActor2DToLayer(vtkSmartPointer<vtkActor2D> actor, int layer);
	void AddActorToFirstLayer(vtkSmartPointer<vtkActor> actor);

	void HideLayer(int layer);
	void ShowLayer(int layer);
	int GetLayerCount() const;
	void RunInteractor();
	void SetInteractorStyle(vtkSmartPointer<vtkInteractorStyle> style);
	std::string GetName() const;

private:
	std::string name;
	vtkSmartPointer<vtkRenderWindowInteractor> interactor;
	vtkSmartPointer<vtkRenderWindow> renderWindow;
	std::vector<vtkSmartPointer<vtkRenderer>> layerRenderers;
	vtkSmartPointer<vtkOrientationMarkerWidget> orientationWidget;

};

class VisualizationWindowManager {
public:
	static VisualizationWindowManager& Instance() {
		static VisualizationWindowManager instance;
		return instance;
	}

	ChartWindow* MakeOrGetChartWindow(const std::string& name,
	                                     const std::string& title = "VTK Window",
	                                     int width = 1024, int height = 768);

	Window3D* MakeOrGet3DWindow(const std::string& name,
	                               const std::string& title = "VTK Window",
	                               int width = -1, int height = -1);
	void CloseAndDelete3DWindow(const std::string& name);

	VisualizationWindowManager(VisualizationWindowManager const&) = delete;
	void operator=(VisualizationWindowManager const&)  = delete;
private:
	std::unordered_map<std::string, ChartWindow> chartWindows;
	std::unordered_map<std::string, Window3D> _3dWindows;

	VisualizationWindowManager() = default;
	~VisualizationWindowManager() = default;
};
}//namespace ITMLib