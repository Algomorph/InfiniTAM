//  ================================================================
//  Created by Gregory Kramida on 6/12/18.
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

//VTK
#include <vtkWindowToImageFilter.h>
#include <vtkDoubleArray.h>
#include <vtkVariantArray.h>
#include <vtkTable.h>
#include <vtkAxis.h>
#include <vtkPlotStacked.h>
#include <vtkColorSeries.h>
#include <vtkColor.h>
#include <vtkWindow.h>
#include <vtkWindowToImageFilter.h>
#include <vtkRenderWindow.h>
#include <vtkPNGWriter.h>
#include <vtkIntArray.h>
#include <vtkStringArray.h>
#include <vtkDoubleArray.h>


//Local
#include "ITMSceneTrackingEnergyPlotter.h"
#include "../ITMMath.h"

using namespace ITMLib;

ITMSceneTrackingEnergyPlotter::ITMSceneTrackingEnergyPlotter() :
	window(ITMVisualizationWindowManager::Instance().MakeOrGetChartWindow("TrackingEnergyPlotter",
	                                                                      "Surface Tracking Energy"))
	{
	PreparePlot();
}

void ITMSceneTrackingEnergyPlotter::PreparePlot(Vector3i colorDataEnergy,
                                               Vector3i colorSmoothingEnergy, Vector3i colorLevelSetEnergy,
                                               Vector3i colorKillingEnergy) {
	// set up table & columns

	const int _DEBUG_row_count = 10;

	vtkSmartPointer<vtkDoubleArray> dataEnergyValues = vtkSmartPointer<vtkDoubleArray>::New();
	dataEnergyValues->SetName("Data Energy");
	dataEnergyValues->SetNumberOfValues(_DEBUG_row_count);

	vtkSmartPointer<vtkDoubleArray> smoothingEnergyValues = vtkSmartPointer<vtkDoubleArray>::New();
	smoothingEnergyValues->SetName("Smoothing Energy");
	smoothingEnergyValues->SetNumberOfValues(_DEBUG_row_count);

	vtkSmartPointer<vtkDoubleArray> levelSetEnergyValues = vtkSmartPointer<vtkDoubleArray>::New();
	levelSetEnergyValues->SetName("Level Set Energy");
	levelSetEnergyValues->SetNumberOfValues(_DEBUG_row_count);

	vtkSmartPointer<vtkDoubleArray> killingEnergyValues = vtkSmartPointer<vtkDoubleArray>::New();
	killingEnergyValues->SetName("Killing Energy");
	killingEnergyValues->SetNumberOfValues(_DEBUG_row_count);

	table = vtkSmartPointer<vtkTable>::New();
	table->AddColumn(dataEnergyValues);
	table->AddColumn(smoothingEnergyValues);
	table->AddColumn(levelSetEnergyValues);
	table->AddColumn(killingEnergyValues);
	table->SetNumberOfRows(0);

	vtkSmartPointer<vtkChartXY> chart = this->window->GetChart();

	chart->ClearPlots();
	chart->GetAxis(1)->SetTitle("Iteration");
	chart->GetAxis(0)->SetTitle("Energy");

	vtkPlotStacked* stack = vtkPlotStacked::SafeDownCast(chart->AddPlot(vtkChart::STACKED));
	stack->SetInputData(table);
	stack->SetUseIndexForXSeries(true);
	stack->SetInputArray(1, "Data Energy");
	stack->SetInputArray(2, "Smoothing Energy");
	stack->SetInputArray(3, "Level Set Energy");
	stack->SetInputArray(4, "Killing Energy");

	chart->SetShowLegend(true);

	vtkSmartPointer<vtkColorSeries> colorSeries =
			vtkSmartPointer<vtkColorSeries>::New();

	colorSeries->SetColorScheme(vtkColorSeries::BREWER_QUALITATIVE_PASTEL2);
	if(!(colorDataEnergy == Vector3i(0,0,0) &&
	     colorDataEnergy == colorSmoothingEnergy &&
	     colorDataEnergy == colorLevelSetEnergy &&
	     colorDataEnergy == colorKillingEnergy)){
		colorSeries->SetColor(0, vtkColor3ub(static_cast<unsigned char>(colorDataEnergy.r),
		                                     static_cast<unsigned char>(colorDataEnergy.g),
		                                     static_cast<unsigned char>(colorDataEnergy.b)));
		colorSeries->SetColor(1, vtkColor3ub(static_cast<unsigned char>(colorSmoothingEnergy.r),
		                                     static_cast<unsigned char>(colorSmoothingEnergy.g),
		                                     static_cast<unsigned char>(colorSmoothingEnergy.b)));
		colorSeries->SetColor(2, vtkColor3ub(static_cast<unsigned char>(colorLevelSetEnergy.r),
		                                     static_cast<unsigned char>(colorLevelSetEnergy.g),
		                                     static_cast<unsigned char>(colorLevelSetEnergy.b)));
		colorSeries->SetColor(3, vtkColor3ub(static_cast<unsigned char>(colorKillingEnergy.r),
		                                     static_cast<unsigned char>(colorKillingEnergy.g),
		                                     static_cast<unsigned char>(colorKillingEnergy.b)));
	}
	stack->SetColorSeries(colorSeries);
	chart->Update();
	this->window->Update();
}



void ITMSceneTrackingEnergyPlotter::AddDataPoints(double dataEnergy, double smoothingEnergy, double levelSetEnergy,
                                                  double killingEnergy) {

	vtkSmartPointer<vtkVariantArray> row = vtkSmartPointer<vtkVariantArray>::New();
	row->InsertNextValue(dataEnergy);
	row->InsertNextValue(smoothingEnergy);
	row->InsertNextValue(levelSetEnergy);
	row->InsertNextValue(killingEnergy);

	double totalEnergy = dataEnergy + smoothingEnergy + levelSetEnergy +killingEnergy;


	table->InsertNextRow(row);
	table->Modified();
	
	vtkSmartPointer<vtkChartXY> chart = this->window->GetChart();
	chart->GetAxis(1)->SetRange(0, table->GetNumberOfRows());
	if(maxEnergyValue < totalEnergy){
		maxEnergyValue = totalEnergy;
		chart->GetAxis(0)->SetRange(0, maxEnergyValue);
	}

	window->GetChart()->Update();
	window->Update();
	
}

void ITMSceneTrackingEnergyPlotter::SaveScreenshot(std::string path) {
	vtkSmartPointer<vtkWindowToImageFilter> windowToImageFilter = vtkSmartPointer<vtkWindowToImageFilter>::New();
	vtkSmartPointer<vtkRenderWindow> renderWindow = window->GetRenderWindow();
	windowToImageFilter->SetInput(renderWindow);
	windowToImageFilter->SetInputBufferTypeToRGB();
	windowToImageFilter->ReadFrontBufferOff();
	windowToImageFilter->Update();
	vtkSmartPointer<vtkPNGWriter> writer = vtkSmartPointer<vtkPNGWriter>::New();
	writer->SetFileName(path.c_str());
	writer->SetInputConnection(windowToImageFilter->GetOutputPort());
	writer->Write();
}
