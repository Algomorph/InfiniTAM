//  ================================================================
//  Created by Gregory Kramida on 1/3/18.
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
#include "SDFViz.h"

//stdlib
#include <utility>

//vtk
#include <vtkCylinderSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkProperty.h>
#include <vtkCamera.h>
#include <vtkPointData.h>
#include <vtkSphereSource.h>
#include <vtkBridgeDataSet.h>
#include <vtkGlyph3D.h>
#include <vtkFloatArray.h>
#include <vtkGlyph3DMapper.h>
#include <vtkLookupTable.h>
#include <vtkLegendBoxActor.h>
#include <vtkNamedColors.h>
#include <vtkClipPolyData.h>
#include <vtkCubeSource.h>
#include <vtkBox.h>
#include <vtkExtractPolyDataGeometry.h>
#include <vtkImageData.h>
#include <vtkSetGet.h>
#include <vtkDataSetMapper.h>
#include <vtkStructuredGrid.h>
#include <vtkUnstructuredGrid.h>
#include <vtkUniformGrid.h>
#include <vtkTextActor.h>
#include <vtkTextProperty.h>
#include <vtkVertexGlyphFilter.h>
#include <vtk-8.1/vtkAxesActor.h>
#include <vtkTransform.h>
#include <vtk-8.1/vtkOrientationMarkerWidget.h>

//local
#include "../../ITMLib/Utils/Visualization/SceneSliceVisualizer3D.h"
#include "SDFVizInteractorStyle.h"

//ITMLib
#include "../../ITMLib/Utils/FileIO/SceneLogger.h"
#include "../../ITMLib/Utils/Configuration.h"
#include "../../ITMLib/Utils/Analytics/SceneStatisticsCalculator/CPU/SceneStatisticsCalculator_CPU.h"
#include "SDFVizGlobalDefines.h"
#include "../../ITMLib/Utils/Visualization/SceneSliceVisualizer3DCommon.h"
//TODO: organize regions -Greg (GitHub: Algomorph)

using namespace ITMLib::Viz;

SDFViz::SDFViz(std::string pathToScene, bool hideNonInterestCanonicalVoxels, bool hideLiveVoxels,
               bool hideInterestCanonicalRegions, bool hideUnknownCanonicalVoxels, bool useInitialCoords,
               Vector3i initialCoords, unsigned int initialFrame, bool loadSlices, bool slicesOnly)
		:
		rootPath(std::move(pathToScene)),
		canonicalScenePipe(canonicalPositiveTruncatedVoxelColor, canonicalPositiveNonTruncatedVoxelColor,
		                   canonicalNegativeNonTruncatedVoxelColor, canonicalNegativeTruncatedVoxelColor,
		                   canonicalUnknownVoxelColor, canonicalPositiveInterestVoxelColor,
		                   canonicalNegativeInterestVoxelColor, highlightVoxelColor, canonicalHashBlockEdgeColor, 0),
		liveScenePipe(livePositiveTruncatedVoxelColor,
		              livePositiveNonTruncatedVoxelColor,
		              liveNegativeNonTruncatedVoxelColor,
		              liveNegativeTruncatedVoxelColor,
		              liveUnknownVoxelColor,
		              highlightVoxelColor,
		              liveHashBlockEdgeColor, true),
		highlightVisualizer(),
		sdfRenderer(vtkSmartPointer<vtkRenderer>::New()),
		markerRenderer(vtkSmartPointer<vtkRenderer>::New()),
		guiOverlayRenderer(vtkSmartPointer<vtkRenderer>::New()),
		renderWindowInteractor(vtkSmartPointer<vtkRenderWindowInteractor>::New()),
		sphere(vtkSmartPointer<vtkSphereSource>::New()),
		cube(vtkSmartPointer<vtkCubeSource>::New()),
		iterationIndicator(vtkSmartPointer<vtkTextActor>::New()),
		frameIndicator(vtkSmartPointer<vtkTextActor>::New()),
		frameIndex(initialFrame),
		iterationIndex(0),
		legend(vtkSmartPointer<vtkLegendBoxActor>::New()),
		messageBar(vtkSmartPointer<vtkLegendBoxActor>::New()),
		canonicalVoxelsVisible(!hideNonInterestCanonicalVoxels),
		canonicalInterestVoxelsVisible(!hideInterestCanonicalRegions),
		canonicalUnknownVoxelsVisible(!hideUnknownCanonicalVoxels),
		canonicalHashBlocksVisible(false),
		liveVoxelsVisible(!hideLiveVoxels),
		liveUnknownVoxelsVisible(false),
		liveHashBlocksVisible(false) {
	
	auto* settings = new ITMLibSettings();
	liveScene = new ITMScene<ITMVoxelLive, ITMVoxelIndex>(
			&settings->sceneParams, settings->swappingMode == ITMLibSettings::SWAPPINGMODE_ENABLED,
			settings->device_type);
	if (slicesOnly) {
		canonicalScene = nullptr;
		sceneLogger = new ITMSceneLogger<ITMVoxelCanonical, ITMVoxelLive, ITMVoxelIndex>(liveScene,
		                                                                                 GenerateExpectedFramePath());
		sliceIdentifiers = sceneLogger->GetSliceIds();
	} else {
		canonicalScene = new ITMScene<ITMVoxelCanonical, ITMVoxelIndex>(
				&settings->sceneParams, settings->swappingMode == ITMLibSettings::SWAPPINGMODE_ENABLED,
				settings->device_type);
		sceneLogger = new ITMSceneLogger<ITMVoxelCanonical, ITMVoxelLive, ITMVoxelIndex>(canonicalScene, liveScene,
		                                                                                 GenerateExpectedFramePath());
	}
	delete settings;

	InitializeRendering();

	DrawLegend();
	DrawMessageBar();
	DrawIterationCounter();
	DrawFrameCounter();
	UpdateFrameDisplay();


	//read scenes from disk
	SetUpGeometrySources();
	LoadFrameData();
	ReinitializePipelines();
	UpdateLiveSceneBounds();
	AddActors();

	// to prevent VTK from doing excessive near-clipping with multi-layered renderers
	DrawDummyMarkers();

	// set up visibilities
	UpdatePipelineVisibilitiesUsingLocalState();
	// load initial warps
	InitializeWarps();

	// set up initial camera position & orientation

	if (useInitialCoords) {
		MoveFocusToVoxelAt(initialCoords.toDouble());
	} else {
		if (hasHighlightInfo) {
			currentHighlight = highlights.GetFirstArray();
			RefocusAtCurrentHighlight();
			RefocusAtCurrentHighlight();//double call as work-around for bug
		} else {
			sdfRenderer->ResetCamera();
			sdfRenderer->GetActiveCamera()->Zoom(1.6);
		}
	}

	if (loadSlices) {
		LoadAllSlices();
	}

}

int SDFViz::Run() {
	renderWindow->Render();
	renderWindowInteractor->Start();
	return EXIT_SUCCESS;
}


SDFViz::~SDFViz() {
	delete sceneLogger;
	delete canonicalScene;
	delete liveScene;
}


void SDFViz::InitializeRendering() {
	double backgroundColor[4];
	memcpy(backgroundColor, backgroundColors[currentBackgrounColorIx].data(), 4 * sizeof(double));
	sdfRenderer->SetBackground(backgroundColor);

	renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
	renderWindow->SetSize(renderWindow->GetScreenSize());

	renderWindow->SetWindowName("SDF Viz (pre-alpha)");//TODO insert git hash here --Greg (GitHub:Algomorph)
	renderWindow->SetNumberOfLayers(3);
	renderWindow->AddRenderer(sdfRenderer);
	sdfRenderer->SetLayer(0);
	sdfRenderer->InteractiveOn();
	renderWindow->AddRenderer(markerRenderer);
	markerRenderer->SetLayer(1);
	markerRenderer->InteractiveOn();
	markerRenderer->SetActiveCamera(sdfRenderer->GetActiveCamera());
	renderWindow->AddRenderer(guiOverlayRenderer);
	guiOverlayRenderer->SetLayer(2);
	guiOverlayRenderer->InteractiveOff();

	vtkSmartPointer<SDFVizInteractorStyle> interactorStyle = vtkSmartPointer<SDFVizInteractorStyle>::New();
	interactorStyle->parent = this;
	interactorStyle->SetMouseWheelMotionFactor(0.05);
	renderWindowInteractor->SetInteractorStyle(interactorStyle);
	renderWindowInteractor->SetRenderWindow(renderWindow);

	InitializeAxes();
}

bool SDFViz::NextNonInterestWarps() {
	if (!sceneLogger->BufferCurrentWarpState(this->allWarpBuffer->GetVoidPointer(0))) {
		return false;
	}
	canonicalScenePipe.UpdatePointPositionsFromBuffer(allWarpBuffer->GetVoidPointer(0));
	return true;
}

bool SDFViz::NonInterestWarpsAt(unsigned int iteration) {
	if (!sceneLogger->BufferWarpStateAt(this->allWarpBuffer->GetVoidPointer(0), iteration)) {
		return false;
	}
	canonicalScenePipe.UpdatePointPositionsFromBuffer(allWarpBuffer->GetVoidPointer(0));
	return true;
}

bool SDFViz::PreviousNonInterestWarps() {
	if (!sceneLogger->BufferPreviousWarpState(this->allWarpBuffer->GetVoidPointer(0))) {
		return false;
	}
	canonicalScenePipe.UpdatePointPositionsFromBuffer(allWarpBuffer->GetVoidPointer(0));
	return true;
}


void SDFViz::InterestWarpBufferHelper() {
	canonicalScenePipe.UpdateInterestRegionsFromBuffers(this->interestWarpBuffer->GetVoidPointer(0));
	//update highlight viz
	const ITMHighlightIterationInfo info = (*currentHighlight)[iterationIndex];
	Vector3d cameraRight = SDFViz::ComputeCameraRightVector(sdfRenderer->GetActiveCamera());
	Vector3d position = canonicalScenePipe.GetHighlightPosition(info.hash, info.localId);
	std::vector<Vector3d> neighborPositions = canonicalScenePipe.GetHighlightNeighborPositions(info.hash, info.localId);
	highlightVisualizer.SetData(position, info, neighborPositions, cameraRight);
}

bool SDFViz::NextInterestWarps() {
	bool success = sceneLogger->BufferCurrentInterestWarpState(this->interestWarpBuffer->GetVoidPointer(0));
	if (success) { InterestWarpBufferHelper(); }
	return success;
}

bool SDFViz::PreviousInterestWarps() {
	bool success = sceneLogger->BufferPreviousInterestWarpState(this->interestWarpBuffer->GetVoidPointer(0));
	if (success) { InterestWarpBufferHelper(); }
	return success;
}


bool SDFViz::InterestWarpsAt(unsigned int iteration) {
	bool success = sceneLogger->BufferInterestWarpStateAtIteration(this->interestWarpBuffer->GetVoidPointer(0),
	                                                               iteration);
	if (success) { InterestWarpBufferHelper(); }
	return success;
}

void SDFViz::DrawLegend() {
	legend->SetNumberOfEntries(7);
	vtkSmartPointer<vtkSphereSource> legendSphere = vtkSmartPointer<vtkSphereSource>::New();
	legendSphere->Update();

	//set up legend entries
	legend->SetEntry(0, legendSphere->GetOutput(), "Positive Interest",
	                 (double*) canonicalPositiveInterestVoxelColor.data());
	legend->SetEntry(1, legendSphere->GetOutput(), "Negative Interest",
	                 (double*) canonicalNegativeInterestVoxelColor.data());
	legend->SetEntry(2, legendSphere->GetOutput(), "Highlight",
	                 (double*) highlightVoxelColor.data());
	legend->SetEntry(3, legendSphere->GetOutput(), "Positive Canonical",
	                 (double*) canonicalPositiveNonTruncatedVoxelColor.data());
	legend->SetEntry(4, legendSphere->GetOutput(), "Negative Canonical",
	                 (double*) canonicalNegativeNonTruncatedVoxelColor.data());
	legend->SetEntry(5, legendSphere->GetOutput(), "Positive Live",
	                 (double*) livePositiveNonTruncatedVoxelColor.data());
	legend->SetEntry(6, legendSphere->GetOutput(), "Negative Live",
	                 (double*) liveNegativeNonTruncatedVoxelColor.data());

	legend->GetPositionCoordinate()->SetCoordinateSystemToView();
	legend->GetPositionCoordinate()->SetValue(0.8, -1.0);
	legend->GetPosition2Coordinate()->SetCoordinateSystemToView();
	legend->GetPosition2Coordinate()->SetValue(1.0, -0.5);

	//set up legend background
	legend->UseBackgroundOn();
	double backgroundColor[4];
	memcpy(backgroundColor, backgroundColors[1].data(), 4 * sizeof(double));//use only black for legend background color
	legend->SetBackgroundColor(backgroundColor);

	guiOverlayRenderer->AddActor(legend);
}

void SDFViz::DrawMessageBar() {
	messageBar->SetNumberOfEntries(1);
	messageBar->GetPositionCoordinate()->SetCoordinateSystemToView();
	messageBar->GetPositionCoordinate()->SetValue(-1.0, -1.0);
	messageBar->GetPosition2Coordinate()->SetCoordinateSystemToView();
	messageBar->GetPosition2Coordinate()->SetValue(8.65, -0.96);
	vtkSmartPointer<vtkNamedColors> colors = vtkSmartPointer<vtkNamedColors>::New();
	double color[4];
	vtkSmartPointer<vtkCubeSource> consoleBullet = vtkSmartPointer<vtkCubeSource>::New();
	consoleBullet->SetBounds(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	consoleBullet->Update();
	colors->GetColor("white", color);
	messageBar->SetEntry(0, consoleBullet->GetOutput(), "", color);
	messageBar->GetEntryTextProperty()->SetFontSize(14);

	//set up console background
	messageBar->UseBackgroundOn();
	double backgroundColor[4];
	memcpy(backgroundColor, backgroundColors[1].data(),
	       4 * sizeof(double));//use only black for console background color
	messageBar->SetBackgroundColor(backgroundColor);
	guiOverlayRenderer->AddActor(messageBar);
}

void SDFViz::DrawIterationCounter() {
	iterationIndicator->SetInput("Iteration: 0");

	iterationIndicator->GetPositionCoordinate()->SetCoordinateSystemToView();
	iterationIndicator->GetPositionCoordinate()->SetValue(-0.95, 0.9);
	iterationIndicator->GetPosition2Coordinate()->SetCoordinateSystemToView();
	iterationIndicator->GetPosition2Coordinate()->SetValue(-0.9, 1.0);
	iterationIndicator->GetTextProperty()->SetFontSize(24);
	iterationIndicator->GetTextProperty()->SetColor(0.1, 0.8, 0.5);

	sdfRenderer->AddActor2D(iterationIndicator);
}

void SDFViz::DrawFrameCounter() {
	frameIndicator->SetInput("Frame: 0");

	frameIndicator->GetPositionCoordinate()->SetCoordinateSystemToView();
	frameIndicator->GetPositionCoordinate()->SetValue(0.85, 0.9);
	frameIndicator->GetPosition2Coordinate()->SetCoordinateSystemToView();
	frameIndicator->GetPosition2Coordinate()->SetValue(0.95, 1.0);
	frameIndicator->GetTextProperty()->SetFontSize(24);
	frameIndicator->GetTextProperty()->SetColor(0.1, 0.8, 0.5);

	sdfRenderer->AddActor2D(frameIndicator);
}

void SDFViz::UpdateIterationDisplay() {
	iterationIndicator->SetInput(("Iteration: " + std::to_string(iterationIndex)).c_str());
	renderWindow->Render();
}

void SDFViz::UpdateFrameDisplay() {
	frameIndicator->SetInput(("Frame: " + std::to_string(frameIndex)).c_str());
	renderWindow->Render();
}

void SDFViz::UpdateMessageBar(std::string text) {
	messageBar->SetEntryString(0, text.c_str());
	renderWindow->Render();
}

void SDFViz::ClearMessageBar() {
	messageBar->SetEntryString(0, "");
	renderWindow->Render();
}

void SDFViz::ToggleCanonicalHashBlockVisibility() {
	canonicalHashBlocksVisible = !canonicalHashBlocksVisible;
	canonicalScenePipe.GetHashBlockActor()->SetVisibility(canonicalHashBlocksVisible);
	renderWindow->Render();
}

void SDFViz::ToggleLiveHashBlockVisibility() {
	liveHashBlocksVisible = !liveHashBlocksVisible;
	liveScenePipe.GetHashBlockActor()->SetVisibility(liveHashBlocksVisible);
	renderWindow->Render();
}

void SDFViz::ToggleCanonicalVoxelVisibility() {
	canonicalVoxelsVisible = !canonicalVoxelsVisible;
	if (canonicalVoxelsVisible && hasWarpIterationInfo &&
	    (sceneLogger->GetGeneralIterationCursor() != iterationIndex)) {
		NonInterestWarpsAt(iterationIndex); //when showing, advance cursor as necessary
	}
	canonicalScenePipe.GetVoxelActor()->SetVisibility(canonicalVoxelsVisible);
	renderWindow->Render();
}

void SDFViz::ToggleCanonicalUnknownVoxelVisibility() {
	canonicalUnknownVoxelsVisible = !canonicalUnknownVoxelsVisible;
	canonicalScenePipe.ToggleScaleMode();
	renderWindow->Render();
}


void SDFViz::ToggleInterestVoxelVisibility() {
	canonicalInterestVoxelsVisible = !canonicalInterestVoxelsVisible;
	if (canonicalInterestVoxelsVisible && hasHighlightInfo &&
	    (sceneLogger->GetInterestIterationCursor() != iterationIndex)) {
		InterestWarpsAt(iterationIndex); //when showing, advance cursor as necessary
	}
	canonicalScenePipe.GetInterestVoxelActor()->SetVisibility(canonicalInterestVoxelsVisible);
	renderWindow->Render();
}

void SDFViz::ToggleLiveVoxelVisibility() {
	liveVoxelsVisible = !liveVoxelsVisible;
	liveScenePipe.GetVoxelActor()->SetVisibility(liveVoxelsVisible);
	renderWindow->Render();
}


void SDFViz::ToggleLiveUnknownVoxelVisibility() {
	liveUnknownVoxelsVisible = !liveUnknownVoxelsVisible;
	liveScenePipe.ToggleScaleMode();
	renderWindow->Render();
}


/**
 * \brief if the scenes are loaded, this will allocate the buffers for voxel warps. If the interest regions are set up,
 * it allocates the appropriate buffers for interest regions as well.
 * scene voxel size should be known before this is called,
 */
void SDFViz::InitializeWarpBuffers() {
	if (!sceneLogger->GetScenesLoaded()) {
		DIEWITHEXCEPTION("Scenes not yet loaded, cannot initialize WarpBuffers");
	}
	this->allWarpBuffer = vtkSmartPointer<vtkFloatArray>::New();
	allWarpBuffer->SetNumberOfComponents(3);
	allWarpBuffer->SetNumberOfTuples(sceneLogger->GetVoxelCount() * 2);
	if (sceneLogger->GetInterestRegionsSetUp()) {
		this->interestWarpBuffer = vtkSmartPointer<vtkFloatArray>::New();
		interestWarpBuffer->SetNumberOfComponents(3);
		interestWarpBuffer->SetNumberOfTuples(sceneLogger->GetTotalInterestVoxelCount() * 2);
	}
}

void SDFViz::DecreaseCanonicalVoxelOpacity() {
	canonicalScenePipe.GetVoxelActor()->GetProperty()->SetOpacity(
			std::max(0.0, canonicalScenePipe.GetVoxelActor()->GetProperty()->GetOpacity() - 0.05));
	renderWindow->Render();
}

void SDFViz::IncreaseCanonicalVoxelOpacity() {
	canonicalScenePipe.GetVoxelActor()->GetProperty()->SetOpacity(
			std::max(0.0, canonicalScenePipe.GetVoxelActor()->GetProperty()->GetOpacity() + 0.05));
	renderWindow->Render();
}


void SDFViz::DecreaseLiveVoxelOpacity() {
	liveScenePipe.GetVoxelActor()->GetProperty()->SetOpacity(
			std::max(0.0, liveScenePipe.GetVoxelActor()->GetProperty()->GetOpacity() - 0.05));
	renderWindow->Render();
}

void SDFViz::IncreaseLiveVoxelOpacity() {
	liveScenePipe.GetVoxelActor()->GetProperty()->SetOpacity(
			std::max(0.0, liveScenePipe.GetVoxelActor()->GetProperty()->GetOpacity() + 0.05));
	renderWindow->Render();
}

void SDFViz::MoveFocusToHighlightAt(int hash, int localId) {
	Vector3d position = canonicalScenePipe.GetHighlightPosition(hash, localId);
	std::vector<Vector3d> neighborPositions = canonicalScenePipe.GetHighlightNeighborPositions(hash, localId);
	const ITMHighlightIterationInfo info = (*highlights.GetArrayAt(hash, localId, frameIndex))[iterationIndex];

	std::cout << "Now viewing highlight at hash " << hash << ", voxel "
	          << localId << " in the canonical frame." << std::endl;

	//TODO: bug -- theoretically, needs to be done after camera repositioning, but that doesn't work for some obscure reason -Greg
	Vector3d cameraRight = SDFViz::ComputeCameraRightVector(sdfRenderer->GetActiveCamera());
	highlightVisualizer.SetData(position, info, neighborPositions, cameraRight);
	MoveFocusToVoxelAt(position);
	renderWindow->Render();
}

void SDFViz::MoveFocusToVoxelAt(Vector3d absoluteCoordinates) {
	sdfRenderer->GetActiveCamera()->SetFocalPoint(absoluteCoordinates.values);
	sdfRenderer->GetActiveCamera()->SetViewUp(0.0, -1.0, 0.0);
	double factor = 3.0;
	absoluteCoordinates.x += factor * 0.9;
	absoluteCoordinates.y += factor * 0.5;
	absoluteCoordinates.z += factor * 1.0;

	sdfRenderer->GetActiveCamera()->SetPosition(absoluteCoordinates.values);
}

void SDFViz::RefocusAtCurrentHighlight() {
	const ITMHighlightIterationInfo& highlightNew = (*currentHighlight)[iterationIndex];
	MoveFocusToHighlightAt(highlightNew.hash, highlightNew.localId);
}

void SDFViz::MoveFocusToNextHighlight() {
	const ITMHighlightIterationInfo& highlightOld = (*currentHighlight)[iterationIndex];
	currentHighlight = highlights.GetArrayAfter(highlightOld.hash, highlightOld.localId, 0);
	const ITMHighlightIterationInfo& highlightNew = (*currentHighlight)[iterationIndex];
	MoveFocusToHighlightAt(highlightNew.hash, highlightNew.localId);
}

void SDFViz::MoveFocusToPreviousHighlight() {
	const ITMHighlightIterationInfo& highlightOld = (*currentHighlight)[iterationIndex];
	currentHighlight = highlights.GetArrayBefore(highlightOld.hash, highlightOld.localId, 0);
	const ITMHighlightIterationInfo& highlightNew = (*currentHighlight)[iterationIndex];
	MoveFocusToHighlightAt(highlightNew.hash, highlightNew.localId);
}

void SDFViz::DrawDummyMarkers() {
	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
	points->InsertNextPoint(-500.0, -500.0, -500.0);
	points->InsertNextPoint(500.0, 500.0, 500.0);
	vtkSmartPointer<vtkPolyData> dummyPolyData = vtkSmartPointer<vtkPolyData>::New();
	dummyPolyData->SetPoints(points);
	vtkSmartPointer<vtkVertexGlyphFilter> vertexFilter =
			vtkSmartPointer<vtkVertexGlyphFilter>::New();
	vertexFilter->SetInputData(dummyPolyData);
	vertexFilter->Update();
	vtkSmartPointer<vtkPolyData> polydata =
			vtkSmartPointer<vtkPolyData>::New();
	polydata->ShallowCopy(vertexFilter->GetOutput());
	// Visualization
	vtkSmartPointer<vtkPolyDataMapper> mapper =
			vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputData(polydata);
	vtkSmartPointer<vtkActor> actor =
			vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(mapper);
	actor->GetProperty()->SetPointSize(1);
	actor->GetProperty()->SetColor(0.0, 0.0, 0.0);
	markerRenderer->AddActor(actor);
}

Vector3d SDFViz::ComputeCameraRightVector(vtkCamera* camera) {
	Vector3d viewPlaneNormal;
	Vector3d viewUp;
	Vector3d viewRight;
	camera->GetViewPlaneNormal(viewPlaneNormal.values);
	camera->GetViewUp(viewUp.values);
	vtkMath::Cross(viewUp.values, viewPlaneNormal.values, viewRight.values);
	return viewRight;
}

void SDFViz::NextBackgroundColor() {
	currentBackgrounColorIx = (currentBackgrounColorIx + 1) % static_cast<int>(backgroundColors.size());
	double backgroundColor[4];
	memcpy(backgroundColor, backgroundColors[currentBackgrounColorIx].data(), 4 * sizeof(double));
	sdfRenderer->SetBackground(backgroundColor);
	renderWindow->Render();
}

void SDFViz::PreviousBackgroundColor() {
	int reversedIndex = static_cast<int>(backgroundColors.size()) - 1 - currentBackgrounColorIx;
	reversedIndex = (reversedIndex + 1) % static_cast<int>(backgroundColors.size());
	currentBackgrounColorIx = static_cast<int>(backgroundColors.size()) - 1 - reversedIndex;
	double backgroundColor[4];
	memcpy(backgroundColor, backgroundColors[currentBackgrounColorIx].data(), 4 * sizeof(double));
	sdfRenderer->SetBackground(backgroundColor);
	renderWindow->Render();
}

std::string SDFViz::GenerateExpectedFramePath() {
	return this->rootPath + "/Frame_" + std::to_string(this->frameIndex);
}

/**
 * \brief Go to / load / view next frame data
 * \return true on success, false on failure (if we're at last frame or there is any problem with the loading)
 */
bool SDFViz::AdvanceFrame() {
	ITMSceneLogger<ITMVoxelCanonical, ITMVoxelLive, ITMVoxelIndex>* nextLogger;
	frameIndex++;
	try {
		this->sceneLogger->SetPath(GenerateExpectedFramePath());
		LoadFrameData();
		ReinitializePipelines();
		UpdatePipelineVisibilitiesUsingLocalState();
		InitializeWarps();
		iterationIndex = 0;
		UpdateIterationDisplay();
		UpdateFrameDisplay();
		renderWindow->Render();
		return true;
	} catch (std::exception& e) {
		frameIndex--;
		std::cerr << "Could not advance to next frame." << std::endl;
		return false;
	}
}

/**
 * \brief Go to / load / view previous frame data
 * \return true on success, false on failure (if we're at frame 0)
 */
bool SDFViz::RetreatFrame() {
	if (frameIndex > 0) {
		ITMSceneLogger<ITMVoxelCanonical, ITMVoxelLive, ITMVoxelIndex>* nextLogger;
		frameIndex--;
		this->sceneLogger->SetPath(GenerateExpectedFramePath());
		LoadFrameData();
		ReinitializePipelines();
		UpdatePipelineVisibilitiesUsingLocalState();
		InitializeWarps();
		iterationIndex = 0;
		UpdateIterationDisplay();
		UpdateFrameDisplay();
		renderWindow->Render();
		return true;
	} else {
		std::cerr << "Could not retreat to previous frame." << std::endl;
		return false;
	}
}

/**
 * \brief Loads the data for the current frame into the scenes, highlights (if any), and begins the warp loading
 */
void SDFViz::LoadFrameData() {
	// set up viz pipelines
	sceneLogger->LoadScenesCompact();
	hasHighlightInfo = sceneLogger->LoadHighlights(false, "continuous");
	if (hasHighlightInfo) {
		sceneLogger->SetUpInterestRegionsForLoading();
	}
	hasWarpIterationInfo = sceneLogger->StartLoadingWarpState(frameIndex);
	if (hasWarpIterationInfo) {
		canonicalScenePipe.SetFrameIndex(frameIndex);
		InitializeWarpBuffers();
	}
	highlights = sceneLogger->GetHighlights();
}

void SDFViz::ReinitializePipelines() {
	canonicalScenePipe.SetInterestRegionInfo(sceneLogger->GetInterestRegionHashes(), highlights);
	canonicalScenePipe.PreparePipeline();
	canonicalScenePipe.PrepareInterestRegions(sphere->GetOutputPort());
	canonicalScenePipe.PrepareWarplessVoxels(sphere->GetOutputPort());
	liveScenePipe.PreparePipeline();
}

bool SDFViz::AdvanceIteration() {
	if (!canonicalScenePipe.GetWarpEnabled()) {
		std::cout << "Caution: advancing warp while warp result display is disabled." << std::endl;
	}
	bool success = true;
	iterationIndex++;
	if (hasHighlightInfo && canonicalScenePipe.GetInterestVoxelActor()->GetVisibility()) {
		success = NextInterestWarps();
		if (success) {
			std::cout << "Loaded next iteration interest warps." << std::endl;
		} else {
			std::cout << "Cound not load next iteration interest warps." << std::endl;
		}
	}
	if (hasWarpIterationInfo && canonicalScenePipe.GetVoxelActor()->GetVisibility()) {
		std::cout << "Loading next iteration warp & updates." << std::endl;
		success = NextNonInterestWarps();
		if (success) {
			std::cout << "Next iteration warps loaded." << std::endl;
		} else {
			std::cout << "Could not load next iteration warp." << std::endl;
		}
	}
	if (success) {
		UpdateIterationDisplay();
		renderWindow->Render();
	} else {
		iterationIndex--;
	}
	return success;
}

bool SDFViz::RetreatIteration() {
	if (!canonicalScenePipe.GetWarpEnabled()) {
		std::cout << "Caution: retreating warp while warp result display is disabled." << std::endl;
	}
	if (iterationIndex == 0) {
		return false;
	}
	bool success = true;
	iterationIndex--;
	if (hasHighlightInfo && canonicalScenePipe.GetInterestVoxelActor()->GetVisibility()) {
		success = PreviousInterestWarps();
		if (success) {
			std::cout << "Loaded previous iteration interest warps." << std::endl;
		} else {
			std::cout << "Could not load previous iteration interest warps." << std::endl;
		}
	}
	if (hasWarpIterationInfo && canonicalScenePipe.GetVoxelActor()->GetVisibility()) {
		std::cout << "Loading previous iteration warp & updates." << std::endl;
		success = PreviousNonInterestWarps();
		if (success) {
			std::cout << "Previous iteration warps loaded." << std::endl;
		} else {
			std::cout << "Could not load previous iteration warp & updates." << std::endl;
		}
	}
	if (success) {
		UpdateIterationDisplay();
		renderWindow->Render();
	} else {
		iterationIndex++;
	}
	return success;
}

void SDFViz::UpdatePipelineVisibilitiesUsingLocalState() {
	canonicalScenePipe.GetVoxelActor()->SetVisibility(canonicalVoxelsVisible);
	if (canonicalScenePipe.GetWarpEnabled()) canonicalScenePipe.GetWarplessVoxelActor()->VisibilityOff();
	canonicalScenePipe.GetInterestVoxelActor()->SetVisibility(canonicalInterestVoxelsVisible);
	if (canonicalScenePipe.GetCurrentScaleMode() == VoxelScaleMode::VOXEL_SCALE_HIDE_UNKNOWNS &&
	    canonicalUnknownVoxelsVisible ||
	    canonicalScenePipe.GetCurrentScaleMode() == VoxelScaleMode::VOXEL_SCALE_SHOW_UNKNOWNS &&
	    !canonicalUnknownVoxelsVisible) {
		canonicalScenePipe.ToggleScaleMode();
	}
	canonicalScenePipe.GetHashBlockActor()->SetVisibility(canonicalHashBlocksVisible);
	liveScenePipe.GetVoxelActor()->SetVisibility(liveVoxelsVisible);
	if (liveScenePipe.GetCurrentScaleMode() == VoxelScaleMode::VOXEL_SCALE_HIDE_UNKNOWNS &&
	    liveUnknownVoxelsVisible ||
	    liveScenePipe.GetCurrentScaleMode() == VoxelScaleMode::VOXEL_SCALE_SHOW_UNKNOWNS &&
	    !liveUnknownVoxelsVisible) {
		liveScenePipe.ToggleScaleMode();
	}
	liveScenePipe.GetHashBlockActor()->SetVisibility(liveHashBlocksVisible);
}

void SDFViz::InitializeWarps() {
	// load initial warps
	if (hasWarpIterationInfo && canonicalVoxelsVisible) {
		NextNonInterestWarps();
	}
	if (hasHighlightInfo && canonicalInterestVoxelsVisible) {
		NextInterestWarps();
	}
}

void SDFViz::AddActors() {
	// add voxel & grid actors
	sdfRenderer->AddActor(canonicalScenePipe.GetVoxelActor());
	sdfRenderer->AddActor(canonicalScenePipe.GetInterestVoxelActor());
	sdfRenderer->AddActor(canonicalScenePipe.GetWarplessVoxelActor());
	sdfRenderer->AddActor(canonicalScenePipe.GetHashBlockActor());
	sdfRenderer->AddActor(liveScenePipe.GetVoxelActor());
	sdfRenderer->AddActor(liveScenePipe.GetHashBlockActor());

	// add marker/highlight actors
	markerRenderer->AddActor(canonicalScenePipe.GetSelectionVoxelActor());
	markerRenderer->AddActor(canonicalScenePipe.GetSliceSelectionActor(0));
	markerRenderer->AddActor(canonicalScenePipe.GetSliceSelectionActor(1));
	markerRenderer->AddActor(canonicalScenePipe.GetSlicePreviewActor());
	markerRenderer->AddActor(highlightVisualizer.GetHighlightActor());
}

// region ============================================= SCENE SLICING ==================================================
bool SDFViz::MakeSlice() {
	if (canonicalScenePipe.GetSliceCoordinatesAreSet()) {
		Vector3i coord0, coord1;
		canonicalScenePipe.GetSliceCoordinates(coord0, coord1);
		std::string sliceIdentifier;
		sceneLogger->MakeSlice(coord0, coord1, sliceIdentifier);
		sliceIdentifiers.push_back(sliceIdentifier);
		return true;
	}
	return false;
}

void SDFViz::UpdateLiveSceneBounds(){
	if(sceneLogger->GetIsActiveSceneASlice()) {
		Vector3i minPoint, maxPoint;
		sceneLogger->GetActiveSceneBounds(minPoint);
		//TODO: get rid of magic number here -Greg (GitHub: Algomorph)
		minPoint -= Vector3i(2);
		maxPoint += Vector3i(2);
		liveScenePipe.SetExtractionBounds(minPoint, maxPoint);
	}else{
		liveScenePipe.ResetExtractionBounds();
	}
}

bool SDFViz::SwitchToSlice(unsigned int sliceIndex) {
	if (sliceIndex >= sliceIdentifiers.size()) {
		return false;
	}
	if (!sceneLogger->SwitchActiveScene(sliceIdentifiers[sliceIndex])) {
		return false;
	}
	LoadFrameData();
	ReinitializePipelines();
	UpdatePipelineVisibilitiesUsingLocalState();
	InitializeWarps();
	UpdateLiveSceneBounds();
	renderWindow->Render();
	return true;
}

bool SDFViz::SliceModeEnabled() const {
	return this->sceneLogger->GetIsActiveSceneASlice();
}

bool SDFViz::ToggleSliceMode(unsigned int sliceIndex) {
	if (SliceModeEnabled()) {
		return SwitchToFullScene();
	}
	return SwitchToSlice(sliceIndex);
}

bool SDFViz::SwitchToFullScene() {
	if (!sceneLogger->SwitchActiveScene()) {
		return false;
	}
	LoadFrameData();
	ReinitializePipelines();
	UpdatePipelineVisibilitiesUsingLocalState();
	InitializeWarps();
	UpdateLiveSceneBounds();
	renderWindow->Render();
}

void SDFViz::LoadAllSlices() {
	auto loadedIdentifiers = sceneLogger->LoadAllSlices();
	this->sliceIdentifiers.insert(std::end(sliceIdentifiers),
	                              std::make_move_iterator(std::begin(loadedIdentifiers)),
	                              std::make_move_iterator(std::end(loadedIdentifiers)));
}

void SDFViz::InitializeAxes() {
	vtkSmartPointer<vtkNamedColors> colors =
			vtkSmartPointer<vtkNamedColors>::New();

	vtkSmartPointer<vtkAxesActor> axes =
			vtkSmartPointer<vtkAxesActor>::New();

	orientationWidget =
			vtkSmartPointer<vtkOrientationMarkerWidget>::New();
	double rgba[4]{0.0, 0.0, 0.0, 0.0};
	colors->GetColor("Carrot",rgba);
	orientationWidget->SetOutlineColor(rgba[0], rgba[1], rgba[2]);
	orientationWidget->SetOrientationMarker( axes );
	orientationWidget->SetInteractor( renderWindowInteractor );
	orientationWidget->SetViewport( 0.0, 0.0, 0.4, 0.4 );
	orientationWidget->SetEnabled( 1 );
	orientationWidget->InteractiveOn();
}

void SDFViz::MoveFocusToSelectedVoxel() {
	MoveFocusToVoxelAt(canonicalScenePipe.GetSelectedVoxelCoordinates().toDouble());
}
// endregion