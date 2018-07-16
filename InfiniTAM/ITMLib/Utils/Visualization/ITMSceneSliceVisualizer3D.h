//  ================================================================
//  Created by Gregory Kramida on 1/22/18.
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
#include <thread>
#include <condition_variable>
#include <unordered_map>

//VTK
#include <vtkSmartPointer.h>
#include <vtkExtractPolyDataGeometry.h>
#include <vtkSphereSource.h>
#include <vtkCubeSource.h>
#include <vtkPointSet.h>


//ITMLib
#include "../../Objects/Scene/ITMScene.h"
#include "../FileIO/ITMSceneLogger.h"
#include "ITMSceneSliceVisualizer3DCommon.h"
#include "ITMVisualizationWindowManager.h"
#include "ITMVisualizationCommon.h"
#include "../../ITMLibDefines.h"

class vtkPoints;
class vtkPolyData;
class vtkActor;
class vtkGlyph3DMapper;

class vtkPolyDataMapper;

namespace ITMLib {

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
class ThreadInteropCommand;

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
class ITMSceneSliceVisualizer3DInteractorStyle;

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
class ITMSceneSliceVisualizer3D {

	friend class ThreadInteropCommand<TVoxelCanonical, TVoxelLive, TIndex>;
	friend class ITMSceneSliceVisualizer3DInteractorStyle<TVoxelCanonical, TVoxelLive, TIndex>;

public:
	//================= CONSTANTS ================
	static const char* colorAttributeName;
	static const char* scaleUnknownsHiddenAttributeName;
	static const char* scaleUnknownsVisibleAttributeName;

	// ====================== CONSTRUCTORS / DESTRUCTORS ==================
	ITMSceneSliceVisualizer3D(ITMScene<TVoxelCanonical, TIndex>* canonicalScene,
	                          ITMScene<TVoxelLive, TIndex>* liveScene,
	                          Vector3i focusCoordinates, Plane plane = PLANE_XY,
	                          int radiusInPlane = 10, int radiusOutOfPlane = 0);
	virtual ~ITMSceneSliceVisualizer3D();

	// ====================== MEMBER FUNCTIONS ===========================
	VoxelScaleMode GetCurrentScaleMode();
	void TriggerDrawWarpUpdates();
	void TriggerDrawSmoothingVectors();
	void TriggerUpdateLiveState();
	void TriggerBuildFusedCanonical();
	virtual void ToggleScaleMode();

	void SetVisibilityMode(VisibilityMode mode);
	void BuildFusedCanonicalFromCurrentScene();
	void UpdateLiveState();
protected:
	// region =============== STATIC FUNCTIONS ============================

	static void SetUpSDFColorLookupTable(vtkSmartPointer<vtkLookupTable>& table,
	                                     const double* highlightColor,
	                                     const double* positiveTruncatedColor,
	                                     const double* positiveNonTruncatedColor,
	                                     const double* negativeNonTruncatedColor,
	                                     const double* negativeTruncatedColor,
	                                     const double* unknownColor);
	// endregion
	// region ============= MEMBER FUNCTIONS =============================

	//endregion


	VoxelScaleMode scaleMode;

	//** highlights **
	ITM3DNestedMapOfArrays<ITMHighlightIterationInfo> highlights;

	// scene limits/boundaries/extent
	Vector6i bounds;
	Vector3i focusCoordinates;

	ITM3DWindow* window;


private:

	struct SceneSlice{
		void Initialize(){
			voxelVizData = vtkSmartPointer<vtkPolyData>::New();
			voxelMapper = vtkSmartPointer<vtkGlyph3DMapper>::New();
			voxelActor = vtkSmartPointer<vtkActor>::New();
			voxelColorLookupTable = vtkSmartPointer<vtkLookupTable>::New();
			hashBlockGridVizData = vtkSmartPointer<vtkPolyData>::New();
			hashBlockActor = vtkSmartPointer<vtkActor>::New();
			hashBlockMapper = vtkSmartPointer<vtkGlyph3DMapper>::New();
		}
		void SetUpMappersAndActors(
				std::array<double, 3>& hashBlockEdgeColor, vtkAlgorithmOutput* hashBlockGeometrySourceOutput,
				vtkAlgorithmOutput* voxelGeometrySourceOutput);
		int voxelCount = 0;
		vtkSmartPointer<vtkPolyData> voxelVizData;
		vtkSmartPointer<vtkGlyph3DMapper> voxelMapper;
		vtkSmartPointer<vtkActor> voxelActor;
		vtkSmartPointer<vtkLookupTable> voxelColorLookupTable;
		vtkSmartPointer<vtkPolyData> hashBlockGridVizData;
		vtkSmartPointer<vtkGlyph3DMapper> hashBlockMapper;
		vtkSmartPointer<vtkActor> hashBlockActor;
	};

	// region ============== MEMBER FUNCTIONS ===========================
	void InitializeVoxels();
	template <typename TVoxel, typename TVoxelDataRetriever>
	void BuildVoxelAndHashBlockPolyDataFromScene(
			ITMScene <TVoxel, TIndex>* scene, SceneSlice& sceneSlice);
	void BuildInitialSlices();
	void InitializeWarps();
	void AddSliceActorsToRenderers();
	void SetUpGeometrySources();
	void Run();
	void DrawWarpUpdates();
	void DrawSmoothnessVectors();
	void AdvanceLiveStateVizualization();
	void RetreatLiveStateVizualization();

	// endregion
	// region ============== MEMBER VARIABLES ===========================

	// ===================== MEMBER VARIABLES ============================


	ITMScene<TVoxelCanonical, TIndex>* canonicalScene;
	ITMScene<TVoxelLive, TIndex>* liveScene;

	// ** visualization modes / states **
	VisibilityMode visibilityMode;

	// ** visualization structures & auxiliaries **
	vtkSmartPointer<vtkSphereSource> voxelVizGeometrySource;
	vtkSmartPointer<vtkCubeSource> hashBlockVizGeometrySource;


	SceneSlice liveSlice;
	SceneSlice canonicalSlice;
	SceneSlice fusedCanonicalSlice;

	// ** warp updates **
	vtkSmartPointer<vtkPolyData> updatesData = vtkSmartPointer<vtkPolyData>::New();
	vtkSmartPointer<vtkPolyDataMapper> updatesMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	vtkSmartPointer<vtkActor> updatesActor = vtkSmartPointer<vtkActor>::New();

	vtkSmartPointer<vtkPolyData> smoothingVectorData = vtkSmartPointer<vtkPolyData>::New();
	vtkSmartPointer<vtkPolyDataMapper> smoothingVectorMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	vtkSmartPointer<vtkActor> smoothingVectorActor = vtkSmartPointer<vtkActor>::New();
	std::unordered_map<std::string, Vector3d> smoothingHedgehogEndpoints;


	// ** live updates **
	std::vector<vtkSmartPointer<vtkPolyData>> liveSliceStates;

	// ** colors **
	// = live =
	std::array<double, 4> livePositiveTruncatedVoxelColor = ITMLib::Viz::livePositiveTruncatedVoxelColor;
	std::array<double, 4> livePositiveNonTruncatedVoxelColor = ITMLib::Viz::livePositiveNonTruncatedVoxelColor;
	std::array<double, 4> liveNegativeNonTruncatedVoxelColor = ITMLib::Viz::liveNegativeNonTruncatedVoxelColor;
	std::array<double, 4> liveNegativeTruncatedVoxelColor = ITMLib::Viz::liveNegativeTruncatedVoxelColor;
	std::array<double, 4> liveUnknownVoxelColor = ITMLib::Viz::liveUnknownVoxelColor;
	std::array<double, 4> liveHighlightVoxelColor = ITMLib::Viz::highlightVoxelColor;
	std::array<double, 3> liveHashBlockEdgeColor = ITMLib::Viz::liveHashBlockEdgeColor;
	// = canonical =
	std::array<double, 4> canonicalPositiveTruncatedVoxelColor = ITMLib::Viz::canonicalPositiveTruncatedVoxelColor;
	std::array<double, 4> canonicalPositiveNonTruncatedVoxelColor = ITMLib::Viz::canonicalPositiveNonTruncatedVoxelColor;
	std::array<double, 4> canonicalNegativeNonTruncatedVoxelColor = ITMLib::Viz::canonicalNegativeNonTruncatedVoxelColor;
	std::array<double, 4> canonicalNegativeTruncatedVoxelColor = ITMLib::Viz::canonicalNegativeTruncatedVoxelColor;
	std::array<double, 4> canonicalUnknownVoxelColor = ITMLib::Viz::canonicalUnknownVoxelColor;
	std::array<double, 4> canonicalHighlightVoxelColor = ITMLib::Viz::highlightVoxelColor;
	std::array<double, 3> canonicalHashBlockEdgeColor = ITMLib::Viz::canonicalHashBlockEdgeColor;

	// ** threading controls **
	std::mutex mutex;
	std::condition_variable conditionVariable;
	bool initialized = false;
	bool warpUpdatePerformed = false;
	bool smoothingVectorsDrawn = false;
	bool fusedCanonicalBuilt = false;
	bool liveStateUpdated = false;
	int liveSamplingFieldIndex = 0;
	int visibleOptimizationStepIndex = 0;
	std::thread* thread = nullptr;
	vtkSmartPointer<ThreadInteropCommand<TVoxelCanonical, TVoxelLive, TIndex>> threadCallback;


	//endregion


};


}//namespace ITMLib;


