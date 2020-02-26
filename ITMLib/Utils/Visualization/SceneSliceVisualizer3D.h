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

//ITMLib
#include "../../Objects/Volume/VoxelVolume.h"
#include "../FileIO/VolumeSequenceRecorder.h"
#include "SceneSliceVisualizer3DCommon.h"
#include "VisualizationWindowManager.h"
#include "../Geometry/CardinalAxesAndPlanes.h"
#include "../../GlobalTemplateDefines.h"

//VTK
#include <vtkSmartPointer.h>
#include <vtkExtractPolyDataGeometry.h>
#include <vtkSphereSource.h>
#include <vtkCubeSource.h>
#include <vtkPointSet.h>


class vtkPoints;
class vtkPolyData;
class vtkActor;
class vtkGlyph3DMapper;

class vtkPolyDataMapper;

namespace ITMLib {

template<typename TVoxel, typename TWarp, typename TIndex>
class ThreadInteropCommand;

template<typename TVoxel, typename TWarp, typename TIndex>
class SceneSliceVisualizer3DInteractorStyle;

template<typename TVoxel, typename TWarp, typename TIndex>
class SceneSliceVisualizer3D {

	friend class ThreadInteropCommand<TVoxel, TWarp, TIndex>;
	friend class SceneSliceVisualizer3DInteractorStyle<TVoxel, TWarp, TIndex>;

public:
	//================= CONSTANTS ================
	static const char* colorAttributeName;
	static const char* scaleUnknownsHiddenAttributeName;
	static const char* scaleUnknownsVisibleAttributeName;

	// ====================== CONSTRUCTORS / DESTRUCTORS ==================
	SceneSliceVisualizer3D(VoxelVolume<TVoxel, TIndex>* canonicalScene,
	                       VoxelVolume<TVoxel, TIndex>* liveScene,
	                       VoxelVolume<TWarp, TIndex>* warpField,
	                       Vector3i focusCoordinates, Plane plane = PLANE_XY,
	                       int radiusInPlane = 10, int radiusOutOfPlane = 0);
	virtual ~SceneSliceVisualizer3D();

	// ====================== MEMBER FUNCTIONS ===========================
	VoxelScaleMode GetCurrentScaleMode();
	void TriggerDrawWarpUpdates();
	void TriggerUpdateLiveState();
	void TriggerBuildFusedCanonical();
	void TriggerRebuildSlices();
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
	NestedMap3DOfArrays<ITMHighlightIterationInfo> highlights;

	// scene limits/boundaries/extent
	Vector6i bounds;
	Vector3i focusCoordinates;

	Window3D* window;


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
	void BuildVoxelAndHashBlockPolyDataFromScene(
			VoxelVolume <TVoxel, TIndex>* scene, SceneSlice& sceneSlice);
	void BuildInitialSlices();
	void RebuildSlices();
	void InitializeWarps();
	void AddSliceActorsToRenderers();
	void SetUpGeometrySources();
	void Run();
	void DrawWarpUpdates();
	void AdvanceLiveStateVizualization();
	void RetreatLiveStateVizualization();

	// endregion
	// region ============== MEMBER VARIABLES ===========================

	// ===================== MEMBER VARIABLES ============================


	VoxelVolume<TVoxel, TIndex>* canonicalScene;
	VoxelVolume<TVoxel, TIndex>* liveScene;
	VoxelVolume<TWarp, TIndex>* warpField;

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

	vtkSmartPointer<vtkPolyData> smoothingTermVectorData = vtkSmartPointer<vtkPolyData>::New();
	vtkSmartPointer<vtkPolyDataMapper> smoothingVectorMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	vtkSmartPointer<vtkActor> smoothingTermVectorActor = vtkSmartPointer<vtkActor>::New();

	vtkSmartPointer<vtkPolyData> dataTermVectorData = vtkSmartPointer<vtkPolyData>::New();
	vtkSmartPointer<vtkPolyDataMapper> dataTermVectorMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	vtkSmartPointer<vtkActor> dataTermVectorActor = vtkSmartPointer<vtkActor>::New();

	std::unordered_map<std::string, std::pair<Vector3d,Vector3d>> componentHedgehogEndpoints;
	bool currentZebraIndex = 0;


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
	bool fusedCanonicalBuilt = false;
	bool liveStateUpdated = false;
	bool slicesRebuilt = false;
	int visibleOptimizationStepIndex = 0;
	std::thread* thread = nullptr;
	vtkSmartPointer<ThreadInteropCommand<TVoxel, TWarp, TIndex>> threadCallback;


	//endregion


};


}//namespace ITMLib;



