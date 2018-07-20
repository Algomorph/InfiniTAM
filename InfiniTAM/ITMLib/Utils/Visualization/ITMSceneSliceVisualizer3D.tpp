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

//stdlib
#include <sstream>

//VTK
#include <vtkActor.h>
#include <vtkFloatArray.h>
#include <vtkPointData.h>
#include <vtkGlyph3DMapper.h>
#include <vtkBox.h>
#include <vtkProperty.h>
#include <vtkLookupTable.h>
#include <vtkPolyDataMapper.h>
#include <vtkCubeSource.h>
#include <vtkBox.h>
#include <vtkRenderWindow.h>
#include <mutex>
#include <vtkHedgeHog.h>
#include <vtkVertexGlyphFilter.h>


//Local
#include "ITMSceneSliceVisualizer3D.h"
#include "ITMSceneSliceVisualizer3DCommon.h"
#include "ITMSceneSliceVisualizerCommon.h"
#include "ITMVisualizationCommon.h"
#include "ITMSceneSliceVisualizer3DInteractorStyle.h"
#include "../../Objects/Scene/ITMSceneTraversal_PlainVoxelArray.h"
#include "../../Objects/Scene/ITMSceneTraversal_VoxelBlockHash.h"
#include "../ITMLibSettings.h"



//TODO: alter OO design such that constructor isn't calling virual / override member functions -Greg (GitHub: Algomorph)

using namespace ITMLib;

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
const char* ITMSceneSliceVisualizer3D<TVoxelCanonical, TVoxelLive, TIndex>::colorAttributeName = "color";
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
const char* ITMSceneSliceVisualizer3D<TVoxelCanonical, TVoxelLive, TIndex>::scaleUnknownsHiddenAttributeName = "scale";
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
const char* ITMSceneSliceVisualizer3D<TVoxelCanonical, TVoxelLive, TIndex>::scaleUnknownsVisibleAttributeName = "alternative_scale";


enum RequestedOperation {
	NONE,
	DRAW_WARP_UPDATES,
	BUILD_FUSED_CANONICAL,
	UPDATE_LIVE_STATE,
	REBUILD_SLICES,

};

// Used for multithreaded interop with VTK
namespace ITMLib {
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
class ThreadInteropCommand : public vtkCommand {
public:
vtkTypeMacro(ThreadInteropCommand, vtkCommand);
	RequestedOperation operation = NONE;

	static ThreadInteropCommand* New() {
		return new ThreadInteropCommand;
	}

	void Execute(vtkObject*vtkNotUsed(caller), unsigned long vtkNotUsed(eventId),
	             void*vtkNotUsed(callData)) {
		switch (operation) {
			case NONE:
				break;
			case DRAW_WARP_UPDATES:
				parent->DrawWarpUpdates();
				break;
			case BUILD_FUSED_CANONICAL:
				parent->BuildFusedCanonicalFromCurrentScene();
				break;
			case UPDATE_LIVE_STATE:
				parent->UpdateLiveState();
				break;
			case REBUILD_SLICES:
				parent->RebuildSlices();
				break;
			default:
				break;
		}
		operation = NONE;
	}

	ITMSceneSliceVisualizer3D<TVoxelCanonical, TVoxelLive, TIndex>* parent;
};
} //namespace ITMLib


template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMSceneSliceVisualizer3D<TVoxelCanonical, TVoxelLive, TIndex>::SceneSlice::SetUpMappersAndActors(
		std::array<double, 3>& hashBlockEdgeColor, vtkAlgorithmOutput* hashBlockGeometrySourceOutput,
		vtkAlgorithmOutput* voxelGeometrySourceOutput) {

	voxelMapper->SetInputData(voxelVizData);
	voxelMapper->SetSourceConnection(voxelGeometrySourceOutput);
	voxelMapper->SetLookupTable(voxelColorLookupTable);
	voxelMapper->ScalingOn();
	voxelMapper->SetScaleArray(scaleUnknownsVisibleAttributeName);
	voxelMapper->SetScaleModeToScaleByMagnitude();
	voxelMapper->ScalarVisibilityOn();
	voxelMapper->SetScalarModeToUsePointData();
	voxelMapper->SetColorModeToMapScalars();
	voxelMapper->SetScalarRange(0.0, static_cast<double>(COLOR_INDEX_COUNT));
	voxelMapper->InterpolateScalarsBeforeMappingOff();
	voxelMapper->Update();

	hashBlockMapper->SetInputData(hashBlockGridVizData);
	hashBlockMapper->SetSourceConnection(hashBlockGeometrySourceOutput);
	hashBlockMapper->ScalarVisibilityOff();
	hashBlockMapper->ScalingOff();
	hashBlockMapper->SetScaleFactor(1.0);

// set up voxel actors
	voxelActor->SetMapper(voxelMapper);
	voxelActor->GetProperty()->SetPointSize(20.0f);
	voxelActor->VisibilityOn();

	hashBlockActor->SetMapper(hashBlockMapper);
	hashBlockActor->GetProperty()->SetRepresentationToWireframe();
	hashBlockActor->GetProperty()->SetColor(hashBlockEdgeColor.data());
	hashBlockActor->VisibilityOff();

}

inline std::string to_string(Vector6i vector) {
	std::ostringstream s;
	s << vector;
	return s.str();
}


// region ====================================== CONSTRUCTORS / DESTRUCTORS ============================================

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
ITMSceneSliceVisualizer3D<TVoxelCanonical, TVoxelLive, TIndex>::ITMSceneSliceVisualizer3D(
		ITMScene<TVoxelCanonical, TIndex>* canonicalScene,
		ITMScene<TVoxelLive, TIndex>* liveScene,
		Vector3i focusCoordinates, Plane plane,
		int radiusInPlane, int radiusOutOfPlane):
		canonicalScene(canonicalScene),
		liveScene(liveScene),
		bounds(ComputeBoundsAroundPoint(focusCoordinates, radiusInPlane, radiusOutOfPlane, plane)),
		focusCoordinates(focusCoordinates),
		scaleMode(VOXEL_SCALE_HIDE_UNKNOWNS) {
	this->thread = new std::thread(&ITMSceneSliceVisualizer3D<TVoxelCanonical, TVoxelLive, TIndex>::Run, this);
	std::unique_lock<std::mutex> lock(mutex);
	conditionVariable.wait(lock, [this] { return this->initialized; });
}


template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
ITMSceneSliceVisualizer3D<TVoxelCanonical, TVoxelLive, TIndex>::~ITMSceneSliceVisualizer3D() {
}

// endregion ===========================================================================================================


// region ========================================== BUILD POLYDATA FROM VOXELS ========================================

template<typename TVoxel>
struct RetrieveVoxelDataBasicStaticFunctior {
	static float GetSdf(const TVoxel& voxel) {
		return TVoxel::valueToFloat(voxel.sdf);
	}

	static ITMLib::VoxelFlags GetFlags(const TVoxel& voxel) {
		return static_cast<ITMLib::VoxelFlags>(voxel.flags);
	}
};

template<typename TVoxel>
struct RetrieveVoxelDataFieldIndex0StaticFunctior {
	static float GetSdf(const TVoxel& voxel) {
		return TVoxel::valueToFloat(voxel.sdf_values[0]);
	}

	static ITMLib::VoxelFlags GetFlags(const TVoxel& voxel) {
		return static_cast<ITMLib::VoxelFlags>(voxel.flag_values[0]);
	}
};

template<typename TVoxel>
struct RetrieveVoxelDataFieldIndex1StaticFunctior {
	static float GetSdf(const TVoxel& voxel) {
		return TVoxel::valueToFloat(voxel.sdf_values[1]);
	}

	static ITMLib::VoxelFlags GetFlags(const TVoxel& voxel) {
		return static_cast<ITMLib::VoxelFlags>(voxel.flag_values[1]);
	}
};

template<typename TVoxel, typename VoxelDataRetrievalFunctor>
struct AddVoxelPointFunctor {
public:
	AddVoxelPointFunctor(vtkFloatArray* scaleAttribute,
	                     vtkFloatArray* alternativeScaleAttribute,
	                     vtkIntArray* colorAttribute,
	                     vtkPoints* voxelPoints,
	                     vtkPoints* hashBlockPoints,
	                     Vector3i focusCoordinates) :
			scaleAttribute(scaleAttribute),
			alternativeScaleAttribute(alternativeScaleAttribute),
			colorAttribute(colorAttribute),
			voxelPoints(voxelPoints),
			hashBlockPoints(hashBlockPoints),
			focusCoordinates(focusCoordinates) {}

	void operator()(const TVoxel& voxel, const Vector3i& position) {

		float sdf = VoxelDataRetrievalFunctor::GetSdf(voxel);
		ITMLib::VoxelFlags flags = VoxelDataRetrievalFunctor::GetFlags(voxel);
		float voxelScale = COMPUTE_VOXEL_SCALE_HIDE_UNKNOWNS(sdf, flags);
		float alternativeVoxelScale = COMPUTE_VOXEL_SCALE(sdf);

		int voxelColorIndex;

		if (position == focusCoordinates) {
			voxelColorIndex = HIGHLIGHT_SDF_COLOR_INDEX;
		} else {
			bool truncated = voxel.flags == ITMLib::VOXEL_TRUNCATED;
			voxelColorIndex = voxel.flags == ITMLib::VOXEL_UNKNOWN ? UNKNOWN_SDF_COLOR_INDEX :
			                  sdf > 0 ?
			                  (truncated ? POSITIVE_TRUNCATED_SDF_COLOR_INDEX : POSITIVE_NON_TRUNCATED_SDF_COLOR_INDEX)
			                          :
			                  (truncated ? NEGATIVE_TRUNCATED_SDF_COLOR_INDEX : NEGATIVE_NON_TRUNCATED_SDF_COLOR_INDEX);
		}

		voxelPoints->InsertNextPoint(position.x, position.y, position.z);
		scaleAttribute->InsertNextValue(voxelScale);
		alternativeScaleAttribute->InsertNextValue(alternativeVoxelScale);
		colorAttribute->InsertNextValue(voxelColorIndex);
		voxelCount++;
	}

	void processHashEntry(const ITMHashEntry& hashEntry) {
		Vector3i currentBlockPositionVoxels = hashEntry.pos.toInt() * SDF_BLOCK_SIZE;
		const double centerOffset = -0.5;
		//draw hash block
		hashBlockPoints->InsertNextPoint((currentBlockPositionVoxels.x + centerOffset),
		                                 (currentBlockPositionVoxels.y + centerOffset),
		                                 (currentBlockPositionVoxels.z + centerOffset));
	}

	int voxelCount = 0;
private:
	Vector3i focusCoordinates;
	vtkFloatArray* scaleAttribute;
	vtkFloatArray* alternativeScaleAttribute;
	vtkIntArray* colorAttribute;
	vtkPoints* voxelPoints;
	vtkPoints* hashBlockPoints;

};


template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
template<typename TVoxel, typename TVoxelDataRetriever>
void ITMSceneSliceVisualizer3D<TVoxelCanonical, TVoxelLive, TIndex>::BuildVoxelAndHashBlockPolyDataFromScene(
		ITMScene<TVoxel, TIndex>* scene, SceneSlice& sceneSlice) {
	vtkSmartPointer<vtkPoints> voxelPoints = vtkSmartPointer<vtkPoints>::New();
	vtkSmartPointer<vtkPoints> hashBlockPoints = vtkSmartPointer<vtkPoints>::New();

	//holds color for each voxel
	vtkSmartPointer<vtkIntArray> colorAttribute = vtkSmartPointer<vtkIntArray>::New();
	colorAttribute->SetName(colorAttributeName);

	//holds scale of each voxel
	vtkSmartPointer<vtkFloatArray> scaleAttribute = vtkSmartPointer<vtkFloatArray>::New();
	scaleAttribute->SetName(scaleUnknownsHiddenAttributeName);

	//holds alternative scale of each voxel (showing -1 value voxels)
	vtkSmartPointer<vtkFloatArray> alternativeScaleAttribute = vtkSmartPointer<vtkFloatArray>::New();
	alternativeScaleAttribute->SetName(scaleUnknownsVisibleAttributeName);

	AddVoxelPointFunctor<TVoxel, TVoxelDataRetriever> addVoxelPointFunctor(
			scaleAttribute, alternativeScaleAttribute, colorAttribute, voxelPoints, hashBlockPoints, focusCoordinates);
	VoxelPositionAndHashEntryTraversalWithinBounds_CPU(scene, addVoxelPointFunctor, bounds);
	sceneSlice.voxelCount = addVoxelPointFunctor.voxelCount;

	vtkSmartPointer<vtkPolyData> voxelVizData = sceneSlice.voxelVizData;
	voxelVizData->SetPoints(voxelPoints);
	voxelVizData->GetPointData()->AddArray(colorAttribute);
	voxelVizData->GetPointData()->AddArray(scaleAttribute);
	voxelVizData->GetPointData()->AddArray(alternativeScaleAttribute);
	voxelVizData->GetPointData()->SetActiveScalars(colorAttributeName);

	sceneSlice.hashBlockGridVizData->SetPoints(hashBlockPoints);
}

// endregion ===========================================================================================================
// region ========================== PRIVATE INIT MEMBER FUNCTIONS =====================================================

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMSceneSliceVisualizer3D<TVoxelCanonical, TVoxelLive, TIndex>::BuildInitialSlices() {
	BuildVoxelAndHashBlockPolyDataFromScene<TVoxelCanonical, RetrieveVoxelDataBasicStaticFunctior<TVoxelCanonical>>(
			canonicalScene, canonicalSlice);
	BuildVoxelAndHashBlockPolyDataFromScene<TVoxelLive, RetrieveVoxelDataFieldIndex1StaticFunctior<TVoxelLive>>(
			liveScene, liveSlice);

	this->liveSliceStates.push_back(liveSlice.voxelVizData);

	canonicalSlice.SetUpMappersAndActors(canonicalHashBlockEdgeColor, hashBlockVizGeometrySource->GetOutputPort(),
	                                     voxelVizGeometrySource->GetOutputPort());
	liveSlice.SetUpMappersAndActors(liveHashBlockEdgeColor, hashBlockVizGeometrySource->GetOutputPort(),
	                                voxelVizGeometrySource->GetOutputPort());
	liveSlice.voxelActor->VisibilityOff();
}


template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMSceneSliceVisualizer3D<TVoxelCanonical, TVoxelLive, TIndex>::RebuildSlices() {
	std::unique_lock<std::mutex> lock(mutex);
	BuildVoxelAndHashBlockPolyDataFromScene<TVoxelCanonical, RetrieveVoxelDataBasicStaticFunctior<TVoxelCanonical>>(
			canonicalScene, canonicalSlice);
	BuildVoxelAndHashBlockPolyDataFromScene<TVoxelLive, RetrieveVoxelDataFieldIndex1StaticFunctior<TVoxelLive>>(
			liveScene, liveSlice);
	canonicalSlice.voxelVizData->Modified();
	liveSlice.voxelVizData->Modified();
	window->Update();
	slicesRebuilt=true;
	lock.unlock();
	conditionVariable.notify_all();

}


template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMSceneSliceVisualizer3D<TVoxelCanonical, TVoxelLive, TIndex>::AddSliceActorsToRenderers() {
	window->AddActorToFirstLayer(this->canonicalSlice.voxelActor);
	window->AddActorToFirstLayer(this->canonicalSlice.hashBlockActor);
	window->AddActorToFirstLayer(this->liveSlice.voxelActor);
	window->AddActorToFirstLayer(this->liveSlice.hashBlockActor);
}
// endregion ===========================================================================================================
// region ================================== STATIC INIT FUNCTIONS =====================================================
/**
 * \brief Sets up the geometry to use for voxel & hash block display
 */
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMSceneSliceVisualizer3D<TVoxelCanonical, TVoxelLive, TIndex>::SetUpGeometrySources() {
	//Individual voxel shape
	voxelVizGeometrySource->SetThetaResolution(6);
	voxelVizGeometrySource->SetPhiResolution(6);
	voxelVizGeometrySource->SetRadius(0.5);//size of half a voxel
	voxelVizGeometrySource->Update();

	//Voxel hash block shape
	hashBlockVizGeometrySource->SetBounds(0, SDF_BLOCK_SIZE, 0, SDF_BLOCK_SIZE, 0, SDF_BLOCK_SIZE);
}


template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMSceneSliceVisualizer3D<TVoxelCanonical, TVoxelLive, TIndex>::SetUpSDFColorLookupTable(
		vtkSmartPointer<vtkLookupTable>& table,
		const double* highlightColor,
		const double* positiveTruncatedColor,
		const double* positiveNonTruncatedColor,
		const double* negativeNonTruncatedColor,
		const double* negativeTruncatedColor,
		const double* unknownColor) {
	table->SetTableRange(0.0, static_cast<double>(COLOR_INDEX_COUNT));
	table->SetNumberOfTableValues(COLOR_INDEX_COUNT);
	table->SetNumberOfColors(COLOR_INDEX_COUNT);
	table->SetTableValue(POSITIVE_TRUNCATED_SDF_COLOR_INDEX, positiveTruncatedColor);
	table->SetTableValue(POSITIVE_NON_TRUNCATED_SDF_COLOR_INDEX, positiveNonTruncatedColor);
	table->SetTableValue(NEGATIVE_NON_TRUNCATED_SDF_COLOR_INDEX, negativeNonTruncatedColor);
	table->SetTableValue(NEGATIVE_TRUNCATED_SDF_COLOR_INDEX, negativeTruncatedColor);
	table->SetTableValue(UNKNOWN_SDF_COLOR_INDEX, unknownColor);
	table->SetTableValue(HIGHLIGHT_SDF_COLOR_INDEX, highlightColor);
	table->SetNanColor(0.4, 0.7, 0.1, 1.0);
}


// endregion ===========================================================================================================

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMSceneSliceVisualizer3D<TVoxelCanonical, TVoxelLive, TIndex>::ToggleScaleMode() {
	if (scaleMode == VoxelScaleMode::VOXEL_SCALE_HIDE_UNKNOWNS) {
		scaleMode = VoxelScaleMode::VOXEL_SCALE_SHOW_UNKNOWNS;
		canonicalSlice.voxelMapper->SetScaleArray(scaleUnknownsVisibleAttributeName);
		liveSlice.voxelMapper->SetScaleArray(scaleUnknownsVisibleAttributeName);
	} else {
		scaleMode = VoxelScaleMode::VOXEL_SCALE_HIDE_UNKNOWNS;
		canonicalSlice.voxelMapper->SetScaleArray(scaleUnknownsHiddenAttributeName);
		liveSlice.voxelMapper->SetScaleArray(scaleUnknownsHiddenAttributeName);
	}
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
VoxelScaleMode ITMSceneSliceVisualizer3D<TVoxelCanonical, TVoxelLive, TIndex>::GetCurrentScaleMode() {
	return this->scaleMode;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMSceneSliceVisualizer3D<TVoxelCanonical, TVoxelLive, TIndex>::InitializeVoxels() {

	// *** initializations ***
	liveSlice.Initialize();
	canonicalSlice.Initialize();
	fusedCanonicalSlice.Initialize();
	scaleMode = VOXEL_SCALE_SHOW_UNKNOWNS;

	voxelVizGeometrySource = vtkSmartPointer<vtkSphereSource>::New();
	hashBlockVizGeometrySource = vtkSmartPointer<vtkCubeSource>::New();


	window = ITMVisualizationWindowManager::Instance().MakeOrGet3DWindow(
			"SceneSliceVisualizer3D" + to_string(this->bounds),
			"Scene 3D Slice Visualizer for bounds (" + to_string(this->bounds) + "))");

	// Create the color maps
	SetUpSDFColorLookupTable(liveSlice.voxelColorLookupTable, liveHighlightVoxelColor.data(),
	                         livePositiveTruncatedVoxelColor.data(),
	                         livePositiveNonTruncatedVoxelColor.data(),
	                         liveNegativeNonTruncatedVoxelColor.data(),
	                         liveNegativeTruncatedVoxelColor.data(), liveUnknownVoxelColor.data());
	SetUpSDFColorLookupTable(canonicalSlice.voxelColorLookupTable, canonicalHighlightVoxelColor.data(),
	                         canonicalPositiveTruncatedVoxelColor.data(),
	                         canonicalPositiveNonTruncatedVoxelColor.data(),
	                         canonicalNegativeNonTruncatedVoxelColor.data(),
	                         canonicalNegativeTruncatedVoxelColor.data(), canonicalUnknownVoxelColor.data());
	fusedCanonicalSlice.voxelColorLookupTable = canonicalSlice.voxelColorLookupTable;

	SetUpGeometrySources();
	BuildInitialSlices();
	AddSliceActorsToRenderers();

}


template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMSceneSliceVisualizer3D<TVoxelCanonical, TVoxelLive, TIndex>::InitializeWarps() {

	this->window->AddLayer(Vector4d(1.0, 1.0, 1.0, 0.0));

	auto initializeWarpSet = [this](vtkSmartPointer<vtkPolyData> polyData, vtkSmartPointer<vtkPolyDataMapper> mapper,
	                                vtkSmartPointer<vtkActor> actor, const char* vectorArrayName, Vector3d color) {
		vtkSmartPointer<vtkPoints> updatePoints = vtkSmartPointer<vtkPoints>::New();
		vtkSmartPointer<vtkFloatArray> updateVectors = vtkSmartPointer<vtkFloatArray>::New();
		updateVectors->SetName(vectorArrayName);
		updateVectors->SetNumberOfComponents(3);

		polyData->SetPoints(updatePoints);
		polyData->GetPointData()->SetVectors(updateVectors);

		vtkSmartPointer<vtkHedgeHog> hedgehog = vtkSmartPointer<vtkHedgeHog>::New();
		hedgehog->SetInputData(polyData);


		mapper->SetColorModeToDirectScalars();
		mapper->SetInputConnection(hedgehog->GetOutputPort());


		actor->SetMapper(mapper);
		actor->GetProperty()->SetColor(color.values);
		this->window->AddActorToLayer(actor, 1);
	};

	initializeWarpSet(updatesData, updatesMapper, updatesActor, "Warp update vectors", Vector3d(0.0,0.0,0.0));
	initializeWarpSet(dataTermVectorData, dataTermVectorMapper, dataTermVectorActor, "Warp data term vectors",
	                  Vector3d(0.0,0.0,1.0));
	initializeWarpSet(smoothingTermVectorData, smoothingVectorMapper, smoothingTermVectorActor,
	                  "Warp smoothing term vectors", Vector3d(0.0,1.0,0.0));
}

//region ============================= VISUALIZE WARP UPDATES ==========================================================
static inline
std::string stringifyVoxelCoordinate(const Vector3i& position) {
	std::stringstream ss;
	ss << position;
	return ss.str();
}

template<typename TVoxel>
struct TransferWarpUpdatesToVtkStructuresFunctor {
public:
	TransferWarpUpdatesToVtkStructuresFunctor(vtkPoints* updatePoints, vtkFloatArray* updateVectors) :
			updatePoints(updatePoints), updateVectors(updateVectors) {}

	void operator()(const TVoxel& voxel, const Vector3i& position) {
		Vector3f updateStartPoint = position.toFloat() + voxel.framewise_warp - voxel.warp_update;
		Vector3f updateVector = voxel.warp_update;
		updatePoints->InsertNextPoint(updateStartPoint.x, updateStartPoint.y, updateStartPoint.z);
		updateVectors->InsertNextTuple(updateVector.values);
	}

private:
	vtkPoints* updatePoints;
	vtkFloatArray* updateVectors;
};


template<typename TVoxel>
struct TransferWarpUpdatesToVtkStructuresFunctor_WithComponents {
public:
	TransferWarpUpdatesToVtkStructuresFunctor_WithComponents(
			vtkSmartPointer<vtkPolyData> updatesData,
			vtkSmartPointer<vtkPolyData> dataTermData,
			vtkSmartPointer<vtkPolyData> smoothingTermData,
			std::unordered_map<std::string, std::pair<Vector3d,Vector3d>>& componentHedgehogEndpoints)
			:
			updatePoints(updatesData->GetPoints()),updateVectors(vtkFloatArray::SafeDownCast(updatesData->GetPointData()->GetVectors())),
			dataTermPoints(dataTermData->GetPoints()), dataTermVectors(vtkFloatArray::SafeDownCast(dataTermData->GetPointData()->GetVectors())),
			smoothingTermPoints(smoothingTermData->GetPoints()), smoothingTermVectors(vtkFloatArray::SafeDownCast(smoothingTermData->GetPointData()->GetVectors())), componentHedgehogEndpoints(componentHedgehogEndpoints){}

	void operator()(const TVoxel& voxel, const Vector3i& position) {
		std::string positionAsString = stringifyVoxelCoordinate(position);

		// whole update vector
		Vector3f updateStartPoint = position.toFloat() + voxel.framewise_warp - voxel.warp_update;
		Vector3f updateVector = voxel.warp_update;
		updatePoints->InsertNextPoint(updateStartPoint.x, updateStartPoint.y, updateStartPoint.z);
		updateVectors->InsertNextTuple(updateVector.values);

		// data & smoothing terms
		Vector3d dataTermVectorStartPoint,smoothingTermVectorStartPoint;

		auto iterator = componentHedgehogEndpoints.find(positionAsString);
		if (iterator == componentHedgehogEndpoints.end()) {
			dataTermVectorStartPoint = position.toDouble();
			smoothingTermVectorStartPoint = position.toDouble();
		} else {
			dataTermVectorStartPoint = iterator->second.first;
			smoothingTermVectorStartPoint = iterator->second.second;
		}
		Vector3f dataTermVector =
				-ITMLibSettings::Instance().sceneTrackingGradientDescentLearningRate * voxel.data_term_gradient;
		Vector3f smoothingTermVector =
				-ITMLibSettings::Instance().sceneTrackingGradientDescentLearningRate * voxel.smoothing_term_gradient;

		dataTermPoints->InsertNextPoint(dataTermVectorStartPoint.values);
		dataTermVectors->InsertNextTuple(dataTermVector.values);
		smoothingTermPoints->InsertNextPoint(smoothingTermVectorStartPoint.values);
		smoothingTermVectors->InsertNextTuple(smoothingTermVector.values);
		componentHedgehogEndpoints[positionAsString] =
				std::make_pair(dataTermVectorStartPoint + dataTermVector.toDouble(),
				               smoothingTermVectorStartPoint + smoothingTermVector.toDouble());
	}

private:
	std::unordered_map<std::string, std::pair<Vector3d,Vector3d>>& componentHedgehogEndpoints;
	vtkPoints* smoothingTermPoints;
	vtkFloatArray* smoothingTermVectors;
	vtkPoints* dataTermPoints;
	vtkFloatArray* dataTermVectors;

	vtkPoints* updatePoints;
	vtkFloatArray* updateVectors;
};

template<typename TVoxelCanonical, typename TIndex, bool hasDebugInfo>
struct DrawWarpUpdateFunctor;

template<typename TVoxelCanonical, typename TIndex>
struct DrawWarpUpdateFunctor<TVoxelCanonical, TIndex, false> {
	static void
	run(vtkSmartPointer<vtkPolyData> updatesData,vtkSmartPointer<vtkPolyData> dataTermData,
	    vtkSmartPointer<vtkPolyData> smoothingTermData, ITMScene<TVoxelCanonical, TIndex>* canonicalScene,
	    Vector6i bounds, std::unordered_map<std::string, std::pair<Vector3d,Vector3d>>& componentHedgehogEndpoints) {

		vtkPoints* updatePoints = updatesData->GetPoints();
		vtkFloatArray* updateVectors = vtkFloatArray::SafeDownCast(updatesData->GetPointData()->GetVectors());
		TransferWarpUpdatesToVtkStructuresFunctor<ITMVoxelCanonical> transferWarpUpdatesToVtkStructuresFunctor(
				updatePoints, updateVectors);
		VoxelPositionTraversalWithinBounds_CPU(canonicalScene, transferWarpUpdatesToVtkStructuresFunctor, bounds);
		updatesData->Modified();
	}
};

template<typename TVoxelCanonical, typename TIndex>
struct DrawWarpUpdateFunctor<TVoxelCanonical, TIndex, true> {
	static void
	run(vtkSmartPointer<vtkPolyData> updatesData,vtkSmartPointer<vtkPolyData> dataTermData,
	    vtkSmartPointer<vtkPolyData> smoothingTermData, ITMScene<TVoxelCanonical, TIndex>* canonicalScene,
	    Vector6i bounds, std::unordered_map<std::string, std::pair<Vector3d,Vector3d>>& componentHedgehogEndpoints) {

		TransferWarpUpdatesToVtkStructuresFunctor_WithComponents<ITMVoxelCanonical>
		        transferSmoothingVectorsToVtkStructuresFunctor(
				updatesData, dataTermData, smoothingTermData, componentHedgehogEndpoints);
		VoxelPositionTraversalWithinBounds_CPU(canonicalScene, transferSmoothingVectorsToVtkStructuresFunctor,bounds);
		updatesData->Modified();
		dataTermData->Modified();
		smoothingTermData->Modified();
	}
};


template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMSceneSliceVisualizer3D<TVoxelCanonical, TVoxelLive, TIndex>::DrawWarpUpdates() {
	std::unique_lock<std::mutex> lock(mutex);

	DrawWarpUpdateFunctor<TVoxelCanonical, TIndex, TVoxelCanonical::hasDebugInformation>::
	        run(updatesData,dataTermVectorData, smoothingTermVectorData, canonicalScene, bounds, this->componentHedgehogEndpoints);

	this->dataTermVectorData->Modified();
	this->window->Update();
	this->warpUpdatePerformed = true;

	lock.unlock();
	conditionVariable.notify_all();
}




// endregion ===========================================================================================================


template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMSceneSliceVisualizer3D<TVoxelCanonical, TVoxelLive, TIndex>::BuildFusedCanonicalFromCurrentScene() {
	std::unique_lock<std::mutex> lock(mutex);
	BuildVoxelAndHashBlockPolyDataFromScene<TVoxelCanonical, RetrieveVoxelDataBasicStaticFunctior<TVoxelCanonical>>(
			canonicalScene, fusedCanonicalSlice);

	fusedCanonicalSlice.SetUpMappersAndActors(canonicalHashBlockEdgeColor, hashBlockVizGeometrySource->GetOutputPort(),
	                                          voxelVizGeometrySource->GetOutputPort());
	this->window->AddActorToFirstLayer(fusedCanonicalSlice.voxelActor);
	this->window->AddActorToFirstLayer(fusedCanonicalSlice.hashBlockActor);
	SetVisibilityMode(VISIBILITY_FUSED_CANONICAL);
	this->fusedCanonicalBuilt = true;
	lock.unlock();
	conditionVariable.notify_all();
}


template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMSceneSliceVisualizer3D<TVoxelCanonical, TVoxelLive, TIndex>::UpdateLiveState() {
	std::unique_lock<std::mutex> lock(mutex);
	liveSlice.voxelVizData = vtkSmartPointer<vtkPolyData>::New();
	switch (liveSamplingFieldIndex) {
		case 0:
			BuildVoxelAndHashBlockPolyDataFromScene<TVoxelLive, RetrieveVoxelDataFieldIndex0StaticFunctior<TVoxelLive>>(
					liveScene, liveSlice);
			break;
		case 1:
			BuildVoxelAndHashBlockPolyDataFromScene<TVoxelLive, RetrieveVoxelDataFieldIndex1StaticFunctior<TVoxelLive>>(
					liveScene, liveSlice);
			break;
		default:
			break;
	}
	liveSlice.voxelMapper->SetInputData(liveSlice.voxelVizData);
	liveSlice.voxelMapper->Modified();
	this->liveSliceStates.push_back(liveSlice.voxelVizData);
	this->window->Update();

	visibleOptimizationStepIndex++;
	liveSamplingFieldIndex = !liveSamplingFieldIndex;

	this->liveStateUpdated = true;
	lock.unlock();
	conditionVariable.notify_all();
}


template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMSceneSliceVisualizer3D<TVoxelCanonical, TVoxelLive, TIndex>::Run() {
	std::unique_lock<std::mutex> lock(mutex);

	this->InitializeVoxels();
	this->InitializeWarps();


	threadCallback = vtkSmartPointer<ThreadInteropCommand<TVoxelCanonical, TVoxelLive, TIndex>>::New();
	threadCallback->parent = this;
	this->window->AddLoopCallback(threadCallback);
	vtkSmartPointer<ITMSceneSliceVisualizer3DInteractorStyle<TVoxelCanonical, TVoxelLive, TIndex>> style =
			vtkSmartPointer<ITMSceneSliceVisualizer3DInteractorStyle<TVoxelCanonical, TVoxelLive, TIndex>>::New();
	style->SetParent(this);

	this->window->SetInteractorStyle(style);

	window->ResetCamera();
	window->Update();

	initialized = true;
	lock.unlock();
	conditionVariable.notify_all();
	this->window->RunInteractor();
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMSceneSliceVisualizer3D<TVoxelCanonical, TVoxelLive, TIndex>::TriggerDrawWarpUpdates() {
	this->threadCallback->operation = DRAW_WARP_UPDATES;
	std::unique_lock<std::mutex> lock(mutex);
	conditionVariable.wait(lock, [this] { return this->warpUpdatePerformed; });
	this->warpUpdatePerformed = false;
}


template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMSceneSliceVisualizer3D<TVoxelCanonical, TVoxelLive, TIndex>::TriggerUpdateLiveState() {
	this->threadCallback->operation = UPDATE_LIVE_STATE;
	std::unique_lock<std::mutex> lock(mutex);
	conditionVariable.wait(lock, [this] { return this->liveStateUpdated; });
	this->liveStateUpdated = false;
}


template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMSceneSliceVisualizer3D<TVoxelCanonical, TVoxelLive, TIndex>::TriggerBuildFusedCanonical() {
	this->threadCallback->operation = BUILD_FUSED_CANONICAL;
	std::unique_lock<std::mutex> lock(mutex);
	conditionVariable.wait(lock, [this] { return this->fusedCanonicalBuilt; });
	this->fusedCanonicalBuilt = false;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMSceneSliceVisualizer3D<TVoxelCanonical, TVoxelLive, TIndex>::TriggerRebuildSlices() {
	this->threadCallback->operation = REBUILD_SLICES;
	std::unique_lock<std::mutex> lock(mutex);
	conditionVariable.wait(lock, [this] { return this->slicesRebuilt; });
	this->slicesRebuilt = false;
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMSceneSliceVisualizer3D<TVoxelCanonical, TVoxelLive, TIndex>::SetVisibilityMode(VisibilityMode mode) {
	this->visibilityMode = mode;
	switch (mode) {
		case VISIBILITY_CANONICAL_WITH_UPDATES:
			this->canonicalSlice.voxelActor->VisibilityOn();
			this->updatesActor->VisibilityOn(); 
			this->dataTermVectorActor->VisibilityOn(); this->smoothingTermVectorActor->VisibilityOn();
			this->liveSlice.voxelActor->VisibilityOff();
			this->fusedCanonicalSlice.voxelActor->VisibilityOff();
			break;
		case VISIBILITY_LIVE:
			this->canonicalSlice.voxelActor->VisibilityOff();
			this->updatesActor->VisibilityOff();
			this->dataTermVectorActor->VisibilityOff(); this->smoothingTermVectorActor->VisibilityOff();
			this->liveSlice.voxelActor->VisibilityOn();
			this->liveSlice.voxelActor->GetProperty()->SetOpacity(1.0);
			this->fusedCanonicalSlice.voxelActor->VisibilityOff();
			break;
		case VISIBILITY_LIVE_AND_CANONICAL_WITH_UPDATES:
			this->canonicalSlice.voxelActor->VisibilityOn();
			this->updatesActor->VisibilityOn();
			this->dataTermVectorActor->VisibilityOff(); this->smoothingTermVectorActor->VisibilityOff();
			this->liveSlice.voxelActor->VisibilityOn();
			this->liveSlice.voxelActor->GetProperty()->SetOpacity(0.5);
			this->fusedCanonicalSlice.voxelActor->VisibilityOff();
			break;
		case VISIBILITY_FUSED_CANONICAL:
			this->canonicalSlice.voxelActor->VisibilityOff();
			this->updatesActor->VisibilityOff();
			this->dataTermVectorActor->VisibilityOff(); this->smoothingTermVectorActor->VisibilityOff();
			this->liveSlice.voxelActor->VisibilityOff();
			this->fusedCanonicalSlice.voxelActor->VisibilityOn();
			break;
	}
	this->window->Update();
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMSceneSliceVisualizer3D<TVoxelCanonical, TVoxelLive, TIndex>::AdvanceLiveStateVizualization() {
	if (visibleOptimizationStepIndex < this->liveSliceStates.size() - 1) {
		visibleOptimizationStepIndex++;
		this->liveSlice.voxelMapper->SetInputData(liveSliceStates[visibleOptimizationStepIndex]);
		liveSlice.voxelMapper->Modified();
		window->Update();
	}
}

template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
void ITMSceneSliceVisualizer3D<TVoxelCanonical, TVoxelLive, TIndex>::RetreatLiveStateVizualization() {
	if (visibleOptimizationStepIndex > 0) {
		visibleOptimizationStepIndex--;
		this->liveSlice.voxelMapper->SetInputData(liveSliceStates[visibleOptimizationStepIndex]);
		liveSlice.voxelMapper->Modified();
		window->Update();
	}
}



