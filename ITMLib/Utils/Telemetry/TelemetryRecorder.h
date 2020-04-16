//  ================================================================
//  Created by Gregory Kramida on 5/25/18.
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

#include "../../Objects/Volume/VoxelVolume.h"
#include "VolumeSequenceRecorder.h"
#include "../Visualization/SceneSliceVisualizer2D.h"
#include "../Geometry/CardinalAxesAndPlanes.h"
#ifdef WITH_VTK
#include "../Visualization/SceneSliceVisualizer1D.h"
#include "../Visualization/SceneSliceVisualizer3D.h"
#include "../Visualization/SceneTrackingEnergyPlotter.h"
#endif


namespace ITMLib {

//TODO: adapt to record warpField properly (+test)
//TODO: adapt to live-scene-pair structure; the live scene is now split into two that are being ping-ponged (+test)

class TelemetryRecorder_Interface{
public:

// region ============================= SETTERS & SWITCHES =============================================================
	virtual void SetOutputDirectory(std::string outputDirectory) = 0;
	virtual void SetFocusCoordinates(Vector3i focusCoordinates) = 0;
	virtual void SetPlaneFor2Dand3DSlices(Plane plane) = 0;
	virtual void Set3DSliceInPlaneRadius(unsigned int _3dSliceInPlaneRadius) = 0;
	virtual void Set3DSliceOutOfPlaneRadius(unsigned int _3dSliceOutOfPlaneRadius) = 0;
	virtual void SetShutdownRequestedFlagLocation(bool* flag) = 0;


	virtual void RequestAppShutdown() = 0;

	virtual void TurnRecordingLiveSceneAs2DSlicesOn() = 0;
	virtual void TurnRecordingLiveSceneAs2DSlicesOff() = 0;
	virtual void ToggleRecordingLiveSceneAs2DSlices() = 0;
	virtual void TurnRecordingCanonicalSceneAs2DSlicesOn() = 0;
	virtual void TurnRecordingCanonicalSceneAs2DSlicesOff() = 0;
	virtual void TurnRecordingScene1DSlicesWithUpdatesOn() = 0;
	virtual void TurnRecordingScene1DSlicesWithUpdatesOff() = 0;
	virtual void TurnRecordingScene2DSlicesWithUpdatesOn() = 0;
	virtual void TurnRecordingScene2DSlicesWithUpdatesOff() = 0;
	virtual void TurnRecordingScene3DSlicesWithUpdatesOn() = 0;
	virtual void TurnRecordingScene3DSlicesWithUpdatesOff() = 0;
	virtual void ToggleRecordingScene2DSlicesWithUpdates() = 0;
	virtual void TurnRecording3DSceneAndWarpProgressionOn() = 0;
	virtual void TurnRecording3DSceneAndWarpProgressionOff() = 0;
	virtual void ToggleRecording3DSceneAndWarpProgression() = 0;
	virtual void TurnRecordingEnergiesToFilesOn() = 0;
	virtual void TurnRecordingEnergiesToFilesOff() = 0;
	virtual void TurnPlottingEnergiesOn() = 0;
	virtual void TurnPlottingEnergiesOff() = 0;
	
//endregion ============================================================================================================
// region ============================= GETTERS ========================================================================

	virtual std::string GetOutputDirectory() const = 0;

	virtual bool IsRecordingLiveSceneAs2DSlices() const = 0;
	virtual bool IsRecordingCanonicalSceneAs2DSlices() const = 0;
	virtual bool IsRecordingScene1DSlicesWithUpdates() const = 0;
	virtual bool IsRecordingScene2DSlicesWithUpdates() const = 0;
	virtual bool IsRecordingScene3DSlicesWithUpdates() const = 0;
	virtual bool IsRecording3DSceneAndWarpProgression() const = 0;
	virtual bool IsRecordingEnergiesToFile() const = 0;
	virtual bool IsPlottingEnergies() const = 0;
	virtual bool NeedsFramewiseOutputFolder() const = 0;

};

template<typename TVoxel, typename TWarp, typename TIndex>
class TelemetryRecorder : public TelemetryRecorder_Interface {
public:
// where to save the images within the output directory
	static const std::string iterationFramesFolderName;
	static const std::string liveIterationFramesFolderName;
	static const std::string canonicalSceneRasterizedFolderName;
	static const std::string liveSceneRasterizedFolderName;

	static TelemetryRecorder& Instance(){
		static TelemetryRecorder instance;
		return instance;
	}

// region ============================= SETTERS & SWITCHES =============================================================

	void SetScenes(VoxelVolume<TVoxel, TIndex>* canonicalScene, VoxelVolume<TVoxel, TIndex>* liveScene,
	               VoxelVolume<TWarp, TIndex>* warpField);
	void SetOutputDirectory(std::string outputDirectory) override;
	void SetFocusCoordinates(Vector3i focusCoordinates) override;
	void SetPlaneFor2Dand3DSlices(Plane plane) override;
	void Set3DSliceInPlaneRadius(unsigned int _3dSliceInPlaneRadius) override;
	void Set3DSliceOutOfPlaneRadius(unsigned int _3dSliceOutOfPlaneRadius) override;
	void SetShutdownRequestedFlagLocation(bool* flag) override;

	void RequestAppShutdown() override;

	void TurnRecordingLiveSceneAs2DSlicesOn() override;
	void TurnRecordingLiveSceneAs2DSlicesOff() override;
	void ToggleRecordingLiveSceneAs2DSlices() override;
	void TurnRecordingCanonicalSceneAs2DSlicesOn() override;
	void TurnRecordingCanonicalSceneAs2DSlicesOff() override;
	void TurnRecordingScene1DSlicesWithUpdatesOn() override;
	void TurnRecordingScene1DSlicesWithUpdatesOff() override;
	void TurnRecordingScene2DSlicesWithUpdatesOn() override;
	void TurnRecordingScene2DSlicesWithUpdatesOff() override;
	void TurnRecordingScene3DSlicesWithUpdatesOn() override;
	void TurnRecordingScene3DSlicesWithUpdatesOff() override;
	void ToggleRecordingScene2DSlicesWithUpdates() override;
	void TurnRecording3DSceneAndWarpProgressionOn() override;
	void TurnRecording3DSceneAndWarpProgressionOff() override;
	void ToggleRecording3DSceneAndWarpProgression() override;
	void TurnRecordingEnergiesToFilesOn() override;
	void TurnRecordingEnergiesToFilesOff() override;
	void TurnPlottingEnergiesOn() override;
	void TurnPlottingEnergiesOff() override;

// endregion ===========================================================================================================
// region ============================= GETTERS ========================================================================

	std::string GetOutputDirectory() const override;

	bool IsRecordingLiveSceneAs2DSlices() const override;
	bool IsRecordingCanonicalSceneAs2DSlices() const override;
	bool IsRecordingScene1DSlicesWithUpdates() const override;
	bool IsRecordingScene2DSlicesWithUpdates() const override;
	bool IsRecordingScene3DSlicesWithUpdates() const override;
	bool IsRecording3DSceneAndWarpProgression() const override;
	bool IsRecordingEnergiesToFile() const override;
	bool IsPlottingEnergies() const override;
	bool NeedsFramewiseOutputFolder() const override;

	void InitializeFrameRecording();



	void SaveWarpSlices(int iteration);
	void SaveWarps();
	void FinalizeFrameRecording();
	void RecordAndPlotEnergies(double totalDataEnergy,
	                           double totalLevelSetEnergy,
	                           double totalKillingEnergy,
	                           double totalSmoothnessEnergy,
	                           double totalEnergy);
	bool IsRecordingWarp2DSlices();
	bool IsRecordingWarps();
	void LogHighlight(int hash, int locId, ITMHighlightIterationInfo info);

	TelemetryRecorder(TelemetryRecorder const&) = delete;
	void operator=(TelemetryRecorder const&) = delete;


private:
	TelemetryRecorder();
	~TelemetryRecorder();

	void InitializeWarp2DSliceRecording(VoxelVolume<TVoxel, TIndex>* canonicalScene,
	                                    VoxelVolume<TVoxel, TIndex>* sourceLiveScene);
	std::string GetOutputDirectoryFor2DSceneSlicesWithWarps() const;
	std::string GetOutputDirectoryFor2DLiveSceneSliceProgression() const;
	std::string GetOutputDirectoryPrefixForLiveSceneAsSlices() const;
	std::string GetOutputDirectoryPrefixForCanonicalSceneAsSlices() const;
	void MakeOrClearOutputDirectoriesFor2DSceneSlices() const;

	// various loggers & visualizers
#ifdef WITH_OPENCV
	std::unique_ptr<SceneSliceVisualizer2D<TVoxel, TWarp, TIndex>> scene2DSliceVisualizer;
#endif
#ifdef WITH_VTK
	std::unique_ptr<SceneSliceVisualizer1D> scene1DSliceVisualizer;
	std::unique_ptr<SceneSliceVisualizer3D<TVoxel, TWarp, TIndex>> scene3DSliceVisualizer;
	std::unique_ptr<ITMSceneTrackingEnergyPlotter> energyPlotter;
#endif
	VolumeSequenceRecorder<TVoxel, TWarp, TIndex>* scene3DLogger = nullptr;

	// internal references to the scenes
	VoxelVolume<TVoxel, TIndex>* canonicalScene = nullptr;
	VoxelVolume<TVoxel, TIndex>* liveScene = nullptr;
	VoxelVolume<TWarp, TIndex>* warpField = nullptr;

	std::ofstream energyStatisticsFile;

	// templates //TODO outsource to ITMScene2DSliceLogger
#ifdef WITH_OPENCV
	cv::Mat blank;
	cv::Mat liveImgTemplate;
#endif

	// state flags
	bool recordingLiveSceneAs2DSlices = false;
	bool recordingCanonicalSceneAs2DSlices = false;
	bool recordingScene1DSlicesWithUpdates = false;
	bool recordingScene2DSlicesWithUpdates = false;
	bool recordingScene3DSlicesWithUpdates = false;
	bool recording3DSceneAndWarpProgression = false;
	bool recordingEnergiesToFile = true;
	bool plottingEnergies = false;
	bool hasFocusCoordinates = false;
	bool* shutdownRequestedFlag = nullptr;

	// configuration
	Plane planeFor2Dand3DSlices = PLANE_XY;
	std::string outputDirectory;
	Vector3i focus_coordinates;
	int _3dSliceInPlaneRadius;
	unsigned int _3dSliceOutOfPlaneRadius;
	const int focusSliceRadius;
};

} //namespace InfiniTAM