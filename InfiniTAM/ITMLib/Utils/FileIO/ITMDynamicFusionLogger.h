//  ================================================================
//  Created by Gregory Kramida on 5/25/18.
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

#include "../../Objects/Scene/ITMScene.h"
#include "ITMSceneLogger.h"
#include "../Visualization/ITMSceneSliceVisualizer1D.h"
#include "../Visualization/ITMSceneSliceVisualizer2D.h"
#include "../Visualization/ITMSceneSliceVisualizer3D.h"
#include "../Visualization/ITMSceneTrackingEnergyPlotter.h"

namespace ITMLib {


template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
class ITMDynamicFusionLogger {
public:
// where to save the images within the output directory
	static const std::string iterationFramesFolderName;
	static const std::string liveIterationFramesFolderName;
	static const std::string canonicalSceneRasterizedFolderName;
	static const std::string liveSceneRasterizedFolderName;

	static ITMDynamicFusionLogger& Instance(){
		static ITMDynamicFusionLogger instance;
		return instance;
	}

// region ============================= SETTERS & SWITCHES =============================================================

	void SetScenes(ITMScene<TVoxelCanonical, TIndex>* canonicalScene,ITMScene<TVoxelLive, TIndex>* liveScene);
	void SetOutputDirectory(std::string outputDirectory);
	void SetFocusCoordinates(Vector3i focusCoordinates);
	void SetPlaneFor2Dand3DSlices(Plane plane);
	void Set3DSliceInPlaneRadius(unsigned int _3dSliceInPlaneRadius);
	void Set3DSliceOutOfPlaneRadius(unsigned int _3dSliceOutOfPlaneRadius);
	void SetShutdownRequestedFlagLocation(bool* flag);

	void RequestAppShutdown();

	void TurnRecordingLiveSceneAs2DSlicesOn();
	void TurnRecordingLiveSceneAs2DSlicesOff();
	void ToggleRecordingLiveSceneAs2DSlices();
	void TurnRecordingCanonicalSceneAs2DSlicesOn();
	void TurnRecordingCanonicalSceneAs2DSlicesOff();
	void TurnRecordingScene1DSlicesWithUpdatesOn();
	void TurnRecordingScene1DSlicesWithUpdatesOff();
	void TurnRecordingScene2DSlicesWithUpdatesOn();
	void TurnRecordingScene2DSlicesWithUpdatesOff();
	void TurnRecordingScene3DSlicesWithUpdatesOn();
	void TurnRecordingScene3DSlicesWithUpdatesOff();
	void ToggleRecordingScene2DSlicesWithUpdates();
	void TurnRecording3DSceneAndWarpProgressionOn();
	void TurnRecording3DSceneAndWarpProgressionOff();
	void ToggleRecording3DSceneAndWarpProgression();
	void TurnRecordingEnergiesToFilesOn();
	void TurnRecordingEnergiesToFilesOff();
	void TurnPlottingEnergiesOn();
	void TurnPlottingEnergiesOff();

// endregion ===========================================================================================================
// region ============================= GETTERS ========================================================================

	std::string GetOutputDirectory() const;

	bool IsRecordingLiveSceneAs2DSlices() const;
	bool IsRecordingCanonicalSceneAs2DSlices() const;
	bool IsRecordingScene1DSlicesWithUpdates() const;
	bool IsRecordingScene2DSlicesWithUpdates() const;
	bool IsRecordingScene3DSlicesWithUpdates() const;
	bool IsRecording3DSceneAndWarpProgression() const;
	bool IsRecordingEnergiesToFile() const;
	bool IsPlottingEnergies() const;

// endregion ===========================================================================================================
	void InitializeFrameRecording();


	void SaveWarpSlices(int iteration);
	void UpdateSmoothingVectors();
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

	ITMDynamicFusionLogger(ITMDynamicFusionLogger const&) = delete;
	void operator=(ITMDynamicFusionLogger const&) = delete;


private:
	ITMDynamicFusionLogger();
	~ITMDynamicFusionLogger();

	void InitializeWarp2DSliceRecording(ITMScene<TVoxelCanonical, TIndex>* canonicalScene,
	                                    ITMScene<TVoxelLive, TIndex>* sourceLiveScene);
	std::string GetOutputDirectoryFor2DSceneSlicesWithWarps() const;
	std::string GetOutputDirectoryFor2DLiveSceneSliceProgression() const;
	std::string GetOutputDirectoryPrefixForLiveSceneAsSlices() const;
	std::string GetOutputDirectoryPrefixForCanonicalSceneAsSlices() const;
	void MakeOrClearOutputDirectoriesFor2DSceneSlices() const;

	// various loggers & visualizers
	std::unique_ptr<ITMSceneSliceVisualizer1D> scene1DSliceVisualizer;
	std::unique_ptr<ITMSceneSliceVisualizer2D<TVoxelCanonical, TVoxelLive, TIndex>> scene2DSliceVisualizer;
	std::unique_ptr<ITMSceneSliceVisualizer3D<TVoxelCanonical, TVoxelLive, TIndex>> scene3DSliceVisualizer;
	std::unique_ptr<ITMSceneTrackingEnergyPlotter> energyPlotter;
	ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>* scene3DLogger = nullptr;

	// internal references to the scenes
	ITMScene<TVoxelCanonical, TIndex>* canonicalScene = nullptr;
	ITMScene<TVoxelLive, TIndex>* liveScene = nullptr;

	std::ofstream energyStatisticsFile;

	// templates //TODO outsource to ITMScene2DSliceLogger
	cv::Mat blank;
	cv::Mat liveImgTemplate;

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
	Vector3i focusCoordinates;
	int _3dSliceInPlaneRadius;
	unsigned int _3dSliceOutOfPlaneRadius;
	const int focusSliceRadius;
};

} //namespace InfiniTAM