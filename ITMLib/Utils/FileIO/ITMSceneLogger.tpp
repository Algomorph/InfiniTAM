//  ================================================================
//  Created by Gregory Kramida on 12/20/17.
//  Copyright (c) 2017-2025 Gregory Kramida
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
#include "ITMSceneLogger.h"

#include <regex>

//boost
#include <boost/filesystem.hpp>

//local
#include "ITMSceneLogger_SceneSlice.tpp"
#include "../../Engines/DepthFusion/DepthFusionEngineFactory.h"
#include "../../Engines/VolumeFileIO/VolumeFileIOEngine.h"
#include "../Analytics/SceneStatisticsCalculator/CPU/ITMSceneStatisticsCalculator_CPU.h"



namespace fs = boost::filesystem;

using namespace ITMLib;

// region ================================= CONSTANT DEFINITIONS =======================================================

template<typename TVoxel, typename TWarp, typename TIndex>
const std::string ITMSceneLogger<TVoxel, TWarp, TIndex>::warpUpdatesFilename =
		ITMWarpFieldLogger<TWarp, TIndex>::warpUpdatesFilename;
template<typename TVoxel, typename TWarp, typename TIndex>
const std::string ITMSceneLogger<TVoxel, TWarp, TIndex>::binaryFileExtension =
		ITMWarpFieldLogger<TWarp, TIndex>::binaryFileExtension;
template<typename TVoxel, typename TWarp, typename TIndex>
const std::string ITMSceneLogger<TVoxel, TWarp, TIndex>::liveName = "live";
template<typename TVoxel, typename TWarp, typename TIndex>
const std::string ITMSceneLogger<TVoxel, TWarp, TIndex>::continuousHighlightsPostfix =
		ITMWarpFieldLogger<TWarp, TIndex>::continuousHighlightsPostfix;
//endregion
// region ================================= CONSTRUCTORS & DESTRUCTORS =================================================

/**
 * \brief Constructor: build a logger with the given path and scenes.
 * \tparam TVoxelCanonical
 * \tparam TVoxelLive
 * \tparam TIndex
 * \param path
 * \param canonicalScene
 * \param liveScene
 */
template<typename TVoxel, typename TWarp, typename TIndex>
ITMSceneLogger<TVoxel, TWarp, TIndex>::ITMSceneLogger(
		VoxelVolume<TVoxel, TIndex>* canonicalScene,
		VoxelVolume<TVoxel, TIndex>* liveScene,
		VoxelVolume<TWarp, TIndex>* warpField,
		std::string path) :
		fullWarpLogger(new ITMWarpFieldLogger<TWarp, TIndex>(warpField, path)),
		activeWarpLogger(fullWarpLogger),
		canonicalScene(canonicalScene),
		liveScene(liveScene),

		mode(FULL_SCENE) {
	if (path != "") {
		SetPath(path);
	}
}

template<typename TVoxel, typename TWarp, typename TIndex>
ITMSceneLogger<TVoxel, TWarp, TIndex>::ITMSceneLogger(VoxelVolume<TVoxel, TIndex>* liveScene,
                                                      std::string path):
		fullWarpLogger(nullptr), activeWarpLogger(nullptr),
		liveScene(liveScene), mode(SLICE) {
	SetPath(path);
	std::vector<std::string> ids = LoadAllSlices();
	if (slices.empty()) {
		DIEWITHEXCEPTION_REPORTLOCATION("Could not find or load any slices in the provided directory.");
	}
	activeWarpLogger = slices[ids[0]];
}


template<typename TVoxel, typename TWarp, typename TIndex>
ITMSceneLogger<TVoxel, TWarp, TIndex>::~ITMSceneLogger() {
	for (auto idAndSliceIterator = this->slices.begin();
	     idAndSliceIterator != this->slices.end(); ++idAndSliceIterator) {
		delete idAndSliceIterator->second;
	}
	delete fullWarpLogger;
}
//endregion
// region ================================= GETTERS, SETTERS, PRINTERS =================================================

template<typename TVoxel, typename TWarp, typename TIndex>
int ITMSceneLogger<TVoxel, TWarp, TIndex>::GetVoxelCount() const {
	return activeWarpLogger->GetVoxelCount();
}

/**
 * \brief Retreive minimum and maximum of currently active scene
 * \param bounds [out] minimum point
 * \param maxPoint [out] maximum point
 */
template<typename TVoxel, typename TWarp, typename TIndex>
void ITMSceneLogger<TVoxel, TWarp, TIndex>::GetActiveSceneBounds(Vector6i& bounds) const {
	if (this->activeWarpLogger) {
		if (this->activeWarpLogger->isSlice) {
			bounds = activeWarpLogger->bounds;
		} else {
			ITMSceneStatisticsCalculator<TWarp, TIndex, MEMORYDEVICE_CPU>::Instance().ComputeVoxelBounds(
					this->activeWarpLogger->warpField);
		}
	}
}

template<typename TVoxel, typename TWarp, typename TIndex>
bool ITMSceneLogger<TVoxel, TWarp, TIndex>::GetScenesLoaded() const {
	return activeWarpLogger->Loaded();
}

template<typename TVoxel, typename TWarp, typename TIndex>
void ITMSceneLogger<TVoxel, TWarp, TIndex>::SetScenes(
		VoxelVolume<TVoxel, TIndex>* canonicalScene,
		VoxelVolume<TVoxel, TIndex>* liveScene,
		VoxelVolume<TWarp, TIndex>* warpField) {
	this->liveScene = liveScene;
	this->canonicalScene = canonicalScene;
	this->activeWarpLogger->warpField = warpField;
}

template<typename TVoxel, typename TWarp, typename TIndex>
std::string ITMSceneLogger<TVoxel, TWarp, TIndex>::GetPath() const {
	return this->path.string();
}

template<typename TVoxel, typename TWarp, typename TIndex>
unsigned int ITMSceneLogger<TVoxel, TWarp, TIndex>::GetGeneralIterationCursor() const {
	return this->activeWarpLogger->GetIterationCursor();
}

template<typename TVoxel, typename TWarp, typename TIndex>
ITM3DNestedMapOfArrays<ITMHighlightIterationInfo>
ITMSceneLogger<TVoxel, TWarp, TIndex>::GetHighlights() const {
	return ITM3DNestedMapOfArrays<ITMHighlightIterationInfo>(activeWarpLogger->highlights);
}

template<typename TVoxel, typename TWarp, typename TIndex>
void ITMSceneLogger<TVoxel, TWarp, TIndex>::PrintHighlights() {
	std::cout << "*** Highlights ***" << std::endl;
	std::cout << this->activeWarpLogger->highlights << std::endl;
}

template<typename TVoxel, typename TWarp, typename TIndex>
bool ITMSceneLogger<TVoxel, TWarp, TIndex>::CheckPath() {
	if (!fs::is_directory(this->path)) {
		std::cout << "The directory '" << path << "' was not found.";
		return false;
	}
	return true;
}

/**
 * \brief Set the path to save everything to or load everything from. If a directory doesn't exist at that path, one
 * will be created.
 * \param path the new path.
 */
template<typename TVoxel, typename TWarp, typename TIndex>
void ITMSceneLogger<TVoxel, TWarp, TIndex>::SetPath(std::string path) {
	if (this->path != "") {
		//clean up
		slices.clear();
	}
	this->path = path;
	if (!fs::create_directories(this->path) && !fs::is_directory(this->path)) {
		DIEWITHEXCEPTION_REPORTLOCATION(std::string("Could not create the directory '") + path + "'. Exiting.");
	}
	if (activeWarpLogger) {
		this->activeWarpLogger->SetPath(path);
	}
	this->livePath = this->path / liveName;

}

template<typename TVoxel, typename TWarp, typename TIndex>
const VoxelVolume<TWarp, TIndex>* ITMSceneLogger<TVoxel, TWarp, TIndex>::GetActiveWarpScene() const {
	return this->activeWarpLogger->warpField;
}

template<typename TVoxel, typename TWarp, typename TIndex>
const VoxelVolume<TVoxel, TIndex>* ITMSceneLogger<TVoxel, TWarp, TIndex>::GetLiveScene() const {
	return this->liveScene;
}

template<typename TVoxel, typename TWarp, typename TIndex>
bool ITMSceneLogger<TVoxel, TWarp, TIndex>::GetIsActiveSceneASlice() const {
	return this->activeWarpLogger->isSlice;
}

template<typename TVoxel, typename TWarp, typename TIndex>
std::vector<std::string> ITMSceneLogger<TVoxel, TWarp, TIndex>::GetSliceIds() const {
	std::vector<std::string> ids;
	for (auto iterator = slices.begin(); iterator != slices.end(); iterator++) {
		ids.push_back(iterator->first);
	}
	return ids;
}
//endregion
// region ================================= SCENE SAVING / LOADING =====================================================

template<typename TVoxel, typename TWarp, typename TIndex>
bool ITMSceneLogger<TVoxel, TWarp, TIndex>::SaveScenes() {
	if (!CheckPath()) {
		return false;
	}
	std::cout << "Saving scenes for current frame (this might take awhile)..." << std::endl;
	std::cout.flush();
	liveScene->SaveToDirectory(livePath.string());
	activeWarpLogger->Save();
	std::cout << "Scenes saved." << std::endl;
	return true;
}


template<typename TVoxel, typename TWarp, typename TIndex>
bool ITMSceneLogger<TVoxel, TWarp, TIndex>::LoadScenes() {
	if (!CheckPath()) {
		return false;
	}
	std::cout << "Loading scenes for current frame (this might take awhile)..." << std::endl;
	std::cout.flush();
	liveScene->LoadFromDirectory(livePath.string());
	activeWarpLogger->Load();
	std::cout << "Scenes loaded." << std::endl;
	return true;
}

template<typename TVoxel, typename TWarp, typename TIndex>
bool ITMSceneLogger<TVoxel, TWarp, TIndex>::SaveScenesCompact() {
	if (!liveScene || activeWarpLogger->Empty()) {
		std::cerr << "At least one of the two scenes, canonical/live, was not set to an actual scene. "
		          << __FILE__ << ":" << __LINE__ << std::endl;
	}
	if (!fs::is_directory(this->path)) {
		std::cout << "The directory '" << path << "' was not found.";
		return false;
	}
	std::cout << "Saving scenes for current frame (this might take awhile)..." << std::endl;
	std::cout.flush();
	VolumeFileIOEngine<TVoxel, TIndex>::SaveToDirectoryCompact(liveScene, livePath.string());
	activeWarpLogger->SaveCompact();
	std::cout << "Scenes saved." << std::endl;
	return true;
};

template<typename TVoxel, typename TWarp, typename TIndex>
bool ITMSceneLogger<TVoxel, TWarp, TIndex>::LoadScenesCompact() {
	if (!CheckPath()) {
		return false;
	}
	std::cout << "Loading scenes for current frame (this might take awhile)..." << std::endl;
	std::cout.flush();
	liveScene->Reset();
	VolumeFileIOEngine<TVoxel, TIndex>::LoadFromDirectoryCompact(liveScene, livePath.string());
	if (!activeWarpLogger->isSlice || !activeWarpLogger->sliceLoaded) {
		activeWarpLogger->LoadCompact();
	}
	std::cout << "Scenes loaded." << std::endl;
	return true;
}
//endregion
// region ================================= WARP STATE ONLINE SAVING / LOADING / BUFFERING =============================

template<typename TVoxel, typename TWarp, typename TIndex>
bool ITMSceneLogger<TVoxel, TWarp, TIndex>::StartSavingWarpState() {
	return activeWarpLogger->StartSavingWarpState();
}

template<typename TVoxel, typename TWarp, typename TIndex>
void ITMSceneLogger<TVoxel, TWarp, TIndex>::StopSavingWarpState() {
	activeWarpLogger->StopSavingWarpState();
}

template<typename TVoxel, typename TWarp, typename TIndex>
bool ITMSceneLogger<TVoxel, TWarp, TIndex>::SaveCurrentWarpState() {
	return activeWarpLogger->SaveCurrentWarpState();
}

template<typename TVoxel, typename TWarp, typename TIndex>
bool ITMSceneLogger<TVoxel, TWarp, TIndex>::StartLoadingWarpState() {
	return activeWarpLogger->StartLoadingWarpState();
}

template<typename TVoxel, typename TWarp, typename TIndex>
bool ITMSceneLogger<TVoxel, TWarp, TIndex>::StartLoadingWarpState(unsigned int& frameIx) {
	return activeWarpLogger->StartLoadingWarpState(frameIx);
}

template<typename TVoxel, typename TWarp, typename TIndex>
void ITMSceneLogger<TVoxel, TWarp, TIndex>::StopLoadingWarpState() {
	activeWarpLogger->StopLoadingWarpState();
}

/**
 * \brief Transfers the warp state from the warp file to the scene, imitating the .warp and .gradient fields after
 * the current iteration.
 * \return True on success, false otherwise
 */
template<typename TVoxel, typename TWarp, typename TIndex>
bool ITMSceneLogger<TVoxel, TWarp, TIndex>::LoadCurrentWarpState() {
	return activeWarpLogger->LoadCurrentWarpState();
}

template<typename TVoxel, typename TWarp, typename TIndex>
bool ITMSceneLogger<TVoxel, TWarp, TIndex>::LoadPreviousWarpState() {
	return activeWarpLogger->LoadPreviousWarpState();
}

template<typename TVoxel, typename TWarp, typename TIndex>
bool ITMSceneLogger<TVoxel, TWarp, TIndex>::IsLoadingWarpState() {
	return activeWarpLogger->IsLoadingWarpState();
}

template<typename TVoxel, typename TWarp, typename TIndex>
bool ITMSceneLogger<TVoxel, TWarp, TIndex>::BufferCurrentWarpState(void* externalBuffer) {
	return activeWarpLogger->BufferCurrentWarpState(externalBuffer);
}

template<typename TVoxel, typename TWarp, typename TIndex>
bool ITMSceneLogger<TVoxel, TWarp, TIndex>::BufferPreviousWarpState(void* externalBuffer) {
	return activeWarpLogger->BufferPreviousWarpState(externalBuffer);
}


template<typename TVoxel, typename TWarp, typename TIndex>
bool ITMSceneLogger<TVoxel, TWarp, TIndex>::BufferWarpStateAt(void* externalBuffer,
                                                              unsigned int iterationIndex) {
	return activeWarpLogger->BufferWarpStateAt(externalBuffer, iterationIndex);
}
//endregion
// region ================================= HIGHLIGHTS =================================================================

template<typename TVoxel, typename TWarp, typename TIndex>
void ITMSceneLogger<TVoxel, TWarp, TIndex>::LogHighlight(
		int hashId, int voxelLocalIndex, int frameNumber, ITMHighlightIterationInfo info) {
	activeWarpLogger->highlights.InsertOrdered(hashId, voxelLocalIndex, frameNumber, info);
}

template<typename TVoxel, typename TWarp, typename TIndex>
bool ITMSceneLogger<TVoxel, TWarp, TIndex>::SaveHighlights(std::string filePostfix) {
	return activeWarpLogger->SaveHighlights(filePostfix);
}

template<typename TVoxel, typename TWarp, typename TIndex>
bool ITMSceneLogger<TVoxel, TWarp, TIndex>::LoadHighlights(bool applyFilters, std::string filePostfix) {
	return activeWarpLogger->LoadHighlights(applyFilters, filePostfix);
}

template<typename TVoxel, typename TWarp, typename TIndex>
void ITMSceneLogger<TVoxel, TWarp, TIndex>::FilterHighlights(int anomalyFrameCountMinimum) {
	this->activeWarpLogger->FilterHighlights(anomalyFrameCountMinimum);
}

template<typename TVoxel, typename TWarp, typename TIndex>
void ITMSceneLogger<TVoxel, TWarp, TIndex>::ClearHighlights() {
	this->activeWarpLogger->highlights.Clear();
}

template<typename TVoxel, typename TWarp, typename TIndex>
ITMSceneLogger<TVoxel, TWarp, TIndex>::ITMSceneLogger() {

}

//endregion