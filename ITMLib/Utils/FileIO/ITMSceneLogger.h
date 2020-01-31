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
#pragma once

//stdlib
#include <set>


//local
#include "../../Objects/Volume/VoxelVolume.h"
#include "../Collections/ITM3DNestedMapOfArrays.h"
#include "../Analytics/ITMNeighborVoxelIterationInfo.h"
#include "ITMWarpFieldLogger.h"
#include "../../Engines/EditAndCopy/CPU/EditAndCopyEngine_CPU.h"

//boost
#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;

namespace ITMLib {


/**
 * \brief Wraps the functionality of saving canonical/live scenes or scene slices for dynamic fusion along
 * with warp changes during optimization between frames.
 * \tparam TVoxel Type of voxels containing TSDF information
 * \tparam TWarp Type of voxels containing vector warp information
 * \tparam TIndex Type of index used for the voxel scenes
 */
template<typename TVoxel, typename TWarp, typename TIndex>
class ITMSceneLogger {


public:
// region === STATIC CONSTANTS ===
	static const std::string warpUpdatesFilename;
	static const std::string binaryFileExtension;
	static const std::string liveName;
	static const std::string continuousHighlightsPostfix;

	typedef EditAndCopyEngine_CPU<TVoxel, TIndex> sceneManipulationEngine;

// endregion
// region === PUBLIC ENUMS ===

	enum Mode {
		FULL_SCENE,
		SLICE
	};
//endregion
// region === PUBLIC INNER CLASSES ===

//endregion
// region === CONSTRUCTORS / DESTRUCTORS ===
	ITMSceneLogger(VoxelVolume<TVoxel, TIndex>* canonicalScene,
	               VoxelVolume<TVoxel, TIndex>* liveScene,
	               VoxelVolume<TWarp, TIndex>* warpField,
	               std::string path = "");
	ITMSceneLogger(VoxelVolume<TVoxel, TIndex>* liveScene,
	               std::string path);

	ITMSceneLogger();
	virtual ~ITMSceneLogger();

// endregion
// region === MEMBER FUNCTIONS ===

	//*** setters / preparation
	void SetScenes(
			VoxelVolume<TVoxel, TIndex>* canonicalScene,
			VoxelVolume<TVoxel, TIndex>* liveScene,
			VoxelVolume<TWarp, TIndex>* warpScene);
	void SetPath(std::string path);

	//*** getters
	std::string GetPath() const;
	int GetVoxelCount() const;
	bool GetScenesLoaded() const;
	bool GetInterestRegionsSetUp() const;
	Mode GetMode() const;
	unsigned int GetGeneralIterationCursor() const;

	ITM3DNestedMapOfArrays<ITMHighlightIterationInfo> GetHighlights() const;
	std::vector<int> GetInterestRegionHashes() const;
	const VoxelVolume<TWarp, TIndex>* GetActiveWarpScene() const;
	const VoxelVolume<TVoxel, TIndex>* GetLiveScene() const;
	bool GetIsActiveSceneASlice() const;
	std::vector<std::string> GetSliceIds() const;
	void GetActiveSceneBounds(Vector6i& bounds) const;

	//*** scene loading/saving
	bool SaveScenes();
	bool LoadScenes();
	bool SaveScenesCompact();
	bool LoadScenesCompact();

	//*** saving / loading of highlight information
	void LogHighlight(int hashId, int voxelLocalIndex, int frameNumber, ITMHighlightIterationInfo info);
	bool SaveHighlights(std::string filePostfix = "");
	void ClearHighlights();
	void PrintHighlights();
	bool LoadHighlights(bool applyFilters = true, std::string filePostfix = "");
	void FilterHighlights(int anomalyFrameCountMinimum);

	void SaveAllInterestRegionWarps();
	void SetUpInterestRegionsForLoading();
	bool BufferInterestWarpStateAtIteration(void* externalBuffer, unsigned int iterationIndex);
	bool BufferCurrentInterestWarpState(void* externalBuffer);
	bool BufferPreviousInterestWarpState(void* externalBuffer);
	bool IsHashInInterestRegion(int hashId);
	int GetTotalInterestVoxelCount();

	//** canonical warp-state saving/loading
	bool StartSavingWarpState();
	void StopSavingWarpState();
	bool StartLoadingWarpState();
	bool StartLoadingWarpState(unsigned int& frameIx);
	void StopLoadingWarpState();

	bool SaveCurrentWarpState();
	bool LoadCurrentWarpState();
	bool BufferCurrentWarpState(void* externalBuffer);
	bool BufferPreviousWarpState(void* externalBuffer);
	bool BufferWarpStateAt(void* externalBuffer, unsigned int iterationIndex);
	bool LoadPreviousWarpState();

	bool IsLoadingWarpState();

	//*** slice generation & usage***

	bool MakeSlice(const Vector3i& extremum1, const Vector3i& extremum2,
	               std::string& identifier);
	bool MakeSlice(const Vector3i& extremum1, const Vector3i& extremum2);
	bool SliceExistsInMemory(const std::string& sliceIdentifier) const;
	bool SliceExistsOnDisk(const Vector3i& extremum1,
	                       const Vector3i& extremum2) const;

	bool SliceExistsOnDisk(const std::string& sliceIdentifier) const;
	bool LoadSlice(const std::string& sliceIdentifier);
	bool SwitchActiveScene(
			std::string sliceIdentifier = ITMWarpFieldLogger<TVoxel, TIndex>::fullSceneSliceIdentifier);


//endregion
	std::vector<std::string> LoadAllSlices();
private:
// region === MEMBER FUNCTIONS ===
	void SaveSliceWarp(const Vector6i& voxelRange,
	                   const boost::filesystem::path& path);
	ITM3DNestedMapOfArrays <ITMHighlightIterationInfo>
	MakeSliceHighlights(const Vector6i& bounds);
	bool CheckPath();

// endregion
// region === MEMBER VARIABLES ===

// *** root folder
	fs::path path;

// *** subpaths
	fs::path livePath;

// *** scene structures ***
	ITMWarpFieldLogger<TWarp, TIndex>* fullWarpLogger;
	ITMWarpFieldLogger<TWarp, TIndex>* activeWarpLogger;
	VoxelVolume<TVoxel, TIndex>* liveScene;
	VoxelVolume<TVoxel, TIndex>* canonicalScene;

// *** scene meta-information + reading/writing
	std::map<std::string, ITMWarpFieldLogger<TWarp, TIndex>*> slices;

// *** state ***
	Mode mode;



// endregion

};


}//namespace ITMLib

