//  ================================================================
//  Created by Gregory Kramida on 3/28/18.
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

//boost
#include <boost/filesystem.hpp>

//local
#include "../../Objects/Volume/VoxelVolume.h"
#include "../Collections/ITM3DNestedMapOfArrays.h"
#include "../Analytics/ITMNeighborVoxelIterationInfo.h"

namespace fs = boost::filesystem;

namespace ITMLib {

template<typename TVoxel, typename TWarp, typename TIndex>
class ITMSceneLogger;

/**
 * \brief An internal wrapper logger for a scene consistent of dynamic voxels with warps.
 * \details This data structure is necessary to be able to switch
 * between full-scene and different slice representations of the same scene, abstracting this away from the data
 * consumer.
 * \tparam TVoxel Type of voxel. Needs to have warp information.
 * \tparam TIndex Type of voxel index.
 */
template<typename TWarp, typename TIndex>
class ITMWarpFieldLogger {
	template<typename TVoxelLogger, typename TWarpLogger, typename TIndexLogger>
	friend
	class ITMSceneLogger;
public:
	// region ================================ STATIC CONSTANTS ========================================================
	static const size_t warpByteSize;
	static const size_t warpFloatSize;
	static const size_t updateByteSize;
	static const size_t updateFloatSize;
	static const size_t warpAndUpdateByteSize;
	static const size_t warpAndUpdateFloatSize;

	static const std::string fullSceneSliceIdentifier;
	static const std::string binaryFileExtension;
	static const std::string textFileExtension;
	static const std::string canonicalName;
	static const std::string warpUpdatesFilename;

	static const std::string sliceFolderPrefix;
	static const std::string sliceScenePrefix;
	static const std::string continuousHighlightsPostfix;
	//endregion
	// region ================================ STATIC FUNCTIONS ========================================================

	static void ExtractBoundsFromSliceStringIdentifier(
			const std::string& stringContainingIdentifier, Vector6i& bounds);
	static std::string GenerateSliceStringIdentifier(const Vector6i& bounds);
	static boost::filesystem::path GenerateSliceFolderPath(const fs::path& fullScenePath, const Vector6i& bounds);
	static boost::filesystem::path GenerateSliceFolderPath(const fs::path& fullScenePath,
	                                                       const std::string& sliceIdentifier);
	static std::string GenerateSliceSceneFilename_UpToPostfix(const fs::path& fullScenePath,
	                                                          const Vector6i& bounds);
	static std::string GenerateSliceSceneFilename_UpToPostfix(const fs::path& fullScenePath,
	                                                          const std::string& sliceIdentifier);
	static std::string GenerateSliceSceneFilename_Full(const fs::path& fullScenePath,
	                                                   const Vector6i& bounds);
	static std::string GenerateSliceSceneFilename_Full(const fs::path& fullScenePath,
	                                                   const std::string& sliceIdentifier);
	static std::string GenerateSliceWarpFilename(const fs::path& rootScenePath, const Vector6i& bounds);
	static std::string GenerateSliceWarpFilename(const fs::path& rootScenePath,
	                                             const std::string& sliceIdentifier);

	// endregion
	// region ================================ CONSTRUCTORS & DESTRUCTORS ==============================================

	explicit ITMWarpFieldLogger(VoxelVolume <TWarp, TIndex>* warpField, boost::filesystem::path path);
	explicit ITMWarpFieldLogger(const Vector6i& bounds, boost::filesystem::path fullScenePath);
	~ITMWarpFieldLogger();

	// endregion
	// region ================================ MEMBER FUNCTIONS ========================================================

	//*** getters / setters ***
	unsigned int GetIterationCursor() const;
	bool SetIterationCursor(unsigned int iterationIndex);
	int GetVoxelCount() const;
	bool Empty() const;
	bool Loaded() const;
	void Load();
	std::string GetSliceIdentifier() const;
	const VoxelVolume <TWarp, TIndex>* GetScene() const;

	//*** scene saving / loading ***
	void Save();
	void SaveCompact();
	void LoadCompact();

	//** highlights saving / loading **
	bool SaveHighlights(std::string filePostfix = "");
	bool LoadHighlights(bool applyFilters = true, std::string filePostfix = "");
	void FilterHighlights(int anomalyFrameCountMinimum);

	//*** warp loading / saving / buffering ***
	bool StartSavingWarpState();
	void StopSavingWarpState();
	bool SaveCurrentWarpState();
	bool LoadPreviousWarpState();

	bool BufferWarpStateAt(void* externalBuffer, unsigned int iterationIndex);
	bool BufferPreviousWarpState(void* externalBuffer);
	bool BufferCurrentWarpState(void* externalBuffer);
	bool LoadCurrentWarpState();
	bool StartLoadingWarpState(unsigned int& frameIx);
	bool StartLoadingWarpState();
	void StopLoadingWarpState();
	bool IsLoadingWarpState();

	// endregion
private:
	// region ================================ STATIC CONSTANTS ========================================================
	static const std::string highlightFilterInfoFilename;
	static const std::string minRecurrenceHighlightFilterName;
	// endregion
	// region ================================ MEMBER FUNCTIONS ========================================================

	void SetPath(boost::filesystem::path fullScenePath);

	// endregion
	// region ================================ MEMBER VARIABLES ========================================================

	// paths
	fs::path path;
	fs::path scenePath;
	fs::path warpPath;
	fs::path highlightsBinaryPath;
	fs::path highlightsTextPath;

	// data structures
	VoxelVolume <TWarp, TIndex>* warpField;
	ITM3DNestedMapOfArrays <ITMHighlightIterationInfo> highlights;
	int minHighlightRecurrenceCount = 0;

	// iterators & stats
	unsigned int iterationCursor = 0;
	int voxelCount = -1;

	// *** optimization warp-updates reading/writing
	std::ofstream warpOFStream;
	std::ifstream warpIFStream;

	// *** slice parameters (optional)
	bool isSlice = false;
	bool sliceLoaded = false;
	Vector6i bounds;
	const std::string sliceIdentifier;
	//endregion
};
}//namespace ITMLib

