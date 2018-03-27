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
#include "../../Objects/Scene/ITMScene.h"
#include "../ITM3DNestedMapOfArrays.h"
#include "../ITMNeighborVoxelIterationInfo.h"

//TODO: eventually replace boost::filesystem with stdlib filesystem when that is no longer experimental -Greg (GitHub: Algomorph)
//TODO: add HAVE_BOOST guards / CMake optional boost support -Greg (GitHub: Algomorph)
//TODO: warpByteSize should be a constant and used throughout the .tpp where it is appropriate (instead of 2*sizeof(Vector3f)) -Greg (GitHub: Algomorph)

//TODO: need only 3 public buffering/reading methods & 2 seek methods (described below) -Greg (GitHub: Algomorph)
/*
 * This would keep iIteration consistent for both interest regions & general warps
 * Buffering:
 * 1) Buffer interest warps & advance file cursor for general warps
 * 2) Buffer general warps & advance file cursors for all interest warps
 * 3) Buffer both interest & general warps
 *
 * Seeking:
 * 1) Move all file cursors to previous (if possible)
 * 2) Move all file cursors to arbitrary iIteration (if possible) -- should peek one-by-one if iteration count is unknown,
 * but move directly to proper position when iteration count becomes known (end of file is reached)
 */
//boost
#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;

namespace ITMLib {

/**
 * \brief Wraps the functionality of saving canonical/live scenes or scene slices for dynamic fusion along
 * with warp changes during optimization between frames.
 * \tparam TVoxelCanonical Type of canonical ("initial"/"source"/"reference") scene voxels
 * \tparam TVoxelLive Type of live ("streaming"/"target") scene voxels
 * \tparam TIndex Type of index used for the voxel scenes
 */
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
class ITMCombinedSceneLogger {

public:
// === STATIC CONSTANTS ===
	static const std::string warpUpdatesFilename;
	static const std::string binaryFileExtension;
	static const std::string textFileExtension;

// === PUBLIC INNER CLASSES ===
	/**
	 * \brief cube-shaped interest region with fixed edge length consistent of hash blocks within the scene
	 */
	class InterestRegionInfo {
		friend class ITMCombinedSceneLogger;
	public:
		static const Vector3s blockTraversalOrder[];
		static const std::string prefix;

		InterestRegionInfo(std::vector<int>& hashBlockIds, int centerHashBlockId,
		                   ITMCombinedSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>& parent);
		InterestRegionInfo(fs::path path, ITMCombinedSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>& parent);


		void SaveCurrentWarpState();

		bool BufferCurrentWarpState(void* externalBuffer);
		bool SeekPrevious();
		bool SeekAt(unsigned int cursor);
		unsigned int GetIterationCursor() const;
		size_t GetIterationWarpBytesize() const;

		const std::vector<int>& GetHashes() const;


		virtual ~InterestRegionInfo();

	private:
		void RewriteHeader();
		// ** member variables **
		bool isLoading = false;
		bool isSaving = false;
		int centerHashBlockId;
		std::vector<int> hashBlockIds;
		fs::path path;
		std::ofstream ofStream;
		std::ifstream ifStream;
		ITMCombinedSceneLogger& parent;
		unsigned int iterationCursor = 0;
		int voxelCount;

	};

// === CONSTRUCTORS / DESTRUCTORS ===
	ITMCombinedSceneLogger(std::string path = "", ITMScene<TVoxelCanonical, TIndex>* canonicalScene = NULL,
	               ITMScene<TVoxelLive, TIndex>* liveScene = NULL);

	ITMCombinedSceneLogger() = delete;//disable default constructor generation
	virtual ~ITMCombinedSceneLogger();

// === MEMBER FUNCTIONS ===
	//*** setters / preparation
	void SetScenes(ITMScene<TVoxelCanonical, TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>* liveScene);
	void SetPath(std::string path);

	//*** scene loading/saving
	bool SaveScenes();
	bool LoadScenes();
	bool SaveScenesCompact();
	bool LoadScenesCompact();
	bool SaveCanonicalSceneSlice(const Vector3i& extremum1,
		                             const Vector3i& extremum2,
		                             unsigned int frameIndex);


	//*** information getters
	std::string GetPath() const;
	int GetVoxelCount() const;
	bool GetScenesLoaded() const;
	bool GetInterestRegionsSetUp() const;
	unsigned int GetGeneralIterationCursor() const;
	unsigned int GetInterestIterationCursor() const;
	const std::map<int, std::shared_ptr<InterestRegionInfo>>& GetInterestRegionsByHash();
	ITM3DNestedMapOfArrays<ITMHighlightIterationInfo> GetHighlights() const;
	std::vector<int> GetInterestRegionHashes() const;

	//*** saving of meta-information & interest regions
	void LogHighlight(int hashId, int voxelLocalIndex, int frameNumber, ITMHighlightIterationInfo info);
	bool SaveHighlights(std::string filePostfix = "");
	void ClearHighlights();
	void PrintHighlights();
	bool LoadHighlights(bool applyFilters = true, std::string filePostfix = "");
	void FilterHighlights(int anomalyFrameCountMinimum);
	void SetUpInterestRegionsForSaving();
	void SetUpInterestRegionsForSaving(const ITM3DNestedMapOfArrays<ITMHighlightIterationInfo>& highlights);
	void SaveAllInterestRegionWarps();
	void SetUpInterestRegionsForLoading();
	bool BufferInterestWarpStateAtIteration(void* externalBuffer, unsigned int iterationIndex);
	bool BufferCurrentInterestWarpState(void* externalBuffer);
	bool BufferPreviousInterestWarpState(void* externalBuffer);
	bool IsHashInInterestRegion(int hashId);
	int GetTotalInterestVoxelCount();

	//** global warp-state saving/loading
	bool StartSavingWarpState(unsigned int frameIx);
	void StopSavingWarpState();
	bool StartLoadingWarpState();
	bool StartLoadingWarpState(unsigned int& frameIx);
	void StopLoadingWarpState();

	bool SaveCurrentWarpState();
	bool LoadCurrentWarpState();
	bool BufferNextWarpState();
	bool BufferPreviousWarpState();
	bool BufferCurrentWarpState(void* externalBuffer);
	bool BufferPreviousWarpState(void* externalBuffer);
	bool BufferWarpStateAt(void* externalBuffer, unsigned int iterationIndex);
	bool LoadPreviousWarpState();

	bool IsLoadingWarpState();
	bool CopyWarpBuffer(float* warpDestination, float* warpUpdateDestination, int& iUpdate);
	bool CopyWarpAt(int index, float voxelWarpDestination[3]) const;
	bool CopyWarpAt(int index, float voxelWarpDestination[3], float voxelUpdateDestination[3]) const;
	const float* WarpAt(int index) const;
	const float* UpdateAt(int index) const;

private:

//========= CONSTANTS =========================
	static const std::string highlightFilterInfoFilename;
	static const std::string minRecurrenceHighlightFilterName;
//========= MEMBER FUNCTIONS ==================
	bool CheckDirectory();
	void SaveWarpSlice(const Vector3i& minPoint, const Vector3i& maxPoint,
		                   unsigned int frameIndex);
	std::string GenerateSliceStringIdentifier(const Vector3i& minPoint, const Vector3i& maxPoint);
	std::string GenerateSliceSceneFilename_UpToPostfix(const Vector3i& minPoint, const Vector3i& maxPoint);
	std::string GenerateSliceSceneFilename_Full(const Vector3i& minPoint, const Vector3i& maxPoint);
	std::string GenerateSliceWarpFilename(const Vector3i& minPoint, const Vector3i& maxPoint);
//========= MEMBER VARIABLES ==================

// *** root folder
	fs::path path;
// *** subpaths
	fs::path canonicalPath;
	fs::path livePath;
	fs::path highlightsBinaryPath;
	fs::path highlightsTextPath;

	ITMScene<TVoxelCanonical, TIndex>* canonicalScene;
	ITMScene<TVoxelLive, TIndex>* liveScene;

// *** scene meta-information + reading/writing
	int voxelCount = -1;
	// map of hash blocks to voxels, voxels to frame numbers, frame numbers to iteration numbers
	ITM3DNestedMapOfArrays<ITMHighlightIterationInfo> highlights;

	std::map<int, std::shared_ptr<InterestRegionInfo>> interestRegionInfoByHashId;
	std::vector<std::shared_ptr<InterestRegionInfo>> interestRegionInfos;
	bool interestRegionsHaveBeenSetUp = false;

// *** optimization warp-updates reading/writing
	fs::path warpUpdatesPath;
	std::ofstream warpOFStream;
	std::ifstream warpIFStream;
	//TODO: the way these update numbers are tracked are less than ideal (see comment below) -Greg (GitHub: Algomorph)
	// There is no way to ensure proper iteration number, since it is not kept track of in the scene.
	// It would be ideal to extend the scene class and log that number there, since it reflects the state of the scene.
	unsigned int generalIterationCursor = 0;
	unsigned int interestIterationCursor = 0;
	Vector3f* warpBuffer = NULL;

// *** data manipulation information ***
	int minHighlightRecurrenceCount = 0;
};


}//namespace ITMLib

