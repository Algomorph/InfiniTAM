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
#include "../Objects/Scene/ITMScene.h"
#include "ITMIntArrayMap3D.h"

//TODO: eventually replace boost::filesystem with stdlib filesystem when that is no longer experimental -Greg (GitHub: Algomorph)
//TODO: add HAVE_BOOST guards / CMake optional boost support -Greg (GitHub: Algomorph)

//boost
#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;

namespace ITMLib {

/**
 * \brief Wraps the functionality of saving canonical/live scenes for dynamic fusion along with warp changes during optimization between frames.
 * \tparam TVoxelCanonical Type of canonical ("initial"/"source"/"reference") scene voxels
 * \tparam TVoxelLive Type of live ("streaming"/"target") scene voxels
 * \tparam TIndex Type of index used for the voxel scenes
 */
template<typename TVoxelCanonical, typename TVoxelLive, typename TIndex>
class ITMSceneLogger {

public:
// === STATIC CONSTANTS ===
	static const std::string binaryFileExtension;
	static const std::string textFileExtension;

// === PUBLIC INNER CLASSES ===
	/**
	 * \brief cube-shaped interest region with fixed edge length consistent of hash blocks within the scene
	 */
	class InterestRegionInfo {
	public:
		static constexpr int edgeLengthHashBlocks = 3;
		static constexpr int maxHashBlockCount = edgeLengthHashBlocks * edgeLengthHashBlocks * edgeLengthHashBlocks;
		static const Vector3s blockTraversalOrder[];
		static const std::string prefix;

		InterestRegionInfo(std::vector<int>& hashBlockIds, int centerHashBlockId,
		                   ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>& parent);
		InterestRegionInfo(fs::path path, ITMSceneLogger<TVoxelCanonical, TVoxelLive, TIndex>& parent);
		void SaveCurrentWarpState();

		void BufferNextWarpState(void* externalBuffer);

		void BufferPreviousWarpState(void* externalBuffer);

		const std::vector<int>& GetHashBlockIds() const;

		virtual ~InterestRegionInfo();

	private:
		// ** member variables **
		bool isLoading = false;
		bool isSaving = false;
		int centerHashBlockId;
		std::vector<int> hashBlockIds;
		fs::path path;
		std::ofstream ofStream;
		std::ifstream ifStream;
		ITMSceneLogger& parent;
		int iUpdate = 0;
		int voxelCount;

	};

// === CONSTRUCTORS / DESTRUCTORS ===
	ITMSceneLogger(std::string path, ITMScene<TVoxelCanonical, TIndex>* canonicalScene = NULL,
	               ITMScene<TVoxelLive, TIndex>* liveScene = NULL);

	ITMSceneLogger() = delete;//disable default constructor generation
	virtual ~ITMSceneLogger();

// === MEMBER FUNCTIONS ===
	//*** setters / preparation
	void SetScenes(ITMScene<TVoxelCanonical, TIndex>* canonicalScene, ITMScene<TVoxelLive, TIndex>* liveScene);

	//*** scene loading/saving
	bool SaveScenes();
	bool LoadScenes();
	bool SaveScenesCompact();
	bool LoadScenesCompact();

	//*** information getters
	int GetVoxelCount() const;
	bool GetScenesLoaded() const;
	const std::map<int, std::shared_ptr<InterestRegionInfo>>& GetInterestRegionsByHash();
	const ITMIntArrayMap3D& GetHighlights();
	std::set<int> GetInterestRegionHashes();

	//*** saving of meta-information & interest regions
	void LogHighlight(int hashId, int voxelLocalIndex, int frameNumber, int iterationNumber);
	bool SaveHighlights();
	void PrintHighlights();
	bool LoadHighlights();
	void FilterHighlights(int anomalyFrameCountMinimum);
	void SetUpInterestRegionsForSaving();
	void SaveAllInterestRegionWarps();
	void SetUpInterestRegionsForLoading();
	bool IsHashInInterestRegion(int hashId);

	//** global warp-state saving/loading
	bool StartSavingWarpState();
	bool SaveCurrentWarpState();
	void StopSavingWarpState();
	bool StartLoadingWarpState();
	bool LoadNextWarpState();
	bool BufferNextWarpState();
	bool BufferPreviousWarpState();
	bool BufferNextWarpState(void* externalBuffer);
	bool BufferPreviousWarpState(void* externalBuffer);
	bool LoadPreviousWarpState();
	void StopLoadingWarpState();
	bool IsLoadingWarpState();
	bool CopyWarpBuffer(float* warpDestination, float* warpUpdateDestination, int& iUpdate);
	bool CopyWarpAt(int index, float voxelWarpDestination[3]) const;
	bool CopyWarpAt(int index, float voxelWarpDestination[3], float voxelUpdateDestination[3]) const;
	const float* WarpAt(int index) const;
	const float* UpdateAt(int index) const;


private:
//========= MEMBER VARIABLES ==================

// *** root folder
	fs::path path;
// *** canonical/live scene saving/loading
	fs::path canonicalPath;
	fs::path livePath;
	ITMScene<TVoxelCanonical, TIndex>* canonicalScene;
	ITMScene<TVoxelLive, TIndex>* liveScene;
// *** scene meta-information + reading/writing
	int voxelCount = -1;
	// map of hash blocks to voxels, voxels to frame numbers, frame numbers to iteration numbers
	ITMIntArrayMap3D highlights;
	fs::path highlightsBinaryPath;
	fs::path highlightsTextPath;
	std::map<int, std::shared_ptr<InterestRegionInfo>> interestRegionInfoByHashId;
	std::vector<std::shared_ptr<InterestRegionInfo>> interestRegionInfos;

// *** optimization warp-updates reading/writing
	fs::path warpUpdatesPath;
	std::ofstream warpOFStream;
	std::ifstream warpIFStream;
	//TODO: the way these update numbers are tracked are less than ideal (see comment below) -Greg (GitHub: Algomorph)
	// There is no way to ensure proper iteration number, since it is not kept track of in the scene.
	// It would be ideal to extend the scene class and log that number there, since it reflects the state of the scene.
	unsigned int iUpdate = 0;
	Vector3f* warpBuffer = NULL;

//======== MEMBER FUNCTIONS =====================
	bool CheckDirectory();

};


}//namespace ITMLib


