//  ================================================================
//  Created by Gregory Kramida on 3/22/18.
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

//stdlib
#include <algorithm>
#include <regex>

//boost
#include <boost/range/iterator_range_core.hpp>

//ITMLib
#include "VolumeSequenceRecorder.h"
#include "../../Utils/Configuration.h"
#include "../EditAndCopy/CPU/EditAndCopyEngine_CPU.h"
#include "../Traversal/CPU/VolumeTraversal_CPU_VoxelBlockHash.h"
#include "../Traversal/CPU/VolumeTraversal_CPU_PlainVoxelArray.h"
#include "WriteAndReadFunctors.h"

using namespace ITMLib;

//======================================================================================================================
//================================================= SCENE LOGGER CLASS CONSTANTS & METHODS RELATING TO SLICES ==========
//======================================================================================================================
// region ======================================== SLICE-RELATED CONSTANTS =============================================

// endregion
// region ======================================== SLICE PATH GENERATION ===============================================

//endregion
// region ======================================== MAKING & SAVING SLICE AND SLICE WARP ================================


//TODO: (big task) fix the logic so that not only the warp field gets sliced, but the canonical as well??
//  (Earlier, canonical scene contained the warp, so it's slice represented both)

//Assumes we have a scene loaded, as well as a warp file available.
/**
 * \brief Make slice of the canonical scene with the specified extrema, store it locally,
 * and save it to disk, (overwrites if files exist already on disk)
 * \param extremum1 [in] the first of the two points defining the slice bounds
 * \param extremum2 [in] the second of the two points defining the slice bounds
 * \param frameIndex [in] index of the frame to write to the saved warp file
 * \param identifier [out] string slice identifier to later retrive / switch to the slice
 * \return true on success, false on failure
 */
template<typename TVoxel, typename TWarp, typename TIndex>
bool
VolumeSequenceRecorder<TVoxel, TWarp, TIndex>::MakeSlice(const Vector3i& extremum1, const Vector3i& extremum2,
                                                         std::string& identifier) {

	Vector6i bounds;
	BoundsFromExtrema(bounds, extremum1, extremum2);
	identifier = WarpFieldSequenceRecorder<TWarp, TIndex>::GenerateSliceStringIdentifier(bounds);

	if (slices.find(identifier) != slices.end()) {
		return false;
	}
	auto logger = new WarpFieldSequenceRecorder<TWarp, TIndex>(bounds, this->path);
	if (!EditAndCopyEngine_CPU<TWarp, TIndex>::Inst().CopyVolumeSlice(
			logger->warpField, fullWarpLogger->warpField, bounds)) {
		return false;
	}
	fs::path outputPath = logger->path;
	if (fs::exists(outputPath)) {
		fs::remove_all(outputPath);//overwrite
	}
	fs::create_directories(outputPath);

	SaveSliceWarp(bounds, logger->warpPath);

	logger->bounds = bounds;
	logger->SaveCompact();
	logger->sliceLoaded = true;
	logger->highlights = MakeSliceHighlights(bounds);
	logger->SaveHighlights(WarpFieldSequenceRecorder<TWarp,TIndex>::continuousHighlightsPostfix);
	slices[identifier] = logger;
	return true;
}

template<typename TVoxel, typename TWarp, typename TIndex>
bool
VolumeSequenceRecorder<TVoxel, TWarp, TIndex>::MakeSlice(const Vector3i& extremum1, const Vector3i& extremum2) {
	std::string identifier;
	return MakeSlice(extremum1, extremum2, identifier);
};



template<typename TVoxel, typename TWarp, typename TIndex>
void
VolumeSequenceRecorder<TVoxel, TWarp, TIndex>::SaveSliceWarp(const Vector6i& voxelRange,
                                                             const boost::filesystem::path& path) {

	std::ofstream sliceWarpOfstream(path.c_str(), std::ofstream::binary | std::ofstream::out);
	if (!sliceWarpOfstream){
		throw std::runtime_error("Could not open '" + path.string() + "' for writing. ["  __FILE__  ": " +
		                         std::to_string(__LINE__) + "]");
	}
	int frameIndex = 0; //TODO: deprecate
	sliceWarpOfstream.write(reinterpret_cast<const char*>(&frameIndex), sizeof(frameIndex));


	bool wasCanonicalLoadingWarp = fullWarpLogger->IsLoadingWarpState();
	unsigned int originalfullSceneIterationCursor = fullWarpLogger->GetIterationCursor();
	fullWarpLogger->StopLoadingWarpState();
	fullWarpLogger->StartLoadingWarpState();
	int zRangeStart, zRangeEnd, yRangeStart, yRangeEnd, xRangeStart, xRangeEnd;

	WarpVoxelWriteFunctor<TWarp, TWarp::hasFramewiseWarp, TWarp::hasWarpUpdate> warp_writer(&sliceWarpOfstream, sizeof(Vector3f), sizeof(Vector3f));

	while (fullWarpLogger->LoadCurrentWarpState()) {
		unsigned int sliceIterationCursor = fullWarpLogger->GetIterationCursor();
		sliceWarpOfstream.write(reinterpret_cast<const char* >(&sliceIterationCursor), sizeof(sliceIterationCursor));

		VolumeTraversalEngine<TWarp, TIndex, MEMORYDEVICE_CPU>::TraverseAllWithinBounds(fullWarpLogger->warpField,
		                                                                                warp_writer,voxelRange);
	}
	sliceWarpOfstream.close();
	fullWarpLogger->StopLoadingWarpState();
	if (wasCanonicalLoadingWarp) {
		fullWarpLogger->SetIterationCursor(originalfullSceneIterationCursor);
	}
}

template<typename TVoxel, typename TWarp, typename TIndex>
NestedMap3DOfArrays<ITMHighlightIterationInfo>
VolumeSequenceRecorder<TVoxel, TWarp, TIndex>::MakeSliceHighlights(const Vector6i& bounds) {
	NestedMap3DOfArrays<ITMHighlightIterationInfo> sliceHighlights;
	std::vector<std::vector<ITMHighlightIterationInfo>> highlightArrays = fullWarpLogger->highlights.GetArrays();
	auto VoxelInRange = [&bounds](Vector3i voxelPosition) {
		Vector3i& vp = voxelPosition;
		return (vp.x >= bounds.min_x && vp.x <= bounds.max_x) &&
		       (vp.y >= bounds.min_y && vp.y <= bounds.max_y) &&
		       (vp.z >= bounds.min_z && vp.z <= bounds.max_z);
	};
	for (auto& array : highlightArrays) {
		for (auto& highlight : array) {
			if (VoxelInRange(highlight.position)){
				sliceHighlights.InsertOrdered(highlight.hash,highlight.localId,0,highlight);
			}
		}
	}
	return sliceHighlights;
};

//endregion

// region =============================== OTHER SLICE-RELATED FUNCTIONS ================================================

/**
 * \brief Verifies whether the slice specified by identifier (by looking at names of folder & files within)
 * \param sliceIdentifier a string-form scene slice identifier, usually consistent of min & max point coordinates
 * separated by '_'
 * \return true if all files corresponding to the slice exist, false otherwise.
 */
template<typename TVoxel, typename TWarp, typename TIndex>
bool VolumeSequenceRecorder<TVoxel, TWarp, TIndex>::SliceExistsOnDisk(const std::string& sliceIdentifier) const {
	fs::path sliceFolderPath =
			WarpFieldSequenceRecorder<TWarp, TIndex>::GenerateSliceFolderPath(path, sliceIdentifier);
	fs::path sliceScenePath =
			WarpFieldSequenceRecorder<TWarp, TIndex>::GenerateSliceSceneFilename_Full(path, sliceIdentifier);
	fs::path sliceWarpPath =
			WarpFieldSequenceRecorder<TWarp, TIndex>::GenerateSliceWarpFilename(path, sliceIdentifier);
	return fs::is_directory(sliceFolderPath) && fs::is_regular_file(sliceScenePath) &&
	       fs::is_regular_file(sliceWarpPath);
}

/**
 * \brief Verifies whether the slice specified by the bounds exists (by looking at names of folder & files within)
 * \param extremum1 the first of the two points defining the slice bounds
 * \param extremum2 the second of the two points defining the slice bounds
 * \return true if all files corresponding to the slice exist, false otherwise.
 */
template<typename TVoxel, typename TWarp, typename TIndex>
bool VolumeSequenceRecorder<TVoxel, TWarp, TIndex>::SliceExistsOnDisk(const Vector3i& extremum1,
                                                                      const Vector3i& extremum2) const {
	Vector6i bounds;
	BoundsFromExtrema(bounds, extremum1,extremum2);
	fs::path sliceFolderPath =
			WarpFieldSequenceRecorder<TWarp, TIndex>::GenerateSliceFolderPath(path, bounds);
	fs::path sliceScenePath =
			WarpFieldSequenceRecorder<TWarp, TIndex>::GenerateSliceSceneFilename_Full(path, bounds);
	fs::path sliceWarpPath =
			WarpFieldSequenceRecorder<TWarp, TIndex>::GenerateSliceWarpFilename(path, bounds);
	return fs::is_directory(sliceFolderPath) && fs::is_regular_file(sliceScenePath) &&
	       fs::is_regular_file(sliceWarpPath);
}

/**
 * \brief Verifies whether the slice exists in memory within the current object data structure
 * \param sliceIdentifier identifier of the slice (string form of min & max points with the six coordinates separated by '_')
 * \return true if the slice exists, false otherwise
 */
template<typename TVoxel, typename TWarp, typename TIndex>
bool
VolumeSequenceRecorder<TVoxel, TWarp, TIndex>::SliceExistsInMemory(const std::string& sliceIdentifier) const {
	return this->slices.find(sliceIdentifier) != slices.end();
}

/**
 * \brief Loads the specified scene slice into destinationScene, and sets up internal structures for loading its warp
 * \param sliceIdentifier identifier of the slice (string form of min & max points with the six coordinates separated by '_')
 * \param destinationScene where to store the slice voxels
 * \return true on success, false on failure
 */
template<typename TVoxel, typename TWarp, typename TIndex>
bool VolumeSequenceRecorder<TVoxel, TWarp, TIndex>::LoadSlice(const std::string& sliceIdentifier) {
	if (!SliceExistsOnDisk(sliceIdentifier)) {
		return false;
	}
	Vector6i bounds;
	WarpFieldSequenceRecorder<TWarp, TIndex>::ExtractBoundsFromSliceStringIdentifier(sliceIdentifier, bounds);
	WarpFieldSequenceRecorder<TWarp, TIndex>* logger =
			new WarpFieldSequenceRecorder<TWarp, TIndex>(bounds, path);
	logger->LoadCompact();
	logger->sliceLoaded = true;
	slices[sliceIdentifier] = logger;
	return true;
}

/**
 * \brief Switch to a different scene slice or to the full scene (if the requested slice or the full scene is available)
 * \param sliceIdentifier either identifier of the slice (string form of min & max points with the six coordinates separated by '_'), or the full scene identifier, \sa ITMWarpSceneLogger::fullSceneSliceIdentifier
 * \return true on success, false on failure
 */
template<typename TVoxel, typename TWarp, typename TIndex>
bool VolumeSequenceRecorder<TVoxel, TWarp, TIndex>::SwitchActiveScene(std::string sliceIdentifier) {
	bool isLoadingWarps = activeWarpLogger->IsLoadingWarpState();
	int iterationCursor = static_cast<int>(activeWarpLogger->GetIterationCursor());

	if (sliceIdentifier == WarpFieldSequenceRecorder<TWarp, TIndex>::fullSceneSliceIdentifier) {
		if (fullWarpLogger != nullptr) {
			mode = FULL_SCENE;
			activeWarpLogger->StopLoadingWarpState();
			activeWarpLogger = fullWarpLogger;
		} else {
			return false;
		}
	} else {
		if (SliceExistsInMemory(sliceIdentifier)) {
			mode = SLICE;
			activeWarpLogger->StopLoadingWarpState();
			activeWarpLogger = slices[sliceIdentifier];
		} else {
			return false;
		}
	}
	if (isLoadingWarps) {
		activeWarpLogger->StartLoadingWarpState();
		//set cursor to the *previous* iteration
		//(std::max) in parenthesis because of some <insert descripitive curseword> Microsoft compiler quirk concerning min/max macros
		activeWarpLogger->SetIterationCursor(static_cast<unsigned int>((std::max)(--iterationCursor, 0)));
	}
	return true;
}

/**
 * \brief Loads all slices available in the logger's root path (if any).
 * \details If the path is invalid or not properly set, silently returns an empty vector of strings.
 * \return A vector of slice string identifiers for all slices that were discovered.
 */
template<typename TVoxel, typename TWarp, typename TIndex>
std::vector<std::string> VolumeSequenceRecorder<TVoxel, TWarp, TIndex>::LoadAllSlices() {
	std::vector<std::string> identifiers;
	if (!CheckPath()) {
		return identifiers;
	}
	const int coordCount = 6;
	std::regex sliceDirectoryRegex("slice(?:_(?:[-]|\\d)\\d{2}){" + std::to_string(coordCount) + "}");
	std::vector<std::pair<std::string,std::time_t>> sliceDirectoryNamesAndWriteTimes;

	for (auto& entry : boost::make_iterator_range(fs::directory_iterator(this->path))) {
		std::string directoryName = entry.path().filename().string();
		std::time_t lastModifiedTime = fs::last_write_time(entry.path());
		if (fs::is_directory(entry.path()) && std::regex_match(directoryName, sliceDirectoryRegex)) {
			sliceDirectoryNamesAndWriteTimes.push_back(std::make_pair(directoryName,lastModifiedTime));
		}
	}

	auto SortBySecondDescending = [&](const std::pair<std::string,std::time_t> &a,
	                        const std::pair<std::string,std::time_t> & b){
		return (a.second > b.second);
	};

	//Sort by modification time in decreasing order
	std::sort(sliceDirectoryNamesAndWriteTimes.begin(), sliceDirectoryNamesAndWriteTimes.end(), SortBySecondDescending);

	for (auto& directoryNameAndWriteTime : sliceDirectoryNamesAndWriteTimes){
		Vector6i bounds;
		WarpFieldSequenceRecorder<TWarp, TIndex>::ExtractBoundsFromSliceStringIdentifier(
				std::get<0>(directoryNameAndWriteTime), bounds);
		std::string sliceIdentifier =
				WarpFieldSequenceRecorder<TWarp, TIndex>::GenerateSliceStringIdentifier(bounds);
		if (LoadSlice(sliceIdentifier)) {
			identifiers.push_back(sliceIdentifier);
		}
	}
	return identifiers;
}
// endregion