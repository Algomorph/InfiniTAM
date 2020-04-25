//  ================================================================
//  Created by Gregory Kramida on 8/27/19.
//  Copyright (c) 2019 Gregory Kramida
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

#include <iostream>

namespace ITMLib{

template<typename TVoxel, bool hasFramewiseWarp, bool hasWarpUpdate>
struct WarpVoxelWriteFunctor;

template<typename TVoxel>
struct WarpVoxelWriteFunctor<TVoxel, true, true> {
	WarpVoxelWriteFunctor(std::ofstream* warp_output_file_stream,
	                      size_t warpByteSize,
	                      size_t updateByteSize) : warpOFStream(warp_output_file_stream),
	                                                   warpByteSize(warpByteSize),
	                                                   updateByteSize(updateByteSize) {}

	void operator()(TVoxel& voxel) {
		warpOFStream->write(reinterpret_cast<const char* >(&voxel.framewise_warp), warpByteSize);
		warpOFStream->write(reinterpret_cast<const char* >(&voxel.warp_update), updateByteSize);
	}

private:
	std::ofstream* warpOFStream;
	const size_t warpByteSize;
	const size_t updateByteSize;
};



template<typename TVoxel>
struct WarpVoxelWriteFunctor<TVoxel, false, true> {
	WarpVoxelWriteFunctor(std::ofstream* warp_output_file_stream,
	                      size_t warpByteSize,
	                      size_t updateByteSize) : warpOFStream(warp_output_file_stream),
	                                               warpByteSize(warpByteSize),
	                                               updateByteSize(updateByteSize) {}

	void operator()(TVoxel& voxel) {
		warpOFStream->write(reinterpret_cast<const char* >(&voxel.warp_update), updateByteSize);
	}

private:
	std::ofstream* warpOFStream;
	const size_t warpByteSize;
	const size_t updateByteSize;
};


template<typename TVoxel, bool hasFramewiseWarp, bool hasWarpUpdate>
struct WarpAndUpdateReadFunctor;

template<typename TVoxel>
struct WarpAndUpdateReadFunctor<TVoxel, true, true> {
	WarpAndUpdateReadFunctor(std::ifstream* warpIFStream,
	                         size_t warpByteSize,
	                         size_t updateByteSize) : warpIFStream(warpIFStream),
	                                                  warpByteSize(warpByteSize),
	                                                  updateByteSize(updateByteSize) {}

	void operator()(TVoxel& voxel) {
		warpIFStream->read(reinterpret_cast<char*>(&voxel.framewise_warp), warpByteSize);
		warpIFStream->read(reinterpret_cast<char*>(&voxel.warp_update), updateByteSize);
	}

private:
	std::ifstream* warpIFStream;
	const size_t warpByteSize;
	const size_t updateByteSize;
};


template<typename TVoxel>
struct WarpAndUpdateReadFunctor<TVoxel, false, true> {
	WarpAndUpdateReadFunctor(std::ifstream* warpIFStream,
	                         size_t warpByteSize,
	                         size_t updateByteSize) : warpIFStream(warpIFStream),
	                                                  warpByteSize(warpByteSize),
	                                                  updateByteSize(updateByteSize) {}

	void operator()(TVoxel& voxel) {
		warpIFStream->read(reinterpret_cast<char*>(&voxel.warp_update), updateByteSize);
	}

private:
	std::ifstream* warpIFStream;
	const size_t warpByteSize;
	const size_t updateByteSize;
};

} // namespace ITMLib
