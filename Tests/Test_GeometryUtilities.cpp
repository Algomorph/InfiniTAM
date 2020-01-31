//  ================================================================
//  Created by Gregory Kramida on 11/26/19.
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
#define BOOST_TEST_MODULE GeometryUtilities
#ifndef WIN32
#define BOOST_TEST_DYN_LINK
#endif

//boost
#include <boost/test/unit_test.hpp>
#include <iostream>

//local
#include "../ITMLib/Utils/Geometry/ITMIntersectionChecks.h"
#include "../ITMLib/Objects/Volume/VoxelBlockHash.h"


using namespace ITMLib;

BOOST_AUTO_TEST_CASE(Segment_AABB_IntersectionCheck){
	Vector3f segmentStart_VoxelBlocks(-5.48471975, 7.65884304, 25.545948);
	Vector3f segmentEnd_VoxelBlocks(-6.11422205, 8.53787804, 28.477953);
	// add offset, like in the algorithm
	segmentStart_VoxelBlocks += Vector3f(1.0f / (2.0f * VOXEL_BLOCK_SIZE));
	segmentEnd_VoxelBlocks += Vector3f(1.0f / (2.0f * VOXEL_BLOCK_SIZE));
	ITMSegment segment(segmentStart_VoxelBlocks, segmentEnd_VoxelBlocks);
	Vector3f blockMinima[] = {
			Vector3f(-6.f, 7.0f,25.0f),
			Vector3f(-6.f, 7.0f,26.0f),
			Vector3f(-6.f, 8.0f,27.0f),
			Vector3f(-6.f, 8.0f,28.0f),
			Vector3f(-7.f, 8.0f,27.0f),
			Vector3f(-7.f, 8.0f,28.0f),
	};
	float blockSideLength = 1.0f;
	for (auto & blockMin : blockMinima){
		bool intersects = SegmentIntersectsGridAlignedCube3D(segment, blockMin, blockSideLength);
		std::cout << "Segment intersects block " << blockMin.toInt() << ": " << (intersects ? "true" : "false") << std::endl;
	}
	Vector3f voxelMinimum(-48.f,67.f,224.f);
	Vector3f voxelMinimumBlocks1 = voxelMinimum / VOXEL_BLOCK_SIZE;
	bool intersects = SegmentIntersectsGridAlignedCube3D(segment, voxelMinimumBlocks1, 1.0f / VOXEL_BLOCK_SIZE);
	std::cout << "VBH trace segment intersects voxel " << voxelMinimum.toInt() << ": " << (intersects ? "true" : "false") << std::endl;

	Vector3f depthRayStart(0.0f,0.0f,0.0f);
	Vector3f depthRayEnd(-0.183568671, 0.256334633, 0.855000019);
	depthRayEnd *= 2.0f;
	ITMSegment depthSegment(depthRayStart,depthRayEnd);

	Vector3f voxelMinimumMeters(-0.192000002, 0.268000007, 0.896000028);
	voxelMinimumMeters -= Vector3f(0.002);
	float voxelSizeMeters = 0.004;

	intersects = SegmentIntersectsGridAlignedCube3D(depthSegment, voxelMinimumMeters, voxelSizeMeters);
	std::cout << "Direct camera->depth segment intersects voxel " << voxelMinimum.toInt() << ": " << (intersects ? "true" : "false") << std::endl;
}
