//  ================================================================
//  Created by Gregory Kramida on 2/20/18.
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

#include "../../Objects/Volume/RepresentationAccess.h"
#include "../../Utils/HashBlockProperties.h"
#include "../../Utils/Analytics/NeighborVoxelIterationInfo.h"
#include "../../Utils/CPPPrintHelpers.h"
#include "../../Utils/CPrintHelpers.h"
#include "../../Objects/Volume/TrilinearInterpolation.h"
#include "../../Utils/Geometry/SpatialIndexConversions.h"

using namespace ITMLib;

//======================================================================================================================
// region ==================================== ROUTINES FOR SAVING DIAGNOSTIC INFORMATION DURING OPTIMIZATION ==========
//======================================================================================================================

template<class TVoxel>
_CPU_AND_GPU_CODE_ inline TVoxel
ReadVoxelAndLinearIndex(const CONSTPTR(TVoxel)* voxelData,
                        const CONSTPTR(ITMLib::VoxelBlockHash::IndexData)* voxelIndex,
                        const THREADPTR(Vector3i)& point, THREADPTR(int)& vmIndex,
                        THREADPTR(ITMLib::VoxelBlockHash::IndexCache)& cache, THREADPTR(int)& linearIdx) {
	Vector3i blockPos;
	linearIdx = pointToVoxelBlockPos(point, blockPos);

	if IS_EQUAL3(blockPos, cache.blockPos) {
		return voxelData[cache.blockPtr + linearIdx];
	}

	int hashIdx = HashCodeFromBlockPosition(blockPos);

	while (true) {
		HashEntry hashEntry = voxelIndex[hashIdx];

		if (IS_EQUAL3(hashEntry.pos, blockPos) && hashEntry.ptr >= 0) {
			cache.blockPos = blockPos;
			cache.blockPtr = hashEntry.ptr * VOXEL_BLOCK_SIZE3;
			vmIndex = hashIdx + 1; // add 1 to support legacy true / false operations for isFound
			return voxelData[cache.blockPtr + linearIdx];
		}

		if (hashEntry.offset < 1) break;
		hashIdx = ORDERED_LIST_SIZE + hashEntry.offset - 1;
	}

	vmIndex = false;
	linearIdx = 0;
	return TVoxel();
}




template<class TVoxel>
_CPU_AND_GPU_CODE_ inline TVoxel
ReadVoxelAndLinearIndex(const CONSTPTR(TVoxel)* voxel_data,
                        const CONSTPTR(ITMLib::PlainVoxelArray::IndexData)* voxel_index,
                        const THREADPTR(Vector3i)& point, THREADPTR(int)& vm_index,
                        THREADPTR(ITMLib::PlainVoxelArray::IndexCache)& cache, THREADPTR(int)& linear_index) {

	linear_index = ComputeLinearIndexFromPosition_PlainVoxelArray(voxel_index, point);
	return voxel_data[linear_index];
}


template<typename TVoxelCanonical, typename TVoxelLive, typename TIndexData, typename TCache>
inline void FindHighlightNeighborInfo(std::array<ITMLib::NeighborVoxelIterationInfo, 9>& neighbors,
                                      const CONSTPTR(Vector3i)& highlightPosition,
                                      const CONSTPTR(int)& highlightHash,
                                      const CONSTPTR(TVoxelCanonical)* canonicalVoxelData,
                                      const CONSTPTR(TIndexData)* canonicalIndexData,
                                      THREADPTR(TCache)& canonicalCache,
                                      const CONSTPTR(TVoxelLive)* liveVoxelData,
                                      const CONSTPTR(TIndexData)* liveIndexData,
                                      THREADPTR(TCache)& liveCache
) {
	//    0        1        2          3         4         5           6         7         8
	//(-1,0,0) (0,-1,0) (0,0,-1)   (1, 0, 0) (0, 1, 0) (0, 0, 1)   (1, 1, 0) (0, 1, 1) (1, 0, 1)
	Vector3i locations[9] = {Vector3i(-1, 0, 0), Vector3i(0, -1, 0), Vector3i(0, 0, -1),
	                         Vector3i(1, 0, 0), Vector3i(0, 1, 0), Vector3i(0, 0, 1),
	                         Vector3i(1, 1, 0), Vector3i(0, 1, 1), Vector3i(1, 0, 1)};
	int vmIndex, localId = 0;
	vmIndex = highlightHash + 1;
	int iNeighbor = 0;
	for (auto location : locations) {
		ITMLib::NeighborVoxelIterationInfo& info = neighbors[iNeighbor];
		Vector3i neighborPosition = highlightPosition + (location);
		TVoxelCanonical voxelCanonical = ReadVoxelAndLinearIndex(canonicalVoxelData, canonicalIndexData,
		                                                         neighborPosition,
		                                                         vmIndex, canonicalCache, localId);
		const TVoxelLive& voxelLive = readVoxel(liveVoxelData, liveIndexData, neighborPosition, vmIndex, liveCache);
		if (vmIndex != 0) {
			info.unknown = voxelCanonical.flags == ITMLib::VOXEL_TRUNCATED;
			info.hash = vmIndex - 1;
		} else {
			info.notAllocated = true;
			info.hash = 0;
			vmIndex = highlightHash + 1;//reset
		}
		info.localId = localId;
		info.warp = voxelCanonical.framewise_warp;
		info.warpGradient = voxelCanonical.gradient0;
		info.sdf = TVoxelCanonical::valueToFloat(voxelCanonical.sdf);
		info.liveSdf = TVoxelLive::valueToFloat(voxelLive.sdf);
		iNeighbor++;
	}
}
// endregion ===========================================================================================================
//======================================================================================================================
// region ============================== DIAGNOSTIC PRINTING ROUTINES ==================================================
//======================================================================================================================

_CPU_AND_GPU_CODE_
inline void PrintDataTermInformation(const CONSTPTR(Vector3f)& live_sdf_gradient) {
	printf("Gradient of live SDF at current warp: %s%E,%E,%E%s\n",
	       c_cyan, live_sdf_gradient.x, live_sdf_gradient.y, live_sdf_gradient.z, c_reset);
}

_CPU_AND_GPU_CODE_
inline void PrintLevelSetTermInformation(const CONSTPTR(Vector3f)& live_sdf_gradient,
                                         const CONSTPTR(Matrix3f)& live_sdf_2nd_derivative,
                                         const CONSTPTR(float)& sdf_gradient_norm_minus_unity) {
	printf("Gradient of live SDF at current warp: %s%E,%E,%E%s\n2nd derivative of live SDF at current warp: %s\n"
	       "%E, %E, %E,\n"
	       "%E, %E, %E,\n"
	       "%E, %E, %E\n"
	       "%sGradient norm minus unity: %s%E%s\n",
	       c_cyan, live_sdf_gradient.x, live_sdf_gradient.y, live_sdf_gradient.z, c_reset, c_green,
	       live_sdf_2nd_derivative.xx, live_sdf_2nd_derivative.xy, live_sdf_2nd_derivative.xz,
	       live_sdf_2nd_derivative.yx, live_sdf_2nd_derivative.yy, live_sdf_2nd_derivative.yz,
	       live_sdf_2nd_derivative.zx, live_sdf_2nd_derivative.zy, live_sdf_2nd_derivative.zz,
	       c_reset, c_blue, sdf_gradient_norm_minus_unity, c_reset);
}

_CPU_AND_GPU_CODE_
inline void PrintKillingTermInformation(const CONSTPTR(Vector3f*) neighbor_warps,
                                        const CONSTPTR(bool*) neighbor_allocated,
                                        const CONSTPTR(bool*) neighbor_truncated,
                                        THREADPTR(Matrix3f)& warp_update_Jacobian, //in
                                        THREADPTR(Matrix3f)* warp_update_Hessian //in, x3
) {

	const int neighborhood_size = 9;
	//(-1,0,0) (0,-1,0) (0,0,-1)   (1, 0, 0) (0, 1, 0) (0, 0, 1)   (1, 1, 0) (0, 1, 1) (1, 0, 1)
	Vector3i neighbor_positions[] = {Vector3i(-1, 0, 0), Vector3i(0, -1, 0), Vector3i(0, 0, -1), Vector3i(1, 0, 0),
	                                 Vector3i(0, 1, 0), Vector3i(0, 0, 1), Vector3i(1, 1, 0), Vector3i(0, 1, 1),
	                                 Vector3i(1, 0, 1),};
	printf("%sNeighbors' warps: \n", c_green);

	for (int i_neighbor = 0; i_neighbor < neighborhood_size; i_neighbor++) {
		printf("%s%d, %d, %d (Neighbor %d): %s%E, %E, %E\n",
		       c_reset, neighbor_positions[i_neighbor].x, neighbor_positions[i_neighbor].y, neighbor_positions[i_neighbor].z, i_neighbor, c_green,
		       neighbor_warps[i_neighbor].x, neighbor_warps[i_neighbor].y, neighbor_warps[i_neighbor].z);
	}
	printf("%sUnallocated neighbors: ", c_reset);
	bool first = true;
	for (int i_neighbor = 0; i_neighbor < neighborhood_size; i_neighbor++) {
		if (!neighbor_allocated[i_neighbor]) {
			if (first == true) {
				printf("%d", i_neighbor);
				first = false;
			} else {
				printf(", %d", i_neighbor);
			}
		}
	}
	printf("\nTruncated neighbors: ");
	first = true;
	for (int i_neighbor = 0; i_neighbor < neighborhood_size; i_neighbor++) {
		if (neighbor_truncated[i_neighbor]) {
			if (first == true) {
				printf("%d", i_neighbor);
				first = false;
			} else {
				printf(", %d", i_neighbor);
			}
		}
	}
	printf("\n\n");

//TODO: figure out how to properly print this on CUDA & remove header guards
#ifdef __CUDACC__
	printf("Cannot print warp update Jacobian & Hessian from CUDA device. Set device to 'cpu' for more info here.\n\n");
	printf("Jacobian of warp updates at current warp (U-component only): %s\n"
	       "%E %E %E\n"
	       "%s\n",
	       c_yellow,
	       warp_update_Jacobian.xx, warp_update_Jacobian.xy, warp_update_Jacobian.xz,
	       c_reset);
#else
	printf("Jacobian of warp updates at current warp: %s\n"
	       "%E %E %E\n"
	       "%E %E %E\n"
	       "%E %E %E\n"
	       "%s\n",
	       c_yellow,
	       warp_update_Jacobian.xx, warp_update_Jacobian.xy, warp_update_Jacobian.xz,
	       warp_update_Jacobian.yx, warp_update_Jacobian.yy, warp_update_Jacobian.yz,
	       warp_update_Jacobian.zx, warp_update_Jacobian.zy, warp_update_Jacobian.zz,
	       c_reset);

	Matrix3f& H_u = warp_update_Hessian[0];
	Matrix3f& H_v = warp_update_Hessian[1];
	Matrix3f& H_w = warp_update_Hessian[2];

	printf("*** Hessian of warp updates at current warp *** \n"
	       "U-component:\n%s"
	       "%E %E %E\n"
	       "%E %E %E\n"
	       "%E %E %E\n"
	       "%s",
	       c_cyan,
	       H_u.xx, H_u.xy, H_u.xz,
	       H_u.yx, H_u.yy, H_u.yz,
	       H_u.zx, H_u.zy, H_u.zz,
	       c_reset);

	printf("V-component:\n%s"
	       "%E %E %E\n"
	       "%E %E %E\n"
	       "%E %E %E\n"
	       "%s",
	       c_cyan,
	       H_v.xx, H_v.xy, H_v.xz,
	       H_v.yx, H_v.yy, H_v.yz,
	       H_v.zx, H_v.zy, H_v.zz,
	       c_reset);

	printf("W-component:\n%s"
	       "%E %E %E\n"
	       "%E %E %E\n"
	       "%E %E %E\n"
	       "%s\n",
	       c_cyan,
	       H_w.xx, H_w.xy, H_w.xz,
	       H_w.yx, H_w.yy, H_w.yz,
	       H_w.zx, H_w.zy, H_w.zz,
	       c_reset);
#endif
};

_CPU_AND_GPU_CODE_
inline void PrintTikhonovTermInformation(const CONSTPTR(Vector3f*) neighbor_warps,
                                         const CONSTPTR(Vector3f)& laplacian) {

	const int neighborhood_size = 6;
	//(-1,0,0) (0,-1,0) (0,0,-1)   (1, 0, 0) (0, 1, 0) (0, 0, 1)   (1, 1, 0) (0, 1, 1) (1, 0, 1)
	const Vector3i neighbor_positions[] = {Vector3i(-1, 0, 0), Vector3i(0, -1, 0), Vector3i(0, 0, -1), Vector3i(1, 0, 0),
	                                       Vector3i(0, 1, 0), Vector3i(0, 0, 1)};

	printf("%sNeighbors' warps: \n ", c_green);
	for (int i_neightbor = 0; i_neightbor < neighborhood_size; i_neightbor++) {
		const Vector3i& pos = neighbor_positions[i_neightbor];
		const Vector3f& warp = neighbor_warps[i_neightbor];
		printf("%s%d, %d, %d  (Neighbor %d): %s%f, %f, %f\n", c_reset, pos.x, pos.y, pos.z,
		       i_neightbor, c_green, warp.x, warp.y, warp.z);
	}
	printf("\nLaplacian:\n%E, %E, %E%s\n", laplacian.x, laplacian.y, laplacian.z, c_reset);
};

_CPU_AND_GPU_CODE_
inline
void PrintLocalEnergyGradients(const Vector3f& local_data_energy_gradient,
                               const Vector3f& local_level_set_energy_gradient,
                               const Vector3f& local_smoothing_energy_gradient,
                               const Vector3f& local_complete_energy_gradient,
                               float energy_gradient_length
) {

	printf("%s(Gradients) Data: %E, %E, %E %sLevel set: %E, %E, %E %sSmoothing: %E, %E, %E\n"
	       "%sCombined: %E, %E, %E%s Combined length: %E\n",
	       c_blue, local_data_energy_gradient.x, local_data_energy_gradient.y, local_data_energy_gradient.z,
	       c_cyan, local_level_set_energy_gradient.x, local_level_set_energy_gradient.y, local_level_set_energy_gradient.z,
	       c_yellow, local_smoothing_energy_gradient.x, local_smoothing_energy_gradient.y, local_smoothing_energy_gradient.z,
	       c_green, local_complete_energy_gradient.x, local_complete_energy_gradient.y, local_complete_energy_gradient.z,
	       c_reset, energy_gradient_length);
}


template<typename TVoxel, typename TIndexData, typename TCache>
_CPU_AND_GPU_CODE_
inline void Find6ConnectedNeighborInfo(
		THREADPTR(bool)* neighbor_known, //x6, out
		THREADPTR(bool)* neighbor_truncated, //x6, out
		THREADPTR(bool)* neighbor_allocated, //x6, out
		THREADPTR(float)* neighbor_sdf, //x6, out
		const CONSTPTR(Vector3i)& voxel_position,
		const CONSTPTR(TVoxel)* voxels,
		const CONSTPTR(TIndexData)* hash_entries,
		THREADPTR(TCache)& cache) {
	int vmIndex = 0;

	TVoxel voxel;
	auto process_voxel = [&](Vector3i location, int index) {
		voxel = readVoxel(voxels, hash_entries, voxel_position + (location), vmIndex, cache);
		neighbor_allocated[index] = vmIndex != 0;
		neighbor_known[index] = voxel.flags != ITMLib::VOXEL_UNKNOWN;
		neighbor_truncated[index] = voxel.flags != ITMLib::VOXEL_NONTRUNCATED;
		neighbor_sdf[index] = TVoxel::valueToFloat(voxel.sdf);
	};
	process_voxel(Vector3i(-1, 0, 0), 0);
	process_voxel(Vector3i(0, -1, 0), 1);
	process_voxel(Vector3i(0, 0, -1), 2);

	process_voxel(Vector3i(1, 0, 0), 3);
	process_voxel(Vector3i(0, 1, 0), 4);
	process_voxel(Vector3i(0, 0, 1), 5);

}


template<typename TVoxel, typename TIndexData, typename TCache>
_CPU_AND_GPU_CODE_
inline void Print6ConnectedNeighborInfo(
		const CONSTPTR(Vector3i)& voxel_position,
		const CONSTPTR(TVoxel)* voxels,
		const CONSTPTR(TIndexData)* index_data,
		THREADPTR(TCache)& cache) {
	const int neighborhood_size = 6;
	bool neighbor_known[neighborhood_size];
	bool neighbor_truncated[neighborhood_size];
	bool neighbor_allocated[neighborhood_size];
	float neighbor_sdf[neighborhood_size];

	Find6ConnectedNeighborInfo(neighbor_known, neighbor_truncated, neighbor_allocated, neighbor_sdf,
	                           voxel_position, voxels, index_data, cache);
	const Vector3i positions[6] = {Vector3i(-1, 0, 0), Vector3i(0, -1, 0), Vector3i(0, 0, -1),
	                               Vector3i(1, 0, 0), Vector3i(0, 1, 0), Vector3i(0, 0, 1)};

	for (int i_neighbor = 0; i_neighbor < 6; i_neighbor++) {
		const Vector3i& position = positions[i_neighbor];
		printf("%s[%d,%d,%d]%s allocated: %s truncated: %s known: %s tsdf: %f\n", c_yellow, position.x, position.y,
		       position.z, c_reset,
		       neighbor_allocated[i_neighbor] ? "true" : "false", neighbor_truncated[i_neighbor] ? "true" : "false",
		       neighbor_known[i_neighbor] ? "true" : "false", neighbor_sdf[i_neighbor]);
	}
}

// endregion
//======================================================================================================================