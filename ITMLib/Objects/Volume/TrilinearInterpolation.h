//  ================================================================
//  Created by Gregory Kramida on 10/27/17.
//  Copyright (c) 2017-2000 Gregory Kramida
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

//TODO: Clean out unused versions -Greg (GitHub: Algomorph)

#include <cfloat>
#include "../../Utils/Logging/ConsolePrintColors.h"
#include "../../Utils/Enums/VoxelFlags.h"
#include "RepresentationAccess.h"

namespace ITMLib {
template<typename TVoxel, typename TCache, typename TIndexData>
_CPU_AND_GPU_CODE_
inline float ProcessTrilinearNeighbor(const CONSTPTR(TVoxel)* voxelData,
                                      const CONSTPTR(TIndexData)* voxelIndex,
                                      const CONSTPTR(Vector3i)& position,
                                      const CONSTPTR(float*) coefficients,
                                      THREADPTR(TCache)& cache,
                                      THREADPTR(bool)& struckKnownVoxels,
                                      int iNeighbor,
                                      THREADPTR(float)& sdf,
                                      THREADPTR(float)& cumulativeWeight) {
	int vmIndex;

#if !defined(__CUDACC__) && !defined(WITH_OPENMP)
	const TVoxel& v = readVoxel(voxelData, voxelIndex, position, vmIndex, cache);
#else
	const TVoxel& v = readVoxel(voxelData, voxelIndex, position, vmIndex);
#endif
	bool curKnown = v.flags != ITMLib::VOXEL_UNKNOWN;
	//_DEBUG
	//trilinear unknown filter: ON
	//float weight = coefficients[iNeighbor] * curKnown;
	//trilinear unknown filter: OFF
	float weight = coefficients[iNeighbor];

	sdf += weight * TVoxel::valueToFloat(v.sdf);
	struckKnownVoxels |= curKnown;
	cumulativeWeight += weight;
	return weight;
};

_CPU_AND_GPU_CODE_
inline void ComputeTrilinearCoefficents(const CONSTPTR(Vector3f)& point, THREADPTR(Vector3i)& pos,
                                        THREADPTR(float*) coefficients /*x8*/) {
	Vector3f ratios;
	Vector3f inverseRatios(1.0f);

	TO_INT_FLOOR3(pos, ratios, point);
	inverseRatios -= ratios;

//@formatter:off
	coefficients[0] = inverseRatios.x * inverseRatios.y * inverseRatios.z; //000
	coefficients[1] = ratios.x *        inverseRatios.y * inverseRatios.z; //100
	coefficients[2] = inverseRatios.x * ratios.y *        inverseRatios.z; //010
	coefficients[3] = ratios.x *        ratios.y *        inverseRatios.z; //110
	coefficients[4] = inverseRatios.x * inverseRatios.y * ratios.z;        //001
	coefficients[5] = ratios.x *        inverseRatios.y * ratios.z;        //101
	coefficients[6] = inverseRatios.x * ratios.y *        ratios.z;        //011
	coefficients[7] = ratios.x *        ratios.y *        ratios.z;        //111
//@formatter:on
}


/**
 * \brief Given an arbitrary (float-valued) point, use trilinear interpolation to get the signed distance function value
 * at this point.
 * \details also determines whether any of the voxels in the sampling space (2x2x2 voxels) has a known (established)
 * value to discriminate it from the newly-initialized voxels set to the default sdf value.
 * \tparam TVoxel
 * \tparam TCache
 * \param voxelData
 * \param voxelIndex
 * \param point
 * \param cache
 * \param struckKnownVoxels
 * \param verbose - print additional information about the operation
 * \return
 */
template<typename TVoxel, typename TCache, typename TIndexData>
_CPU_AND_GPU_CODE_
inline float _DEBUG_InterpolateTrilinearly_StruckKnown(const CONSTPTR(TVoxel)* voxelData,
                                                       const CONSTPTR(TIndexData)* voxelIndex,
                                                       const CONSTPTR(Vector3f)& point,
                                                       THREADPTR(TCache)& cache,
                                                       bool& struckKnownVoxels,
                                                       bool verbose) {

	const int neighborCount = 8;
	const Vector3i positions[neighborCount] = {Vector3i(0, 0, 0), Vector3i(1, 0, 0),
	                                           Vector3i(0, 1, 0), Vector3i(1, 1, 0),
	                                           Vector3i(0, 0, 1), Vector3i(1, 0, 1),
	                                           Vector3i(0, 1, 1), Vector3i(1, 1, 1)};
	float coefficients[neighborCount];
	Vector3i pos;
	ComputeTrilinearCoefficents(point, pos, coefficients);

	if (verbose) {
		printf("%s*** Printing interpolation data for point (%f, %f, %f) ***%s)\nTruncated position: (%d, %d, %d).\n",
		       bright_cyan, point.x, point.y, point.z, reset, pos.x, pos.y, pos.z);
	}

	struckKnownVoxels = false;
	float cumulativeWeight = 0.0f;
	float sdf = 0.0f;

	for (int iNeighbor = 0; iNeighbor < neighborCount; iNeighbor++) {
		float weight = ProcessTrilinearNeighbor(voxelData, voxelIndex, pos + positions[iNeighbor],
		                                        coefficients, cache,
		                                        struckKnownVoxels, iNeighbor, sdf, cumulativeWeight);
		if (verbose) {
			const Vector3i& npos = positions[iNeighbor];
			printf("Neighbor position: (%d, %d, %d) Sdf: %E Weight: %E\n", npos.x, npos.y, npos.z,
			       sdf, weight);
		}

	}
	if (cumulativeWeight > FLT_EPSILON) {
		sdf /= cumulativeWeight;
	} else {
		sdf = TVoxel::SDF_initialValue();
	}

	if (verbose) {
		printf("New sdf: %E\n", sdf);
	}

	return sdf;
}


//sdf and color + knownVoxelHit + smart weights
/**
 * \brief Trilinearly interpolates the SDF value and color in the given voxel grid at the provided 3D coordinate
 * @details Computes cumulative interpolation weight for voxels that are not marked as VOXEL_UNKNOWN,
 * only takes the "known" voxels into account
 * \tparam TVoxel
 * \tparam TCache
 * \param voxelData [in] Array of voxels in memory
 * \param voxelHash [in] Indexing structure, containing voxel hash block entries
 * \param point 3D [in] location at which to interpolate
 * \param cache [in,out] Cache to speed up hash indexing
 * \param color [out] interpolated color value, as float vector
 * \param struckKnownVoxels [out] whether or not known voxels with non-zero interpolation weights were sampled during the procedure
 * \param struckNonTruncated [out] whether or not narrow band voxels with non-zero weights were sampled during the procedure
 * \param cumulativeWeight [out] cumulative interpolation weight for all sampled voxels excluding those marked VOXEL_UNKNOWN
 * \return interpolated sdf value
 */
template<class TVoxel, typename TCache>
_CPU_AND_GPU_CODE_
inline float InterpolateTrilinearly_SdfColor_StruckNonTruncatedAndKnown_SmartWeights(
		const CONSTPTR(TVoxel)* voxelData,
		const CONSTPTR(ITMLib::HashEntry)* voxelHash,
		const THREADPTR(Vector3f)& point,
		THREADPTR(TCache)& cache,
		THREADPTR(Vector3f)& color,
		THREADPTR(bool)& struckKnownVoxels,
		THREADPTR(bool)& struckNonTruncated,
		THREADPTR(float)& cumulativeWeight) {
	Vector3f ratios;
	Vector3f inverseRatios(1.0f);
	Vector3i pos;
	int vmIndex;
	TO_INT_FLOOR3(pos, ratios, point);
	inverseRatios -= ratios;
	const int neighborCount = 8;
	const Vector3i positions[neighborCount] = {Vector3i(0, 0, 0), Vector3i(1, 0, 0),
	                                           Vector3i(0, 1, 0), Vector3i(1, 1, 0),
	                                           Vector3i(0, 0, 1), Vector3i(1, 0, 1),
	                                           Vector3i(0, 1, 1), Vector3i(1, 1, 1)};
	float sdf = 0.0f;
	color = Vector3f(0.0f);
	float coefficients[neighborCount];

	struckKnownVoxels = false;
	struckNonTruncated = false;
	cumulativeWeight = 0.0f;

	//@formatter:off
	coefficients[0] = inverseRatios.x * inverseRatios.y * inverseRatios.z; //000
	coefficients[1] = ratios.x *        inverseRatios.y * inverseRatios.z; //100
	coefficients[2] = inverseRatios.x * ratios.y *        inverseRatios.z; //010
	coefficients[3] = ratios.x *        ratios.y *        inverseRatios.z; //110
	coefficients[4] = inverseRatios.x * inverseRatios.y * ratios.z;        //001
	coefficients[5] = ratios.x *        inverseRatios.y * ratios.z;        //101
	coefficients[6] = inverseRatios.x * ratios.y *        ratios.z;        //011
	coefficients[7] = ratios.x *        ratios.y *        ratios.z;        //111
	//@formatter:on

	for (int iNeighbor = 0; iNeighbor < neighborCount; iNeighbor++) {
		const TVoxel& v = readVoxel(voxelData, voxelHash, pos + (positions[iNeighbor]), vmIndex, cache);
		bool curKnown = v.flags != ITMLib::VOXEL_UNKNOWN;
		bool curNonTruncated = v.flags == ITMLib::VOXEL_NONTRUNCATED;
		float weight = coefficients[iNeighbor] * curKnown;
		sdf += weight * TVoxel::valueToFloat(v.sdf);
		color += weight * TO_FLOAT3(v.clr);
		struckKnownVoxels |= (bool) (weight * curKnown);
		struckNonTruncated |= (bool) (weight * curNonTruncated);
		cumulativeWeight += weight;
	}

	return sdf;
}

//sdf + knownVoxelHit + smart weights
template<class TVoxel, typename TCache>
_CPU_AND_GPU_CODE_
inline float InterpolateTrilinearly_MultiSdf_StruckNonTruncatedAndKnown_SmartWeights(
		const CONSTPTR(TVoxel)* voxelData,
		const CONSTPTR(ITMLib::HashEntry)* voxelHash,
		const THREADPTR(Vector3f)& point,
		const CONSTPTR(int)& sourceSdfIndex,

		THREADPTR(TCache)& cache,
		THREADPTR(bool)& struckKnownVoxels,
		THREADPTR(bool)& struckNonTruncated,
		THREADPTR(float)& cumulativeWeight) {
	Vector3f ratios;
	Vector3f inverseRatios(1.0f);
	Vector3i pos;
	int vmIndex;
	TO_INT_FLOOR3(pos, ratios, point);
	inverseRatios -= ratios;
	const int neighborCount = 8;
	const Vector3i positions[neighborCount] = {Vector3i(0, 0, 0), Vector3i(1, 0, 0),
	                                           Vector3i(0, 1, 0), Vector3i(1, 1, 0),
	                                           Vector3i(0, 0, 1), Vector3i(1, 0, 1),
	                                           Vector3i(0, 1, 1), Vector3i(1, 1, 1)};
	float sdf = 0.0f;
	float coefficients[neighborCount];

	struckKnownVoxels = false;
	struckNonTruncated = false;
	cumulativeWeight = 0.0f;

	//@formatter:off
	coefficients[0] = inverseRatios.x * inverseRatios.y * inverseRatios.z; //000
	coefficients[1] = ratios.x *        inverseRatios.y * inverseRatios.z; //100
	coefficients[2] = inverseRatios.x * ratios.y *        inverseRatios.z; //010
	coefficients[3] = ratios.x *        ratios.y *        inverseRatios.z; //110
	coefficients[4] = inverseRatios.x * inverseRatios.y * ratios.z;        //001
	coefficients[5] = ratios.x *        inverseRatios.y * ratios.z;        //101
	coefficients[6] = inverseRatios.x * ratios.y *        ratios.z;        //011
	coefficients[7] = ratios.x *        ratios.y *        ratios.z;        //111
	//@formatter:on

	for (int iNeighbor = 0; iNeighbor < neighborCount; iNeighbor++) {
		const TVoxel& v = readVoxel(voxelData, voxelHash, pos + (positions[iNeighbor]), vmIndex, cache);
		bool curKnown = v.flags != ITMLib::VOXEL_UNKNOWN;
		bool curNonTruncated = v.flags == ITMLib::VOXEL_NONTRUNCATED;
		float weight = coefficients[iNeighbor] * curKnown;
		sdf += weight * TVoxel::valueToFloat(v.sdf_values[sourceSdfIndex]);
		struckKnownVoxels |= (bool) (weight * curKnown);
		struckNonTruncated |= (bool) (weight * curNonTruncated);
		cumulativeWeight += weight;
	}

	return sdf;
}

//sdf + knownVoxelHit + smart weights
template<class TVoxel, typename TCache>
_CPU_AND_GPU_CODE_
inline float InterpolateTrilinearly_Sdf_StruckNonTruncatedAndKnown_SmartWeights(
		const CONSTPTR(TVoxel)* voxelData,
		const CONSTPTR(ITMLib::HashEntry)* voxelHash,
		const THREADPTR(Vector3f)& point,
		THREADPTR(TCache)& cache,
		THREADPTR(bool)& struckKnownVoxels,
		THREADPTR(bool)& struckNonTruncated,
		THREADPTR(float)& cumulativeWeight) {
	Vector3f ratios;
	Vector3f inverseRatios(1.0f);
	Vector3i pos;
	int vmIndex;
	TO_INT_FLOOR3(pos, ratios, point);
	inverseRatios -= ratios;
	const int neighborCount = 8;
	const Vector3i positions[neighborCount] = {Vector3i(0, 0, 0), Vector3i(1, 0, 0),
	                                           Vector3i(0, 1, 0), Vector3i(1, 1, 0),
	                                           Vector3i(0, 0, 1), Vector3i(1, 0, 1),
	                                           Vector3i(0, 1, 1), Vector3i(1, 1, 1)};
	float sdf = 0.0f;
	float coefficients[neighborCount];

	struckKnownVoxels = false;
	struckNonTruncated = false;
	cumulativeWeight = 0.0f;

	//@formatter:off
	coefficients[0] = inverseRatios.x * inverseRatios.y * inverseRatios.z; //000
	coefficients[1] = ratios.x *        inverseRatios.y * inverseRatios.z; //100
	coefficients[2] = inverseRatios.x * ratios.y *        inverseRatios.z; //010
	coefficients[3] = ratios.x *        ratios.y *        inverseRatios.z; //110
	coefficients[4] = inverseRatios.x * inverseRatios.y * ratios.z;        //001
	coefficients[5] = ratios.x *        inverseRatios.y * ratios.z;        //101
	coefficients[6] = inverseRatios.x * ratios.y *        ratios.z;        //011
	coefficients[7] = ratios.x *        ratios.y *        ratios.z;        //111
	//@formatter:on

	for (int iNeighbor = 0; iNeighbor < neighborCount; iNeighbor++) {
		const TVoxel& v = readVoxel(voxelData, voxelHash, pos + (positions[iNeighbor]), vmIndex, cache);
		bool curKnown = v.flags != ITMLib::VOXEL_UNKNOWN;
		bool curNonTruncated = v.flags == ITMLib::VOXEL_NONTRUNCATED;
		float weight = coefficients[iNeighbor] * curKnown;
		sdf += weight * TVoxel::valueToFloat(v.sdf);
		struckKnownVoxels |= (bool) (weight * curKnown);
		struckNonTruncated |= (bool) (weight * curNonTruncated);
		cumulativeWeight += weight;
	}

	return sdf;
}


//_DEBUG -- special treatment of truncated values, use voxels with semantic information only!
//sdf, color, pick maximum weights, get confidence
template<class TVoxel, typename TCache>
_CPU_AND_GPU_CODE_
inline float InterpolateTrilinearly_TruncatedSignCopy_StruckNarrowBand(const CONSTPTR(TVoxel)* voxelData,
                                                                       const CONSTPTR(ITMLib::HashEntry)* voxelHash,
                                                                       const CONSTPTR(Vector3f)& point,
                                                                       THREADPTR(TCache)& cache,
                                                                       THREADPTR(Vector3f)& color,
                                                                       THREADPTR(float)& confidence,
                                                                       THREADPTR(bool)& struckNarrowBand) {
	float sdfRes1, sdfRes2;
	float confRes1, confRes2;
	Vector3f clrRes1, clrRes2;
	int vmIndex = false;
	Vector3f coeff;
	Vector3i pos;

	TO_INT_FLOOR3(pos, coeff, point);
	const int neighborCount = 8;
	float sdfs[neighborCount];
	Vector3f colors[neighborCount];
	float confs[neighborCount];
	bool found[neighborCount] = {0};
	const Vector3i positions[neighborCount] = {Vector3i(0, 0, 0), Vector3i(1, 0, 0),
	                                           Vector3i(0, 1, 0), Vector3i(1, 1, 0),
	                                           Vector3i(0, 0, 1), Vector3i(1, 0, 1),
	                                           Vector3i(0, 1, 1), Vector3i(1, 1, 1)};
	struckNarrowBand = false;
	float sumFound = 0.0f;
	for (int iNeighbor = 0; iNeighbor < neighborCount; iNeighbor++) {
		const TVoxel& v = readVoxel(voxelData, voxelHash, pos + (positions[iNeighbor]), vmIndex, cache);
		sdfs[iNeighbor] = TVoxel::valueToFloat(v.sdf);
		confs[iNeighbor] = v.confidence;
		colors[iNeighbor] = TO_FLOAT3(v.clr);
		bool foundCur = v.flags != ITMLib::VOXEL_TRUNCATED;
		struckNarrowBand |= foundCur;
		found[iNeighbor] = foundCur;
		sumFound += foundCur * v.sdf;
	}
	if (!struckNarrowBand) {
		return TVoxel::SDF_initialValue();//cannot really say anything about the sdf here
	}

	float truncated = std::copysign(1.0f, sumFound);
	for (int iNeighbor = 0; iNeighbor < neighborCount; iNeighbor++) {
		if (!found[iNeighbor]) {
			sdfs[iNeighbor] = truncated;
		}
	}

	float oneMinusCoeffX = 1.0f - coeff.x;
	float oneMinusCoeffY = 1.0f - coeff.y;

	sdfRes1 = oneMinusCoeffY * (oneMinusCoeffX * sdfs[0] + coeff.x * sdfs[1])
	          + coeff.y * (oneMinusCoeffX * sdfs[2] + coeff.x * sdfs[3]);
	sdfRes2 = oneMinusCoeffY * (oneMinusCoeffX * sdfs[4] + coeff.x * sdfs[5])
	          + coeff.y * (oneMinusCoeffX * sdfs[6] + coeff.x * sdfs[7]);
	confRes1 = oneMinusCoeffY * (oneMinusCoeffX * confs[0] + coeff.x * confs[1])
	           + coeff.y * (oneMinusCoeffX * confs[2] + coeff.x * confs[3]);
	confRes2 = oneMinusCoeffY * (oneMinusCoeffX * confs[4] + coeff.x * confs[5])
	           + coeff.y * (oneMinusCoeffX * confs[6] + coeff.x * confs[7]);
	clrRes1 = oneMinusCoeffY * (oneMinusCoeffX * colors[0] + coeff.x * colors[1])
	          + coeff.y * (oneMinusCoeffX * colors[2] + coeff.x * colors[3]);
	clrRes2 = oneMinusCoeffY * (oneMinusCoeffX * colors[4] + coeff.x * colors[5])
	          + coeff.y * (oneMinusCoeffX * colors[6] + coeff.x * colors[7]);

	float sdf = (1.0f - coeff.z) * sdfRes1 + coeff.z * sdfRes2;
	confidence = (1.0f - coeff.z) * confRes1 + coeff.z * confRes2;
	color = (1.0f - coeff.z) * clrRes1 + coeff.z * clrRes2;
	return sdf;
}

//_DEBUG -- special treatment of truncated values, use voxels with semantic information only!
//sdf, color, pick maximum weights, get confidence
template<class TVoxel, typename TCache>
_CPU_AND_GPU_CODE_
inline float InterpolateTrilinearly_NegativeTruncatedSignCopy_PositiveTruncatedNoChange(
		const CONSTPTR(TVoxel)* voxelData,
		const CONSTPTR(ITMLib::HashEntry)* voxelHash,
		const CONSTPTR(Vector3f)& point,
		THREADPTR(TCache)& cache,
		THREADPTR(Vector3f)& color,
		THREADPTR(bool)& struckKnownVoxels) {
	float sdfRes1, sdfRes2;
	Vector3f clrRes1, clrRes2;
	int vmIndex = false;
	Vector3f coeff;
	Vector3i pos;

	TO_INT_FLOOR3(pos, coeff, point);
	const int neighborCount = 8;
	float sdfs[neighborCount];
	Vector3f colors[neighborCount];
	bool found[neighborCount] = {0};
	const Vector3i positions[neighborCount] = {Vector3i(0, 0, 0), Vector3i(1, 0, 0),
	                                           Vector3i(0, 1, 0), Vector3i(1, 1, 0),
	                                           Vector3i(0, 0, 1), Vector3i(1, 0, 1),
	                                           Vector3i(0, 1, 1), Vector3i(1, 1, 1)};
	struckKnownVoxels = false;
	float sumFound = 0.0f;
	for (int iNeighbor = 0; iNeighbor < neighborCount; iNeighbor++) {
		const TVoxel& v = readVoxel(voxelData, voxelHash, pos + (positions[iNeighbor]), vmIndex, cache);
		float currentSdf = TVoxel::valueToFloat(v.sdf);
		sdfs[iNeighbor] = currentSdf;
		colors[iNeighbor] = TO_FLOAT3(v.clr);
		bool foundCur = currentSdf != TVoxel::SDF_initialValue();
		struckKnownVoxels |= foundCur;
		found[iNeighbor] = foundCur;
		sumFound += foundCur * v.sdf;
	}
	if (!struckKnownVoxels) {
		return TVoxel::SDF_initialValue();//cannot really say anything about the sdf here
	}

	float truncated = std::copysign(1.0f, sumFound);
	for (int iNeighbor = 0; iNeighbor < neighborCount; iNeighbor++) {
		if (!found[iNeighbor]) {
			sdfs[iNeighbor] = truncated;
		}
	}

	float oneMinusCoeffX = 1.0f - coeff.x;
	float oneMinusCoeffY = 1.0f - coeff.y;

	sdfRes1 = oneMinusCoeffY * (oneMinusCoeffX * sdfs[0] + coeff.x * sdfs[1])
	          + coeff.y * (oneMinusCoeffX * sdfs[2] + coeff.x * sdfs[3]);
	sdfRes2 = oneMinusCoeffY * (oneMinusCoeffX * sdfs[4] + coeff.x * sdfs[5])
	          + coeff.y * (oneMinusCoeffX * sdfs[6] + coeff.x * sdfs[7]);
	clrRes1 = oneMinusCoeffY * (oneMinusCoeffX * colors[0] + coeff.x * colors[1])
	          + coeff.y * (oneMinusCoeffX * colors[2] + coeff.x * colors[3]);
	clrRes2 = oneMinusCoeffY * (oneMinusCoeffX * colors[4] + coeff.x * colors[5])
	          + coeff.y * (oneMinusCoeffX * colors[6] + coeff.x * colors[7]);

	float sdf = (1.0f - coeff.z) * sdfRes1 + coeff.z * sdfRes2;
	color = (1.0f - coeff.z) * clrRes1 + coeff.z * clrRes2;
	return sdf;
}

//sdf only
template<class TVoxel, typename TCache>
_CPU_AND_GPU_CODE_
inline float InterpolateTrilinearly_NegativeTruncatedSignCopy_PositiveTruncatedNoChange(
		const CONSTPTR(TVoxel)* voxelData,
		const CONSTPTR(ITMLib::HashEntry)* voxelHash,
		const CONSTPTR(Vector3f)& point,
		THREADPTR(TCache)& cache,
		THREADPTR(bool)& struckKnownVoxels) {
	float sdfRes1, sdfRes2;
	int vmIndex = false;
	Vector3f coeff;
	Vector3i pos;
	struckKnownVoxels = false;

	TO_INT_FLOOR3(pos, coeff, point);
	const int neighborCount = 8;
	float sdfs[neighborCount];
	bool found[neighborCount] = {0};
	const Vector3i positions[neighborCount] = {Vector3i(0, 0, 0), Vector3i(1, 0, 0),
	                                           Vector3i(0, 1, 0), Vector3i(1, 1, 0),
	                                           Vector3i(0, 0, 1), Vector3i(1, 0, 1),
	                                           Vector3i(0, 1, 1), Vector3i(1, 1, 1)};
	float sumFound = 0.0f;
	for (int iNeighbor = 0; iNeighbor < neighborCount; iNeighbor++) {
		const TVoxel& v = readVoxel(voxelData, voxelHash, pos + (positions[iNeighbor]), vmIndex, cache);
		float currentSdf = TVoxel::valueToFloat(v.sdf);
		sdfs[iNeighbor] = currentSdf;
		bool foundCur = currentSdf != TVoxel::SDF_initialValue();
		struckKnownVoxels |= foundCur;
		found[iNeighbor] = foundCur;
		sumFound += foundCur * v.sdf;
	}
	if (!struckKnownVoxels) {
		return TVoxel::SDF_initialValue();//cannot really say anything about the sdf here
	}

	float truncatedValue = std::copysign(1.0f, sumFound);
	for (int iNeighbor = 0; iNeighbor < neighborCount; iNeighbor++) {
		if (!found[iNeighbor]) {
			sdfs[iNeighbor] = truncatedValue;
		}
	}

	float oneMinusCoeffX = 1.0f - coeff.x;
	float oneMinusCoeffY = 1.0f - coeff.y;

	sdfRes1 = oneMinusCoeffY * (oneMinusCoeffX * sdfs[0] + coeff.x * sdfs[1])
	          + coeff.y * (oneMinusCoeffX * sdfs[2] + coeff.x * sdfs[3]);
	sdfRes2 = oneMinusCoeffY * (oneMinusCoeffX * sdfs[4] + coeff.x * sdfs[5])
	          + coeff.y * (oneMinusCoeffX * sdfs[6] + coeff.x * sdfs[7]);

	float sdf = (1.0f - coeff.z) * sdfRes1 + coeff.z * sdfRes2;
	return sdf;
}


//sdf only, version with replacing all truncated voxels with given value
template<class TVoxel, typename TCache>
_CPU_AND_GPU_CODE_
inline float InterpolateTrilinearly_SetTruncatedToVal(const CONSTPTR(TVoxel)* voxelData,
                                                      const CONSTPTR(ITMLib::HashEntry)* voxelHash,
                                                      const CONSTPTR(float)& truncationReplacement,
                                                      const CONSTPTR(Vector3f)& point,
                                                      THREADPTR(TCache)& cache) {
	float sdfRes1, sdfRes2, sdfV1, sdfV2;
	int vmIndex = false;
	Vector3f coeff;
	Vector3i pos;
	TO_INT_FLOOR3(pos, coeff, point);
#define PROCESS_VOXEL(suffix, coord)\
    {\
        const TVoxel& v = readVoxel(voxelData, voxelHash, pos + (coord), vmIndex, cache);\
        sdfV##suffix = v.flags != ITMLib::VOXEL_TRUNCATED ? v.sdf : truncationReplacement;\
    }
	PROCESS_VOXEL(1, Vector3i(0, 0, 0))
	PROCESS_VOXEL(2, Vector3i(1, 0, 0))
	sdfRes1 = (1.0f - coeff.x) * sdfV1 + coeff.x * sdfV2;
	PROCESS_VOXEL(1, Vector3i(0, 1, 0))
	PROCESS_VOXEL(2, Vector3i(1, 1, 0))
	sdfRes1 = (1.0f - coeff.y) * sdfRes1 + coeff.y * ((1.0f - coeff.x) * sdfV1 + coeff.x * sdfV2);
	PROCESS_VOXEL(1, Vector3i(0, 0, 1))
	PROCESS_VOXEL(2, Vector3i(1, 0, 1))
	sdfRes2 = (1.0f - coeff.x) * sdfV1 + coeff.x * sdfV2;
	PROCESS_VOXEL(1, Vector3i(0, 1, 1))
	PROCESS_VOXEL(2, Vector3i(1, 1, 1))
	sdfRes2 = (1.0f - coeff.y) * sdfRes2 + coeff.y * ((1.0f - coeff.x) * sdfV1 + coeff.x * sdfV2);
#undef PROCESS_VOXEL
	float sdf = TVoxel::valueToFloat((1.0f - coeff.z) * sdfRes1 + coeff.z * sdfRes2);

	return sdf;
}

//sdf only, version with replacing all truncated voxels with given value; determines whether narrow band was hit
template<class TVoxel, typename TCache>
_CPU_AND_GPU_CODE_
inline float InterpolateTrilinearly_SetTruncatedToVal_StruckNarrowBand(const CONSTPTR(TVoxel)* voxelData,
                                                                       const CONSTPTR(ITMLib::HashEntry)* voxelHash,
                                                                       const CONSTPTR(float)& truncationReplacement,
                                                                       const CONSTPTR(Vector3f)& point,
                                                                       THREADPTR(TCache)& cache,
                                                                       THREADPTR(bool)& struckNarrowBand) {
	float sdfRes1, sdfRes2, sdfV1, sdfV2;
	int vmIndex = false;
	Vector3f coeff;
	Vector3i pos;
	TO_INT_FLOOR3(pos, coeff, point);
	struckNarrowBand = false;
#define PROCESS_VOXEL(suffix, coord)\
    {\
        const TVoxel& v = readVoxel(voxelData, voxelHash, pos + (coord), vmIndex, cache);\
        if(v.flags == ITMLib::VOXEL_TRUNCATED){\
            sdfV##suffix = truncationReplacement;\
        }else{\
            struckNarrowBand = true;\
            sdfV##suffix =  v.sdf;\
        }\
    }
	PROCESS_VOXEL(1, Vector3i(0, 0, 0))
	PROCESS_VOXEL(2, Vector3i(1, 0, 0))
	sdfRes1 = (1.0f - coeff.x) * sdfV1 + coeff.x * sdfV2;
	PROCESS_VOXEL(1, Vector3i(0, 1, 0))
	PROCESS_VOXEL(2, Vector3i(1, 1, 0))
	sdfRes1 = (1.0f - coeff.y) * sdfRes1 + coeff.y * ((1.0f - coeff.x) * sdfV1 + coeff.x * sdfV2);
	PROCESS_VOXEL(1, Vector3i(0, 0, 1))
	PROCESS_VOXEL(2, Vector3i(1, 0, 1))
	sdfRes2 = (1.0f - coeff.x) * sdfV1 + coeff.x * sdfV2;
	PROCESS_VOXEL(1, Vector3i(0, 1, 1))
	PROCESS_VOXEL(2, Vector3i(1, 1, 1))
	sdfRes2 = (1.0f - coeff.y) * sdfRes2 + coeff.y * ((1.0f - coeff.x) * sdfV1 + coeff.x * sdfV2);
#undef PROCESS_VOXEL
	float sdf = TVoxel::valueToFloat((1.0f - coeff.z) * sdfRes1 + coeff.z * sdfRes2);

	return sdf;
}

template<class TVoxel, typename TCache>
_CPU_AND_GPU_CODE_
inline float _DEBUG_InterpolateTrilinearly_SetUnknownToVal(const CONSTPTR(TVoxel)* voxelData,
                                                           const CONSTPTR(ITMLib::HashEntry)* voxelHash,
                                                           const CONSTPTR(float)& defaultReplacement,
                                                           const CONSTPTR(Vector3f)& point,
                                                           THREADPTR(TCache)& cache,
                                                           THREADPTR(bool)& struckNonTruncated,
                                                           THREADPTR(bool)& struckKnownVoxels) {
	Vector3f ratios;
	Vector3f inverseRatios(1.0f);
	Vector3i pos;
	int vmIndex;
	TO_INT_FLOOR3(pos, ratios, point);
	inverseRatios -= ratios;
	const int neighborCount = 8;
	const Vector3i positions[neighborCount] = {Vector3i(0, 0, 0), Vector3i(1, 0, 0),
	                                           Vector3i(0, 1, 0), Vector3i(1, 1, 0),
	                                           Vector3i(0, 0, 1), Vector3i(1, 0, 1),
	                                           Vector3i(0, 1, 1), Vector3i(1, 1, 1)};
	float sdf = 0.0f;
	float coefficients[neighborCount];

	struckKnownVoxels = false;
	struckNonTruncated = false;
	float cumulativeWeight = 0.0f;

	//@formatter:off
	coefficients[0] = inverseRatios.x * inverseRatios.y * inverseRatios.z; //000
	coefficients[1] = ratios.x *        inverseRatios.y * inverseRatios.z; //100
	coefficients[2] = inverseRatios.x * ratios.y *        inverseRatios.z; //010
	coefficients[3] = ratios.x *        ratios.y *        inverseRatios.z; //110
	coefficients[4] = inverseRatios.x * inverseRatios.y * ratios.z;        //001
	coefficients[5] = ratios.x *        inverseRatios.y * ratios.z;        //101
	coefficients[6] = inverseRatios.x * ratios.y *        ratios.z;        //011
	coefficients[7] = ratios.x *        ratios.y *        ratios.z;        //111

	for (int iNeighbor = 0; iNeighbor < neighborCount; iNeighbor++) {
		const TVoxel& v = readVoxel(voxelData, voxelHash, pos + (positions[iNeighbor]), vmIndex, cache);
		bool curKnown = v.flags != ITMLib::VOXEL_UNKNOWN;
		bool curNonTruncated = v.flags == ITMLib::VOXEL_NONTRUNCATED;
		float weight = coefficients[iNeighbor] * curKnown;
		sdf += weight * (curKnown ? TVoxel::valueToFloat(v.sdf) : defaultReplacement);
		struckKnownVoxels |= weight * curKnown > FLT_EPSILON;
		struckNonTruncated |= weight * curNonTruncated > FLT_EPSILON;
		cumulativeWeight += weight;

	}
	return sdf;
}


template<class TVoxel, typename TCache>
_CPU_AND_GPU_CODE_
inline float _DEBUG_InterpolateTrilinearly(const CONSTPTR(TVoxel)* voxelData,
                                           const CONSTPTR(ITMLib::HashEntry)* voxelHash,
                                           const CONSTPTR(Vector3f)& point,
                                           THREADPTR(TCache)& cache,
                                           THREADPTR(bool)& struckNonTruncated,
                                           THREADPTR(bool)& struckKnownVoxels) {
	Vector3f ratios;
	Vector3f inverseRatios(1.0f);
	Vector3i pos;
	int vmIndex;
	TO_INT_FLOOR3(pos, ratios, point);
	inverseRatios -= ratios;
	const int neighborCount = 8;
	const Vector3i positions[neighborCount] = {Vector3i(0, 0, 0), Vector3i(1, 0, 0),
	                                           Vector3i(0, 1, 0), Vector3i(1, 1, 0),
	                                           Vector3i(0, 0, 1), Vector3i(1, 0, 1),
	                                           Vector3i(0, 1, 1), Vector3i(1, 1, 1)};
	float sdf = 0.0f;
	float coefficients[neighborCount];

	struckKnownVoxels = false;
	struckNonTruncated = false;
	float cumulativeWeight = 0.0f;

	//@formatter:off
	coefficients[0] = inverseRatios.x * inverseRatios.y * inverseRatios.z; //000
	coefficients[1] = ratios.x *        inverseRatios.y * inverseRatios.z; //100
	coefficients[2] = inverseRatios.x * ratios.y *        inverseRatios.z; //010
	coefficients[3] = ratios.x *        ratios.y *        inverseRatios.z; //110
	coefficients[4] = inverseRatios.x * inverseRatios.y * ratios.z;        //001
	coefficients[5] = ratios.x *        inverseRatios.y * ratios.z;        //101
	coefficients[6] = inverseRatios.x * ratios.y *        ratios.z;        //011
	coefficients[7] = ratios.x *        ratios.y *        ratios.z;        //111

	for (int iNeighbor = 0; iNeighbor < neighborCount; iNeighbor++) {
		const TVoxel& v = readVoxel(voxelData, voxelHash, pos + (positions[iNeighbor]), vmIndex, cache);
		bool curKnown = v.flags != ITMLib::VOXEL_UNKNOWN;
		bool curNonTruncated = v.flags == ITMLib::VOXEL_NONTRUNCATED;
		float weight = coefficients[iNeighbor] * curKnown;
		sdf += TVoxel::valueToFloat(v.sdf);
		struckKnownVoxels |= weight * curKnown > FLT_EPSILON;
		struckNonTruncated |= weight * curNonTruncated > FLT_EPSILON;
		cumulativeWeight += weight;

	}
	return sdf;
}

//VoxelBlockHash version of the same function
template<class TVoxel, typename TCache>
_CPU_AND_GPU_CODE_
inline float _DEBUG_InterpolateTrilinearly(const CONSTPTR(TVoxel)* voxelData,
                                           const CONSTPTR(ITMLib::HashEntry)* voxelHash,
                                           const CONSTPTR(Vector3f)& point,
                                           THREADPTR(TCache)& cache) {
	Vector3f ratios;
	Vector3f inverseRatios(1.0f);
	Vector3i pos;
	int vmIndex;
	TO_INT_FLOOR3(pos, ratios, point);
	inverseRatios -= ratios;
	const int neighborCount = 8;
	const Vector3i positions[neighborCount] = {Vector3i(0, 0, 0), Vector3i(1, 0, 0),
	                                           Vector3i(0, 1, 0), Vector3i(1, 1, 0),
	                                           Vector3i(0, 0, 1), Vector3i(1, 0, 1),
	                                           Vector3i(0, 1, 1), Vector3i(1, 1, 1)};
	float sdf = 0.0f;
	float coefficients[neighborCount];

	float cumulativeWeight = 0.0f;

	//@formatter:off
	coefficients[0] = inverseRatios.x * inverseRatios.y * inverseRatios.z; //000
	coefficients[1] = ratios.x *        inverseRatios.y * inverseRatios.z; //100
	coefficients[2] = inverseRatios.x * ratios.y *        inverseRatios.z; //010
	coefficients[3] = ratios.x *        ratios.y *        inverseRatios.z; //110
	coefficients[4] = inverseRatios.x * inverseRatios.y * ratios.z;        //001
	coefficients[5] = ratios.x *        inverseRatios.y * ratios.z;        //101
	coefficients[6] = inverseRatios.x * ratios.y *        ratios.z;        //011
	coefficients[7] = ratios.x *        ratios.y *        ratios.z;        //111

	for (int iNeighbor = 0; iNeighbor < neighborCount; iNeighbor++) {
		const TVoxel& v = readVoxel(voxelData, voxelHash, pos + (positions[iNeighbor]), vmIndex, cache);
		bool curKnown = v.flags != ITMLib::VOXEL_UNKNOWN;
		float weight = coefficients[iNeighbor] * curKnown;
		sdf += weight * TVoxel::valueToFloat(v.sdf);
		cumulativeWeight += weight;
	}
	return sdf;
}

template<class TVoxel, typename TCache>
_CPU_AND_GPU_CODE_
inline float _DEBUG_InterpolateTrilinearly_SetTruncatedToVal_StruckChecks(const CONSTPTR(TVoxel)* voxelData,
                                                                        const CONSTPTR(ITMLib::HashEntry)* voxelHash,
                                                                        const CONSTPTR(float)& truncationReplacement,
                                                                        const CONSTPTR(Vector3f)& point,
                                                                        THREADPTR(TCache)& cache,
                                                                        THREADPTR(bool)& struckNonTruncated,
                                                                        THREADPTR(bool)& struckKnownVoxels) {
	Vector3f ratios;
	Vector3f inverseRatios(1.0f);
	Vector3i pos;
	int vmIndex;
	TO_INT_FLOOR3(pos, ratios, point);
	inverseRatios -= ratios;
	const int neighborCount = 8;
	const Vector3i positions[neighborCount] = {Vector3i(0, 0, 0), Vector3i(1, 0, 0),
	                                           Vector3i(0, 1, 0), Vector3i(1, 1, 0),
	                                           Vector3i(0, 0, 1), Vector3i(1, 0, 1),
	                                           Vector3i(0, 1, 1), Vector3i(1, 1, 1)};
	float sdf = 0.0f;
	float coefficients[neighborCount];

	struckKnownVoxels = false;
	struckNonTruncated = false;
	float cumulativeWeight = 0.0f;

	//@formatter:off
	coefficients[0] = inverseRatios.x * inverseRatios.y * inverseRatios.z; //000
	coefficients[1] = ratios.x *        inverseRatios.y * inverseRatios.z; //100
	coefficients[2] = inverseRatios.x * ratios.y *        inverseRatios.z; //010
	coefficients[3] = ratios.x *        ratios.y *        inverseRatios.z; //110
	coefficients[4] = inverseRatios.x * inverseRatios.y * ratios.z;        //001
	coefficients[5] = ratios.x *        inverseRatios.y * ratios.z;        //101
	coefficients[6] = inverseRatios.x * ratios.y *        ratios.z;        //011
	coefficients[7] = ratios.x *        ratios.y *        ratios.z;        //111

	for (int iNeighbor = 0; iNeighbor < neighborCount; iNeighbor++) {
		const TVoxel& v = readVoxel(voxelData, voxelHash, pos + (positions[iNeighbor]), vmIndex, cache);
		bool curKnown = v.flags != ITMLib::VOXEL_UNKNOWN;
		bool curNonTruncated = v.flags == ITMLib::VOXEL_NONTRUNCATED;
		float weight = coefficients[iNeighbor] * curNonTruncated;
		sdf += weight * (curNonTruncated ? TVoxel::valueToFloat(v.sdf) : truncationReplacement);
		struckKnownVoxels |= (bool) (weight * curKnown);
		struckNonTruncated |= (bool) (weight * curNonTruncated);
		cumulativeWeight += weight;
	}
	return sdf;
}



//sdf only, version with replacing all truncated voxels with given value; determines whether narrow band was hit
template<class TVoxel, typename TCache>
_CPU_AND_GPU_CODE_
inline float InterpolateTrilinearly_SetUnknownToVal_StruckChecks(const CONSTPTR(TVoxel)* voxelData,
                                                                 const CONSTPTR(ITMLib::HashEntry)* voxelHash,
                                                                 const CONSTPTR(float)& defaultReplacement,
                                                                 const CONSTPTR(Vector3f)& point,
                                                                 THREADPTR(TCache)& cache,
                                                                 THREADPTR(bool)& struckNonTruncated,
                                                                 THREADPTR(bool)& struckKnownVoxels) {
	float sdfRes1, sdfRes2, sdfV1, sdfV2;
	int vmIndex = false;
	Vector3f coeff;
	Vector3i pos;
	TO_INT_FLOOR3(pos, coeff, point);
	struckNonTruncated = false;
	struckKnownVoxels = false;
#define PROCESS_VOXEL(suffix, coord)\
    {\
        const TVoxel& v = readVoxel(voxelData, voxelHash, pos + (coord), vmIndex, cache);\
        float currentSdf = TVoxel::valueToFloat(v.sdf);\
        if(v.flags == ITMLib::VOXEL_UNKNOWN){\
            sdfV##suffix = defaultReplacement;\
        }else{\
            sdfV##suffix = currentSdf;\
            struckNonTruncated |= v.flags == ITMLib::VOXEL_NONTRUNCATED;\
            struckKnownVoxels = true;\
        }\
    }
	PROCESS_VOXEL(1, Vector3i(0, 0, 0))
	PROCESS_VOXEL(2, Vector3i(1, 0, 0))
	sdfRes1 = (1.0f - coeff.x) * sdfV1 + coeff.x * sdfV2;
	PROCESS_VOXEL(1, Vector3i(0, 1, 0))
	PROCESS_VOXEL(2, Vector3i(1, 1, 0))
	sdfRes1 = (1.0f - coeff.y) * sdfRes1 + coeff.y * ((1.0f - coeff.x) * sdfV1 + coeff.x * sdfV2);
	PROCESS_VOXEL(1, Vector3i(0, 0, 1))
	PROCESS_VOXEL(2, Vector3i(1, 0, 1))
	sdfRes2 = (1.0f - coeff.x) * sdfV1 + coeff.x * sdfV2;
	PROCESS_VOXEL(1, Vector3i(0, 1, 1))
	PROCESS_VOXEL(2, Vector3i(1, 1, 1))
	sdfRes2 = (1.0f - coeff.y) * sdfRes2 + coeff.y * ((1.0f - coeff.x) * sdfV1 + coeff.x * sdfV2);
#undef PROCESS_VOXEL
	float sdf = TVoxel::valueToFloat((1.0f - coeff.z) * sdfRes1 + coeff.z * sdfRes2);

	return sdf;
}



//sdf only, version with replacing all truncated voxels with given value; determines whether narrow band was hit
template<class TVoxel, typename TCache>
_CPU_AND_GPU_CODE_
inline float InterpolateTrilinearly_SetUnknownToVal(const CONSTPTR(TVoxel)* voxelData,
                                                    const CONSTPTR(ITMLib::HashEntry)* voxelHash,
                                                    const CONSTPTR(float)& defaultReplacement,
                                                    const CONSTPTR(Vector3f)& point,
                                                    THREADPTR(TCache)& cache) {
	float sdfRes1, sdfRes2, sdfV1, sdfV2;
	int vmIndex = false;
	Vector3f coeff;
	Vector3i pos;
	TO_INT_FLOOR3(pos, coeff, point);
#define PROCESS_VOXEL(suffix, coord)\
    {\
        const TVoxel& v = readVoxel(voxelData, voxelHash, pos + (coord), vmIndex, cache);\
        float currentSdf = TVoxel::valueToFloat(v.sdf);\
        sdfV##suffix = v.flags == ITMLib::VOXEL_UNKNOWN ? defaultReplacement : currentSdf;\
    }
	PROCESS_VOXEL(1, Vector3i(0, 0, 0))
	PROCESS_VOXEL(2, Vector3i(1, 0, 0))
	sdfRes1 = (1.0f - coeff.x) * sdfV1 + coeff.x * sdfV2;
	PROCESS_VOXEL(1, Vector3i(0, 1, 0))
	PROCESS_VOXEL(2, Vector3i(1, 1, 0))
	sdfRes1 = (1.0f - coeff.y) * sdfRes1 + coeff.y * ((1.0f - coeff.x) * sdfV1 + coeff.x * sdfV2);
	PROCESS_VOXEL(1, Vector3i(0, 0, 1))
	PROCESS_VOXEL(2, Vector3i(1, 0, 1))
	sdfRes2 = (1.0f - coeff.x) * sdfV1 + coeff.x * sdfV2;
	PROCESS_VOXEL(1, Vector3i(0, 1, 1))
	PROCESS_VOXEL(2, Vector3i(1, 1, 1))
	sdfRes2 = (1.0f - coeff.y) * sdfRes2 + coeff.y * ((1.0f - coeff.x) * sdfV1 + coeff.x * sdfV2);
#undef PROCESS_VOXEL
	float sdf = TVoxel::valueToFloat((1.0f - coeff.z) * sdfRes1 + coeff.z * sdfRes2);

	return sdf;
}



//pick maximum weights, get confidence
template<class TVoxel, typename TCache>
_CPU_AND_GPU_CODE_
inline float InterpolateTrilinearly(const CONSTPTR(TVoxel)* voxelData,
                                    const CONSTPTR(ITMLib::HashEntry)* hashIndex,
                                    const CONSTPTR(Vector3f)& point,
                                    THREADPTR(TCache)& cache,
                                    THREADPTR(Vector3f)& color,
                                    THREADPTR(int)& wDepth,
                                    THREADPTR(int)& wColor,
                                    THREADPTR(float)& confidence) {
	float sdfRes1, sdfRes2, sdfV1, sdfV2;
	Vector3f colorRes1, colorRes2, colorV1, colorV2;
	float confRes1, confRes2, confV1, confV2;
	int vmIndex = false;
	Vector3f coeff;
	Vector3i pos;
	TO_INT_FLOOR3(pos, coeff, point);

#define PROCESS_VOXEL(suffix, coord)\
    {\
        const TVoxel& v = readVoxel(voxelData, hashIndex, pos + (coord), vmIndex, cache);\
        sdfV##suffix = v.sdf;\
        colorV##suffix = TO_FLOAT3(v.clr);\
        confV##suffix = v.confidence;\
        if (v.w_depth > wDepth) wDepth = v.w_depth;\
        if (v.w_color > wColor) wColor = v.w_color;\
    }
	PROCESS_VOXEL(1, Vector3i(0, 0, 0))
	PROCESS_VOXEL(2, Vector3i(1, 0, 0))
	sdfRes1 = (1.0f - coeff.x) * sdfV1 + coeff.x * sdfV2;
	colorRes1 = (1.0f - coeff.x) * colorV1 + coeff.x * colorV2;
	confRes1 = (1.0f - coeff.x) * confV1 + coeff.x * confV2;
	PROCESS_VOXEL(1, Vector3i(0, 1, 0))
	PROCESS_VOXEL(2, Vector3i(1, 1, 0))
	sdfRes1 = (1.0f - coeff.y) * sdfRes1 + coeff.y * ((1.0f - coeff.x) * sdfV1 + coeff.x * sdfV2);
	colorRes1 = (1.0f - coeff.y) * colorRes1 + coeff.y * ((1.0f - coeff.x) * colorV1 + coeff.x * colorV2);
	confRes1 = (1.0f - coeff.y) * confRes1 + coeff.y * ((1.0f - coeff.x) * confV1 + coeff.x * confV2);
	PROCESS_VOXEL(1, Vector3i(0, 0, 1))
	PROCESS_VOXEL(2, Vector3i(1, 0, 1))
	sdfRes2 = (1.0f - coeff.x) * sdfV1 + coeff.x * sdfV2;
	colorRes2 = (1.0f - coeff.x) * colorV1 + coeff.x * colorV2;
	confRes2 = (1.0f - coeff.x) * confV1 + coeff.x * confV2;
	PROCESS_VOXEL(1, Vector3i(0, 1, 1))
	PROCESS_VOXEL(2, Vector3i(1, 1, 1))
	sdfRes2 = (1.0f - coeff.y) * sdfRes2 + coeff.y * ((1.0f - coeff.x) * sdfV1 + coeff.x * sdfV2);
	colorRes2 = (1.0f - coeff.y) * colorRes2 + coeff.y * ((1.0f - coeff.x) * colorV1 + coeff.x * colorV2);
	confRes2 = (1.0f - coeff.y) * confRes2 + coeff.y * ((1.0f - coeff.x) * confV1 + coeff.x * confV2);
	color = ((1.0f - coeff.z) * colorRes1 + coeff.z * colorRes2) / 255.0f;
	confidence = (1.0f - coeff.z) * confRes1 + coeff.z * confRes2;
#undef PROCESS_VOXEL
	return TVoxel::valueToFloat((1.0f - coeff.z) * sdfRes1 + coeff.z * sdfRes2);
}

//sdf, color, and interpolated weights
template<class TVoxel, typename TCache>
_CPU_AND_GPU_CODE_
inline float InterpolateTrilinearly(const CONSTPTR(TVoxel)* voxelData,
                                    const CONSTPTR(ITMLib::HashEntry)* hashIndex,
                                    const CONSTPTR(Vector3f)& point,
                                    THREADPTR(TCache)& cache,
                                    THREADPTR(Vector3f)& color,
                                    THREADPTR(float)& wDepth,
                                    THREADPTR(float)& wColor) {
	float sdfRes1, sdfRes2, sdfV1, sdfV2;
	float wDepthRes1, wDepthRes2, wDepthV1, wDepthV2;
	float wColorRes1, wColorRes2, wColorV1, wColorV2;
	Vector3f colorRes1, colorRes2, colorV1, colorV2;
	int vmIndex = false;
	Vector3f coeff;
	Vector3i pos;
	TO_INT_FLOOR3(pos, coeff, point);

#define PROCESS_VOXEL(suffix, coord)\
    {\
        const TVoxel& v = readVoxel(voxelData, hashIndex, pos + (coord), vmIndex, cache);\
        sdfV##suffix = v.sdf;\
        colorV##suffix = TO_FLOAT3(v.clr);\
        wDepthV##suffix = v.w_depth;\
        wColorV##suffix = v.w_color;\
    }
	PROCESS_VOXEL(1, Vector3i(0, 0, 0))
	PROCESS_VOXEL(2, Vector3i(1, 0, 0))
	sdfRes1 = (1.0f - coeff.x) * sdfV1 + coeff.x * sdfV2;
	colorRes1 = (1.0f - coeff.x) * colorV1 + coeff.x * colorV2;
	wDepthRes1 = (1.0f - coeff.x) * wDepthV1 + coeff.x * wDepthV2;
	wColorRes1 = (1.0f - coeff.x) * wColorV1 + coeff.x * wColorV2;
	PROCESS_VOXEL(1, Vector3i(0, 1, 0))
	PROCESS_VOXEL(2, Vector3i(1, 1, 0))
	sdfRes1 = (1.0f - coeff.y) * sdfRes1 + coeff.y * ((1.0f - coeff.x) * sdfV1 + coeff.x * sdfV2);
	colorRes1 = (1.0f - coeff.y) * colorRes1 + coeff.y * ((1.0f - coeff.x) * colorV1 + coeff.x * colorV2);
	wDepthRes1 = (1.0f - coeff.y) * wDepthRes1 + coeff.y * ((1.0f - coeff.x) * wDepthV1 + coeff.x * wDepthV2);
	wColorRes1 = (1.0f - coeff.y) * wColorRes1 + coeff.y * ((1.0f - coeff.x) * wColorV1 + coeff.x * wColorV2);
	PROCESS_VOXEL(1, Vector3i(0, 0, 1))
	PROCESS_VOXEL(2, Vector3i(1, 0, 1))
	sdfRes2 = (1.0f - coeff.x) * sdfV1 + coeff.x * sdfV2;
	colorRes2 = (1.0f - coeff.x) * colorV1 + coeff.x * colorV2;
	wDepthRes2 = (1.0f - coeff.x) * wDepthV1 + coeff.x * wDepthV2;
	wColorRes2 = (1.0f - coeff.x) * wColorV1 + coeff.x * wColorV2;
	PROCESS_VOXEL(1, Vector3i(0, 1, 1))
	PROCESS_VOXEL(2, Vector3i(1, 1, 1))
	sdfRes2 = (1.0f - coeff.y) * sdfRes2 + coeff.y * ((1.0f - coeff.x) * sdfV1 + coeff.x * sdfV2);
	colorRes2 = (1.0f - coeff.y) * colorRes2 + coeff.y * ((1.0f - coeff.x) * colorV1 + coeff.x * colorV2);
	wDepthRes2 = (1.0f - coeff.y) * wDepthRes2 + coeff.y * ((1.0f - coeff.x) * wDepthV1 + coeff.x * wDepthV2);
	wColorRes2 = (1.0f - coeff.y) * wColorRes2 + coeff.y * ((1.0f - coeff.x) * wColorV1 + coeff.x * wColorV2);

	color = ((1.0f - coeff.z) * colorRes1 + coeff.z * colorRes2) / 255.0f;
	wDepth = (1.0f - coeff.z) * wDepthRes1 + coeff.z * wDepthRes2;
	wColor = (1.0f - coeff.z) * wColorRes1 + coeff.z * wColorRes2;
#undef PROCESS_VOXEL
	return TVoxel::valueToFloat((1.0f - coeff.z) * sdfRes1 + coeff.z * sdfRes2);
}


//sdf and color + knownVoxelHit
template<class TVoxel, typename TCache>
_CPU_AND_GPU_CODE_
inline float InterpolateTrilinearly_StruckKnownVoxels(const CONSTPTR(TVoxel)* voxelData,
                                                      const CONSTPTR(ITMLib::HashEntry)* hashIndex,
                                                      const THREADPTR(Vector3f)& point,
                                                      THREADPTR(TCache)& cache,
                                                      THREADPTR(Vector3f)& color,
                                                      THREADPTR(bool)& struckKnownVoxels) {
	float sdfRes1, sdfRes2, sdfV1, sdfV2;
	Vector3f colorRes1, colorRes2, colorV1, colorV2;
	int vmIndex = false;
	Vector3f coeff;
	Vector3i pos;
	struckKnownVoxels = false;
	TO_INT_FLOOR3(pos, coeff, point);

#define PROCESS_VOXEL(suffix, coord)\
    {\
        const TVoxel& v = readVoxel(voxelData, hashIndex, pos + (coord), vmIndex, cache);\
        sdfV##suffix = TVoxel::valueToFloat(v.sdf);\
        colorV##suffix = TO_FLOAT3(v.clr);\
        struckKnownVoxels |= v.flags != ITMLib::VOXEL_UNKNOWN;\
    }
	PROCESS_VOXEL(1, Vector3i(0, 0, 0))
	PROCESS_VOXEL(2, Vector3i(1, 0, 0))
	sdfRes1 = (1.0f - coeff.x) * sdfV1 + coeff.x * sdfV2;
	colorRes1 = (1.0f - coeff.x) * colorV1 + coeff.x * colorV2;
	PROCESS_VOXEL(1, Vector3i(0, 1, 0))
	PROCESS_VOXEL(2, Vector3i(1, 1, 0))
	sdfRes1 = (1.0f - coeff.y) * sdfRes1 + coeff.y * ((1.0f - coeff.x) * sdfV1 + coeff.x * sdfV2);
	colorRes1 = (1.0f - coeff.y) * colorRes1 + coeff.y * ((1.0f - coeff.x) * colorV1 + coeff.x * colorV2);
	PROCESS_VOXEL(1, Vector3i(0, 0, 1))
	PROCESS_VOXEL(2, Vector3i(1, 0, 1))
	sdfRes2 = (1.0f - coeff.x) * sdfV1 + coeff.x * sdfV2;
	colorRes2 = (1.0f - coeff.x) * colorV1 + coeff.x * colorV2;
	PROCESS_VOXEL(1, Vector3i(0, 1, 1))
	PROCESS_VOXEL(2, Vector3i(1, 1, 1))

	sdfRes2 = (1.0f - coeff.y) * sdfRes2 + coeff.y * ((1.0f - coeff.x) * sdfV1 + coeff.x * sdfV2);
	colorRes2 = (1.0f - coeff.y) * colorRes2 + coeff.y * ((1.0f - coeff.x) * colorV1 + coeff.x * colorV2);
	color = ((1.0f - coeff.z) * colorRes1 + coeff.z * colorRes2) / 255.0f;
#undef PROCESS_VOXEL
	return TVoxel::valueToFloat((1.0f - coeff.z) * sdfRes1 + coeff.z * sdfRes2);
}


//sdf and color + struckNarrowBand
template<class TVoxel, typename TCache>
_CPU_AND_GPU_CODE_
inline float InterpolateTrilinearly_StruckNarrowBand(const CONSTPTR(TVoxel)* voxelData,
                                                     const CONSTPTR(ITMLib::HashEntry)* hashIndex,
                                                     const THREADPTR(Vector3f)& point,
                                                     THREADPTR(TCache)& cache,
                                                     THREADPTR(Vector3f)& color,
                                                     THREADPTR(bool)& struckNarrowBand) {
	float sdfRes1, sdfRes2, sdfV1, sdfV2;
	Vector3f colorRes1, colorRes2, colorV1, colorV2;
	int vmIndex = false;
	Vector3f coeff;
	Vector3i pos;
	struckNarrowBand = false;
	TO_INT_FLOOR3(pos, coeff, point);
#define PROCESS_VOXEL(suffix, coord)\
    {\
        const TVoxel& v = readVoxel(voxelData, hashIndex, pos + (coord), vmIndex, cache);\
        sdfV##suffix = v.sdf;\
        colorV##suffix = TO_FLOAT3(v.clr);\
        struckNarrowBand |= (v.flags == ITMLib::VOXEL_NONTRUNCATED);\
    }
	PROCESS_VOXEL(1, Vector3i(0, 0, 0))
	PROCESS_VOXEL(2, Vector3i(1, 0, 0))
	sdfRes1 = (1.0f - coeff.x) * sdfV1 + coeff.x * sdfV2;
	colorRes1 = (1.0f - coeff.x) * colorV1 + coeff.x * colorV2;
	PROCESS_VOXEL(1, Vector3i(0, 1, 0))
	PROCESS_VOXEL(2, Vector3i(1, 1, 0))
	sdfRes1 = (1.0f - coeff.y) * sdfRes1 + coeff.y * ((1.0f - coeff.x) * sdfV1 + coeff.x * sdfV2);
	colorRes1 = (1.0f - coeff.y) * colorRes1 + coeff.y * ((1.0f - coeff.x) * colorV1 + coeff.x * colorV2);
	PROCESS_VOXEL(1, Vector3i(0, 0, 1))
	PROCESS_VOXEL(2, Vector3i(1, 0, 1))
	sdfRes2 = (1.0f - coeff.x) * sdfV1 + coeff.x * sdfV2;
	colorRes2 = (1.0f - coeff.x) * colorV1 + coeff.x * colorV2;
	PROCESS_VOXEL(1, Vector3i(0, 1, 1))
	PROCESS_VOXEL(2, Vector3i(1, 1, 1))

	sdfRes2 = (1.0f - coeff.y) * sdfRes2 + coeff.y * ((1.0f - coeff.x) * sdfV1 + coeff.x * sdfV2);
	colorRes2 = (1.0f - coeff.y) * colorRes2 + coeff.y * ((1.0f - coeff.x) * colorV1 + coeff.x * colorV2);
	color = ((1.0f - coeff.z) * colorRes1 + coeff.z * colorRes2) / 255.0f;
#undef PROCESS_VOXEL
	return TVoxel::valueToFloat((1.0f - coeff.z) * sdfRes1 + coeff.z * sdfRes2);
}

//sdf and color
template<class TVoxel, typename TCache>
_CPU_AND_GPU_CODE_
inline float InterpolateTrilinearly(const CONSTPTR(TVoxel)* voxelData,
                                    const CONSTPTR(ITMLib::HashEntry)* hashIndex,
                                    const CONSTPTR(Vector3f)& point,
                                    THREADPTR(TCache)& cache,
                                    THREADPTR(Vector3f)& color) {
	float sdfRes1, sdfRes2, sdfV1, sdfV2;
	Vector3f colorRes1, colorRes2, colorV1, colorV2;
	int vmIndex = false;
	Vector3f coeff;
	Vector3i pos;
	TO_INT_FLOOR3(pos, coeff, point);
#define PROCESS_VOXEL(suffix, coord)\
    {\
        const TVoxel& v = readVoxel(voxelData, hashIndex, pos + (coord), vmIndex, cache);\
        sdfV##suffix = v.sdf;\
        colorV##suffix = TO_FLOAT3(v.clr);\
    }
	PROCESS_VOXEL(1, Vector3i(0, 0, 0))
	PROCESS_VOXEL(2, Vector3i(1, 0, 0))
	sdfRes1 = (1.0f - coeff.x) * sdfV1 + coeff.x * sdfV2;
	colorRes1 = (1.0f - coeff.x) * colorV1 + coeff.x * colorV2;
	PROCESS_VOXEL(1, Vector3i(0, 1, 0))
	PROCESS_VOXEL(2, Vector3i(1, 1, 0))
	sdfRes1 = (1.0f - coeff.y) * sdfRes1 + coeff.y * ((1.0f - coeff.x) * sdfV1 + coeff.x * sdfV2);
	colorRes1 = (1.0f - coeff.y) * colorRes1 + coeff.y * ((1.0f - coeff.x) * colorV1 + coeff.x * colorV2);
	PROCESS_VOXEL(1, Vector3i(0, 0, 1))
	PROCESS_VOXEL(2, Vector3i(1, 0, 1))
	sdfRes2 = (1.0f - coeff.x) * sdfV1 + coeff.x * sdfV2;
	colorRes2 = (1.0f - coeff.x) * colorV1 + coeff.x * colorV2;
	PROCESS_VOXEL(1, Vector3i(0, 1, 1))
	PROCESS_VOXEL(2, Vector3i(1, 1, 1))

	sdfRes2 = (1.0f - coeff.y) * sdfRes2 + coeff.y * ((1.0f - coeff.x) * sdfV1 + coeff.x * sdfV2);
	colorRes2 = (1.0f - coeff.y) * colorRes2 + coeff.y * ((1.0f - coeff.x) * colorV1 + coeff.x * colorV2);
	color = ((1.0f - coeff.z) * colorRes1 + coeff.z * colorRes2) / 255.0f;
#undef PROCESS_VOXEL
	return TVoxel::valueToFloat((1.0f - coeff.z) * sdfRes1 + coeff.z * sdfRes2);
}

//_DEBUG -- special treatment of truncated values
//sdf without color, found/not
template<class TVoxel, typename TCache>
_CPU_AND_GPU_CODE_
inline float InterpolateTrilinearly_TruncatedCopySign(const CONSTPTR(TVoxel)* voxelData,
                                                      const CONSTPTR(ITMLib::HashEntry)* voxelHash,
                                                      const CONSTPTR(Vector3f)& point,
                                                      THREADPTR(TCache)& cache) {
	float sdfRes1, sdfRes2;
	int vmIndex = false;
	Vector3f coeff;
	Vector3i pos;

	TO_INT_FLOOR3(pos, coeff, point);
	const int neighborCount = 8;
	float sdfs[neighborCount];
	bool found[neighborCount] = {0};
	const Vector3i positions[neighborCount] = {Vector3i(0, 0, 0), Vector3i(1, 0, 0),
	                                           Vector3i(0, 1, 0), Vector3i(1, 1, 0),
	                                           Vector3i(0, 0, 1), Vector3i(1, 0, 1),
	                                           Vector3i(0, 1, 1), Vector3i(1, 1, 1)};
	float sumFound = 0.0f;
	for (int iNeighbor = 0; iNeighbor < neighborCount; iNeighbor++) {
		const TVoxel& v = readVoxel(voxelData, voxelHash, pos + (positions[iNeighbor]), vmIndex, cache);
		sdfs[iNeighbor] = v.sdf;
		bool foundCur = vmIndex != 0;
		found[iNeighbor] = foundCur;
		sumFound += foundCur * v.sdf;
	}
	float truncated = std::copysign(1.0f, sumFound);
	for (int iNeighbor = 0; iNeighbor < neighborCount; iNeighbor++) {
		if (!found[iNeighbor]) {
			sdfs[iNeighbor] = truncated;
		}
	}


	float oneMinusCoeffX = 1.0f - coeff.x;
	float oneMinusCoeffY = 1.0f - coeff.y;

	sdfRes1 = (oneMinusCoeffY) * ((oneMinusCoeffX) * sdfs[0] + coeff.x * sdfs[1])
	          + coeff.y * ((oneMinusCoeffX) * sdfs[2] + coeff.x * sdfs[3]);
	sdfRes2 = (oneMinusCoeffY) * ((oneMinusCoeffX) * sdfs[4] + coeff.x * sdfs[5])
	          + coeff.y * ((oneMinusCoeffX) * sdfs[6] + coeff.x * sdfs[7]);

	float sdf = TVoxel::valueToFloat((1.0f - coeff.z) * sdfRes1 + coeff.z * sdfRes2);
	return sdf;
}

//sdf without color, struck non-Truncated check, struck known check,
template<class TVoxel, typename TCache, typename TIndexData>
_CPU_AND_GPU_CODE_
inline float InterpolateTrilinearly_StruckKnown(const CONSTPTR(TVoxel)* voxelData,
                                                const CONSTPTR(TIndexData)* index_data,
	                                             const CONSTPTR(Vector3f)& point,
	                                             THREADPTR(TCache)& cache,
	                                             THREADPTR(bool)& struckKnown) {
	float sdfRes1, sdfRes2, sdfV1, sdfV2;
	int vmIndex = false;
	Vector3f coeff;
	Vector3i pos;
	struckKnown = false;
	TO_INT_FLOOR3(pos, coeff, point);
	auto process_voxel = [&voxelData, &index_data, &pos, &vmIndex, &cache, &struckKnown](float& sdfV, const Vector3i& coord){
#if !defined(__CUDACC__) && !defined(WITH_OPENMP)
		const TVoxel& v = readVoxel(voxelData, index_data, pos + coord, vmIndex, cache);
#else
		const TVoxel& v = readVoxel(voxelData, index_data, pos + coord, vmIndex);
#endif
	    sdfV = TVoxel::valueToFloat(v.sdf);
        struckKnown |= (v.flags != ITMLib::VoxelFlags::VOXEL_UNKNOWN);
	};
	process_voxel(sdfV1, Vector3i(0, 0, 0));
	process_voxel(sdfV2, Vector3i(1, 0, 0));
	sdfRes1 = (1.0f - coeff.x) * sdfV1 + coeff.x * sdfV2;
	process_voxel(sdfV1, Vector3i(0, 1, 0));
	process_voxel(sdfV2, Vector3i(1, 1, 0));
	sdfRes1 = (1.0f - coeff.y) * sdfRes1 + coeff.y * ((1.0f - coeff.x) * sdfV1 + coeff.x * sdfV2);
	process_voxel(sdfV1, Vector3i(0, 0, 1));
	process_voxel(sdfV2, Vector3i(1, 0, 1));
	sdfRes2 = (1.0f - coeff.x) * sdfV1 + coeff.x * sdfV2;
	process_voxel(sdfV1, Vector3i(0, 1, 1));
	process_voxel(sdfV2, Vector3i(1, 1, 1));
	sdfRes2 = (1.0f - coeff.y) * sdfRes2 + coeff.y * ((1.0f - coeff.x) * sdfV1 + coeff.x * sdfV2);
	float sdf = (1.0f - coeff.z) * sdfRes1 + coeff.z * sdfRes2;

	return sdf;
}

//sdf without color, struck non-Truncated check
template<class TVoxel, typename TCache>
_CPU_AND_GPU_CODE_
inline float InterpolateTrilinearly_StruckNarrowBand(const CONSTPTR(TVoxel)* voxelData,
                                                     const CONSTPTR(ITMLib::HashEntry)* voxelHash,
                                                     const CONSTPTR(Vector3f)& point,
                                                     THREADPTR(TCache)& cache,
                                                     THREADPTR(bool)& struckNarrowBand) {
	float sdfRes1, sdfRes2, sdfV1, sdfV2;
	int vmIndex = false;
	Vector3f coeff;
	Vector3i pos;
	struckNarrowBand = false;
	TO_INT_FLOOR3(pos, coeff, point);
#define PROCESS_VOXEL(suffix, coord)\
    {\
        const TVoxel& v = readVoxel(voxelData, voxelHash, pos + (coord), vmIndex, cache);\
        sdfV##suffix = v.sdf;\
        struckNarrowBand |= (v.flags == ITMLib::VoxelFlags::VOXEL_NONTRUNCATED);\
    }
	PROCESS_VOXEL(1, Vector3i(0, 0, 0))
	PROCESS_VOXEL(2, Vector3i(1, 0, 0))
	sdfRes1 = (1.0f - coeff.x) * sdfV1 + coeff.x * sdfV2;
	PROCESS_VOXEL(1, Vector3i(0, 1, 0))
	PROCESS_VOXEL(2, Vector3i(1, 1, 0))
	sdfRes1 = (1.0f - coeff.y) * sdfRes1 + coeff.y * ((1.0f - coeff.x) * sdfV1 + coeff.x * sdfV2);
	PROCESS_VOXEL(1, Vector3i(0, 0, 1))
	PROCESS_VOXEL(2, Vector3i(1, 0, 1))
	sdfRes2 = (1.0f - coeff.x) * sdfV1 + coeff.x * sdfV2;
	PROCESS_VOXEL(1, Vector3i(0, 1, 1))
	PROCESS_VOXEL(2, Vector3i(1, 1, 1))
	sdfRes2 = (1.0f - coeff.y) * sdfRes2 + coeff.y * ((1.0f - coeff.x) * sdfV1 + coeff.x * sdfV2);
#undef PROCESS_VOXEL
	float sdf = TVoxel::valueToFloat((1.0f - coeff.z) * sdfRes1 + coeff.z * sdfRes2);

	return sdf;
}

//sdf without color
template<class TVoxel, typename TCache>
_CPU_AND_GPU_CODE_
inline float InterpolateTrilinearly(const CONSTPTR(TVoxel)* voxelData,
                                    const CONSTPTR(ITMLib::HashEntry)* voxelHash,
                                    const CONSTPTR(Vector3f)& point,
                                    THREADPTR(TCache)& cache) {
	float sdfRes1, sdfRes2, sdfV1, sdfV2;
	int vmIndex = false;
	Vector3f coeff;
	Vector3i pos;
	TO_INT_FLOOR3(pos, coeff, point);
#define PROCESS_VOXEL(suffix, coord)\
    {\
        const TVoxel& v = readVoxel(voxelData, voxelHash, pos + (coord), vmIndex, cache);\
        sdfV##suffix = v.sdf;\
    }
	PROCESS_VOXEL(1, Vector3i(0, 0, 0))
	PROCESS_VOXEL(2, Vector3i(1, 0, 0))
	sdfRes1 = (1.0f - coeff.x) * sdfV1 + coeff.x * sdfV2;
	PROCESS_VOXEL(1, Vector3i(0, 1, 0))
	PROCESS_VOXEL(2, Vector3i(1, 1, 0))
	sdfRes1 = (1.0f - coeff.y) * sdfRes1 + coeff.y * ((1.0f - coeff.x) * sdfV1 + coeff.x * sdfV2);
	PROCESS_VOXEL(1, Vector3i(0, 0, 1))
	PROCESS_VOXEL(2, Vector3i(1, 0, 1))
	sdfRes2 = (1.0f - coeff.x) * sdfV1 + coeff.x * sdfV2;
	PROCESS_VOXEL(1, Vector3i(0, 1, 1))
	PROCESS_VOXEL(2, Vector3i(1, 1, 1))
	sdfRes2 = (1.0f - coeff.y) * sdfRes2 + coeff.y * ((1.0f - coeff.x) * sdfV1 + coeff.x * sdfV2);
#undef PROCESS_VOXEL
	float sdf = TVoxel::valueToFloat((1.0f - coeff.z) * sdfRes1 + coeff.z * sdfRes2);

	return sdf;
}


template<class TVoxel, typename TCache>
_CPU_AND_GPU_CODE_
inline void FindPointNeighbors(THREADPTR(Vector3f)* p,
                               THREADPTR(float)* sdf,
                               const CONSTPTR(Vector3i) blockLocation,
                               const CONSTPTR(TVoxel)* localVBA,
                               const CONSTPTR(ITMLib::HashEntry)* hashTable,
                               THREADPTR(TCache)& cache) {
	int vmIndex;
	Vector3i localBlockLocation;

	Vector3i(0, 0, 0);
	TVoxel voxel;
#define PROCESS_VOXEL(location, index)\
    localBlockLocation = blockLocation + (location);\
    p[index] = localBlockLocation.toFloat();\
    voxel = readVoxel(localVBA, hashTable, localBlockLocation, vmIndex, cache);\
    sdf[index] = TVoxel::valueToFloat(voxel.sdf);

	PROCESS_VOXEL(Vector3i(0, 0, 0), 0);
	PROCESS_VOXEL(Vector3i(0, 0, 1), 1);
	PROCESS_VOXEL(Vector3i(0, 1, 0), 2);
	PROCESS_VOXEL(Vector3i(0, 1, 1), 3);
	PROCESS_VOXEL(Vector3i(1, 0, 0), 4);
	PROCESS_VOXEL(Vector3i(1, 0, 1), 5);
	PROCESS_VOXEL(Vector3i(1, 1, 0), 6);
	PROCESS_VOXEL(Vector3i(1, 1, 1), 7);
#undef PROCESS_VOXEL
}

//_DEBUG
//sdf without color
template<class TVoxel, typename TCache>
_CPU_AND_GPU_CODE_
inline float InterpolateTrilinearly_Alternative(const CONSTPTR(TVoxel)* voxelData,
                                                const CONSTPTR(ITMLib::HashEntry)* voxelHash,
                                                const CONSTPTR(Vector3f)& point,
                                                THREADPTR(TCache)& cache) {
	const int neighborCount = 8;
	Vector3f points[neighborCount];
	float sdfVals[neighborCount];
	FindPointNeighbors(points, sdfVals, point.toIntFloor(), voxelData, voxelHash, cache);
	float sdf;

	Vector3f ratios = (point - points[0]) / (points[7] - points[0]);
	Vector3f invRatios = Vector3f(1.f) - ratios;
	Vector3f coeff;
	point.toIntFloor(coeff);

//#define INTERPOLATE_TRILINEAR(type, prefix, output, array, ratios, invRatios)\
//                    type prefix##_00 = (array)[0]*(invRatios).x + (array)[4]*(ratios).x;\
//                    type prefix##_01 = (array)[1]*(invRatios).x + (array)[5]*(ratios).x;\
//                    type prefix##_10 = (array)[2]*(invRatios).x + (array)[6]*(ratios).x;\
//                    type prefix##_11 = (array)[3]*(invRatios).x + (array)[7]*(ratios).x;\
//                    type prefix##_0 = prefix##_00*(invRatios).y + prefix##_10*(ratios).y;\
//                    type prefix##_1 = prefix##_01*(invRatios).y + prefix##_11*(ratios).y;\
//                    (output) = prefix##_0*(invRatios).z + prefix##_1 * (ratios).z;
//	INTERPOLATE_TRILINEAR(float, sdf, sdf, sdfVals, ratios, invRatios);
//#undef INTERPOLATE_TRILINEAR
	float sdf_00 = sdfVals[0] * (invRatios).x + sdfVals[4] * (ratios).x;
	float sdf_01 = sdfVals[1] * (invRatios).x + sdfVals[5] * (ratios).x;
	float sdf_10 = sdfVals[2] * (invRatios).x + sdfVals[6] * (ratios).x;
	float sdf_11 = sdfVals[3] * (invRatios).x + sdfVals[7] * (ratios).x;
	float sdf_0 = sdf_00 * (invRatios).y + sdf_10 * (ratios).y;
	float sdf_1 = sdf_01 * (invRatios).y + sdf_11 * (ratios).y;
	sdf = sdf_0 * (invRatios).z + sdf_1 * (ratios).z;
	//_DEBUG
//	if (1.0f - std::abs(sdf) < 10e-10){
//
//	}
	return sdf;
}

} // namespace ITMLib