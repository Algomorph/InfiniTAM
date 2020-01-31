// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "RepresentationAccess.h"

#define MAX_NUM_LOCALMAPS 32

namespace ITMLib {
	struct MultiCache {};

	template<class TIndex>
	class MultiIndex
	{
	public:
		typedef TIndex IndexType;
		typedef MultiCache IndexCache;

		struct IndexData
		{
			int numLocalMaps;
			typedef TIndex IndexType;
			typename TIndex::IndexData *index[MAX_NUM_LOCALMAPS];
			Matrix4f poses_vs[MAX_NUM_LOCALMAPS];
			Matrix4f posesInv[MAX_NUM_LOCALMAPS];
		};
	};

	template<class TVoxel>
	class MultiVoxel
	{
	public:
		typedef TVoxel VoxelType;
		TVoxel* voxels[MAX_NUM_LOCALMAPS];

		static const CONSTPTR(bool) hasColorInformation = TVoxel::hasColorInformation;
		static const CONSTPTR(bool) hasConfidenceInformation = TVoxel::hasConfidenceInformation;
		static const CONSTPTR(bool) hasSemanticInformation = TVoxel::hasSemanticInformation;
		static const CONSTPTR(bool) hasWeightInformation = TVoxel::hasWeightInformation;
	};
}

template<class TMultiVoxel, class TMultiIndex>
_CPU_AND_GPU_CODE_ inline float readFromSDF_float_uninterpolated(const TMultiVoxel *voxelData, const TMultiIndex *voxelIndex, const Vector3f & point, int & vmIndex, ITMLib::MultiCache & _cache)
{
	typedef typename TMultiVoxel::VoxelType TVoxel;
	typedef typename TMultiIndex::IndexType TIndex;

	float sum_sdf = 0.0f;
	int sum_weights = 0;
	vmIndex = false;
	for (int localMapId = 0; localMapId < voxelIndex->numLocalMaps; ++localMapId)
	{
		Vector3f point_local = voxelIndex->poses_vs[localMapId] * point;

		int vmIndex_tmp;
		typename TIndex::IndexCache cache;
		const TVoxel & v = readVoxel(voxelData->voxels[localMapId], voxelIndex->index[localMapId], Vector3i((int)ROUND(point_local.x), (int)ROUND(point_local.y), (int)ROUND(point_local.z)), vmIndex_tmp, cache);
		if (!vmIndex_tmp) continue;

		vmIndex = true;
		sum_sdf += (float)v.w_depth * (float)v.sdf;
		sum_weights += v.w_depth;
	}
	if (sum_weights == 0) return 1.0f;
	return TVoxel::valueToFloat(sum_sdf / (float)sum_weights);
}

template<class TMultiVoxel, class TMultiIndex>
_CPU_AND_GPU_CODE_ inline float readFromSDF_float_interpolated(const TMultiVoxel *voxelData, const TMultiIndex *voxelIndex, const Vector3f & point, int & vmIndex, ITMLib::MultiCache & _cache)
{
	typedef typename TMultiIndex::IndexType TIndex;

	float sum_sdf = 0.0f;
	int sum_weights = 0;
	vmIndex = false;

	for (int localMapId = 0; localMapId < voxelIndex->numLocalMaps; ++localMapId) 
	{
		Vector3f point_local = voxelIndex->poses_vs[localMapId] * point;

		int vmIndex_tmp, maxW;
		typename TIndex::IndexCache cache;
		
		float sdf = readFromSDF_float_interpolated(voxelData->voxels[localMapId], voxelIndex->index[localMapId], point_local, vmIndex_tmp, cache, maxW);
		if (!vmIndex_tmp) continue;
		
		vmIndex = true;

		sum_sdf += (float)maxW * sdf;
		sum_weights += maxW;
	}

	if (sum_weights == 0) return 1.0f;
	
	return (sum_sdf / (float)sum_weights);
}

template<class TMultiVoxel, class TMultiIndex>
_CPU_AND_GPU_CODE_ inline Vector4f readFromSDF_color4u_interpolated(const TMultiVoxel *voxelData,
                                                                    const TMultiIndex *voxelIndex,
                                                                    const Vector3f & point,
                                                                    ITMLib::MultiCache & _cache)
{
	typedef typename TMultiIndex::IndexType TIndex;

	Vector4f accu(0.0f);
	for (int localMapId = 0; localMapId < voxelIndex->numLocalMaps; ++localMapId) 
	{
		Vector3f point_local = voxelIndex->poses_vs[localMapId] * point;

		int maxW;
		typename TIndex::IndexCache cache;
		Vector4f val = readFromSDF_color4u_interpolated(voxelData->voxels[localMapId], voxelIndex->index[localMapId], point_local, cache, maxW);

		accu += (float)maxW * val;
	}
	if (accu.w < 0.001f) accu.w = 1.0f;
	return (accu / accu.w);
}

template<class TMultiVoxel, class TMultiIndex>
_CPU_AND_GPU_CODE_ inline float readWithConfidenceFromSDF_float_interpolated(THREADPTR(float) &confidence, const CONSTPTR(TMultiVoxel) *voxelData,
	const CONSTPTR(TMultiIndex) *voxelIndex, Vector3f point, THREADPTR(int) &vmIndex, ITMLib::MultiCache & _cache)
{
	typedef typename TMultiIndex::IndexType TIndex;

	float sum_sdf = 0.0f, sum_confidence = 0.0f;
	int noLiveScenes = 0;
	
	vmIndex = false;
	for (int localMapId = 0; localMapId < voxelIndex->numLocalMaps; ++localMapId) 
	{
		Vector3f point_local = voxelIndex->poses_vs[localMapId] * point;

		int vmIndex_tmp;
		typename TIndex::IndexCache cache;

		float conf;
		float sdf = readWithConfidenceFromSDF_float_interpolated(conf, voxelData->voxels[localMapId], voxelIndex->index[localMapId], point_local, vmIndex_tmp, cache);
		
		if (!vmIndex_tmp) continue;
		vmIndex = true;

		sum_sdf += (float)conf * sdf;
		sum_confidence += conf;
		noLiveScenes++;
	}

	if (sum_confidence == 0.0f) { confidence = 1.0f;  return 1.0f; }

	confidence = sum_confidence / (float)noLiveScenes + 1.0f;
	return sum_sdf / (float)sum_confidence;
}
