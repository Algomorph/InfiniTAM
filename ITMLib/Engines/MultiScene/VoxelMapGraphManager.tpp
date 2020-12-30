// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "VoxelMapGraphManager.h"

//#include <queue>

namespace ITMLib
{
	template<class TVoxel, class TIndex>
	VoxelMapGraphManager<TVoxel, TIndex>::VoxelMapGraphManager(const RaycastingEngineBase<TVoxel, TIndex> *_VisualizationEngine, const DenseMapper<TVoxel, TIndex> *_denseMapper, const Vector2i & _trackedImageSize)
		: visualization_engine(_VisualizationEngine), denseMapper(_denseMapper), trackedImageSize(_trackedImageSize)
	{
	}

	template<class TVoxel, class TIndex>
	VoxelMapGraphManager<TVoxel, TIndex>::~VoxelMapGraphManager()
	{
		while (allData.size() > 0)
		{
			delete allData.back();
			allData.pop_back();
		}
	}

	template<class TVoxel, class TIndex>
	int VoxelMapGraphManager<TVoxel, TIndex>::createNewLocalMap()
	{
		int newIdx = (int)allData.size();
		allData.push_back(new LocalMap<TVoxel, TIndex>(visualization_engine, trackedImageSize));

		allData[newIdx]->volume->Reset();
		return newIdx;
	}

	template<class TVoxel, class TIndex>
	void VoxelMapGraphManager<TVoxel, TIndex>::removeLocalMap(int localMapId)
	{
		if ((localMapId < 0) || ((unsigned)localMapId >= allData.size())) return;

		// make sure there are no relations anywhere pointing to the local map
		const ConstraintList & l = getConstraints(localMapId);
		for (ConstraintList::const_iterator it = l.begin(); it != l.end(); ++it) eraseRelation(it->first, localMapId);

		// delete the local map
		delete allData[localMapId];
		allData.erase(allData.begin() + localMapId);
	}

	template<class TVoxel, class TIndex>
	PoseConstraint & VoxelMapGraphManager<TVoxel, TIndex>::getRelation(int fromLocalMap, int toLocalMap)
	{
		ConstraintList & m = getLocalMap(fromLocalMap)->relations;
		return m[toLocalMap];
	}

	static const PoseConstraint invalidPoseConstraint;

	template<class TVoxel, class TIndex>
	const PoseConstraint & VoxelMapGraphManager<TVoxel, TIndex>::getRelation_const(int fromLocalMap, int toLocalMap) const
	{
		if ((fromLocalMap < 0) || (fromLocalMap >= (int)allData.size())) return invalidPoseConstraint;

		const ConstraintList & m = getLocalMap(fromLocalMap)->relations;
		ConstraintList::const_iterator it = m.find(toLocalMap);
		if (it == m.end()) return invalidPoseConstraint;

		return it->second;
	}

	template<class TVoxel, class TIndex>
	void VoxelMapGraphManager<TVoxel, TIndex>::eraseRelation(int fromLocalMap, int toLocalMap)
	{
		if ((fromLocalMap < 0) || (fromLocalMap >= (int)allData.size())) return;

		std::map<int, PoseConstraint> & m = getLocalMap(fromLocalMap)->relations;
		m.erase(toLocalMap);
	}

	template<class TVoxel, class TIndex>
	bool VoxelMapGraphManager<TVoxel, TIndex>::resetTracking(int localMapId, const ORUtils::SE3Pose & pose)
	{
		if ((localMapId < 0) || ((unsigned)localMapId >= allData.size())) return false;
		allData[localMapId]->trackingState->pose_d->SetFrom(&pose);
		allData[localMapId]->trackingState->point_cloud_age = -1;
		return true;
	}

	template<class TVoxel, class TIndex>
	int VoxelMapGraphManager<TVoxel, TIndex>::getLocalMapSize(int localMapId) const
	{
		if ((localMapId < 0) || ((unsigned)localMapId >= allData.size())) return -1;

		VoxelVolume<TVoxel, TIndex>* volume = allData[localMapId]->volume;
		return volume->index.GetUtilizedBlockCount() - 1;
	}

	template<class TVoxel, class TIndex>
	int VoxelMapGraphManager<TVoxel, TIndex>::countVisibleBlocks(int localMapId, int minBlockId, int maxBlockId, bool invertIds) const
	{
		if ((localMapId < 0) || ((unsigned)localMapId >= allData.size())) return -1;
		const LocalMap<TVoxel, TIndex> *localMap = allData[localMapId];

		if (invertIds) 
		{
			int tmp = minBlockId;
			minBlockId = localMap->volume->index.GetMaximumBlockCount() - maxBlockId - 1;
			maxBlockId = localMap->volume->index.GetMaximumBlockCount() - tmp - 1;
		}

		return visualization_engine->CountVisibleBlocks(localMap->volume, localMap->renderState, minBlockId, maxBlockId);
	}

	struct LinkPathComparison 
	{
		bool operator()(const std::vector<int> & a, const std::vector<int> & b) { return a.size() > b.size(); }
	};

	template<class TVoxel, class TIndex>
	ORUtils::SE3Pose VoxelMapGraphManager<TVoxel, TIndex>::findTransformation(int fromLocalMapId, int toLocalMapId) const
	{
		ORUtils::SE3Pose fromLocalMapPose, toLocalMapPose;
		if ((fromLocalMapId >= 0) || ((size_t)fromLocalMapId < allData.size())) fromLocalMapPose = allData[fromLocalMapId]->estimatedGlobalPose;
		if ((toLocalMapId >= 0) || ((size_t)toLocalMapId < allData.size())) toLocalMapPose = allData[toLocalMapId]->estimatedGlobalPose;
		return ORUtils::SE3Pose(toLocalMapPose.GetM() * fromLocalMapPose.GetInvM());
	}
}
