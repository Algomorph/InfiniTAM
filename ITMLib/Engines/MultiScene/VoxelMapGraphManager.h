// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include <vector>

#include "../../Objects/Volume/LocalMap.h"
#include "../Main/Mappers/DenseMapper.h"
#include "../Visualization/Interface/VisualizationEngine.h"

namespace ITMLib
{
	/* This helpful abstract interface allows you to ignore the fact that
	scenes are templates.
	*/
	class MapGraphManager
	{
	public:
		virtual ~MapGraphManager() {}

		virtual int createNewLocalMap() = 0;
		virtual void removeLocalMap(int index) = 0;
		virtual size_t numLocalMaps() const = 0;

		virtual const PoseConstraint & getRelation_const(int fromLocalMap, int toLocalMap) const = 0;
		virtual PoseConstraint & getRelation(int fromLocalMap, int toLocalMap) = 0;
		virtual void eraseRelation(int fromLocalMap, int toLocalMap) = 0;
		virtual const ConstraintList & getConstraints(int localMapId) const = 0;

		virtual void setEstimatedGlobalPose(int localMapId, const ORUtils::SE3Pose & pose) = 0;
		virtual const ORUtils::SE3Pose & getEstimatedGlobalPose(int localMapId) const = 0;

		virtual bool resetTracking(int localMapId, const ORUtils::SE3Pose & pose) = 0;

		virtual const ORUtils::SE3Pose* getTrackingPose(int localMapId) const = 0;
		virtual int getLocalMapSize(int localMapId) const = 0;
		virtual int countVisibleBlocks(int localMapId, int minBlockId, int maxBlockId, bool invertIDs) const = 0;
	};

	template<class TVoxel, class TIndex>
	class VoxelMapGraphManager : public MapGraphManager
	{
	private:
		const VisualizationEngine<TVoxel, TIndex> *visualization_engine;
		const DenseMapper<TVoxel, TIndex> *denseMapper;
		Vector2i trackedImageSize;

		std::vector<LocalMap<TVoxel, TIndex>*> allData;

	public:
		VoxelMapGraphManager(const VisualizationEngine<TVoxel, TIndex>* visualizationEngine,
		                     const DenseMapper<TVoxel, TIndex>* denseMapper, const Vector2i& trackedImageSize);
		~VoxelMapGraphManager();

		int createNewLocalMap();
		void removeLocalMap(int index);
		size_t numLocalMaps() const { return allData.size(); }

		const LocalMap<TVoxel, TIndex>* getLocalMap(int localMapId) const { return allData[localMapId]; }

		LocalMap<TVoxel, TIndex>* getLocalMap(int localMapId) { return allData[localMapId]; }

		const PoseConstraint & getRelation_const(int fromLocalMap, int toLocalMap) const;
		PoseConstraint & getRelation(int fromLocalMap, int toLocalMap);
		void eraseRelation(int fromLocalMap, int toLocalMap);
		const ConstraintList & getConstraints(int localMapId) const { return allData[localMapId]->relations; }

		void setEstimatedGlobalPose(int localMapId, const ORUtils::SE3Pose & pose) { allData[localMapId]->estimatedGlobalPose = pose; }
		const ORUtils::SE3Pose & getEstimatedGlobalPose(int localMapId) const { return allData[localMapId]->estimatedGlobalPose; }

		bool resetTracking(int localMapId, const ORUtils::SE3Pose & pose);
		const ORUtils::SE3Pose* getTrackingPose(int localMapId) const { return getLocalMap(localMapId)->trackingState->pose_d; }

		int getLocalMapSize(int localMapId) const;
		int countVisibleBlocks(int localMapId, int minBlockId, int maxBlockId, bool invertIDs) const;

		ORUtils::SE3Pose findTransformation(int fromlocalMapId, int tolocalMapId) const;
	};
}