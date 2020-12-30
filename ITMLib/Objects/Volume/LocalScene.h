// Copyright 2016 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include <map>

#include "../../Engines/Raycasting/Interface/RaycastingEngineInterface.h"
#include "../RenderStates/RenderState.h"
#include "VoxelVolume.h"
#include "../Tracking/CameraTrackingState.h"
#include "../../Utils/Configuration/Configuration.h"

namespace ITMLib {
	struct ITMPoseConstraint
	{
	public:
		ITMPoseConstraint()
		{
			accu_num = 0;
		}

		void AddObservation(const ORUtils::SE3Pose & relative_pose, int weight = 1)
		{
			Matrix4f tmp = accu_poses.GetM() * (float)accu_num + relative_pose.GetM() * (float)weight;
			accu_num += weight;
			accu_poses.SetM(tmp / (float)accu_num);
			accu_poses.Coerce();
			//	accu_poses = (accu_poses * (float)accu_num + relative_pose)/(float)(accu_num+1);
			accu_num++;
		}

		ORUtils::SE3Pose GetAccumulatedObservations() const { return accu_poses; }
		int GetNumAccumulatedObservations() const { return accu_num; }

	private:
		ORUtils::SE3Pose accu_poses;
		int accu_num;
	};

	typedef std::map<int, ITMPoseConstraint> ConstraintList;

	template<class TVoxel, class TIndex>
	class ITMLocalMap
	{
	public:
		ITMScene<TVoxel, TIndex> *scene;
		ITMRenderState *renderState;
		ITMTrackingState *trackingState;
		ConstraintList relations;
		ORUtils::SE3Pose estimatedGlobalPose;

		ITMLocalMap(const ITMLibSettings *settings, const VisualizationEngine<TVoxel, TIndex> *VisualizationEngine, const Vector2i & trackedImageSize)
		{
			MemoryDeviceType memoryType = settings->deviceType == ITMLibSettings::DEVICE_CUDA ? MEMORYDEVICE_CUDA : MEMORYDEVICE_CPU;
			scene = new ITMScene<TVoxel, TIndex>(&settings->sceneParams, settings->swappingMode == ITMLibSettings::SWAPPINGMODE_ENABLED, memoryType);
			renderState = VisualizationEngine->CreateRenderState(scene, trackedImageSize);
			trackingState = new ITMTrackingState(trackedImageSize, memoryType);
		}
		~ITMLocalMap()
		{
			delete scene;
			delete renderState;
			delete trackingState;
		}
	};
}

