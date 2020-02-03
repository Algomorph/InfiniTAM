// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include <map>

#include "../../Engines/Visualization/Interface/VisualizationEngine.h"
#include "../RenderStates/RenderState.h"
#include "VoxelVolume.h"
#include "../Tracking/CameraTrackingState.h"
#include "../../Utils/Configuration.h"

namespace ITMLib {
	struct PoseConstraint
	{
	public:
		PoseConstraint(void)
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

		ORUtils::SE3Pose GetAccumulatedObservations(void) const { return accu_poses; }
		int GetNumAccumulatedObservations(void) const { return accu_num; }

	private:
		ORUtils::SE3Pose accu_poses;
		int accu_num;
	};

	typedef std::map<int, PoseConstraint> ConstraintList;

	template<class TVoxel, class TIndex>
	class LocalMap
	{
	public:
		VoxelVolume<TVoxel, TIndex> *scene;
		RenderState *renderState;
		CameraTrackingState *trackingState;
		ConstraintList relations;
		ORUtils::SE3Pose estimatedGlobalPose;

		LocalMap(const VisualizationEngine<TVoxel, TIndex>* VisualizationEngine, const Vector2i& trackedImageSize)
		{
			auto& settings = configuration::get();
			MemoryDeviceType memoryType = settings.device_type == MEMORYDEVICE_CUDA ? MEMORYDEVICE_CUDA : MEMORYDEVICE_CPU;
			scene = new VoxelVolume<TVoxel, TIndex>(&settings.general_voxel_volume_parameters, settings.swapping_mode == configuration::SWAPPINGMODE_ENABLED, memoryType);
			renderState = new RenderState(trackedImageSize, settings.general_voxel_volume_parameters.near_clipping_distance,
			                              settings.general_voxel_volume_parameters.far_clipping_distance, memoryType);
			trackingState = new CameraTrackingState(trackedImageSize, memoryType);
		}
		~LocalMap()
		{
			delete scene;
			delete renderState;
			delete trackingState;
		}
	};
}

