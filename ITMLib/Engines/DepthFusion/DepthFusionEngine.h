//  ================================================================
//  Created by Gregory Kramida on 5/7/18.
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
#include <cmath>

//local
#include "../../Objects/RenderStates/RenderState.h"
#include "../../Objects/Volume/VoxelVolume.h"
#include "../../Objects/Tracking/CameraTrackingState.h"
#include "../../Objects/Views/View.h"
#include "../../Utils/Enums/WarpType.h"
#include "../Common/WarpAccessFunctors.h"
#include "../Common/Configurable.h"
#include "DepthFusionSettings.h"

namespace ITMLib {

/** \brief
	Interface to engines implementing the main KinectFusion
	depth integration process in dynamic fusion settings that involve 2 different SDFs with different voxel types.

	These classes basically manage
	an ITMLib::Objects::ITMScene and fuse new image information
	into them.
*/
template<typename TVoxel, typename TIndex>
class DepthFusionEngineInterface : public Configurable<DepthFusionSettings> {
protected:
	using Configurable<DepthFusionSettings>::parameters;
public:
	using Configurable<DepthFusionSettings>::Configurable;
	using Configurable<DepthFusionSettings>::GetParameters;

	DepthFusionEngineInterface() = default;
	virtual ~DepthFusionEngineInterface() = default;

	/**
	 * \brief Update the voxel blocks by integrating depth and possibly color information from the given view. Assume
	 * camera is at world origin.
	 */
	virtual void IntegrateDepthImageIntoTsdfVolume(VoxelVolume<TVoxel, TIndex>* volume, const View* view) = 0;

	/** Update the voxel blocks by integrating depth and
	possibly colour information from the given view.*/
	virtual void IntegrateDepthImageIntoTsdfVolume(VoxelVolume<TVoxel, TIndex>* volume, const View* view,
	                                               const CameraTrackingState* trackingState) = 0;

};


template<typename TVoxel, typename TIndex, MemoryDeviceType TMemoryDeviceType>
class DepthFusionEngine :
		public DepthFusionEngineInterface<TVoxel, TIndex> {
public:
	using DepthFusionEngineInterface<TVoxel, TIndex>::DepthFusionEngineInterface;
	~DepthFusionEngine() = default;

	void IntegrateDepthImageIntoTsdfVolume(VoxelVolume<TVoxel, TIndex>* volume, const View* view);
	void IntegrateDepthImageIntoTsdfVolume(VoxelVolume<TVoxel, TIndex>* volume, const View* view,
	                                       const CameraTrackingState* trackingState);
private:


	void IntegrateDepthImageIntoTsdfVolume_Helper(VoxelVolume<TVoxel, TIndex>* volume, const View* view,
	                                              Matrix4f depth_camera_matrix = Matrix4f::Identity());

};

}//namespace ITMLib