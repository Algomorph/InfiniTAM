// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../../Objects/RenderStates/ITMRenderState.h"
#include "../../../Objects/Scene/ITMVoxelVolume.h"
#include "../../../Objects/Views/ITMView.h"

namespace ITMLib
{
	/** \brief
	Interface to engines that swap data in and out of the
	fairly limited GPU memory to some large scale storage
	space.
	*/
	template<class TVoxel, class TIndex>
	class SwappingEngine
	{
	public:
		virtual void IntegrateGlobalIntoLocal(ITMVoxelVolume<TVoxel, TIndex> *scene, ITMRenderState *renderState) = 0;
		virtual void SaveToGlobalMemory(ITMVoxelVolume<TVoxel, TIndex> *scene, ITMRenderState *renderState) = 0;
		virtual void CleanLocalMemory(ITMVoxelVolume<TVoxel, TIndex> *scene, ITMRenderState *renderState) = 0;

		virtual ~SwappingEngine(void) { }
	};
}
