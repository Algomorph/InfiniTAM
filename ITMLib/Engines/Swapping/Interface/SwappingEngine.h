// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../../Objects/RenderStates/RenderState.h"
#include "../../../Objects/Volume/VoxelVolume.h"
#include "../../../Objects/Views/View.h"

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
		virtual void IntegrateGlobalIntoLocal(VoxelVolume<TVoxel, TIndex> *scene, RenderState *renderState) = 0;
		virtual void SaveToGlobalMemory(VoxelVolume<TVoxel, TIndex> *scene, RenderState *renderState) = 0;
		virtual void CleanLocalMemory(VoxelVolume<TVoxel, TIndex> *scene, RenderState *renderState) = 0;
	};
}
