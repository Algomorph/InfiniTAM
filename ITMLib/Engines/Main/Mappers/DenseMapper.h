// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../Reconstruction/Interface/SceneReconstructionEngine.h"
#include "../../Swapping/Interface/SwappingEngine.h"
#include "../../../Utils/Configuration.h"

namespace ITMLib
{
	/** \brief
	*/
	template<class TVoxel, class TIndex>
	class DenseMapper
	{
	private:
		SceneReconstructionEngine<TVoxel,TIndex> *sceneRecoEngine;
		SwappingEngine<TVoxel,TIndex> *swappingEngine;

		configuration::SwappingMode swappingMode;

	public:

		/// Process a single frame
		void ProcessFrame(const ITMView *view, const ITMTrackingState *trackingState, VoxelVolume<TVoxel,TIndex> *scene, RenderState *renderState_live);

		/// Update the visible list (this can be called to update the visible list when fusion is turned off)
		void UpdateVisibleList(const ITMView *view, const ITMTrackingState *trackingState, VoxelVolume<TVoxel, TIndex> *scene, RenderState *renderState, bool resetVisibleList = false);

		/** \brief Constructor
		    Ommitting a separate image size for the depth images
		    will assume same resolution as for the RGB images.
		*/
		explicit DenseMapper(const TIndex& index);
		~DenseMapper();
	};
}
