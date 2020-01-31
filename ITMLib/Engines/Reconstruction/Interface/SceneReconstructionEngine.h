// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include <math.h>

#include "../../../Objects/RenderStates/RenderState.h"
#include "../../../Objects/Scene/VoxelVolume.h"
#include "../../../Objects/Tracking/ITMTrackingState.h"
#include "../../../Objects/Views/ITMView.h"

namespace ITMLib
{
	/** \brief
	    Interface to engines implementing the main KinectFusion
	    depth integration process.

	    These classes basically manage
	    an ITMLib::Objects::ITMScene and fuse new image information
	    into them.
	*/
	template<class TVoxel, class TIndex>
	class SceneReconstructionEngine
	{
	public:
		/** Clear and reset a scene to set up a new empty
		    one.
		*/
		virtual void ResetScene(VoxelVolume<TVoxel, TIndex> *scene) = 0;

		/**
		 * \brief Given a view with a new depth image, compute the
		    visible blocks, allocate them and update the hash
		    table so that the new image data can be integrated.
		 * \param scene [out] the scene whose hash needs additional allocations
		 * \param view [in] a view with a new depth image
		 * \param trackingState [in] tracking state from previous frame to new frame that corresponds to the given view
		 * \param renderState [in] the current renderState with information about which hash entries are visible
		 * \param onlyUpdateVisibleList [in] whether we want to allocate only the hash entry blocks currently visible
		 * \param resetVisibleList  [in] reset visibility list upon completion
		 */
		virtual void AllocateSceneFromDepth(VoxelVolume<TVoxel,TIndex> *scene, const ITMView *view, const ITMTrackingState *trackingState,
		                                    const RenderState *renderState, bool onlyUpdateVisibleList = false, bool resetVisibleList = false) = 0;

		/** Update the voxel blocks by integrating depth and
		    possibly colour information from the given view.
		*/
		virtual void IntegrateIntoScene(VoxelVolume<TVoxel,TIndex> *scene, const ITMView *view, const ITMTrackingState *trackingState,
		                                const RenderState *renderState) = 0;

		SceneReconstructionEngine(void) { }
		virtual ~SceneReconstructionEngine(void) { }
	};
}
