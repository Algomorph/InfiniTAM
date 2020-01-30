// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../../Objects/RenderStates/ITMRenderState.h"
#include "../../../Objects/Scene/ITMVoxelVolume.h"
#include "../../../Objects/Tracking/ITMTrackingState.h"
#include "../../../Objects/Views/ITMView.h"

namespace ITMLib
{
	class IVisualizationEngine
	{
	public:
		enum RenderImageType
		{
			RENDER_SHADED_GREYSCALE,
			RENDER_SHADED_GREEN,
			RENDER_SHADED_OVERLAY,
			RENDER_SHADED_GREYSCALE_IMAGENORMALS,
			RENDER_COLOUR_FROM_VOLUME,
			RENDER_COLOUR_FROM_NORMAL,
			RENDER_COLOUR_FROM_CONFIDENCE
		};

		enum RenderRaycastSelection
		{
			RENDER_FROM_NEW_RAYCAST,
			RENDER_FROM_OLD_RAYCAST,
			RENDER_FROM_OLD_FORWARDPROJ
		};

		virtual ~IVisualizationEngine(void) {}

		static void DepthToUchar4(ITMUChar4Image *dst, const ITMFloatImage *src);
		static void NormalToUchar4(ITMUChar4Image* dst, const ITMFloat4Image *src);
		static void WeightToUchar4(ITMUChar4Image *dst, const ITMFloatImage *src);
	};

	template<class TIndex> struct IndexToRenderState { typedef ITMRenderState type; };
	template<> struct IndexToRenderState<VoxelBlockHash> { typedef ITMRenderState type; };

	/** \brief
		Interface to engines helping with the Visualization of
		the results from the rest of the library.

		This is also used internally to get depth estimates for the
		raycasting done for the trackers. The basic idea there is
		to project down a scene of 8x8x8 voxel
		blocks and look at the bounding boxes. The projection
		provides an idea of the possible depth range for each pixel
		in an image, which can be used to speed up raycasting
		operations.
		*/
	template<class TVoxel, class TIndex>
	class VisualizationEngine : public IVisualizationEngine
	{
	public:

		/** Given a scene, pose and intrinsics, compute the
		visible subset of the scene and store it in an
		appropriate Visualization state object, created
		previously using allocateInternalState().
		*/
		virtual void FindVisibleBlocks(ITMVoxelVolume<TVoxel,TIndex> *scene, const ORUtils::SE3Pose *pose, const ITMIntrinsics *intrinsics,
		                               ITMRenderState *renderState) const = 0;

		/** Given a render state, Count the number of visible blocks
		with minBlockId <= blockID <= maxBlockId .
		*/
		virtual int CountVisibleBlocks(const ITMVoxelVolume<TVoxel,TIndex> *scene, const ITMRenderState *renderState, int minBlockId, int maxBlockId) const = 0;

		/** Given scene, pose and intrinsics, create an estimate
		of the minimum and maximum depths at each pixel of
		an image.
		*/
		virtual void CreateExpectedDepths(const ITMVoxelVolume<TVoxel,TIndex> *scene, const ORUtils::SE3Pose *pose, const ITMIntrinsics *intrinsics,
		                                  ITMRenderState *renderState) const = 0;

		/** This will render an image using raycasting. */
		virtual void RenderImage(ITMVoxelVolume<TVoxel,TIndex> *scene, const ORUtils::SE3Pose *pose, const ITMIntrinsics *intrinsics,
		                         const ITMRenderState *renderState, ITMUChar4Image *outputImage, RenderImageType type = RENDER_SHADED_GREYSCALE, RenderRaycastSelection raycastType = RENDER_FROM_NEW_RAYCAST) const = 0;

		/** Finds the scene surface using raycasting. */
		virtual void FindSurface(ITMVoxelVolume<TVoxel,TIndex> *scene, const ORUtils::SE3Pose *pose, const ITMIntrinsics *intrinsics,
		                         const ITMRenderState *renderState) const = 0;

		/** Create a point cloud as required by the
		ITMLib::Engine::ITMColorTracker classes.
		*/
		virtual void CreatePointCloud(ITMVoxelVolume<TVoxel,TIndex> *scene, const ITMView *view, ITMTrackingState *trackingState,
		                              ITMRenderState *renderState, bool skipPoints) const = 0;

		/** Create an image of reference points and normals as
		required by the ITMLib::Engine::ITMDepthTracker classes.
		*/
		virtual void CreateICPMaps(ITMVoxelVolume<TVoxel,TIndex> *scene, const ITMView *view, ITMTrackingState *trackingState,
		                           ITMRenderState *renderState) const = 0;

		/** Create an image of reference points and normals as
		required by the ITMLib::Engine::ITMDepthTracker classes.

		Incrementally previous raycast result.
		*/
		virtual void ForwardRender(const ITMVoxelVolume<TVoxel,TIndex> *scene, const ITMView *view, ITMTrackingState *trackingState,
		                           ITMRenderState *renderState) const = 0;
	};
}
