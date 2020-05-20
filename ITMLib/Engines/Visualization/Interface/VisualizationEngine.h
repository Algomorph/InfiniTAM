// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../../Objects/RenderStates/RenderState.h"
#include "../../../Objects/Volume/VoxelVolume.h"
#include "../../../Objects/Tracking/CameraTrackingState.h"
#include "../../../Objects/Views/View.h"

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

		virtual ~IVisualizationEngine() {}

		static void DepthToUchar4(UChar4Image *dst, const FloatImage *src);
		static void NormalToUchar4(UChar4Image* dst, const Float4Image *src);
		static void WeightToUchar4(UChar4Image *dst, const FloatImage *src);
	};

	template<class TIndex> struct IndexToRenderState { typedef RenderState type; };
	template<> struct IndexToRenderState<VoxelBlockHash> { typedef RenderState type; };

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

		/** Given a scene, pose and intrinsics, compute_allocated the
		visible subset of the scene and store it in an
		appropriate RenderState object.
		*/
		virtual void FindVisibleBlocks(VoxelVolume<TVoxel,TIndex> *scene, const ORUtils::SE3Pose *pose, const Intrinsics *intrinsics,
		                               RenderState *renderState) const = 0;

		/** Given a render state, Count the number of visible blocks
		with minBlockId <= blockID <= maxBlockId .
		*/
		virtual int CountVisibleBlocks(const VoxelVolume<TVoxel,TIndex> *scene, const RenderState *renderState, int minBlockId, int maxBlockId) const = 0;

		/** Given scene, pose and intrinsics, create an estimate
		of the minimum and maximum depths at each pixel of
		an image.
		*/
		virtual void CreateExpectedDepths(const VoxelVolume<TVoxel,TIndex> *scene, const ORUtils::SE3Pose *pose, const Intrinsics *intrinsics,
		                                  RenderState *renderState) const = 0;

		/** This will render an image using raycasting. */
		virtual void RenderImage(VoxelVolume<TVoxel,TIndex> *scene, const ORUtils::SE3Pose *pose, const Intrinsics *intrinsics,
		                         const RenderState *renderState, UChar4Image *outputImage, RenderImageType type = RENDER_SHADED_GREYSCALE, RenderRaycastSelection raycastType = RENDER_FROM_NEW_RAYCAST) const = 0;

		/** Finds the scene surface using raycasting. */
		virtual void FindSurface(VoxelVolume<TVoxel,TIndex> *scene, const ORUtils::SE3Pose *pose, const Intrinsics *intrinsics,
		                         const RenderState *renderState) const = 0;

		/** Create a point cloud as required by the
		ITMLib::Engine::ITMColorTracker classes.
		*/
		virtual void CreatePointCloud(VoxelVolume<TVoxel,TIndex> *scene, const View *view, CameraTrackingState *trackingState,
		                              RenderState *renderState, bool skipPoints) const = 0;

		/** Create an image of reference points and normals as
		required by the ITMLib::Engine::ITMDepthTracker classes.
		*/
		virtual void CreateICPMaps(VoxelVolume<TVoxel,TIndex> *scene, const View *view, CameraTrackingState *trackingState,
		                           RenderState *renderState) const = 0;

		/** Create an image of reference points and normals as
		required by the ITMLib::Engine::ITMDepthTracker classes.

		Incrementally previous raycast result.
		*/
		virtual void ForwardRender(const VoxelVolume<TVoxel,TIndex> *scene, const View *view, CameraTrackingState *trackingState,
		                           RenderState *renderState) const = 0;
	};
}
