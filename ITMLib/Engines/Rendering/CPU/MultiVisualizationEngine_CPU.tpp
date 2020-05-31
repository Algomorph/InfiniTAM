// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

//stdlib
#include <vector>

//local
#include "MultiVisualizationEngine_CPU.h"

#include "../../../Objects/RenderStates/RenderStateMultiScene.h"
#include "../../../Objects/Volume/MultiSceneAccess.h"

#include "../Shared/RenderingEngine_Shared.h"

using namespace ITMLib;

template<class TVoxel, class TIndex>
void MultiVisualizationEngine_CPU<TVoxel, TIndex>::PrepareRenderState(const VoxelMapGraphManager<TVoxel, TIndex> & mapManager, RenderState *_state)
{
	RenderStateMultiScene<TVoxel, TIndex> *state = (RenderStateMultiScene<TVoxel, TIndex>*)_state;

	state->PrepareLocalMaps(mapManager);
}

template<class TVoxel>
void MultiVisualizationEngine_CPU<TVoxel, VoxelBlockHash>::PrepareRenderState(const VoxelMapGraphManager<TVoxel, VoxelBlockHash> & mapManager, RenderState *_state){
	RenderStateMultiScene<TVoxel, VoxelBlockHash> *state = (RenderStateMultiScene<TVoxel, VoxelBlockHash>*)_state;
	state->PrepareLocalMaps(mapManager);
}


template<class TVoxel, class TIndex>
void MultiVisualizationEngine_CPU<TVoxel, TIndex>::CreateExpectedDepths(const VoxelMapGraphManager<TVoxel, TIndex> & sceneManager, const ORUtils::SE3Pose *pose, const Intrinsics *intrinsics, RenderState *_renderState) const
{
}

template<class TVoxel>
void MultiVisualizationEngine_CPU<TVoxel, VoxelBlockHash>::CreateExpectedDepths(const VoxelMapGraphManager<TVoxel, VoxelBlockHash> & sceneManager, const ORUtils::SE3Pose *pose, const Intrinsics *intrinsics, RenderState *_renderState) const
{
	RenderStateMultiScene<TVoxel, VoxelBlockHash> *renderState = (RenderStateMultiScene<TVoxel, VoxelBlockHash>*)_renderState;

	// reset min max image
	Vector2i imgSize = renderState->renderingRangeImage->dimensions;
	Vector2f *minmaxData = renderState->renderingRangeImage->GetData(MEMORYDEVICE_CPU);

	for (int locId = 0; locId < imgSize.x*imgSize.y; ++locId) 
	{
		Vector2f & pixel = minmaxData[locId];
		pixel.x = FAR_AWAY;
		pixel.y = VERY_CLOSE;
	}

	// add the values from each local map
	for (int localMapId = 0; localMapId < renderState->indexData_host.numLocalMaps; ++localMapId) 
	{
		float voxelSize = renderState->voxelSize;
		const HashEntry *hash_entries = renderState->indexData_host.index[localMapId];
		int hashEntryCount = sceneManager.getLocalMap(0)->volume->index.hash_entry_count;

		std::vector<RenderingBlock> renderingBlocks(MAX_RENDERING_BLOCKS);
		int numRenderingBlocks = 0;

		Matrix4f localPose = pose->GetM() * renderState->indexData_host.posesInv[localMapId];
		for (int blockNo = 0; blockNo < hashEntryCount; ++blockNo) {
			const HashEntry & blockData(hash_entries[blockNo]);

			Vector2i upperLeft, lowerRight;
			Vector2f zRange;
			bool validProjection = false;
			if (blockData.ptr >= 0) {
				validProjection = ProjectSingleBlock(blockData.pos, localPose, intrinsics->projectionParamsSimple.all, imgSize, voxelSize, upperLeft, lowerRight, zRange);
			}
			if (!validProjection) continue;

			Vector2i requiredRenderingBlocks((int)ceilf((float)(lowerRight.x - upperLeft.x + 1) / (float)rendering_block_size_x),
				(int)ceilf((float)(lowerRight.y - upperLeft.y + 1) / (float)rendering_block_size_y));
			int requiredNumBlocks = requiredRenderingBlocks.x * requiredRenderingBlocks.y;

			if (numRenderingBlocks + requiredNumBlocks >= MAX_RENDERING_BLOCKS) continue;
			int offset = numRenderingBlocks;
			numRenderingBlocks += requiredNumBlocks;

			CreateRenderingBlocks(&(renderingBlocks[0]), offset, upperLeft, lowerRight, zRange);
		}

		// go through rendering blocks
		for (int blockNo = 0; blockNo < numRenderingBlocks; ++blockNo) 
		{
			// fill minmaxData
			const RenderingBlock & b(renderingBlocks[blockNo]);

			for (int y = b.upper_left.y; y <= b.lower_right.y; ++y) {
				for (int x = b.upper_left.x; x <= b.lower_right.x; ++x) {
					Vector2f & pixel(minmaxData[x + y*imgSize.x]);
					if (pixel.x > b.z_range.x) pixel.x = b.z_range.x;
					if (pixel.y < b.z_range.y) pixel.y = b.z_range.y;
				}
			}
		}
	}
}

template<typename TVoxel, typename TIndex>
static void RenderImage_common(const ORUtils::SE3Pose *pose, const Intrinsics *intrinsics, RenderState *_renderState, UChar4Image *outputImage, IRenderingEngine::RenderImageType type){
	RenderStateMultiScene<TVoxel, TIndex> *renderState = (RenderStateMultiScene<TVoxel, TIndex>*)_renderState;

	Vector2i imgSize = outputImage->dimensions;
	Matrix4f invM = pose->GetInvM();

	// Generic Raycast
	float voxelSize = renderState->voxelSize;
	{
		Vector4f projParams = intrinsics->projectionParamsSimple.all;
		Vector4f invProjParams = InvertProjectionParams(projParams);

		const Vector2f *minmaximg = renderState->renderingRangeImage->GetData(MEMORYDEVICE_CPU);
		float mu = renderState->mu;
		float oneOverVoxelSize = 1.0f / voxelSize;
		Vector4f *pointsRay = renderState->raycastResult->GetData(MEMORYDEVICE_CPU);

		typedef MultiVoxel<TVoxel> VD;
		typedef MultiIndex<TIndex> ID;

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
		for (int locId = 0; locId < imgSize.x*imgSize.y; ++locId)
		{
			int y = locId / imgSize.x;
			int x = locId - y*imgSize.x;
			int locId2 = (int)floor((float)x / minmaximg_subsample) + (int)floor((float)y / minmaximg_subsample) * imgSize.x;

			castRay<VD, ID, false>(pointsRay[locId], NULL, x, y, &renderState->voxelData_host, &renderState->indexData_host, invM, invProjParams, oneOverVoxelSize, mu, minmaximg[locId2]);
		}
	}

	Vector3f lightSource = -Vector3f(invM.getColumn(2));
	Vector4u *outRendering = outputImage->GetData(MEMORYDEVICE_CPU);
	Vector4f *pointsRay = renderState->raycastResult->GetData(MEMORYDEVICE_CPU);

	if ((type == IRenderingEngine::RENDER_COLOUR_FROM_VOLUME) &&
	    (!TVoxel::hasColorInformation)) type = IRenderingEngine::RENDER_SHADED_GREYSCALE;

	switch (type) {
		case IRenderingEngine::RENDER_COLOUR_FROM_VOLUME:
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
			for (int locId = 0; locId < imgSize.x * imgSize.y; locId++) {
				Vector4f ptRay = pointsRay[locId];
				processPixelColour<MultiVoxel<TVoxel>, MultiIndex<TIndex> >(outRendering[locId], ptRay.toVector3(), ptRay.w > 0, &(renderState->voxelData_host),
				                                                            &(renderState->indexData_host));
			}
			break;
		case IRenderingEngine::RENDER_COLOUR_FROM_NORMAL:
			if (intrinsics->FocalLengthSignsDiffer())
			{
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
				for (int locId = 0; locId < imgSize.x * imgSize.y; locId++)
				{
					int y = locId / imgSize.x, x = locId - y*imgSize.x;
					processPixelNormals_ImageNormals<true, true>(outRendering, pointsRay, imgSize, x, y, voxelSize, lightSource);
				}
			}
			else
			{
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
				for (int locId = 0; locId < imgSize.x * imgSize.y; locId++)
				{
					int y = locId / imgSize.x, x = locId - y*imgSize.x;
					processPixelNormals_ImageNormals<true, false>(outRendering, pointsRay, imgSize, x, y, voxelSize, lightSource);
				}
			}
			break;
		case IRenderingEngine::RENDER_COLOUR_FROM_CONFIDENCE:
			if (intrinsics->FocalLengthSignsDiffer())
			{
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
				for (int locId = 0; locId < imgSize.x * imgSize.y; locId++)
				{
					int y = locId / imgSize.x, x = locId - y*imgSize.x;
					processPixelConfidence_ImageNormals<true, true>(outRendering, pointsRay, imgSize, x, y, voxelSize, lightSource);
				}
			}
			else
			{
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
				for (int locId = 0; locId < imgSize.x * imgSize.y; locId++)
				{
					int y = locId / imgSize.x, x = locId - y*imgSize.x;
					processPixelConfidence_ImageNormals<true, false>(outRendering, pointsRay, imgSize, x, y, voxelSize, lightSource);
				}
			}
			break;
		case IRenderingEngine::RENDER_SHADED_GREYSCALE:
		default:
			if (intrinsics->FocalLengthSignsDiffer())
			{
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
				for (int locId = 0; locId < imgSize.x * imgSize.y; locId++)
				{
					int y = locId / imgSize.x, x = locId - y*imgSize.x;
					processPixelGrey_ImageNormals<true, true>(outRendering, pointsRay, imgSize, x, y, voxelSize, lightSource);
				}
			}
			else
			{
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
				for (int locId = 0; locId < imgSize.x * imgSize.y; locId++)
				{
					int y = locId / imgSize.x, x = locId - y*imgSize.x;
					processPixelGrey_ImageNormals<true, false>(outRendering, pointsRay, imgSize, x, y, voxelSize, lightSource);
				}
			}
			break;
	}
};

template<class TVoxel, class TIndex>
void MultiVisualizationEngine_CPU<TVoxel, TIndex>::RenderImage(const ORUtils::SE3Pose *pose, const Intrinsics *intrinsics, RenderState *_renderState, UChar4Image *outputImage, IRenderingEngine::RenderImageType type) const
{
	RenderImage_common<TVoxel,TIndex>(pose,intrinsics,_renderState,outputImage,type);
}

template<class TVoxel>
void MultiVisualizationEngine_CPU<TVoxel, VoxelBlockHash>::RenderImage(const ORUtils::SE3Pose *pose, const Intrinsics *intrinsics, RenderState *_renderState, UChar4Image *outputImage, IRenderingEngine::RenderImageType type) const
{
	RenderImage_common<TVoxel,VoxelBlockHash>(pose,intrinsics,_renderState,outputImage,type);
}

