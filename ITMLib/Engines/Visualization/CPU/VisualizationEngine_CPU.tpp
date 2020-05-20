// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

//stdlib
#include <vector>

//local
#include "VisualizationEngine_CPU.h"
#include "../Shared/VisualizationEngine_Shared.h"
//#include "../../Reconstruction/Shared/SceneReconstructionEngine_Shared.h"
#include "../../../Utils/Geometry/CheckBlockVisibility.h"

using namespace ITMLib;

template<class TVoxel, class TIndex>
static int RenderPointCloud(Vector4u* outRendering, Vector4f* locations, Vector4f* colours, const Vector4f* ptsRay,
                            const TVoxel* voxelData, const typename TIndex::IndexData* voxelIndex, bool skipPoints,
                            float voxelSize,
                            Vector2i imgSize, Vector3f lightSource);

template<class TVoxel, class TIndex>
void VisualizationEngine_CPU<TVoxel, TIndex>::FindVisibleBlocks(VoxelVolume<TVoxel, TIndex>* scene,
                                                                const ORUtils::SE3Pose* pose,
                                                                const Intrinsics* intrinsics,
                                                                RenderState* renderState) const {
}

template<class TVoxel>
void VisualizationEngine_CPU<TVoxel, VoxelBlockHash>::FindVisibleBlocks(
		VoxelVolume<TVoxel, VoxelBlockHash>* volume, const ORUtils::SE3Pose* pose, const Intrinsics* intrinsics,
		RenderState* renderState) const {
	const HashEntry* hash_table = volume->index.GetEntries();
	int hash_entry_count = volume->index.hash_entry_count;
	float voxel_size = volume->GetParameters().voxel_size;
	Vector2i image_size = renderState->renderingRangeImage->dimensions;

	Matrix4f M = pose->GetM();
	Vector4f projParams = intrinsics->projectionParamsSimple.all;

	int visible_block_count = 0;
	int* visible_block_hash_codes = volume->index.GetVisibleBlockHashCodes();

	//build visible list
	for (int hash_code = 0; hash_code < hash_entry_count; hash_code++) {
		unsigned char block_visibility_type = 0;// = blockVisibilityTypes[targetIdx];
		const HashEntry& hash_entry = hash_table[hash_code];

		if (hash_entry.ptr >= 0) {
			bool is_visible, is_visible_enlarged;
			CheckVoxelHashBlockVisibility<false>(is_visible, is_visible_enlarged, hash_entry.pos, M, projParams,
			                                     voxel_size,
			                                     image_size);
			block_visibility_type = is_visible;
		}

		if (block_visibility_type > 0) {
			visible_block_hash_codes[visible_block_count] = hash_code;
			visible_block_count++;
		}
	}
	volume->index.SetVisibleBlockCount(visible_block_count);
}

template<class TVoxel, class TIndex>
int VisualizationEngine_CPU<TVoxel, TIndex>::CountVisibleBlocks(const VoxelVolume<TVoxel, TIndex>* scene,
                                                                const RenderState* renderState, int minBlockId,
                                                                int maxBlockId) const {
	return 1;
}

template<class TVoxel>
int
VisualizationEngine_CPU<TVoxel, VoxelBlockHash>::CountVisibleBlocks(const VoxelVolume<TVoxel, VoxelBlockHash>* volume,
                                                                    const RenderState* renderState, int minBlockId,
                                                                    int maxBlockId) const {

	int visibleBlockCount = volume->index.GetVisibleBlockCount();
	const int* visibleBlockHashCodes = volume->index.GetVisibleBlockHashCodes();

	int ret = 0;
	for (int i = 0; i < visibleBlockCount; ++i) {
		int blockID = volume->index.GetEntries()[visibleBlockHashCodes[i]].ptr;
		if ((blockID >= minBlockId) && (blockID <= maxBlockId)) ++ret;
	}

	return ret;
}

template<class TVoxel, class TIndex>
void VisualizationEngine_CPU<TVoxel, TIndex>::CreateExpectedDepths(const VoxelVolume<TVoxel, TIndex>* scene,
                                                                   const ORUtils::SE3Pose* pose,
                                                                   const Intrinsics* intrinsics,
                                                                   RenderState* renderState) const {
	Vector2i imgSize = renderState->renderingRangeImage->dimensions;
	Vector2f* minmaxData = renderState->renderingRangeImage->GetData(MEMORYDEVICE_CPU);

	for (int locId = 0; locId < imgSize.x * imgSize.y; ++locId) {
		//TODO : this could be improved a bit...
		Vector2f& pixel = minmaxData[locId];
		pixel.x = 0.2f;
		pixel.y = 3.0f;
	}
}

template<class TVoxel>
void
VisualizationEngine_CPU<TVoxel, VoxelBlockHash>::CreateExpectedDepths(const VoxelVolume<TVoxel, VoxelBlockHash>* volume,
                                                                      const ORUtils::SE3Pose* pose,
                                                                      const Intrinsics* intrinsics,
                                                                      RenderState* renderState) const {
	Vector2i imgSize = renderState->renderingRangeImage->dimensions;
	Vector2f* minmaxData = renderState->renderingRangeImage->GetData(MEMORYDEVICE_CPU);

	for (int locId = 0; locId < imgSize.x * imgSize.y; ++locId) {
		Vector2f& pixel = minmaxData[locId];
		pixel.x = FAR_AWAY;
		pixel.y = VERY_CLOSE;
	}

	float voxelSize = volume->GetParameters().voxel_size;

	std::vector<RenderingBlock> render_blocks(MAX_RENDERING_BLOCKS);
	int render_block_count = 0;

	const int* visible_block_hash_codes = volume->index.GetUtilizedBlockHashCodes();
	int visible_block_count = volume->index.GetUtilizedBlockCount();

	//go through list of visible 8x8x8 blocks
	for (int blockNo = 0; blockNo < visible_block_count; ++blockNo) {
		const HashEntry& blockData(volume->index.GetEntries()[visible_block_hash_codes[blockNo]]);

		Vector2i upperLeft, lowerRight;
		Vector2f zRange;
		bool validProjection = false;
		if (blockData.ptr >= 0) {
			validProjection = ProjectSingleBlock(blockData.pos, pose->GetM(), intrinsics->projectionParamsSimple.all,
			                                     imgSize, voxelSize, upperLeft, lowerRight, zRange);
		}
		if (!validProjection) continue;

		Vector2i requiredRenderingBlocks(
				(int) ceilf((float) (lowerRight.x - upperLeft.x + 1) / (float) renderingBlockSizeX),
				(int) ceilf((float) (lowerRight.y - upperLeft.y + 1) / (float) renderingBlockSizeY));
		int requiredNumBlocks = requiredRenderingBlocks.x * requiredRenderingBlocks.y;

		if (render_block_count + requiredNumBlocks >= MAX_RENDERING_BLOCKS) continue;
		int offset = render_block_count;
		render_block_count += requiredNumBlocks;

		CreateRenderingBlocks(&(render_blocks[0]), offset, upperLeft, lowerRight, zRange);
	}

	// go through rendering blocks
	for (int blockNo = 0; blockNo < render_block_count; ++blockNo) {
		// fill minmaxData
		const RenderingBlock& b(render_blocks[blockNo]);

		for (int y = b.upperLeft.y; y <= b.lowerRight.y; ++y) {
			for (int x = b.upperLeft.x; x <= b.lowerRight.x; ++x) {
				Vector2f& pixel(minmaxData[x + y * imgSize.x]);
				if (pixel.x > b.zRange.x) pixel.x = b.zRange.x;
				if (pixel.y < b.zRange.y) pixel.y = b.zRange.y;
			}
		}
	}
}

template<typename TIndex>
static inline HashBlockVisibility* GetBlockVisibilityTypes(TIndex& index);

template<>
inline HashBlockVisibility* GetBlockVisibilityTypes<VoxelBlockHash>(VoxelBlockHash& index) {
	return index.GetBlockVisibilityTypes();
}

template<>
inline HashBlockVisibility* GetBlockVisibilityTypes<PlainVoxelArray>(PlainVoxelArray& index) {
	return nullptr;
}

template<class TVoxel, class TIndex>
static void GenericRaycast(VoxelVolume<TVoxel, TIndex>* volume, const Vector2i& imgSize, const Matrix4f& invM,
                           const Vector4f& projParams, const RenderState* renderState, bool updateVisibleList) {
	const Vector2f* minmaximg = renderState->renderingRangeImage->GetData(MEMORYDEVICE_CPU);
	const float mu = volume->GetParameters().narrow_band_half_width;
	const float oneOverVoxelSize = 1.0f / volume->GetParameters().voxel_size;
	Vector4f* pointsRay = renderState->raycastResult->GetData(MEMORYDEVICE_CPU);
	const TVoxel* voxelData = volume->GetVoxels();
	const typename TIndex::IndexData* voxelIndex = volume->index.GetIndexData();
	HashBlockVisibility* blockVisibilityTypes = GetBlockVisibilityTypes(volume->index);

#ifdef WITH_OPENMP
#pragma omp parallel for default(none) shared(imgSize, minmaximg, projParams, invM, pointsRay, blockVisibilityTypes, voxelData, voxelIndex)
#endif
	for (int locId = 0; locId < imgSize.x * imgSize.y; ++locId) {
		int y = locId / imgSize.x;
		int x = locId - y * imgSize.x;

		int locId2 =
				(int) floor((float) x / minmaximg_subsample) + (int) floor((float) y / minmaximg_subsample) * imgSize.x;

		if (blockVisibilityTypes != nullptr)
			castRay<TVoxel, TIndex, true>(
					pointsRay[locId],
					blockVisibilityTypes,
					x, y,
					voxelData,
					voxelIndex,
					invM,
					InvertProjectionParams(projParams),
					oneOverVoxelSize,
					mu,
					minmaximg[locId2]
			);
		else
			castRay<TVoxel, TIndex, false>(
					pointsRay[locId],
					NULL,
					x, y,
					voxelData,
					voxelIndex,
					invM,
					InvertProjectionParams(projParams),
					oneOverVoxelSize,
					mu,
					minmaximg[locId2]
			);
	}
}

template<class TVoxel, class TIndex>
static void
RenderImage_common(VoxelVolume<TVoxel, TIndex>* volume, const ORUtils::SE3Pose* pose, const Intrinsics* intrinsics,
                   const RenderState* renderState, UChar4Image* outputImage, IVisualizationEngine::RenderImageType type,
                   IVisualizationEngine::RenderRaycastSelection raycastType) {
	Vector2i imgSize = outputImage->dimensions;
	Matrix4f invM = pose->GetInvM();

	Vector4f* pointsRay;
	if (raycastType == IVisualizationEngine::RENDER_FROM_OLD_RAYCAST)
		pointsRay = renderState->raycastResult->GetData(MEMORYDEVICE_CPU);
	else {
		if (raycastType == IVisualizationEngine::RENDER_FROM_OLD_FORWARDPROJ)
			pointsRay = renderState->forwardProjection->GetData(MEMORYDEVICE_CPU);
		else {
			// this one is generally done for freeview Visualization, so
			// no, do not update the list of visible blocks
			GenericRaycast(volume, imgSize, invM, intrinsics->projectionParamsSimple.all, renderState, false);
			pointsRay = renderState->raycastResult->GetData(MEMORYDEVICE_CPU);
		}
	}

	Vector3f lightSource = -Vector3f(invM.getColumn(2));
	Vector4u* outRendering = outputImage->GetData(MEMORYDEVICE_CPU);
	const TVoxel* voxelData = volume->GetVoxels();
	const typename TIndex::IndexData* voxelIndex = volume->index.GetIndexData();

	if ((type == IVisualizationEngine::RENDER_COLOUR_FROM_VOLUME) &&
	    (!TVoxel::hasColorInformation))
		type = IVisualizationEngine::RENDER_SHADED_GREYSCALE;

	switch (type) {
		case IVisualizationEngine::RENDER_COLOUR_FROM_VOLUME:
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
			for (int locId = 0; locId < imgSize.x * imgSize.y; locId++) {
				Vector4f ptRay = pointsRay[locId];
				processPixelColour<TVoxel, TIndex>(outRendering[locId], ptRay.toVector3(), ptRay.w > 0, voxelData,
				                                   voxelIndex);
			}
			break;
		case IVisualizationEngine::RENDER_COLOUR_FROM_NORMAL:
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
			for (int locId = 0; locId < imgSize.x * imgSize.y; locId++) {
				Vector4f ptRay = pointsRay[locId];
				processPixelNormal<TVoxel, TIndex>(outRendering[locId], ptRay.toVector3(), ptRay.w > 0, voxelData,
				                                   voxelIndex, lightSource);
			}
			break;
		case IVisualizationEngine::RENDER_COLOUR_FROM_CONFIDENCE:
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
			for (int locId = 0; locId < imgSize.x * imgSize.y; locId++) {
				Vector4f ptRay = pointsRay[locId];
				processPixelConfidence<TVoxel, TIndex>(outRendering[locId], ptRay, ptRay.w > 0, voxelData, voxelIndex,
				                                       lightSource);
			}
			break;
		case IVisualizationEngine::RENDER_SHADED_GREYSCALE_IMAGENORMALS:
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
			for (int locId = 0; locId < imgSize.x * imgSize.y; locId++) {
				int y = locId / imgSize.x;
				int x = locId - y * imgSize.x;

				if (intrinsics->FocalLengthSignsDiffer()) {
					processPixelGrey_ImageNormals<true, true>(outRendering, pointsRay, imgSize, x, y,
					                                          volume->GetParameters().voxel_size, lightSource);
				} else {
					processPixelGrey_ImageNormals<true, false>(outRendering, pointsRay, imgSize, x, y,
					                                           volume->GetParameters().voxel_size, lightSource);
				}
			}
			break;
		case IVisualizationEngine::RENDER_SHADED_GREEN: {
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
			for (int locId = 0; locId < imgSize.x * imgSize.y; locId++) {
				Vector4f ptRay = pointsRay[locId];
				processPixelGreen<TVoxel, TIndex>(outRendering[locId], ptRay.toVector3(), ptRay.w > 0, voxelData,
				                                  voxelIndex, lightSource);
			}
			break;
		}
		case IVisualizationEngine::RENDER_SHADED_OVERLAY: {
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
			for (int locId = 0; locId < imgSize.x * imgSize.y; locId++) {
				Vector4f ptRay = pointsRay[locId];
				processPixelOverlay<TVoxel, TIndex>(outRendering[locId], ptRay.toVector3(), ptRay.w > 0, voxelData,
				                                    voxelIndex, lightSource);
			}
			break;
		}
		case IVisualizationEngine::RENDER_SHADED_GREYSCALE:
		default:
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
			for (int locId = 0; locId < imgSize.x * imgSize.y; locId++) {
				Vector4f ptRay = pointsRay[locId];
				processPixelGrey<TVoxel, TIndex>(outRendering[locId], ptRay.toVector3(), ptRay.w > 0, voxelData,
				                                 voxelIndex, lightSource);
			}
	}
}

template<class TVoxel, class TIndex>
static void
CreatePointCloud_common(VoxelVolume<TVoxel, TIndex>* volume, const View* view, CameraTrackingState* trackingState,
                        RenderState* renderState, bool skipPoints) {
	Vector2i imgSize = renderState->raycastResult->dimensions;
	Matrix4f invM = trackingState->pose_d->GetInvM() * view->calib.trafo_rgb_to_depth.calib;

	// this one is generally done for the colour tracker, so yes, update
	// the list of visible blocks if possible
	GenericRaycast(volume, imgSize, invM, view->calib.intrinsics_rgb.projectionParamsSimple.all, renderState, true);
	trackingState->pose_pointCloud->SetFrom(trackingState->pose_d);

	trackingState->pointCloud->noTotalPoints = RenderPointCloud<TVoxel, TIndex>(
			trackingState->pointCloud->locations->GetData(MEMORYDEVICE_CPU),
			trackingState->pointCloud->colours->GetData(MEMORYDEVICE_CPU),
			renderState->raycastResult->GetData(MEMORYDEVICE_CPU),
			volume->GetVoxels(),
			volume->index.GetIndexData(),
			skipPoints,
			volume->GetParameters().voxel_size,
			imgSize,
			-Vector3f(invM.getColumn(2))
	);
}

template<class TVoxel, class TIndex>
static void
CreateICPMaps_common(VoxelVolume<TVoxel, TIndex>* volume, const View* view, CameraTrackingState* trackingState,
                     RenderState* renderState) {
	const Vector2i imgSize = renderState->raycastResult->dimensions;
	Matrix4f invM = trackingState->pose_d->GetInvM();

	// this one is generally done for the ICP tracker, so yes, update
	// the list of visible blocks if possible
	GenericRaycast(volume, imgSize, invM, view->calib.intrinsics_d.projectionParamsSimple.all, renderState, true);
	trackingState->pose_pointCloud->SetFrom(trackingState->pose_d);

	const Vector3f lightSource = -Vector3f(invM.getColumn(2));
	Vector4f* normalsMap = trackingState->pointCloud->colours->GetData(MEMORYDEVICE_CPU);
	Vector4f* pointsMap = trackingState->pointCloud->locations->GetData(MEMORYDEVICE_CPU);
	Vector4f* pointsRay = renderState->raycastResult->GetData(MEMORYDEVICE_CPU);
	const float voxelSize = volume->GetParameters().voxel_size;

#ifdef WITH_OPENMP
#pragma omp parallel for default(none) shared(pointsMap, normalsMap, pointsRay, view)
#endif
	for (int y = 0; y < imgSize.y; y++)
		for (int x = 0; x < imgSize.x; x++) {
			if (view->calib.intrinsics_d.FocalLengthSignsDiffer()) {
				processPixelICP<true, true>(pointsMap, normalsMap, pointsRay, imgSize, x, y, voxelSize, lightSource);
			} else {
				processPixelICP<true, false>(pointsMap, normalsMap, pointsRay, imgSize, x, y, voxelSize, lightSource);
			}

		}
}

template<class TVoxel, class TIndex>
static void
ForwardRender_common(const VoxelVolume<TVoxel, TIndex>* volume, const View* view, CameraTrackingState* trackingState,
                     RenderState* renderState) {
	Vector2i imgSize = renderState->raycastResult->dimensions;
	Matrix4f M = trackingState->pose_d->GetM();
	Matrix4f invM = trackingState->pose_d->GetInvM();
	const Vector4f& projParams = view->calib.intrinsics_d.projectionParamsSimple.all;

	const Vector4f* pointsRay = renderState->raycastResult->GetData(MEMORYDEVICE_CPU);
	Vector4f* forwardProjection = renderState->forwardProjection->GetData(MEMORYDEVICE_CPU);
	float* currentDepth = view->depth->GetData(MEMORYDEVICE_CPU);
	int* fwdProjMissingPoints = renderState->fwdProjMissingPoints->GetData(MEMORYDEVICE_CPU);
	const Vector2f* minmaximg = renderState->renderingRangeImage->GetData(MEMORYDEVICE_CPU);
	float voxelSize = volume->GetParameters().voxel_size;
	const TVoxel* voxelData = volume->GetVoxels();
	const typename TIndex::IndexData* voxelIndex = volume->index.GetIndexData();

	renderState->forwardProjection->Clear();

	for (int y = 0; y < imgSize.y; y++)
		for (int x = 0; x < imgSize.x; x++) {
			int locId = x + y * imgSize.x;
			Vector4f pixel = pointsRay[locId];

			int locId_new = forwardProjectPixel(pixel * voxelSize, M, projParams, imgSize);
			if (locId_new >= 0) forwardProjection[locId_new] = pixel;
		}

	int noMissingPoints = 0;
	for (int y = 0; y < imgSize.y; y++)
		for (int x = 0; x < imgSize.x; x++) {
			int locId = x + y * imgSize.x;
			int locId2 = (int) floor((float) x / minmaximg_subsample) +
			             (int) floor((float) y / minmaximg_subsample) * imgSize.x;

			Vector4f fwdPoint = forwardProjection[locId];
			Vector2f minmaxval = minmaximg[locId2];
			float depth = currentDepth[locId];

			if ((fwdPoint.w <= 0) && ((fwdPoint.x == 0 && fwdPoint.y == 0 && fwdPoint.z == 0) || (depth >= 0)) &&
			    (minmaxval.x < minmaxval.y))
				//if ((fwdPoint.w <= 0) && (minmaxval.x < minmaxval.y))
			{
				fwdProjMissingPoints[noMissingPoints] = locId;
				noMissingPoints++;
			}
		}

	renderState->noFwdProjMissingPoints = noMissingPoints;
	const Vector4f invProjParams = InvertProjectionParams(projParams);

	for (int pointId = 0; pointId < noMissingPoints; pointId++) {
		int locId = fwdProjMissingPoints[pointId];
		int y = locId / imgSize.x, x = locId - y * imgSize.x;
		int locId2 =
				(int) floor((float) x / minmaximg_subsample) + (int) floor((float) y / minmaximg_subsample) * imgSize.x;

		castRay<TVoxel, TIndex, false>(forwardProjection[locId], NULL, x, y, voxelData, voxelIndex, invM, invProjParams,
		                               1.0f / volume->GetParameters().voxel_size,
		                               volume->GetParameters().narrow_band_half_width, minmaximg[locId2]);
	}
}

template<class TVoxel, class TIndex>
void
VisualizationEngine_CPU<TVoxel, TIndex>::RenderImage(VoxelVolume<TVoxel, TIndex>* scene, const ORUtils::SE3Pose* pose,
                                                     const Intrinsics* intrinsics,
                                                     const RenderState* renderState, UChar4Image* outputImage,
                                                     IVisualizationEngine::RenderImageType type,
                                                     IVisualizationEngine::RenderRaycastSelection raycastType) const {
	RenderImage_common(scene, pose, intrinsics, renderState, outputImage, type, raycastType);
}

template<class TVoxel>
void VisualizationEngine_CPU<TVoxel, VoxelBlockHash>::RenderImage(VoxelVolume<TVoxel, VoxelBlockHash>* scene,
                                                                  const ORUtils::SE3Pose* pose,
                                                                  const Intrinsics* intrinsics,
                                                                  const RenderState* renderState,
                                                                  UChar4Image* outputImage,
                                                                  IVisualizationEngine::RenderImageType type,
                                                                  IVisualizationEngine::RenderRaycastSelection raycastType) const {
	RenderImage_common(scene, pose, intrinsics, renderState, outputImage, type, raycastType);
}

template<class TVoxel, class TIndex>
void
VisualizationEngine_CPU<TVoxel, TIndex>::FindSurface(VoxelVolume<TVoxel, TIndex>* scene, const ORUtils::SE3Pose* pose,
                                                     const Intrinsics* intrinsics,
                                                     const RenderState* renderState) const {
	// this one is generally done for freeview Visualization, so no, do not
	// update the list of visible blocks
	GenericRaycast(scene, renderState->raycastResult->dimensions, pose->GetInvM(),
	               intrinsics->projectionParamsSimple.all, renderState, false);
}

template<class TVoxel>
void VisualizationEngine_CPU<TVoxel, VoxelBlockHash>::FindSurface(VoxelVolume<TVoxel, VoxelBlockHash>* scene,
                                                                  const ORUtils::SE3Pose* pose,
                                                                  const Intrinsics* intrinsics,
                                                                  const RenderState* renderState) const {
	// this one is generally done for freeview Visualization, so no, do not
	// update the list of visible blocks
	GenericRaycast(scene, renderState->raycastResult->dimensions, pose->GetInvM(),
	               intrinsics->projectionParamsSimple.all, renderState, false);
}

template<class TVoxel, class TIndex>
void VisualizationEngine_CPU<TVoxel, TIndex>::CreatePointCloud(VoxelVolume<TVoxel, TIndex>* scene, const View* view,
                                                               CameraTrackingState* trackingState,
                                                               RenderState* renderState, bool skipPoints) const {
	CreatePointCloud_common(scene, view, trackingState, renderState, skipPoints);
}

template<class TVoxel>
void VisualizationEngine_CPU<TVoxel, VoxelBlockHash>::CreatePointCloud(VoxelVolume<TVoxel, VoxelBlockHash>* scene,
                                                                       const View* view,
                                                                       CameraTrackingState* trackingState,
                                                                       RenderState* renderState,
                                                                       bool skipPoints) const {
	CreatePointCloud_common(scene, view, trackingState, renderState, skipPoints);
}

template<class TVoxel, class TIndex>
void VisualizationEngine_CPU<TVoxel, TIndex>::CreateICPMaps(VoxelVolume<TVoxel, TIndex>* scene, const View* view,
                                                            CameraTrackingState* trackingState,
                                                            RenderState* renderState) const {
	CreateICPMaps_common(scene, view, trackingState, renderState);
}

template<class TVoxel>
void VisualizationEngine_CPU<TVoxel, VoxelBlockHash>::CreateICPMaps(VoxelVolume<TVoxel, VoxelBlockHash>* scene,
                                                                    const View* view,
                                                                    CameraTrackingState* trackingState,
                                                                    RenderState* renderState) const {
	CreateICPMaps_common(scene, view, trackingState, renderState);
}

template<class TVoxel, class TIndex>
void VisualizationEngine_CPU<TVoxel, TIndex>::ForwardRender(const VoxelVolume<TVoxel, TIndex>* scene, const View* view,
                                                            CameraTrackingState* trackingState,
                                                            RenderState* renderState) const {
	ForwardRender_common(scene, view, trackingState, renderState);
}

template<class TVoxel>
void VisualizationEngine_CPU<TVoxel, VoxelBlockHash>::ForwardRender(const VoxelVolume<TVoxel, VoxelBlockHash>* scene,
                                                                    const View* view,
                                                                    CameraTrackingState* trackingState,
                                                                    RenderState* renderState) const {
	ForwardRender_common(scene, view, trackingState, renderState);
}

template<class TVoxel, class TIndex>
static int RenderPointCloud(Vector4f* locations, Vector4f* colours, const Vector4f* ptsRay,
                            const TVoxel* voxelData, const typename TIndex::IndexData* voxelIndex, bool skipPoints,
                            float voxelSize,
                            Vector2i imgSize, Vector3f lightSource) {
	int noTotalPoints = 0;

	for (int y = 0, locId = 0; y < imgSize.y; y++)
		for (int x = 0; x < imgSize.x; x++, locId++) {
			Vector3f outNormal;
			float angle;
			Vector4f pointRay = ptsRay[locId];
			Vector3f point = pointRay.toVector3();
			bool foundPoint = pointRay.w > 0;

			computeNormalAndAngle<TVoxel, TIndex>(foundPoint, point, voxelData, voxelIndex, lightSource, outNormal,
			                                      angle);

			if (skipPoints && ((x % 2 == 0) || (y % 2 == 0))) foundPoint = false;

			if (foundPoint) {
				Vector4f tmp;
				tmp = VoxelColorReader<TVoxel::hasColorInformation, TVoxel, TIndex>::interpolate(voxelData, voxelIndex,
				                                                                                 point);
				if (tmp.w > 0.0f) {
					tmp.x /= tmp.w;
					tmp.y /= tmp.w;
					tmp.z /= tmp.w;
					tmp.w = 1.0f;
				}
				colours[noTotalPoints] = tmp;

				Vector4f pt_ray_out;
				pt_ray_out.x = point.x * voxelSize;
				pt_ray_out.y = point.y * voxelSize;
				pt_ray_out.z = point.z * voxelSize;
				pt_ray_out.w = 1.0f;
				locations[noTotalPoints] = pt_ray_out;

				noTotalPoints++;
			}
		}

	return noTotalPoints;
}
