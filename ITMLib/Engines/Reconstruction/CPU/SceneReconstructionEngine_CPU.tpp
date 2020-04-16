// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

//local
#include "SceneReconstructionEngine_CPU.h"
#include "../Shared/SceneReconstructionEngine_Shared.h"
#include "../../Common/CheckBlockVisibility.h"
#include "../../../Utils/Configuration.h"

using namespace ITMLib;
template<class TVoxel>
void SceneReconstructionEngine_CPU<TVoxel,VoxelBlockHash>::ResetScene(VoxelVolume<TVoxel, VoxelBlockHash>* volume){
	volume->Reset();
}

template<class TVoxel>
void SceneReconstructionEngine_CPU<TVoxel, VoxelBlockHash>::IntegrateIntoScene(VoxelVolume<TVoxel, VoxelBlockHash>* volume, const View* view,
                                                                               const CameraTrackingState* trackingState, const RenderState* renderState)
{
	Vector2i rgbImgSize = view->rgb->dimensions;
	Vector2i depthImgSize = view->depth->dimensions;
	float voxelSize = volume->parameters->voxel_size;

	Matrix4f M_d, M_rgb;
	Vector4f projParams_d, projParams_rgb;

	M_d = trackingState->pose_d->GetM();
	if (TVoxel::hasColorInformation) M_rgb = view->calib.trafo_rgb_to_depth.calib_inv * M_d;

	projParams_d = view->calib.intrinsics_d.projectionParamsSimple.all;
	projParams_rgb = view->calib.intrinsics_rgb.projectionParamsSimple.all;

	float mu = volume->parameters->narrow_band_half_width; int maxW = volume->parameters->max_integration_weight;

	float *depth = view->depth->GetData(MEMORYDEVICE_CPU);
	float *confidence = view->depthConfidence->GetData(MEMORYDEVICE_CPU);
	Vector4u *rgb = view->rgb->GetData(MEMORYDEVICE_CPU);
	TVoxel *localVBA = volume->GetVoxelBlocks();
	HashEntry *hashTable = volume->index.GetEntries();

	int *visibleBlockHashCodes = volume->index.GetUtilizedBlockHashCodes();
	int visibleHashBlockCount = volume->index.GetUtilizedBlockCount();

	bool stopIntegratingAtMaxW = volume->parameters->stop_integration_at_max_weight;

#ifdef WITH_OPENMP
	#pragma omp parallel for default(shared)
#endif
	for (int visibleHash = 0; visibleHash < visibleHashBlockCount; visibleHash++)
	{
		Vector3i globalPos;
		int hash = visibleBlockHashCodes[visibleHash];

		const HashEntry &currentHashEntry = hashTable[hash];

		if (currentHashEntry.ptr < 0) continue;

		globalPos.x = currentHashEntry.pos.x;
		globalPos.y = currentHashEntry.pos.y;
		globalPos.z = currentHashEntry.pos.z;
		globalPos *= VOXEL_BLOCK_SIZE;

		TVoxel *localVoxelBlock = &(localVBA[currentHashEntry.ptr * (VOXEL_BLOCK_SIZE3)]);

		for (int z = 0; z < VOXEL_BLOCK_SIZE; z++) for (int y = 0; y < VOXEL_BLOCK_SIZE; y++) for (int x = 0; x < VOXEL_BLOCK_SIZE; x++)
		{
			Vector4f pt_model; int locId;

			locId = x + y * VOXEL_BLOCK_SIZE + z * VOXEL_BLOCK_SIZE * VOXEL_BLOCK_SIZE;

			if (stopIntegratingAtMaxW) if (localVoxelBlock[locId].w_depth == maxW) continue;

			pt_model.x = (float)(globalPos.x + x) * voxelSize;
			pt_model.y = (float)(globalPos.y + y) * voxelSize;
			pt_model.z = (float)(globalPos.z + z) * voxelSize;
			pt_model.w = 1.0f;

			ComputeUpdatedVoxelInfo<
					TVoxel::hasColorInformation,
					TVoxel::hasConfidenceInformation,
					TVoxel::hasSemanticInformation,
					TVoxel>::compute(localVoxelBlock[locId], pt_model, M_d,
				projParams_d, M_rgb, projParams_rgb, mu, maxW, depth, confidence, depthImgSize, rgb, rgbImgSize);
		}
	}
}

template<class TVoxel>
void SceneReconstructionEngine_CPU<TVoxel, VoxelBlockHash>::AllocateSceneFromDepth(VoxelVolume<TVoxel, VoxelBlockHash> *scene, const View *view,
                                                                                   const CameraTrackingState *trackingState, const RenderState *renderState, bool onlyUpdateVisibleList, bool resetVisibleList)
{
	Vector2i depthImgSize = view->depth->dimensions;
	float voxelSize = scene->parameters->voxel_size;

	Matrix4f M_d, invM_d;
	Vector4f projParams_d, invProjParams_d;

	if (resetVisibleList) scene->index.SetUtilizedBlockCount(0);

	M_d = trackingState->pose_d->GetM(); M_d.inv(invM_d);

	projParams_d = view->calib.intrinsics_d.projectionParamsSimple.all;
	invProjParams_d = projParams_d;
	invProjParams_d.x = 1.0f / invProjParams_d.x;
	invProjParams_d.y = 1.0f / invProjParams_d.y;

	float mu = scene->parameters->narrow_band_half_width;

	float *depth = view->depth->GetData(MEMORYDEVICE_CPU);
	int *voxelAllocationList = scene->index.GetBlockAllocationList();
	int *excessAllocationList = scene->index.GetExcessEntryList();
	HashEntry *hashTable = scene->index.GetEntries();
	ITMHashSwapState *swapStates = scene->Swapping() ? scene->global_cache->GetSwapStates(false) : 0;
	int* visibleBlockHashCodes = scene->index.GetUtilizedBlockHashCodes();
	HashBlockVisibility* hashBlockVisibilityTypes = scene->index.GetBlockVisibilityTypes();

	int hashEntryCount = scene->index.hash_entry_count;
	HashEntryAllocationState* hashEntryStates_device = scene->index.GetHashEntryAllocationStates();
	Vector3s* blockCoords_device = scene->index.GetAllocationBlockCoordinates();

	bool useSwapping = scene->Swapping();

	float oneOverHashEntrySize = 1.0f / (voxelSize * VOXEL_BLOCK_SIZE);//m
	float band_factor = configuration::get().general_voxel_volume_parameters.block_allocation_band_factor;
	float surface_cutoff_distance = mu * band_factor;

	int lastFreeVoxelBlockId = scene->index.GetLastFreeBlockListId();
	int lastFreeExcessListId = scene->index.GetLastFreeExcessListId();

	int visibleHashBlockCount = 0;

	scene->index.ClearHashEntryAllocationStates();

	for (int i = 0; i < scene->index.GetUtilizedBlockCount(); i++)
		hashBlockVisibilityTypes[visibleBlockHashCodes[i]] = VISIBLE_AT_PREVIOUS_FRAME_AND_UNSTREAMED;

	//build hashVisibility
#ifdef WITH_OPENMP
	#pragma omp parallel for default(shared)
#endif
	for (int locId = 0; locId < depthImgSize.x*depthImgSize.y; locId++)
	{
		int y = locId / depthImgSize.x;
		int x = locId - y * depthImgSize.x;
		bool collisionDetected = false;
		findVoxelBlocksForRayNearSurfaceLegacy_Algomorph(hashEntryStates_device,
		                                                 blockCoords_device, hashBlockVisibilityTypes,
		                                                 hashTable, x, y,
		                                                 depth, surface_cutoff_distance, invM_d,
		                                                 invProjParams_d,
		                                                 oneOverHashEntrySize, depthImgSize, scene->parameters->near_clipping_distance,
		                                                 scene->parameters->far_clipping_distance, collisionDetected);
	}

	if (onlyUpdateVisibleList) useSwapping = false;
	if (!onlyUpdateVisibleList)
	{
		//allocate
		for (int targetIdx = 0; targetIdx < hashEntryCount; targetIdx++)
		{
			int vbaIdx, exlIdx;
			unsigned char hashChangeType = hashEntryStates_device[targetIdx];

			switch (hashChangeType)
			{
			case 1: //needs allocation, fits in the ordered list
				vbaIdx = lastFreeVoxelBlockId; lastFreeVoxelBlockId--;

				if (vbaIdx >= 0) //there is room in the voxel block array
				{
					HashEntry hashEntry;
					hashEntry.pos = blockCoords_device[targetIdx];
					hashEntry.ptr = voxelAllocationList[vbaIdx];
					hashEntry.offset = 0;

					hashTable[targetIdx] = hashEntry;
				}
				else
				{
					// Mark entry as not visible since we couldn't allocate it but findVoxelBlocksForRayNearSurface changed its state.
					hashBlockVisibilityTypes[targetIdx] = INVISIBLE;

					// Restore previous value to avoid leaks.
					lastFreeVoxelBlockId++;
				}

				break;
			case 2: //needs allocation in the excess list
				vbaIdx = lastFreeVoxelBlockId; lastFreeVoxelBlockId--;
				exlIdx = lastFreeExcessListId; lastFreeExcessListId--;

				if (vbaIdx >= 0 && exlIdx >= 0) //there is room in the voxel block array and excess list
				{
					HashEntry hashEntry;
					hashEntry.pos = blockCoords_device[targetIdx];
					hashEntry.ptr = voxelAllocationList[vbaIdx];
					hashEntry.offset = 0;

					int exlOffset = excessAllocationList[exlIdx];

					hashTable[targetIdx].offset = exlOffset + 1; //connect to child

					hashTable[ORDERED_LIST_SIZE + exlOffset] = hashEntry; //add child to the excess list

					hashBlockVisibilityTypes[ORDERED_LIST_SIZE + exlOffset] = IN_MEMORY_AND_VISIBLE;
				}
				else
				{
					// No need to mark the entry as not visible since findVoxelBlocksForRayNearSurface did not mark it.
					// Restore previous value to avoid leaks.
					lastFreeVoxelBlockId++;
					lastFreeExcessListId++;
				}

				break;
			}
		}
	}

	//build visible list
	for (int targetIdx = 0; targetIdx < hashEntryCount; targetIdx++)
	{
		HashBlockVisibility hashVisibleType = hashBlockVisibilityTypes[targetIdx];
		const HashEntry &hashEntry = hashTable[targetIdx];

		if (hashVisibleType == 3)
		{
			bool isVisibleEnlarged, isVisible;

			if (useSwapping)
			{
				checkBlockVisibility<true>(isVisible, isVisibleEnlarged, hashEntry.pos, M_d, projParams_d, voxelSize, depthImgSize);
				if (!isVisibleEnlarged) hashVisibleType = INVISIBLE;
			} else {
				checkBlockVisibility<false>(isVisible, isVisibleEnlarged, hashEntry.pos, M_d, projParams_d, voxelSize, depthImgSize);
				if (!isVisible) { hashVisibleType = INVISIBLE; }
			}
			hashBlockVisibilityTypes[targetIdx] = hashVisibleType;
		}

		if (useSwapping)
		{
			if (hashVisibleType > 0 && swapStates[targetIdx].state != 2) swapStates[targetIdx].state = 1;
		}

		if (hashVisibleType > 0)
		{
			visibleBlockHashCodes[visibleHashBlockCount] = targetIdx;
			visibleHashBlockCount++;
		}

	}

	//reallocate deleted ones from previous swap operation
	if (useSwapping)
	{
		for (int targetIdx = 0; targetIdx < hashEntryCount; targetIdx++)
		{
			int vbaIdx;
			HashEntry hashEntry = hashTable[targetIdx];

			if (hashBlockVisibilityTypes[targetIdx] > 0 && hashEntry.ptr == -1)
			{
				vbaIdx = lastFreeVoxelBlockId; lastFreeVoxelBlockId--;
				if (vbaIdx >= 0) hashTable[targetIdx].ptr = voxelAllocationList[vbaIdx];
				else lastFreeVoxelBlockId++; // Avoid leaks
			}
		}
	}

	scene->index.SetUtilizedBlockCount(visibleHashBlockCount);
	scene->index.SetLastFreeBlockListId(lastFreeVoxelBlockId);
	scene->index.SetLastFreeExcessListId(lastFreeExcessListId);
}

template<class TVoxel>
SceneReconstructionEngine_CPU<TVoxel,PlainVoxelArray>::SceneReconstructionEngine_CPU(void)
{}

template<class TVoxel>
SceneReconstructionEngine_CPU<TVoxel,PlainVoxelArray>::~SceneReconstructionEngine_CPU(void)
{}

template<class TVoxel>
void SceneReconstructionEngine_CPU<TVoxel,PlainVoxelArray>::ResetScene(VoxelVolume<TVoxel, PlainVoxelArray> *scene)
{
	scene->Reset();
}

template<class TVoxel>
void SceneReconstructionEngine_CPU<TVoxel, PlainVoxelArray>::AllocateSceneFromDepth(VoxelVolume<TVoxel, PlainVoxelArray> *scene, const View *view,
                                                                                    const CameraTrackingState *trackingState, const RenderState *renderState, bool onlyUpdateVisibleList, bool resetVisibleList)
{}

template<class TVoxel>
void SceneReconstructionEngine_CPU<TVoxel, PlainVoxelArray>::IntegrateIntoScene(VoxelVolume<TVoxel, PlainVoxelArray>* scene, const View *view,
                                                                                const CameraTrackingState *trackingState, const RenderState *renderState)
{
	const Vector2i rgbImgSize = view->rgb->dimensions;
	const Vector2i depthImgSize = view->depth->dimensions;
	const float voxelSize = scene->parameters->voxel_size;

	const Matrix4f M_d = trackingState->pose_d->GetM();
	const Matrix4f M_rgb = TVoxel::hasColorInformation ? view->calib.trafo_rgb_to_depth.calib_inv * M_d : Matrix4f();

	const Vector4f projParams_d = view->calib.intrinsics_d.projectionParamsSimple.all;
	const Vector4f projParams_rgb = view->calib.intrinsics_rgb.projectionParamsSimple.all;

	const float mu = scene->parameters->narrow_band_half_width;
	const int maxW = scene->parameters->max_integration_weight;

	float *depth = view->depth->GetData(MEMORYDEVICE_CPU);
	float *confidence = view->depthConfidence->GetData(MEMORYDEVICE_CPU);
	Vector4u *rgb = view->rgb->GetData(MEMORYDEVICE_CPU);
	TVoxel *voxelArray = scene->GetVoxelBlocks();

	const PlainVoxelArray::IndexData *arrayInfo = scene->index.GetIndexData();

	const bool stopIntegratingAtMaxW = scene->parameters->stop_integration_at_max_weight;
	//bool approximateIntegration = !trackingState->requiresFullRendering;

#ifdef WITH_OPENMP
	#pragma omp parallel for default(none) shared(scene, voxelArray, arrayInfo, rgb, depth, confidence)
#endif
	for (int locId = 0; locId < scene->index.GetVolumeSize().x * scene->index.GetVolumeSize().y *
	                            scene->index.GetVolumeSize().z; ++locId)
	{
		int z = locId / (scene->index.GetVolumeSize().x * scene->index.GetVolumeSize().y);
		int tmp = locId - z * scene->index.GetVolumeSize().x * scene->index.GetVolumeSize().y;
		int y = tmp / scene->index.GetVolumeSize().x;
		int x = tmp - y * scene->index.GetVolumeSize().x;

		Vector4f pt_model;

		if (stopIntegratingAtMaxW) if (voxelArray[locId].w_depth == maxW) continue;
		//if (approximateIntegration) if (voxelArray[locId].w_depth != 0) continue;

		pt_model.x = (float)(x + arrayInfo->offset.x) * voxelSize;
		pt_model.y = (float)(y + arrayInfo->offset.y) * voxelSize;
		pt_model.z = (float)(z + arrayInfo->offset.z) * voxelSize;
		pt_model.w = 1.0f;

		ComputeUpdatedVoxelInfo<
				TVoxel::hasColorInformation,
				TVoxel::hasConfidenceInformation,
				TVoxel::hasSemanticInformation,
				TVoxel>::compute(voxelArray[locId], pt_model, M_d,
		                         projParams_d, M_rgb, projParams_rgb, mu, maxW, depth, confidence, depthImgSize, rgb, rgbImgSize);
	}
}
