#############################
# CMakeLists.txt for ITMLib #
#############################

###########################
# Specify the target name #
###########################

set(targetname ITMLib)

################################
# Specify the libraries to use #
################################

include(${PROJECT_SOURCE_DIR}/cmake/UseCUDA.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/UseOpenMP.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/UseVTK.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/UseOpenCV.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/UseBoost.cmake)

#############################
# Specify the project files #
#############################

##
set(ITMLIB_TOPLEVEL_CPU_SOURCES
    ## VBH - CPU
    Instantiations/VoxelBlockHash/CPUInstantiations.cpp
    ## PVA - CPU
    Instantiations/PlainVoxelArray/CPUInstantiations.cpp
    )

set(ITMLIB_TOPLEVEL_CUDA_SOURCES
    ## VBH - CUDA
    Instantiations/VoxelBlockHash/CUDAInstantiations.cu
    Instantiations/VoxelBlockHash/SurfelCUDAInstantiations.cu
    ## PVA - CUDA
    Instantiations/PlainVoxelArray/CUDAInstantiations.cu
    )

set(ITMLIB_TOPLEVEL_HEADERS
    ITMLibDefines.h
    )

##
set(ITMLIB_CORE_SOURCES
    Engines/Main/ITMBasicEngine.tpp
    Engines/Main/ITMBasicSurfelEngine.tpp
    Engines/Main/ITMDenseMapper.tpp
    Engines/Main/ITMDenseSurfelMapper.tpp
    Engines/Main/DenseDynamicMapper.tpp
    Engines/Main/DenseDynamicMapper_PlainVoxelArray.cpp
    Engines/Main/DenseDynamicMapper_VoxelBlockHash.cpp
    Engines/Main/NonRigidTrackingParameters.h
    Engines/Main/ITMMultiEngine.tpp
    Engines/Main/ITMDynamicEngine.tpp
    Engines/Main/ITMDynamicEngine_VoxelBlockHash.cpp
    Engines/Main/ITMDynamicEngine_PlainVoxelArray.cpp
    )

set(ITMLIB_CORE_HEADERS
    Engines/Main/ITMBasicEngine.h
    Engines/Main/ITMBasicSurfelEngine.h
    Engines/Main/ITMDenseMapper.h
    Engines/Main/ITMDenseSurfelMapper.h
    Engines/Main/DenseDynamicMapper.h
    Engines/Main/ITMMainEngine.h
    Engines/Main/ITMMultiEngine.h
    Engines/Main/ITMTrackingController.h
    Engines/Main/ITMDynamicEngine.h
    Engines/Main/MianEngineFactory.h
    )

#======================================= COMMON TO MULTIPLE ENGINES ====================================================
set(ITMLIB_ENGINES_COMMON_HEADERS
    Engines/Common/ITMCommonFunctors.h
    Engines/Common/ITMWarpEnums.h
    )
#======================================= LOW-LEVEL ENGINES =============================================================
##
set(ITMLIB_ENGINES_LOWLEVEL_SOURCES
    Engines/LowLevel/ITMLowLevelEngineFactory.cpp
    )

set(ITMLIB_ENGINES_LOWLEVEL_HEADERS
    Engines/LowLevel/ITMLowLevelEngineFactory.h
    )

##
set(ITMLIB_ENGINES_LOWLEVEL_CPU_SOURCES
    Engines/LowLevel/CPU/ITMLowLevelEngine_CPU.cpp
    )

set(ITMLIB_ENGINES_LOWLEVEL_CPU_HEADERS
    Engines/LowLevel/CPU/ITMLowLevelEngine_CPU.h
    )

##
set(ITMLIB_ENGINES_LOWLEVEL_CUDA_SOURCES
    Engines/LowLevel/CUDA/ITMLowLevelEngine_CUDA.cu
    )

set(ITMLIB_ENGINES_LOWLEVEL_CUDA_HEADERS
    Engines/LowLevel/CUDA/ITMLowLevelEngine_CUDA.h
    )

##
set(ITMLIB_ENGINES_LOWLEVEL_INTERFACE_HEADERS
    Engines/LowLevel/Interface/ITMLowLevelEngine.h
    )

##
set(ITMLIB_ENGINES_LOWLEVEL_SHARED_HEADERS
    Engines/LowLevel/Shared/ITMLowLevelEngine_Shared.h
    )

#====================================== SCENE TRAVERSAL ENGINES ======================================================

##
set(ITMLIB_ENGINES_TRAVERSAL_INTERFACE_HEADERS
    Engines/Traversal/Interface/ITMSceneTraversal.h
    )
##
set(ITMLIB_ENGINES_TRAVERSAL_SHARED_HEADERS
    Engines/Traversal/Shared/ITMSceneTraversal_Shared.h
    )

##
set(ITMLIB_ENGINES_TRAVERSAL_CPU_SOURCES
    )

set(ITMLIB_ENGINES_TRAVERSAL_CPU_HEADERS
    Engines/Traversal/CPU/ITMSceneTraversal_CPU_VoxelBlockHash.h
    Engines/Traversal/CPU/ITMSceneTraversal_CPU_PlainVoxelArray.h
    Engines/Traversal/CPU/ITMSceneTraversal_CPU_PVA_to_VBH.h
    )

##
set(ITMLIB_ENGINES_TRAVERSAL_CUDA_SOURCES
    )

set(ITMLIB_ENGINES_TRAVERSAL_CUDA_HEADERS
    Engines/Traversal/CUDA/ITMSceneTraversal_CUDA_VoxelBlockHash.h
    Engines/Traversal/CUDA/ITMSceneTraversal_CUDA_VoxelBlockHash_Kernels.h
    Engines/Traversal/CUDA/ITMSceneTraversal_CUDA_PlainVoxelArray.h
    Engines/Traversal/CUDA/ITMSceneTraversal_CUDA_PlainVoxelArray_Kernels.h
    Engines/Traversal/CUDA/ITMSceneTraversal_CUDA_PVA_to_VBH.h
    Engines/Traversal/CUDA/ITMSceneTraversal_CUDA_PVA_to_VBH_Kernels.h
    )


#====================================== SCENE MANIPULATION ENGINES =====================================================
set(ITMLIB_ENGINES_MANIPULATION_HEADERS
    Engines/EditAndCopy/EditAndCopyEngineFactory.h
    )

##
set(ITMLIB_ENGINES_MANIPULATION_INTERFACE_HEADERS
    Engines/EditAndCopy/Interface/EditAndCopyEngineInterface.h
    )
##
set(ITMLIB_ENGINES_MANIPULATION_SHARED_HEADERS
    Engines/EditAndCopy/Shared/EditAndCopyEngine_Shared.h
    Engines/EditAndCopy/Shared/EditAndCopyEngine_Functors.h
    )

##
set(ITMLIB_ENGINES_MANIPULATION_CPU_SOURCES
    Engines/EditAndCopy/CPU/EditAndCopyEngine_CPU.cpp
    Engines/EditAndCopy/CPU/EditAndCopyEngine_CPU_VoxelBlockHash.tpp
    Engines/EditAndCopy/CPU/EditAndCopyEngine_CPU_PlainVoxelArray.tpp
    Engines/EditAndCopy/CPU/EditAndCopyEngine_CPU_VoxelBlockHash.cpp
    Engines/EditAndCopy/CPU/EditAndCopyEngine_CPU_PlainVoxelArray.cpp
    )

set(ITMLIB_ENGINES_MANIPULATION_CPU_HEADERS
    Engines/EditAndCopy/CPU/EditAndCopyEngine_CPU.h
    )

##
set(ITMLIB_ENGINES_MANIPULATION_CUDA_SOURCES
    Engines/EditAndCopy/CUDA/EditAndCopyEngine_CUDA_VoxelBlockHash.tcu
    Engines/EditAndCopy/CUDA/EditAndCopyEngine_CUDA_PlainVoxelArray.tcu
    Engines/EditAndCopy/CUDA/EditAndCopyEngine_CUDA_VoxelBlockHash.cu
    Engines/EditAndCopy/CUDA/EditAndCopyEngine_CUDA_PlainVoxelArray.cu
    )

set(ITMLIB_ENGINES_MANIPULATION_CUDA_HEADERS
    Engines/EditAndCopy/CUDA/EditAndCopyEngine_CUDA.h
    Engines/EditAndCopy/CUDA/EditAndCopyEngine_CUDA_VoxelBlockHash_Kernels.h
    Engines/EditAndCopy/CUDA/EditAndCopyEngine_CUDA_PlainVoxelArray_Kernels.h
    )

#====================================== MESHING ENGINES ================================================================
##
set(ITMLIB_ENGINES_MESHING_HEADERS
    Engines/Meshing/ITMMeshingEngineFactory.h
    Engines/Meshing/ITMMultiMeshingEngineFactory.h
    )

##
set(ITMLIB_ENGINES_MESHING_CPU_SOURCES
    Engines/Meshing/CPU/ITMMeshingEngine_CPU.tpp
    Engines/Meshing/CPU/ITMMultiMeshingEngine_CPU.tpp
    )

set(ITMLIB_ENGINES_MESHING_CPU_HEADERS
    Engines/Meshing/CPU/ITMMeshingEngine_CPU.h
    Engines/Meshing/CPU/ITMMultiMeshingEngine_CPU.h
    )

##
set(ITMLIB_ENGINES_MESHING_CUDA_SOURCES
    Engines/Meshing/CUDA/ITMMeshingEngine_CUDA.tcu
    Engines/Meshing/CUDA/ITMMultiMeshingEngine_CUDA.tcu
    )

set(ITMLIB_ENGINES_MESHING_CUDA_HEADERS
    Engines/Meshing/CUDA/ITMMeshingEngine_CUDA.h
    Engines/Meshing/CUDA/ITMMultiMeshingEngine_CUDA.h
    )

##
set(ITMLIB_ENGINES_MESHING_INTERFACE_HEADERS
    Engines/Meshing/Interface/ITMMeshingEngine.h
    Engines/Meshing/Interface/ITMMultiMeshingEngine.h
    )

##
set(ITMLIB_ENGINES_MESHING_SHARED_HEADERS
    Engines/Meshing/Shared/ITMMeshingEngine_Shared.h
    Engines/Meshing/Shared/ITMMultiMeshingEngine_Shared.h
    )

##
set(ITMLIB_ENGINES_MULTISCENE_SOURCES
    Engines/MultiScene/ITMActiveMapManager.cpp
    Engines/MultiScene/ITMGlobalAdjustmentEngine.cpp
    Engines/MultiScene/ITMMapGraphManager.tpp
    )

set(ITMLIB_ENGINES_MULTISCENE_HEADERS
    Engines/MultiScene/ITMActiveMapManager.h
    Engines/MultiScene/ITMGlobalAdjustmentEngine.h
    Engines/MultiScene/ITMMapGraphManager.h
    )
#====================================== INDEXING ENGINES ===============================================================
set(ITMLIB_ENGINES_INDEXING_SOURCES
    Engines/Indexing/VBH/IndexingEngine_VoxelBlockHash.tpp
    )

set(ITMLIB_ENGINES_INDEXING_HEADERS
    Engines/Indexing/VBH/IndexingEngine_VoxelBlockHash.h
    )

##
set(ITMLIB_ENGINES_INDEXING_CPU_SOURCES
    Engines/Indexing/VBH/CPU/IndexingEngine_CPU_VoxelBlockHash.tpp
    Engines/Indexing/VBH/CPU/IndexingEngine_CPU_VoxelBlockHash.cpp
    Engines/Indexing/PVA/CPU/IndexingEngine_CPU_PlainVoxelArray.cpp
    )

set(ITMLIB_ENGINES_INDEXING_CPU_HEADERS
    Engines/Indexing/VBH/CPU/IndexingEngine_CPU_VoxelBlockHash.h
    )
##
set(ITMLIB_ENGINES_INDEXING_CUDA_SOURCES
    Engines/Indexing/VBH/CUDA/IndexingEngine_CUDA_VoxelBlockHash.tcu
    Engines/Indexing/VBH/CUDA/IndexingEngine_CUDA_VoxelBlockHash.cu
    Engines/Indexing/PVA/CUDA/IndexingEngine_CUDA_PlainVoxelArray.cu
    )

set(ITMLIB_ENGINES_INDEXING_CUDA_HEADERS
    Engines/Indexing/VBH/CUDA/IndexingEngine_CUDA_VoxelBlockHash_Kernels.h
    )

##
set(ITMLIB_ENGINES_INDEXING_INTERFACE_HEADERS
    Engines/Indexing/Interface/IndexingEngine.h

    )
set(ITMLIB_ENGINES_INDEXING_INTERFACE_SOURCES
    Engines/Indexing/Interface/IndexingEngine.tpp
    )

set(ITMLIB_ENGINES_INDEXING_SHARED_HEADERS
    Engines/Indexing/Shared/IndexingEngine_Shared.h
    )


#====================================== RECONSTRUCTION (SDF GENERATION) ENGINES ========================================
##
set(ITMLIB_ENGINES_RECONSTRUCTION_SOURCES
    Engines/Reconstruction/SurfelSceneReconstructionEngineFactory.tpp
    )

set(ITMLIB_ENGINES_RECONSTRUCTION_HEADERS
    Engines/Reconstruction/SceneReconstructionEngineFactory.h
    Engines/Reconstruction/SurfelSceneReconstructionEngineFactory.h
    Engines/DepthFusion/DepthFusionEngineFactory.h
    )

##
set(ITMLIB_ENGINES_RECONSTRUCTION_CPU_SOURCES
    Engines/Reconstruction/CPU/SceneReconstructionEngine_CPU.tpp
    Engines/DepthFusion/DepthFusionEngine_CPU_VoxelBlockHash.tpp
    Engines/DepthFusion/DepthFusionEngine_CPU_VoxelBlockHash.cpp
    Engines/DepthFusion/DepthFusionEngine_CPU_PlainVoxelArray.tpp
    Engines/DepthFusion/DepthFusionEngine_CPU_PlainVoxelArray.cpp
    Engines/Reconstruction/CPU/SurfelSceneReconstructionEngine_CPU.tpp
    )

set(ITMLIB_ENGINES_RECONSTRUCTION_CPU_HEADERS
    Engines/Reconstruction/CPU/SceneReconstructionEngine_CPU.h
    Engines/DepthFusion/DepthFusionEngine_CPU.h
    Engines/Reconstruction/CPU/SurfelSceneReconstructionEngine_CPU.h
    )

##
set(ITMLIB_ENGINES_RECONSTRUCTION_CUDA_SOURCES
    Engines/Reconstruction/CUDA/SceneReconstructionEngine_CUDA.tcu
    Engines/Reconstruction/CUDA/SurfelSceneReconstructionEngine_CUDA.tcu
    Engines/DepthFusion/DepthFusionEngine_CUDA_PlainVoxelArray.tcu
    Engines/DepthFusion/DepthFusionEngine_CUDA_PlainVoxelArray.cu
    Engines/DepthFusion/DepthFusionEngine_CUDA_VoxelBlockHash.tcu
    Engines/DepthFusion/DepthFusionEngine_CUDA_VoxelBlockHash.cu
    )

set(ITMLIB_ENGINES_RECONSTRUCTION_CUDA_HEADERS
    Engines/Reconstruction/CUDA/SceneReconstructionEngine_CUDA.h
    Engines/Reconstruction/CUDA/SurfelSceneReconstructionEngine_CUDA.h
    Engines/DepthFusion/DepthFusionEngine_CUDA.h
    Engines/DepthFusion/DepthFusionEngine_CUDA_VoxelBlockHash_Kernels.h
    Engines/DepthFusion/DepthFusionEngine_CUDA_PlainVoxelArray_Kernels.h
    )

##
set(ITMLIB_ENGINES_RECONSTRUCTION_INTERFACE_SOURCES
    Engines/Reconstruction/Interface/SurfelSceneReconstructionEngine.tpp
    )

set(ITMLIB_ENGINES_RECONSTRUCTION_INTERFACE_HEADERS

    Engines/Reconstruction/Interface/SceneReconstructionEngine.h
    Engines/Reconstruction/Interface/SurfelSceneReconstructionEngine.h
    Engines/DepthFusion/DepthFusionEngine.h
    )

##
set(ITMLIB_ENGINES_RECONSTRUCTION_SHARED_HEADERS
    Engines/Reconstruction/Shared/SceneReconstructionEngine_Shared.h
    Engines/DepthFusion/DepthFusionEngine_Shared.h

    Engines/Reconstruction/Shared/SurfelSceneReconstructionEngine_Shared.h
    )
## ======================================= VOLUME FUSION ENGINE ========================================================
set(ITMLIB_ENGINES_VOLUME_FUSION_SOURCES
    Engines/VolumeFusion/VolumeFusionEngine.tpp
    Engines/VolumeFusion/VolumeFusionEngine_CPU_PlainVoxelArray.cpp
    Engines/VolumeFusion/VolumeFusionEngine_CPU_VoxelBlockHash.cpp
    Engines/VolumeFusion/VolumeFusionEngine_CUDA_PlainVoxelArray.cu
    Engines/VolumeFusion/VolumeFusionEngine_CUDA_VoxelBlockHash.cu
    )

set(ITMLIB_ENGINES_VOLUME_FUSION_HEADERS
    Engines/VolumeFusion/VolumeFusionEngine.h
    Engines/VolumeFusion/VolumeFusionFunctors.h
    Engines/VolumeFusion/VolumeFusionEngineFactory.h
    )

## ======================================= VOLUME WARP ENGINE ==========================================================
set(ITMLIB_ENGINES_WARP_VOLUME_SOURCES
    Engines/Warping/WarpingEngine.tpp
    Engines/Warping/WarpingEngine_CPU_PlainVoxelArray.cpp
    Engines/Warping/WarpingEngine_CUDA_PlainVoxelArray.cu
    Engines/Warping/WarpingEngine_CPU_VoxelBlockHash.cpp
    Engines/Warping/WarpingEngine_CUDA_VoxelBlockHash.cu
    )

set(ITMLIB_ENGINES_WARP_VOLUME_HEADERS
    Engines/Warping/WarpingEngine.h
    Engines/Warping/WarpingFunctors.h
    Engines/Warping/WarpingEngineFactory.h)

## ======================================= SCENE FILE IO ENGINES =======================================================
set(ITMLIB_ENGINES_SCENEFILEIO_HEADERS
    Engines/SceneFileIO/ITMSceneFileIOEngine.h
    )
set(ITMLIB_ENGINES_SCENEFILEIO_SOURCES
    Engines/SceneFileIO/ITMSceneFileIOEngine.tpp
    Engines/SceneFileIO/ITMSceneFileIOEngine.cpp
    Engines/SceneFileIO/ITMSceneFileIOEngine_PlainVoxelArray.cpp
    Engines/SceneFileIO/ITMSceneFileIOEngine_VoxelBlockHash.cpp
    )
## ======================================= SWAPPING ENGINES ============================================================
set(ITMLIB_ENGINES_SWAPPING_HEADERS
    Engines/Swapping/ITMSwappingEngineFactory.h
    )

##
set(ITMLIB_ENGINES_SWAPPING_CPU_SOURCES
    Engines/Swapping/CPU/ITMSwappingEngine_CPU.tpp
    )

set(ITMLIB_ENGINES_SWAPPING_CPU_HEADERS
    Engines/Swapping/CPU/ITMSwappingEngine_CPU.h
    )

##
set(ITMLIB_ENGINES_SWAPPING_CUDA_SOURCES
    Engines/Swapping/CUDA/ITMSwappingEngine_CUDA.tcu
    )

set(ITMLIB_ENGINES_SWAPPING_CUDA_HEADERS
    Engines/Swapping/CUDA/ITMSwappingEngine_CUDA.h
    )

##
set(ITMLIB_ENGINES_SWAPPING_INTERFACE_HEADERS
    Engines/Swapping/Interface/ITMSwappingEngine.h
    )

##
set(ITMLIB_ENGINES_SWAPPING_SHARED_HEADERS
    Engines/Swapping/Shared/ITMSwappingEngine_Shared.h
    )

## ======================================== VIEW-BUILDNG ENGINES =======================================================
set(ITMLIB_ENGINES_VIEWBUILDING_SOURCES
    Engines/ViewBuilding/ITMViewBuilderFactory.cpp
    )

set(ITMLIB_ENGINES_VIEWBUILDING_HEADERS
    Engines/ViewBuilding/ITMViewBuilderFactory.h
    )

##
set(ITMLIB_ENGINES_VIEWBUILDING_CPU_SOURCES
    Engines/ViewBuilding/CPU/ITMViewBuilder_CPU.cpp
    )

set(ITMLIB_ENGINES_VIEWBUILDING_CPU_HEADERS
    Engines/ViewBuilding/CPU/ITMViewBuilder_CPU.h
    )

##
set(ITMLIB_ENGINES_VIEWBUILDING_CUDA_SOURCES
    Engines/ViewBuilding/CUDA/ITMViewBuilder_CUDA.cu
    )

set(ITMLIB_ENGINES_VIEWBUILDING_CUDA_HEADERS
    Engines/ViewBuilding/CUDA/ITMViewBuilder_CUDA.h
    )

##
set(ITMLIB_ENGINES_VIEWBUILDING_INTERFACE_HEADERS
    Engines/ViewBuilding/Interface/ITMViewBuilder.h
    )

##
set(ITMLIB_ENGINES_VIEWBUILDING_SHARED_HEADERS
    Engines/ViewBuilding/Shared/ITMViewBuilder_Shared.h
    )
## ======================================== VISUALIATION ENGINES =======================================================
##
set(ITMLIB_ENGINES_VISUALISATION_SOURCES
    Engines/Visualization/ITMSurfelVisualizationEngineFactory.tpp
    )

set(ITMLIB_ENGINES_VISUALISATION_HEADERS
    Engines/Visualization/ITMMultiVisualizationEngineFactory.h
    Engines/Visualization/ITMSurfelVisualizationEngineFactory.h
    Engines/Visualization/ITMVisualizationEngineFactory.h
    )

##
set(ITMLIB_ENGINES_VISUALISATION_CPU_SOURCES
    Engines/Visualization/CPU/ITMMultiVisualizationEngine_CPU.tpp
    Engines/Visualization/CPU/ITMSurfelVisualisationEngine_CPU.tpp
    Engines/Visualization/CPU/ITMVisualisationEngine_CPU.tpp
    )

set(ITMLIB_ENGINES_VISUALISATION_CPU_HEADERS
    Engines/Visualization/CPU/ITMMultiVisualizationEngine_CPU.h
    Engines/Visualization/CPU/ITMSurfelVisualizationEngine_CPU.h
    Engines/Visualization/CPU/ITMVisualizationEngine_CPU.h
    )

##
set(ITMLIB_ENGINES_VISUALISATION_CUDA_SOURCES
    Engines/Visualization/CUDA/ITMMultiVisualizationEngine_CUDA.tcu
    Engines/Visualization/CUDA/ITMSurfelVisualizationEngine_CUDA.tcu
    Engines/Visualization/CUDA/ITMVisualizationEngine_CUDA.tcu
    Engines/Visualization/CUDA/ITMVisualizationHelpers_CUDA.cu
    )

set(ITMLIB_ENGINES_VISUALISATION_CUDA_HEADERS
    Engines/Visualization/CUDA/ITMMultiVisualizationEngine_CUDA.h
    Engines/Visualization/CUDA/ITMSurfelVisualizationEngine_CUDA.h
    Engines/Visualization/CUDA/ITMVisualizationEngine_CUDA.h
    Engines/Visualization/CUDA/ITMVisualizationHelpers_CUDA.h
    )

##
set(ITMLIB_ENGINES_VISUALISATION_INTERFACE_SOURCES
    Engines/Visualization/Interface/ITMSurfelVisualisationEngine.tpp
    Engines/Visualization/Interface/ITMVisualisationEngine.cpp
    )

set(ITMLIB_ENGINES_VISUALISATION_INTERFACE_HEADERS
    Engines/Visualization/Interface/ITMMultiVisualisationEngine.h
    Engines/Visualization/Interface/ITMSurfelVisualisationEngine.h
    Engines/Visualization/Interface/ITMVisualisationEngine.h
    )

##
set(ITMLIB_ENGINES_VISUALISATION_SHARED_HEADERS
    Engines/Visualization/Shared/ITMSurfelVisualisationEngine_Settings.h
    Engines/Visualization/Shared/ITMSurfelVisualisationEngine_Shared.h
    Engines/Visualization/Shared/ITMVisualisationEngine_Shared.h
    )

## ================================================== OBJECTS ==========================================================
set(ITMLIB_OBJECTS_CAMERA_SOURCES
    Objects/Camera/ITMCalibIO.cpp
    )

set(ITMLIB_OBJECTS_CAMERA_HEADERS
    Objects/Camera/ITMCalibIO.h
    Objects/Camera/ITMDisparityCalib.h
    Objects/Camera/ITMExtrinsics.h
    Objects/Camera/ITMIntrinsics.h
    Objects/Camera/ITMRGBDCalib.h
    )

##
set(ITMLIB_OBJECTS_MESHING_HEADERS
    Objects/Meshing/ITMMesh.h
    )

##
set(ITMLIB_OBJECTS_MISC_HEADERS
    Objects/Misc/ITMIMUCalibrator.h
    Objects/Misc/ITMIMUMeasurement.h
    Objects/Misc/ITMPointCloud.h
    )

##
set(ITMLIB_OBJECTS_RENDERSTATES_SOURCES
    Objects/RenderStates/ITMSurfelRenderState.cpp
    )

##
set(ITMLIB_OBJECTS_RENDERSTATES_HEADERS
    Objects/RenderStates/ITMRenderState.h
    Objects/RenderStates/ITMRenderStateMultiScene.h
    Objects/RenderStates/ITMSurfelRenderState.h
    )

##
set(ITMLIB_OBJECTS_SCENE_SOURCES
    Objects/Scene/ITMVoxelVolume.tpp
    Objects/Scene/ITMVoxelVolume.cpp
    Objects/Scene/ITMLocalVBA.tpp
    Objects/Scene/ITMLocalVBA.cpp
    Objects/Scene/VoxelBlockHash.cpp
    )

##
set(ITMLIB_OBJECTS_SCENE_HEADERS
    Objects/Scene/ITMGlobalCache.h
    Objects/Scene/ITMLocalMap.h
    Objects/Scene/ITMLocalVBA.h
    Objects/Scene/ITMMultiSceneAccess.h
    Objects/Scene/PlainVoxelArray.h
    Objects/Scene/ITMRepresentationAccess.h
    Objects/Scene/ITMTrilinearInterpolation.h
    Objects/Scene/ITMTrilinearDistribution.h
    Objects/Scene/ITMVoxelVolume.h
    Objects/Scene/ITMSurfelScene.h
    Objects/Scene/ITMSurfelTypes.h
    Objects/Scene/VoxelBlockHash.h
    Objects/Scene/ITMVoxelTypes.h
    )

##
set(ITMLIB_OBJECTS_TRACKING_HEADERS
    Objects/Tracking/ITMDepthHierarchyLevel.h
    Objects/Tracking/ITMImageHierarchy.h
    Objects/Tracking/ITMIntensityHierarchyLevel.h
    Objects/Tracking/ITMRGBHierarchyLevel.h
    Objects/Tracking/ITMSceneHierarchyLevel.h
    Objects/Tracking/ITMTemplatedHierarchyLevel.h
    Objects/Tracking/ITMTrackingState.h
    Objects/Tracking/ITMViewHierarchyLevel.h
    Objects/Tracking/TrackerIterationType.h
    )

##
set(ITMLIB_OBJECTS_VIEWS_HEADERS
    Objects/Views/ITMView.h
    Objects/Views/ITMViewIMU.h
    )

#===================================================== CAMERA TRACKERS =================================================
set(ITMLIB_CAMERA_TRACKERS_HEADERS
    CameraTrackers/ITMCameraTrackerFactory.h
    )
##
set(ITMLIB_CAMERA_TRACKERS_CPU_SOURCES
    CameraTrackers/CPU/ITMColorTracker_CPU.cpp
    CameraTrackers/CPU/ITMDepthTracker_CPU.cpp
    CameraTrackers/CPU/ITMExtendedTracker_CPU.cpp
    CameraTrackers/CPU/ITMDynamicCameraTracker_CPU.cpp
    )

set(ITMLIB_CAMERA_TRACKERS_CPU_HEADERS
    CameraTrackers/CPU/ITMColorTracker_CPU.h
    CameraTrackers/CPU/ITMDepthTracker_CPU.h
    CameraTrackers/CPU/ITMExtendedTracker_CPU.h
    CameraTrackers/CPU/ITMDynamicCameraTracker_CPU.h
    )

##
set(ITMLIB_CAMERA_TRACKERS_CUDA_SOURCES
    CameraTrackers/CUDA/ITMColorTracker_CUDA.cu
    CameraTrackers/CUDA/ITMDepthTracker_CUDA.cu
    CameraTrackers/CUDA/ITMExtendedTracker_CUDA.cu
    CameraTrackers/CUDA/ITMDynamicCameraTracker_CUDA.cu
    )

set(ITMLIB_CAMERA_TRACKERS_CUDA_HEADERS
    CameraTrackers/CUDA/ITMColorTracker_CUDA.h
    CameraTrackers/CUDA/ITMDepthTracker_CUDA.h
    CameraTrackers/CUDA/ITMExtendedTracker_CUDA.h
    CameraTrackers/CUDA/ITMDynamicCameraTracker_CUDA.h
    )

##
set(ITMLIB_CAMERA_TRACKERS_INTERFACE_SOURCES
    CameraTrackers/Interface/ITMColorTracker.cpp
    CameraTrackers/Interface/ITMDepthTracker.cpp
    CameraTrackers/Interface/ITMExtendedTracker.cpp
    CameraTrackers/Interface/ITMFileBasedTracker.cpp
    CameraTrackers/Interface/ITMForceFailTracker.cpp
    CameraTrackers/Interface/ITMIMUTracker.cpp
    CameraTrackers/Interface/ITMDynamicCameraTracker.cpp
    )

set(ITMLIB_CAMERA_TRACKERS_INTERFACE_HEADERS
    CameraTrackers/Interface/ITMColorTracker.h
    CameraTrackers/Interface/ITMCompositeTracker.h
    CameraTrackers/Interface/ITMDepthTracker.h
    CameraTrackers/Interface/ITMExtendedTracker.h
    CameraTrackers/Interface/ITMFileBasedTracker.h
    CameraTrackers/Interface/ITMForceFailTracker.h
    CameraTrackers/Interface/ITMIMUTracker.h
    CameraTrackers/Interface/ITMCameraTracker.h
    CameraTrackers/Interface/ITMDynamicCameraTracker.h
    )

##
set(ITMLIB_CAMERA_TRACKERS_SHARED_HEADERS
    CameraTrackers/Shared/ITMColorTracker_Shared.h
    CameraTrackers/Shared/ITMDepthTracker_Shared.h
    CameraTrackers/Shared/ITMExtendedTracker_Shared.h
    )
# ============================================== SCENE MOTION TRACKERS =================================================
set(ITMLIB_SCENE_MOTION_TRACKERS_HEADERS
    SurfaceTrackers/SurfaceTrackerFactory.h
    )

set(ITMLIB_SCENE_MOTION_TRACKERS_CPU_SOURCES

    SurfaceTrackers/CPU/SurfaceTracker_CPU_VoxelBlockHash.cpp
    SurfaceTrackers/CPU/SurfaceTracker_CPU_PlainVoxelArray.cpp
    )

set(ITMLIB_SCENE_MOTION_TRACKERS_CPU_HEADERS
    )
##

set(ITMLIB_SCENE_MOTION_TRACKERS_CUDA_SOURCES
    SurfaceTrackers/CUDA/SurfaceTracker_CUDA_PlainVoxelArray.cu
    SurfaceTrackers/CUDA/SurfaceTracker_CUDA_VoxelBlockHash.cu
    )
set(ITMLIB_SCENE_MOTION_TRACKERS_CUDA_HEADERS

    )
##

set(ITMLIB_SCENE_MOTION_TRACKERS_INTERFACE_SOURCES
    SurfaceTrackers/Interface/SlavchevaSurfaceTracker.cpp
    SurfaceTrackers/Interface/SurfaceTracker.tpp
    )
set(ITMLIB_SCENE_MOTION_TRACKERS_INTERFACE_HEADERS
    SurfaceTrackers/WarpGradientFunctors/WarpGradientFunctor.h
    SurfaceTrackers/Interface/SurfaceTrackerInterface.h
    SurfaceTrackers/Interface/SlavchevaSufraceTracker.h
    SurfaceTrackers/Interface/SurfaceTracker.h
    )
##
set(ITMLIB_SCENE_MOTION_TRACKERS_SHARED_HEADERS
    SurfaceTrackers/WarpGradientFunctors/WarpGradientFunctor_SlavchevaDiagnostic.h
    SurfaceTrackers/WarpGradientFunctors/WarpGradientFunctor_SlavchevaOptimized.h

    SurfaceTrackers/Shared/ITMWarpGradientCommon.h
    SurfaceTrackers/Shared/SurfaceTrackerSharedRoutines.h
    SurfaceTrackers/Shared/SurfaceTrackerDiagnosticRoutines.h
    SurfaceTrackers/Shared/SurfaceTrackerSharedFunctors.h
    SurfaceTrackers/Shared/SurfaceTrackerOptimizationParameters.h
    SurfaceTrackers/Shared/ITMWarpGradientRoutines.h
    SurfaceTrackers/Shared/ITMWarpGradientAggregates.h
    )

# ======================================================================================================================
# =================================================== UTILITIES (MISC) =================================================
set(ITMLIB_UTILS_SOURCES
    Utils/Collections/ITM3DNestedMapOfArrays.tpp
    Utils/Collections/ITM3DNestedMapOfArrays.cpp
    Utils/Collections/ITM3DNestedMap.tpp
    Utils/Collections/ITM3DNestedMap.cpp

    Utils/Serialization/SerializationDetails.cpp

    Utils/Analytics/ITMNeighborVoxelIterationInfo.cpp
    Utils/Analytics/ITMBenchmarkUtils.cpp

    Utils/Analytics/VoxelVolumeComparison/ITMVoxelVolumeComparison_CPU.tpp
    Utils/Analytics/VoxelVolumeComparison/ITMVoxelVolumeComparison_CPU.cpp
    Utils/Analytics/VoxelVolumeComparison/ITMVoxelVolumeComparison_CUDA.tcu
    Utils/Analytics/VoxelVolumeComparison/ITMVoxelVolumeComparison_CUDA.cu

    Utils/json_utils.cpp

    Utils/Configuration.cpp
    Utils/ITMPrintHelpers.cpp
    )

set(ITMLIB_UTILS_HEADERS
    Utils/Collections/ITM3DNestedMapOfArrays.h
    Utils/Collections/ITM3DNestedMap.h

    Utils/Geometry/ITMGeometryBooleanOperations.h
    Utils/Geometry/ITMGeometryConversionOperations.h
    Utils/Geometry/ITM3DIndexConversions.h
    Utils/Geometry/ITMIntersectionChecks.h
    Utils/Geometry/ITMSegment.h

    Utils/Serialization/Serialization.h
    Utils/Serialization/SerializationDetails.h
    Utils/Serialization/SerializationSequenceMacros.h
    Utils/Serialization/SerializationSequenceLevel2Macros.h
    Utils/Serialization/PreprocessorNargs.h

    Utils/json_utils.h

    Utils/ITMCUDAUtils.h
    Utils/ITMValuePosition.h
    Utils/ITMImageTypes.h
    Utils/ITMHashBlockProperties.h
    Utils/ITMVoxelFlags.h
    Utils/Configuration.h
    Utils/ITMMath.h
    Utils/ITMMemoryBlockTypes.h
    Utils/ITMPixelUtils.h
    Utils/ITMProjectionUtils.h
    Utils/VoxelVolumeParameters.h
    Utils/SurfelVolumeParameters.h
    Utils/ITMPrintHelpers.h
    Utils/ITMCPrintHelpers.h

    Utils/Analytics/ITMBenchmarkUtils.h
    Utils/Analytics/ITMAlmostEqual.h
    Utils/Analytics/ITMIsAltered.h
    Utils/Analytics/ITMNeighborVoxelIterationInfo.h

    Utils/Analytics/VoxelVolumeComparison/ITMVoxelVolumeComparison.h
    Utils/Analytics/VoxelVolumeComparison/ITMVoxelVolumeComparison_CPU.h
    Utils/Analytics/VoxelVolumeComparison/ITMVoxelVolumeComparison_CUDA.h
    Utils/Analytics/VoxelVolumeComparison/ITMVoxelVolumeComparison_Functors.h
    )

## ================================================ UTILS - SCENE STATISTICS CALCULATOR ================================

set(ITMLIB_UTILS_STATCALC_INTERFACE_HEADERS
    Utils/Analytics/SceneStatisticsCalculator/Interface/ITMSceneStatisticsCalculatorInterface.h
    )


set(ITMLIB_UTILS_STATCALC_SHARED_HEADERS
    Utils/Analytics/SceneStatisticsCalculator/Shared/ITMSceneStatisticsCalculator_Functors.h
    )


set(ITMLIB_UTILS_STATCALC_CPU_HEADERS
    Utils/Analytics/SceneStatisticsCalculator/CPU/ITMSceneStatisticsCalculator_CPU.h
    )

set(ITMLIB_UTILS_STATCALC_CPU_SOURCES
    Utils/Analytics/SceneStatisticsCalculator/CPU/ITMSceneStatisticsCalculator_CPU.tpp
    Utils/Analytics/SceneStatisticsCalculator/CPU/ITMSceneStatisticsCalculator_CPU_PlainVoxelArray.cpp
    Utils/Analytics/SceneStatisticsCalculator/CPU/ITMSceneStatisticsCalculator_CPU_VoxelBlockHash.cpp
    )

set(ITMLIB_UTILS_STATCALC_CUDA_HEADERS
    Utils/Analytics/SceneStatisticsCalculator/CUDA/ITMSceneStatisticsCalculator_CUDA.h
    )
set(ITMLIB_UTILS_STATCALC_CUDA_SOURCES
    Utils/Analytics/SceneStatisticsCalculator/CUDA/ITMSceneStatisticsCalculator_CUDA.tcu
    Utils/Analytics/SceneStatisticsCalculator/CUDA/ITMSceneStatisticsCalculator_CUDA_PlainVoxelArray.cu
    Utils/Analytics/SceneStatisticsCalculator/CUDA/ITMSceneStatisticsCalculator_CUDA_VoxelBlockHash.cu
    )


## ================================================ UTILS - FILEIO =====================================================
set(ITMLIB_UTILS_FILEIO_SOURCES
    Utils/FileIO/ITMDynamicFusionLogger_PlainVoxelArray.cpp
    Utils/FileIO/ITMDynamicFusionLogger_VoxelBlockHash.cpp
    Utils/FileIO/ITMSceneLogger.tpp
    Utils/FileIO/ITMSceneLogger_PlainVoxelArray.cpp
    Utils/FileIO/ITMSceneLogger_VoxelBlockHash.cpp
    Utils/FileIO/ITMDynamicFusionLogger.tpp
    Utils/FileIO/ITMSceneLogger_SceneSlice.tpp
    Utils/FileIO/ITMWarpFieldLogger.tpp
    Utils/FileIO/ITMWarpFieldLogger_PlainVoxelArray.cpp
    Utils/FileIO/ITMWarpFieldLogger_VoxelBlockHash.cpp
    )

set(ITMLIB_UTILS_FILEIO_HEADERS
    Utils/FileIO/ITMWriteAndReadFunctors.h
    Utils/FileIO/ITMSceneLogger.h
    Utils/FileIO/ITMDynamicFusionLogger.h
    Utils/FileIO/ITMWarpFieldLogger.h)
##

set(ITMLIB_UTILS_VISUALIZATION_SOURCES
    Utils/Visualization/ITMSceneSliceVisualizer2D.tpp
    Utils/Visualization/ITMSceneSliceVisualizer2D_PlainVoxelArray.cpp
    Utils/Visualization/ITMSceneSliceVisualizer2D_VoxelBlockHash.cpp

    Utils/Visualization/ITMVisualizationCommon.cpp
    )
if (WITH_VTK)
    list(APPEND ITMLIB_UTILS_VISUALIZATION_SOURCES
         Utils/Visualization/ITMSceneSliceVisualizer1D.cpp
         Utils/Visualization/ITMSceneSliceVisualizer1D.tpp
         Utils/Visualization/ITMSceneSliceVisualizer3D.tpp
         Utils/Visualization/ITMSceneSliceVisualizer3DInteractorStyle.tpp
         Utils/Visualization/ITMSceneSliceVisualizer3DCommon.cpp
         Utils/Visualization/ITMSceneSliceVisualizer3D_PlainVoxelArray.cpp
         Utils/Visualization/ITMSceneSliceVisualizer3D_VoxelBlockHash.cpp
         Utils/Visualization/ITMSceneTrackingEnergyPlotter.cpp
         Utils/Visualization/ITMVisualizationWindowManager.cpp
         )
endif ()

set(ITMLIB_UTILS_VISUALIZATION_HEADERS
    Utils/Visualization/ITMSceneSliceVisualizerCommon.h
    Utils/Visualization/ITMSceneSliceVisualizer2D.h

    Utils/Visualization/ITMVisualizationCommon.h
    )
if (WITH_VTK)
    list(APPEND ITMLIB_UTILS_VISUALIZATION_HEADERS
         Utils/Visualization/ITMSceneSliceVisualizer1D.h
         Utils/Visualization/ITMSceneSliceVisualizer3D.h
         Utils/Visualization/ITMSceneSliceVisualizer3DInteractorStyle.h
         Utils/Visualization/ITMSceneSliceVisualizer3DCommon.h
         Utils/Visualization/ITMSceneTrackingEnergyPlotter.h
         Utils/Visualization/ITMVisualizationWindowManager.h
         )
endif ()
#################################################################
# Collect the project files into sources, headers and templates #
#################################################################

set(sources
    ${ITMLIB_OBJECTS_SCENE_SOURCES}
    ${ITMLIB_TOPLEVEL_CPU_SOURCES}
    ${ITMLIB_TOPLEVEL_CUDA_SOURCES}
    ${ITMLIB_CORE_SOURCES}
    ${ITMLIB_ENGINES_LOWLEVEL_SOURCES}
    ${ITMLIB_ENGINES_LOWLEVEL_CPU_SOURCES}
    ${ITMLIB_ENGINES_LOWLEVEL_CUDA_SOURCES}
    ${ITMLIB_ENGINES_MANIPULATION_CPU_SOURCES}
    ${ITMLIB_ENGINES_MANIPULATION_CUDA_SOURCES}
    ${ITMLIB_ENGINES_TRAVERSAL_CPU_SOURCES}
    ${ITMLIB_ENGINES_TRAVERSAL_CUDA_SOURCES}
    ${ITMLIB_ENGINES_MESHING_CPU_SOURCES}
    ${ITMLIB_ENGINES_MESHING_CUDA_SOURCES}
    ${ITMLIB_ENGINES_MULTISCENE_SOURCES}
    ${ITMLIB_ENGINES_INDEXING_SOURCES}
    ${ITMLIB_ENGINES_INDEXING_CPU_SOURCES}
    ${ITMLIB_ENGINES_INDEXING_CUDA_SOURCES}
    ${ITMLIB_ENGINES_INDEXING_INTERFACE_SOURCES}
    ${ITMLIB_ENGINES_RECONSTRUCTION_SOURCES}
    ${ITMLIB_ENGINES_RECONSTRUCTION_CPU_SOURCES}
    ${ITMLIB_ENGINES_RECONSTRUCTION_CUDA_SOURCES}
    ${ITMLIB_ENGINES_RECONSTRUCTION_INTERFACE_SOURCES}
    ${ITMLIB_ENGINES_VOLUME_FUSION_SOURCES}
    ${ITMLIB_ENGINES_WARP_VOLUME_SOURCES}
    ${ITMLIB_ENGINES_SCENEFILEIO_SOURCES}
    ${ITMLIB_ENGINES_SWAPPING_CPU_SOURCES}
    ${ITMLIB_ENGINES_SWAPPING_CUDA_SOURCES}
    ${ITMLIB_ENGINES_VIEWBUILDING_SOURCES}
    ${ITMLIB_ENGINES_VIEWBUILDING_CPU_SOURCES}
    ${ITMLIB_ENGINES_VIEWBUILDING_CUDA_SOURCES}
    ${ITMLIB_ENGINES_VISUALISATION_SOURCES}
    ${ITMLIB_ENGINES_VISUALISATION_CPU_SOURCES}
    ${ITMLIB_ENGINES_VISUALISATION_CUDA_SOURCES}
    ${ITMLIB_ENGINES_VISUALISATION_INTERFACE_SOURCES}
    ${ITMLIB_OBJECTS_CAMERA_SOURCES}
    ${ITMLIB_OBJECTS_RENDERSTATES_SOURCES}
    ${ITMLIB_CAMERA_TRACKERS_CPU_SOURCES}
    ${ITMLIB_CAMERA_TRACKERS_CUDA_SOURCES}
    ${ITMLIB_CAMERA_TRACKERS_INTERFACE_SOURCES}
    ${ITMLIB_SCENE_MOTION_TRACKERS_SOURCES}
    ${ITMLIB_SCENE_MOTION_TRACKERS_CPU_SOURCES}
    ${ITMLIB_SCENE_MOTION_TRACKERS_CUDA_SOURCES}
    ${ITMLIB_SCENE_MOTION_TRACKERS_INTERFACE_SOURCES}
    ${ITMLIB_SCENE_MOTION_TRACKERS_SHARED_SOURCES}
    ${ITMLIB_UTILS_SOURCES}
    ${ITMLIB_UTILS_STATCALC_CPU_SOURCES}
    ${ITMLIB_UTILS_STATCALC_CUDA_SOURCES}
    ${ITMLIB_UTILS_FILEIO_SOURCES}
    ${ITMLIB_UTILS_VISUALIZATION_SOURCES}
    )

set(headers
    ${ITMLIB_TOPLEVEL_HEADERS}
    ${ITMLIB_CORE_HEADERS}
    ${ITMLIB_ENGINES_COMMON_HEADERS}
    ${ITMLIB_ENGINES_LOWLEVEL_HEADERS}
    ${ITMLIB_ENGINES_LOWLEVEL_CPU_HEADERS}
    ${ITMLIB_ENGINES_LOWLEVEL_CUDA_HEADERS}
    ${ITMLIB_ENGINES_MANIPULATION_HEADERS}
    ${ITMLIB_ENGINES_MANIPULATION_INTERFACE_HEADERS}
    ${ITMLIB_ENGINES_MANIPULATION_SHARED_HEADERS}
    ${ITMLIB_ENGINES_MANIPULATION_CPU_HEADERS}
    ${ITMLIB_ENGINES_MANIPULATION_CUDA_HEADERS}
    ${ITMLIB_ENGINES_TRAVERSAL_INTERFACE_HEADERS}
    ${ITMLIB_ENGINES_TRAVERSAL_SHARED_HEADERS}
    ${ITMLIB_ENGINES_TRAVERSAL_CPU_HEADERS}
    ${ITMLIB_ENGINES_TRAVERSAL_CUDA_HEADERS}
    ${ITMLIB_ENGINES_LOWLEVEL_INTERFACE_HEADERS}
    ${ITMLIB_ENGINES_LOWLEVEL_SHARED_HEADERS}
    ${ITMLIB_ENGINES_MESHING_HEADERS}
    ${ITMLIB_ENGINES_MESHING_CPU_HEADERS}
    ${ITMLIB_ENGINES_MESHING_CUDA_HEADERS}
    ${ITMLIB_ENGINES_MESHING_INTERFACE_HEADERS}
    ${ITMLIB_ENGINES_MESHING_SHARED_HEADERS}
    ${ITMLIB_ENGINES_MULTISCENE_HEADERS}
    ${ITMLIB_ENGINES_INDEXING_HEADERS}
    ${ITMLIB_ENGINES_INDEXING_CPU_HEADERS}
    ${ITMLIB_ENGINES_INDEXING_CUDA_HEADERS}
    ${ITMLIB_ENGINES_INDEXING_INTERFACE_HEADERS}
    ${ITMLIB_ENGINES_INDEXING_SHARED_HEADERS}
    ${ITMLIB_ENGINES_RECONSTRUCTION_HEADERS}
    ${ITMLIB_ENGINES_RECONSTRUCTION_CPU_HEADERS}
    ${ITMLIB_ENGINES_RECONSTRUCTION_CUDA_HEADERS}
    ${ITMLIB_ENGINES_RECONSTRUCTION_INTERFACE_HEADERS}
    ${ITMLIB_ENGINES_RECONSTRUCTION_SHARED_HEADERS}
    ${ITMLIB_ENGINES_VOLUME_FUSION_HEADERS}
    ${ITMLIB_ENGINES_WARP_VOLUME_HEADERS}
    ${ITMLIB_ENGINES_SCENEFILEIO_HEADERS}
    ${ITMLIB_ENGINES_SWAPPING_HEADERS}
    ${ITMLIB_ENGINES_SWAPPING_CPU_HEADERS}
    ${ITMLIB_ENGINES_SWAPPING_CUDA_HEADERS}
    ${ITMLIB_ENGINES_SWAPPING_INTERFACE_HEADERS}
    ${ITMLIB_ENGINES_SWAPPING_SHARED_HEADERS}
    ${ITMLIB_ENGINES_VIEWBUILDING_HEADERS}
    ${ITMLIB_ENGINES_VIEWBUILDING_CPU_HEADERS}
    ${ITMLIB_ENGINES_VIEWBUILDING_CUDA_HEADERS}
    ${ITMLIB_ENGINES_VIEWBUILDING_INTERFACE_HEADERS}
    ${ITMLIB_ENGINES_VIEWBUILDING_SHARED_HEADERS}
    ${ITMLIB_ENGINES_VISUALISATION_HEADERS}
    ${ITMLIB_ENGINES_VISUALISATION_CPU_HEADERS}
    ${ITMLIB_ENGINES_VISUALISATION_CUDA_HEADERS}
    ${ITMLIB_ENGINES_VISUALISATION_INTERFACE_HEADERS}
    ${ITMLIB_ENGINES_VISUALISATION_SHARED_HEADERS}
    ${ITMLIB_OBJECTS_CAMERA_HEADERS}
    ${ITMLIB_OBJECTS_MESHING_HEADERS}
    ${ITMLIB_OBJECTS_MISC_HEADERS}
    ${ITMLIB_OBJECTS_RENDERSTATES_HEADERS}
    ${ITMLIB_OBJECTS_SCENE_SOURCES}
    ${ITMLIB_OBJECTS_SCENE_HEADERS}
    ${ITMLIB_OBJECTS_TRACKING_HEADERS}
    ${ITMLIB_OBJECTS_VIEWS_HEADERS}
    ${ITMLIB_CAMERA_TRACKERS_HEADERS}
    ${ITMLIB_CAMERA_TRACKERS_CPU_HEADERS}
    ${ITMLIB_CAMERA_TRACKERS_CUDA_HEADERS}
    ${ITMLIB_CAMERA_TRACKERS_INTERFACE_HEADERS}
    ${ITMLIB_CAMERA_TRACKERS_SHARED_HEADERS}
    ${ITMLIB_SCENE_MOTION_TRACKERS_HEADERS}
    ${ITMLIB_SCENE_MOTION_TRACKERS_CPU_HEADERS}
    ${ITMLIB_SCENE_MOTION_TRACKERS_CUDA_HEADERS}
    ${ITMLIB_SCENE_MOTION_TRACKERS_INTERFACE_HEADERS}
    ${ITMLIB_SCENE_MOTION_TRACKERS_SHARED_HEADERS}
    ${ITMLIB_UTILS_HEADERS}
    ${ITMLIB_UTILS_STATCALC_INTERFACE_HEADERS}
    ${ITMLIB_UTILS_STATCALC_SHARED_HEADERS}
    ${ITMLIB_UTILS_STATCALC_CPU_HEADERS}
    ${ITMLIB_UTILS_STATCALC_CUDA_HEADERS}
    ${ITMLIB_UTILS_FILEIO_HEADERS}
    ${ITMLIB_UTILS_VISUALIZATION_HEADERS}
    )

#############################
# Specify the source groups #
#############################
source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${sources} ${headers})

##########################################
# Specify the target and where to put it #
##########################################

include(${PROJECT_SOURCE_DIR}/cmake/SetCUDALibTarget.cmake)


#TODO: make these optional and consistent with the overal CMake package usage in the project -Greg (GitHub: Algomorph)
##########################################
# Find and link extra packages #
##########################################

if (WITH_VTK)
    target_compile_definitions(${targetname} PUBLIC -DWITH_VTK)
endif ()
target_include_directories(${targetname} PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${targetname} ${OpenCV_LIBS} ${VTK_LIBRARIES})

include(${PROJECT_SOURCE_DIR}/cmake/LinkBoost.cmake)


#fix potential warning causing from error in VTK compile definitions
# "<command-line>:0:20: warning: ISO C++11 requires whitespace after the macro name"
get_property(COMPILE_DEFINITIONS_PROPERTY DIRECTORY PROPERTY COMPILE_DEFINITIONS)
list(FIND COMPILE_DEFINITIONS_PROPERTY "vtkRenderingCore_AUTOINIT=3(vtkInteractionStyle,vtkRenderingFreeType,vtkRenderingOpenGL2)" INDEX)
if (NOT INDEX EQUAL -1)
    list(REMOVE_AT COMPILE_DEFINITIONS_PROPERTY ${INDEX})
    list(APPEND COMPILE_DEFINITIONS_PROPERTY "vtkRenderingCore_AUTOINIT=3(vtkInteractionStyle ,vtkRenderingFreeType ,vtkRenderingOpenGL2 )")
    #replace with the same properties with spaces after the macro name
    set_property(DIRECTORY PROPERTY COMPILE_DEFINITIONS ${COMPILE_DEFINITIONS_PROPERTY})
endif ()