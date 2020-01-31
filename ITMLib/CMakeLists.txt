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

set(ITMLIB_TOPLEVEL_HEADERS
    ITMLibDefines.h
    )

##
set(ITMLIB_CORE_SOURCES
    Engines/Main/Mappers/DenseMapper.tpp
    Engines/Main/Mappers/DenseMapper_PlainVoxelArray.cpp
    Engines/Main/Mappers/DenseMapper_VoxelBlockHash.cpp
    Engines/Main/Mappers/DenseSurfelMapper.tpp
    Engines/Main/Mappers/DenseSurfelMapper.cpp
    Engines/Main/Mappers/DenseDynamicMapper.tpp
    Engines/Main/Mappers/DenseDynamicMapper_PlainVoxelArray.cpp
    Engines/Main/Mappers/DenseDynamicMapper_VoxelBlockHash.cpp

    Engines/Main/BasicVoxelEngine.tpp
    Engines/Main/BasicVoxelEngine_PlainVoxelArray.cpp
    Engines/Main/BasicVoxelEngine_VoxelBlockHash.cpp

    Engines/Main/BasicSurfelEngine.tpp
    Engines/Main/BasicSufelEngine.cpp

    Engines/Main/MultiEngine.tpp
    Engines/Main/MultiEngine_PlainVoxelArray.cpp
    Engines/Main/MultiEngine_VoxelBlockHash.cpp

    Engines/Main/NonRigidTrackingParameters.h
    Engines/Main/DynamicSceneVoxelEngine.tpp
    Engines/Main/DynamicSceneVoxelEngine_PlainVoxelArray.cpp
    Engines/Main/DynamicSceneVoxelEngine_VoxelBlockHash.cpp
    )

set(ITMLIB_CORE_HEADERS
    Engines/Main/Mappers/DenseMapper.h
    Engines/Main/Mappers/DenseSurfelMapper.h
    Engines/Main/Mappers/DenseDynamicMapper.h

    Engines/Main/BasicVoxelEngine.h
    Engines/Main/BasicSurfelEngine.h

    Engines/Main/MainEngine.h
    Engines/Main/MultiEngine.h
    Engines/Main/TrackingController.h
    Engines/Main/DynamicSceneVoxelEngine.h
    Engines/Main/MainEngineFactory.h
    )

#======================================= COMMON TO MULTIPLE ENGINES ====================================================
set(ITMLIB_ENGINES_COMMON_HEADERS
    Engines/Common/AllocationTempData.h
    Engines/Common/CheckBlockVisibility.h
    Engines/Common/CommonFunctors.h
    Engines/Common/WarpType.h
    )
#======================================= LOW-LEVEL ENGINES =============================================================
##
set(ITMLIB_ENGINES_LOWLEVEL_SOURCES
    Engines/LowLevel/LowLevelEngineFactory.cpp
    )

set(ITMLIB_ENGINES_LOWLEVEL_HEADERS
    Engines/LowLevel/LowLevelEngineFactory.h
    )

##
set(ITMLIB_ENGINES_LOWLEVEL_CPU_SOURCES
    Engines/LowLevel/CPU/LowLevelEngine_CPU.cpp
    )

set(ITMLIB_ENGINES_LOWLEVEL_CPU_HEADERS
    Engines/LowLevel/CPU/LowLevelEngine_CPU.h
    )

##
set(ITMLIB_ENGINES_LOWLEVEL_CUDA_SOURCES
    Engines/LowLevel/CUDA/LowLevelEngine_CUDA.cu
    )

set(ITMLIB_ENGINES_LOWLEVEL_CUDA_HEADERS
    Engines/LowLevel/CUDA/LowLevelEngine_CUDA.h
    )

##
set(ITMLIB_ENGINES_LOWLEVEL_INTERFACE_HEADERS
    Engines/LowLevel/Interface/LowLevelEngine.h
    )

##
set(ITMLIB_ENGINES_LOWLEVEL_SHARED_HEADERS
    Engines/LowLevel/Shared/LowLevelEngine_Shared.h
    )

#====================================== SCENE TRAVERSAL ENGINES ======================================================

##
set(ITMLIB_ENGINES_TRAVERSAL_INTERFACE_HEADERS
    Engines/Traversal/Interface/VolumeTraversal.h
    )
##
set(ITMLIB_ENGINES_TRAVERSAL_SHARED_HEADERS
    Engines/Traversal/Shared/VolumeTraversal_Shared.h
    )

##
set(ITMLIB_ENGINES_TRAVERSAL_CPU_SOURCES
    )

set(ITMLIB_ENGINES_TRAVERSAL_CPU_HEADERS
    Engines/Traversal/CPU/VolumeTraversal_CPU_VoxelBlockHash.h
    Engines/Traversal/CPU/VolumeTraversal_CPU_PlainVoxelArray.h
    Engines/Traversal/CPU/VolumeTraversal_CPU_PVA_to_VBH.h
    )

##
set(ITMLIB_ENGINES_TRAVERSAL_CUDA_SOURCES
    )

set(ITMLIB_ENGINES_TRAVERSAL_CUDA_HEADERS
    Engines/Traversal/CUDA/VolumeTraversal_CUDA_VoxelBlockHash.h
    Engines/Traversal/CUDA/VolumeTraversal_CUDA_VoxelBlockHash_Kernels.h
    Engines/Traversal/CUDA/VolumeTraversal_CUDA_PlainVoxelArray.h
    Engines/Traversal/CUDA/VolumeTraversal_CUDA_PlainVoxelArray_Kernels.h
    Engines/Traversal/CUDA/VolumeTraversal_CUDA_PVA_to_VBH.h
    Engines/Traversal/CUDA/VolumeTraversal_CUDA_PVA_to_VBH_Kernels.h
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
    Engines/Meshing/MeshingEngineFactory.h
    Engines/Meshing/MultiMeshingEngineFactory.h
    )

##
set(ITMLIB_ENGINES_MESHING_CPU_SOURCES
    Engines/Meshing/CPU/MeshingEngine_CPU.tpp
    Engines/Meshing/CPU/MeshingEngine_CPU_PlainVoxelArray.cpp
    Engines/Meshing/CPU/MeshingEngine_CPU_VoxelBlockHash.cpp
    Engines/Meshing/CPU/MultiMeshingEngine_CPU_PlainVoxelArray.cpp
    Engines/Meshing/CPU/MultiMeshingEngine_CPU_VoxelBlockHash.cpp
    Engines/Meshing/CPU/MultiMeshingEngine_CPU.tpp
    )

set(ITMLIB_ENGINES_MESHING_CPU_HEADERS
    Engines/Meshing/CPU/MeshingEngine_CPU.h
    Engines/Meshing/CPU/MultiMeshingEngine_CPU.h
    )

##
set(ITMLIB_ENGINES_MESHING_CUDA_SOURCES
    Engines/Meshing/CUDA/MeshingEngine_CUDA.tcu
    Engines/Meshing/CUDA/MeshingEngine_CUDA_PlainVoxelArray.cu
    Engines/Meshing/CUDA/MeshingEngine_CUDA_VoxelBlockHash.cu
    Engines/Meshing/CUDA/MultiMeshingEngine_CUDA.tcu
    Engines/Meshing/CUDA/MultiMeshingEngine_CUDA_PlainVoxelArray.cu
    Engines/Meshing/CUDA/MultiMeshingEngine_CUDA_VoxelBlockHash.cu
    )

set(ITMLIB_ENGINES_MESHING_CUDA_HEADERS
    Engines/Meshing/CUDA/MeshingEngine_CUDA.h
    Engines/Meshing/CUDA/MultiMeshingEngine_CUDA.h
    )

##
set(ITMLIB_ENGINES_MESHING_INTERFACE_HEADERS
    Engines/Meshing/Interface/MeshingEngine.h
    Engines/Meshing/Interface/MultiMeshingEngine.h
    )

##
set(ITMLIB_ENGINES_MESHING_SHARED_HEADERS
    Engines/Meshing/Shared/MeshingEngine_Shared.h
    Engines/Meshing/Shared/MultiMeshingEngine_Shared.h
    )

##
set(ITMLIB_ENGINES_MULTISCENE_SOURCES
    Engines/MultiScene/ActiveMapManager.cpp
    Engines/MultiScene/GlobalAdjustmentEngine.cpp
    Engines/MultiScene/VoxelMapGraphManager.tpp
    Engines/MultiScene/VoxelMapGraphManager_PlainVoxelArray.cpp
    Engines/MultiScene/VoxelMapGraphManager_VoxelBlockHash.cpp
    )

set(ITMLIB_ENGINES_MULTISCENE_HEADERS
    Engines/MultiScene/ActiveMapManager.h
    Engines/MultiScene/GlobalAdjustmentEngine.h
    Engines/MultiScene/VoxelMapGraphManager.h
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
set(ITMLIB_ENGINES_RECONSTRUCTION_FACTORY_HEADERS
    Engines/Reconstruction/SceneReconstructionEngineFactory.h
    Engines/Reconstruction/SurfelSceneReconstructionEngineFactory.h
    )
##
set(ITMLIB_ENGINES_RECONSTRUCTION_CPU_SOURCES
    Engines/Reconstruction/CPU/SceneReconstructionEngine_CPU.tpp
    Engines/Reconstruction/CPU/SceneReconstructionEngine_CPU_VoxelBlockHash.cpp
    Engines/Reconstruction/CPU/SceneReconstructionEngine_CPU_PlainVoxelArray.cpp
    Engines/Reconstruction/CPU/SurfelSceneReconstructionEngine_CPU.tpp
    Engines/Reconstruction/CPU/SurfelSceneReconstructionEngine_CPU.cpp
    )
set(ITMLIB_ENGINES_RECONSTRUCTION_CPU_HEADERS
    Engines/Reconstruction/CPU/SceneReconstructionEngine_CPU.h
    Engines/Reconstruction/CPU/SurfelSceneReconstructionEngine_CPU.h
    )
##
set(ITMLIB_ENGINES_RECONSTRUCTION_CUDA_SOURCES
    Engines/Reconstruction/CUDA/SceneReconstructionEngine_CUDA.tcu
    Engines/Reconstruction/CUDA/SceneReconstructionEngine_CUDA_VoxelBlockHash.cu
    Engines/Reconstruction/CUDA/SceneReconstructionEngine_CUDA_PlainVoxelArray.cu
    Engines/Reconstruction/CUDA/SurfelSceneReconstructionEngine_CUDA.tcu
    Engines/Reconstruction/CUDA/SurfelSceneReconstructionEngine_CUDA.cu
    )
set(ITMLIB_ENGINES_RECONSTRUCTION_CUDA_HEADERS
    Engines/Reconstruction/CUDA/SceneReconstructionEngine_CUDA.h
    Engines/Reconstruction/CUDA/SurfelSceneReconstructionEngine_CUDA.h
    )
##
set(ITMLIB_ENGINES_RECONSTRUCTION_INTERFACE_SOURCES
    Engines/Reconstruction/Interface/SurfelSceneReconstructionEngine.tpp
    Engines/Reconstruction/Interface/SurfelSceneReconstructionEngine.cpp
    )
set(ITMLIB_ENGINES_RECONSTRUCTION_INTERFACE_HEADERS
    Engines/Reconstruction/Interface/SceneReconstructionEngine.h
    Engines/Reconstruction/Interface/SurfelSceneReconstructionEngine.h
    )
##
set(ITMLIB_ENGINES_RECONSTRUCTION_SHARED_HEADERS
    Engines/Reconstruction/Shared/SceneReconstructionEngine_Shared.h
    Engines/Reconstruction/Shared/SurfelSceneReconstructionEngine_Shared.h
    )

set(
        ITMLIB_ENGINES_RECONSTRUCTION_HEADERS

        ${ITMLIB_ENGINES_RECONSTRUCTION_FACTORY_HEADERS}
        ${ITMLIB_ENGINES_RECONSTRUCTION_SHARED_HEADERS}
        ${ITMLIB_ENGINES_RECONSTRUCTION_INTERFACE_HEADERS}
        ${ITMLIB_ENGINES_RECONSTRUCTION_CPU_HEADERS}
        ${ITMLIB_ENGINES_RECONSTRUCTION_CUDA_HEADERS}
)
set(
        ITMLIB_ENGINES_RECONSTRUCTION_SOURCES

        ${ITMLIB_ENGINES_RECONSTRUCTION_FACTORY_SOURCES}
        ${ITMLIB_ENGINES_RECONSTRUCTION_INTERFACE_SOURCES}
        ${ITMLIB_ENGINES_RECONSTRUCTION_CPU_SOURCES}
        ${ITMLIB_ENGINES_RECONSTRUCTION_CUDA_SOURCES}
)
## ======================================= DEPTH FUSION ENGINE =========================================================
# (replaces former "reconstruction" engine for dynamic-scene fusion)
set(ITMLIB_ENGINES_DEPTH_FUSION_SOURCES
    Engines/DepthFusion/DepthFusionEngine_CPU_VoxelBlockHash.tpp
    Engines/DepthFusion/DepthFusionEngine_CPU_VoxelBlockHash.cpp
    Engines/DepthFusion/DepthFusionEngine_CPU_PlainVoxelArray.tpp
    Engines/DepthFusion/DepthFusionEngine_CPU_PlainVoxelArray.cpp
    Engines/DepthFusion/DepthFusionEngine_CUDA_PlainVoxelArray.tcu
    Engines/DepthFusion/DepthFusionEngine_CUDA_PlainVoxelArray.cu
    Engines/DepthFusion/DepthFusionEngine_CUDA_VoxelBlockHash.tcu
    Engines/DepthFusion/DepthFusionEngine_CUDA_VoxelBlockHash.cu
    )

set(ITMLIB_ENGINES_DEPTH_FUSION_HEADERS
    Engines/DepthFusion/DepthFusionEngineFactory.h
    Engines/DepthFusion/DepthFusionEngine_Shared.h
    Engines/DepthFusion/DepthFusionEngine.h
    Engines/DepthFusion/DepthFusionEngine_CUDA.h
    Engines/DepthFusion/DepthFusionEngine_CUDA_VoxelBlockHash_Kernels.h
    Engines/DepthFusion/DepthFusionEngine_CUDA_PlainVoxelArray_Kernels.h
    Engines/DepthFusion/DepthFusionEngine_CPU.h
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
    Engines/VolumeFileIO/VolumeFileIOEngine.h
    )
set(ITMLIB_ENGINES_SCENEFILEIO_SOURCES
    Engines/VolumeFileIO/VolumeFileIOEngine.tpp
    Engines/VolumeFileIO/VolumeFileIOEngine.cpp
    Engines/VolumeFileIO/VolumeFileIOEngine_PlainVoxelArray.cpp
    Engines/VolumeFileIO/VolumeFileIOEngine_VoxelBlockHash.cpp
    )
## ======================================= SWAPPING ENGINES ============================================================
set(ITMLIB_ENGINES_SWAPPING_HEADERS
    Engines/Swapping/SwappingEngineFactory.h
    )

##
set(ITMLIB_ENGINES_SWAPPING_CPU_SOURCES
    Engines/Swapping/CPU/SwappingEngine_CPU.tpp
    Engines/Swapping/CPU/SwappingEngine_CPU_PlainVoxelArray.cpp
    Engines/Swapping/CPU/SwappingEngine_CPU_VoxelBlockHash.cpp
    )

set(ITMLIB_ENGINES_SWAPPING_CPU_HEADERS
    Engines/Swapping/CPU/SwappingEngine_CPU.h
    )

##
set(ITMLIB_ENGINES_SWAPPING_CUDA_SOURCES
    Engines/Swapping/CUDA/SwappingEngine_CUDA.tcu
    Engines/Swapping/CUDA/SwappingEngine_CUDA_PlainVoxelArray.cu
    Engines/Swapping/CUDA/SwappingEngine_CUDA_VoxelBlockHash.cu
    )

set(ITMLIB_ENGINES_SWAPPING_CUDA_HEADERS
    Engines/Swapping/CUDA/SwappingEngine_CUDA.h
    )

##
set(ITMLIB_ENGINES_SWAPPING_INTERFACE_HEADERS
    Engines/Swapping/Interface/SwappingEngine.h
    )

##
set(ITMLIB_ENGINES_SWAPPING_SHARED_HEADERS
    Engines/Swapping/Shared/SwappingEngine_Shared.h
    )

## ======================================== VIEW-BUILDNG ENGINES =======================================================
set(ITMLIB_ENGINES_VIEWBUILDING_SOURCES
    Engines/ViewBuilding/ViewBuilderFactory.cpp
    )

set(ITMLIB_ENGINES_VIEWBUILDING_HEADERS
    Engines/ViewBuilding/ViewBuilderFactory.h
    )

##
set(ITMLIB_ENGINES_VIEWBUILDING_CPU_SOURCES
    Engines/ViewBuilding/CPU/ViewBuilder_CPU.cpp
    )

set(ITMLIB_ENGINES_VIEWBUILDING_CPU_HEADERS
    Engines/ViewBuilding/CPU/ViewBuilder_CPU.h
    )

##
set(ITMLIB_ENGINES_VIEWBUILDING_CUDA_SOURCES
    Engines/ViewBuilding/CUDA/ViewBuilder_CUDA.cu
    )

set(ITMLIB_ENGINES_VIEWBUILDING_CUDA_HEADERS
    Engines/ViewBuilding/CUDA/ViewBuilder_CUDA.h
    )

##
set(ITMLIB_ENGINES_VIEWBUILDING_INTERFACE_HEADERS
    Engines/ViewBuilding/Interface/ViewBuilder.h
    )

##
set(ITMLIB_ENGINES_VIEWBUILDING_SHARED_HEADERS
    Engines/ViewBuilding/Shared/ViewBuilder_Shared.h
    )
## ======================================== VISUALIATION ENGINES =======================================================
##
set(ITMLIB_ENGINES_Visualization_HEADERS
    Engines/Visualization/MultiVisualizationEngineFactory.h
    Engines/Visualization/SurfelVisualizationEngineFactory.h
    Engines/Visualization/VisualizationEngineFactory.h
    )

##
set(ITMLIB_ENGINES_Visualization_CPU_SOURCES
    Engines/Visualization/CPU/MultiVisualizationEngine_CPU.tpp
    Engines/Visualization/CPU/MultiVisualizationEngine_CPU_PlainVoxelArray.cpp
    Engines/Visualization/CPU/MultiVisualizationEngine_CPU_VoxelBlockHash.cpp
    Engines/Visualization/CPU/SurfelVisualizationEngine_CPU.tpp
    Engines/Visualization/CPU/SurfelVisualizationEngine_CPU.cpp
    Engines/Visualization/CPU/VisualizationEngine_CPU.tpp
    Engines/Visualization/CPU/VisualizationEngine_CPU_VoxelBlockHash.cpp
    Engines/Visualization/CPU/VisualizationEngine_CPU_PlainVoxelArray.cpp
    )

set(ITMLIB_ENGINES_Visualization_CPU_HEADERS
    Engines/Visualization/CPU/MultiVisualizationEngine_CPU.h
    Engines/Visualization/CPU/SurfelVisualizationEngine_CPU.h
    Engines/Visualization/CPU/VisualizationEngine_CPU.h
    )

##
set(ITMLIB_ENGINES_Visualization_CUDA_SOURCES
    Engines/Visualization/CUDA/MultiVisualizationEngine_CUDA.tcu
    Engines/Visualization/CUDA/MultiVisualizationEngine_CUDA_PlainVoxelArray.cu
    Engines/Visualization/CUDA/MultiVisualizationEngine_CUDA_VoxelBlockHash.cu
    Engines/Visualization/CUDA/SurfelVisualizationEngine_CUDA.tcu
    Engines/Visualization/CUDA/SurfelVisualizationEngine_CUDA.cu
    Engines/Visualization/CUDA/VisualizationEngine_CUDA.tcu
    Engines/Visualization/CUDA/VisualizationHelpers_CUDA.cu
    Engines/Visualization/CUDA/VisualizationEngine_CUDA_VoxelBlockHash.cu
    Engines/Visualization/CUDA/VisualizationEngine_CUDA_PlainVoxelArray.cu
    )

set(ITMLIB_ENGINES_Visualization_CUDA_HEADERS
    Engines/Visualization/CUDA/MultiVisualizationEngine_CUDA.h
    Engines/Visualization/CUDA/SurfelVisualizationEngine_CUDA.h
    Engines/Visualization/CUDA/VisualizationEngine_CUDA.h
    Engines/Visualization/CUDA/VisualizationHelpers_CUDA.h
    )

##
set(ITMLIB_ENGINES_Visualization_INTERFACE_SOURCES
    Engines/Visualization/Interface/SurfelVisualizationEngine.tpp
    Engines/Visualization/Interface/SurfelVisualizationEngine.cpp
    Engines/Visualization/Interface/VisualizationEngine.cpp
    )

set(ITMLIB_ENGINES_Visualization_INTERFACE_HEADERS
    Engines/Visualization/Interface/MultiVisualizationEngine.h
    Engines/Visualization/Interface/SurfelVisualizationEngine.h
    Engines/Visualization/Interface/VisualizationEngine.h
    )

##
set(ITMLIB_ENGINES_Visualization_SHARED_HEADERS
    Engines/Visualization/Shared/SurfelVisualizationEngine_Settings.h
    Engines/Visualization/Shared/SurfelVisualizationEngine_Shared.h
    Engines/Visualization/Shared/VisualizationEngine_Shared.h
    )

## ================================================== OBJECTS ==========================================================
set(ITMLIB_OBJECTS_CAMERA_SOURCES
    Objects/Camera/CalibIO.cpp
    )

set(ITMLIB_OBJECTS_CAMERA_HEADERS
    Objects/Camera/CalibIO.h
    Objects/Camera/DisparityCalib.h
    Objects/Camera/Extrinsics.h
    Objects/Camera/Intrinsics.h
    Objects/Camera/RGBDCalib.h
    )

##
set(ITMLIB_OBJECTS_MESHING_HEADERS
    Objects/Meshing/Mesh.h
    )

##
set(ITMLIB_OBJECTS_MISC_HEADERS
    Objects/Misc/IMUCalibrator.h
    Objects/Misc/IMUMeasurement.h
    Objects/Misc/PointCloud.h
    )

##
set(ITMLIB_OBJECTS_RENDERSTATES_SOURCES
    Objects/RenderStates/SurfelRenderState.cpp
    )

##
set(ITMLIB_OBJECTS_RENDERSTATES_HEADERS
    Objects/RenderStates/RenderState.h
    Objects/RenderStates/RenderStateMultiScene.h
    Objects/RenderStates/SurfelRenderState.h
    )

##
set(ITMLIB_OBJECTS_SCENE_SOURCES
    Objects/Scene/VoxelVolume.tpp
    Objects/Scene/VoxelVolume.cpp
    Objects/Scene/LocalVBA.tpp
    Objects/Scene/LocalVBA.cpp
    Objects/Scene/VoxelBlockHash.cpp
    )

##
set(ITMLIB_OBJECTS_SCENE_HEADERS
    Objects/Scene/GlobalCache.h
    Objects/Scene/LocalMap.h
    Objects/Scene/LocalVBA.h
    Objects/Scene/MultiSceneAccess.h
    Objects/Scene/PlainVoxelArray.h
    Objects/Scene/RepresentationAccess.h
    Objects/Scene/TrilinearInterpolation.h
    Objects/Scene/TrilinearDistribution.h
    Objects/Scene/VoxelVolume.h
    Objects/Scene/SurfelScene.h
    Objects/Scene/SurfelTypes.h
    Objects/Scene/VoxelBlockHash.h
    Objects/Scene/VoxelTypes.h
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
    CameraTrackers/CameraTrackerFactory.h
    )
##
set(ITMLIB_CAMERA_TRACKERS_CPU_SOURCES
    CameraTrackers/CPU/ColorTracker_CPU.cpp
    CameraTrackers/CPU/DepthTracker_CPU.cpp
    CameraTrackers/CPU/ExtendedTracker_CPU.cpp
    CameraTrackers/CPU/SDF2SDFCameraTracker_CPU.cpp
    )

set(ITMLIB_CAMERA_TRACKERS_CPU_HEADERS
    CameraTrackers/CPU/ColorTracker_CPU.h
    CameraTrackers/CPU/DepthTracker_CPU.h
    CameraTrackers/CPU/ExtendedTracker_CPU.h
    CameraTrackers/CPU/SDF2SDFCameraTracker_CPU.h
    )

##
set(ITMLIB_CAMERA_TRACKERS_CUDA_SOURCES
    CameraTrackers/CUDA/ColorTracker_CUDA.cu
    CameraTrackers/CUDA/DepthTracker_CUDA.cu
    CameraTrackers/CUDA/ExtendedTracker_CUDA.cu
    CameraTrackers/CUDA/DynamicCameraTracker_CUDA.cu
    )

set(ITMLIB_CAMERA_TRACKERS_CUDA_HEADERS
    CameraTrackers/CUDA/ColorTracker_CUDA.h
    CameraTrackers/CUDA/DepthTracker_CUDA.h
    CameraTrackers/CUDA/ExtendedTracker_CUDA.h
    CameraTrackers/CUDA/DynamicCameraTracker_CUDA.h
    )

##
set(ITMLIB_CAMERA_TRACKERS_INTERFACE_SOURCES
    CameraTrackers/Interface/ColorTracker.cpp
    CameraTrackers/Interface/DepthTracker.cpp
    CameraTrackers/Interface/ExtendedTracker.cpp
    CameraTrackers/Interface/FileBasedTracker.cpp
    CameraTrackers/Interface/ForceFailTracker.cpp
    CameraTrackers/Interface/IMUTracker.cpp
    CameraTrackers/Interface/DynamicCameraTracker.cpp
    )

set(ITMLIB_CAMERA_TRACKERS_INTERFACE_HEADERS
    CameraTrackers/Interface/ColorTracker.h
    CameraTrackers/Interface/CompositeTracker.h
    CameraTrackers/Interface/DepthTracker.h
    CameraTrackers/Interface/ExtendedTracker.h
    CameraTrackers/Interface/FileBasedTracker.h
    CameraTrackers/Interface/ForceFailTracker.h
    CameraTrackers/Interface/IMUTracker.h
    CameraTrackers/Interface/CameraTracker.h
    CameraTrackers/Interface/DynamicCameraTracker.h
    )

##
set(ITMLIB_CAMERA_TRACKERS_SHARED_HEADERS
    CameraTrackers/Shared/ColorTracker_Shared.h
    CameraTrackers/Shared/DepthTracker_Shared.h
    CameraTrackers/Shared/ExtendedTracker_Shared.h
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
    ${ITMLIB_ENGINES_DEPTH_FUSION_SOURCES}
    ${ITMLIB_ENGINES_VOLUME_FUSION_SOURCES}
    ${ITMLIB_ENGINES_WARP_VOLUME_SOURCES}
    ${ITMLIB_ENGINES_SCENEFILEIO_SOURCES}
    ${ITMLIB_ENGINES_SWAPPING_CPU_SOURCES}
    ${ITMLIB_ENGINES_SWAPPING_CUDA_SOURCES}
    ${ITMLIB_ENGINES_VIEWBUILDING_SOURCES}
    ${ITMLIB_ENGINES_VIEWBUILDING_CPU_SOURCES}
    ${ITMLIB_ENGINES_VIEWBUILDING_CUDA_SOURCES}
    ${ITMLIB_ENGINES_Visualization_SOURCES}
    ${ITMLIB_ENGINES_Visualization_CPU_SOURCES}
    ${ITMLIB_ENGINES_Visualization_CUDA_SOURCES}
    ${ITMLIB_ENGINES_Visualization_INTERFACE_SOURCES}
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
    ${ITMLIB_ENGINES_DEPTH_FUSION_SOURCES}
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
    ${ITMLIB_ENGINES_Visualization_HEADERS}
    ${ITMLIB_ENGINES_Visualization_CPU_HEADERS}
    ${ITMLIB_ENGINES_Visualization_CUDA_HEADERS}
    ${ITMLIB_ENGINES_Visualization_INTERFACE_HEADERS}
    ${ITMLIB_ENGINES_Visualization_SHARED_HEADERS}
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