# CMakeLists.txt for ITMLib #
#############################

set(ITMLIB_TOPLEVEL_HEADERS
    GlobalTemplateDefines.h
    )

##
set(ITMLIB_ENGINES_MAIN_SOURCES
    Engines/Main/Mappers/DenseMapper.tpp
    Engines/Main/Mappers/DenseMapper_PlainVoxelArray.cpp
    Engines/Main/Mappers/DenseMapper_VoxelBlockHash.cpp
    Engines/Main/Mappers/DenseSurfelMapper.tpp
    Engines/Main/Mappers/DenseSurfelMapper.cpp

    Engines/Main/BasicVoxelEngine.tpp
    Engines/Main/BasicVoxelEngine_PlainVoxelArray.cpp
    Engines/Main/BasicVoxelEngine_VoxelBlockHash.cpp

    Engines/Main/BasicSurfelEngine.tpp
    Engines/Main/BasicSufelEngine.cpp

    Engines/Main/MainEngineSettings.cpp
    Engines/Main/MultiEngine.tpp
    Engines/Main/MultiEngine_PlainVoxelArray.cpp
    Engines/Main/MultiEngine_VoxelBlockHash.cpp

    Engines/Main/DynamicSceneVoxelEngine.tpp
    Engines/Main/DynamicSceneVoxelEngine_PlainVoxelArray.cpp
    Engines/Main/DynamicSceneVoxelEngine_VoxelBlockHash.cpp
    )

set(ITMLIB_ENGINES_MAIN_HEADERS
    Engines/Main/Mappers/DenseMapper.h
    Engines/Main/Mappers/DenseSurfelMapper.h

    Engines/Main/BasicVoxelEngine.h
    Engines/Main/BasicSurfelEngine.h

    Engines/Main/FusionAlgorithm.h
    Engines/Main/MainEngineSettings.h
    Engines/Main/MultiEngine.h
    Engines/Main/CameraTrackingController.h
    Engines/Main/DynamicSceneVoxelEngine.h
    Engines/Main/MainEngineFactory.h
    )

#======================================= COMMON TO MULTIPLE ENGINES ====================================================
set(ITMLIB_ENGINES_COMMON_HEADERS
    Engines/Common/AllocationTempData.h
    Engines/Common/Configurable.h
    Engines/Common/WarpAccessFunctors.h
    )
#======================================= LOW-LEVEL ENGINES =============================================================
##
set(
        ITMLIB_ENGINES_LOW_LEVEL_SOURCES

        Engines/ImageProcessing/ImageProcessingEngineFactory.cpp

        Engines/ImageProcessing/CPU/LowLevelEngine_CPU.cpp
        Engines/ImageProcessing/CUDA/LowLevelEngine_CUDA.cu
)

set(
        ITMLIB_ENGINES_LOW_LEVEL_HEADERS

        Engines/ImageProcessing/ImageProcessingEngineFactory.h

        Engines/ImageProcessing/Interface/ImageProcessingEngineInterface.h

        Engines/ImageProcessing/Shared/LowLevelEngine_Shared.h

        Engines/ImageProcessing/CPU/LowLevelEngine_CPU.h
        Engines/ImageProcessing/CUDA/LowLevelEngine_CUDA.h
)
#====================================== TRAVERSAL ENGINES ======================================================

set(
        ITMLIB_ENGINES_TRAVERSAL_HEADERS

        ## Interface
        Engines/Traversal/Interface/HashTableTraversal.h
        Engines/Traversal/Interface/ImageTraversal.h
        Engines/Traversal/Interface/TwoImageTraversal.h
        Engines/Traversal/Interface/MemoryBlockTraversal.h
        Engines/Traversal/Interface/RawArrayTraversal.h
        Engines/Traversal/Interface/Regular2DSubGridArrayTraversal.h
        Engines/Traversal/Interface/VolumeTraversal.h
        Engines/Traversal/Interface/TwoVolumeTraversal.h
        Engines/Traversal/Interface/ThreeVolumeTraversal.h

        ## Shared
        Engines/Traversal/Shared/ContiguousCollectionTraversal_Shared.h
        Utils/CUDA/CudaCallWrappers.cuh
        Engines/Traversal/Shared/JobCountPolicy.h
        Engines/Traversal/Shared/TraversalMethod.h
        Engines/Traversal/Shared/VolumeTraversal_Shared.h

        ## CPU
        Engines/Traversal/CPU/ImageTraversal_CPU.h
        Engines/Traversal/CPU/TwoImageTraversal_CPU.h
        Engines/Traversal/CPU/MemoryBlockTraversal_CPU.h
        Engines/Traversal/CPU/RawArrayTraversal_CPU.h
        Engines/Traversal/CPU/Regular2DSubGridArrayTraversal_CPU.h
        Engines/Traversal/CPU/HashTableTraversal_CPU.h

        Engines/Traversal/CPU/VolumeTraversal_CPU_PlainVoxelArray.h
        Engines/Traversal/CPU/VolumeTraversal_CPU_VoxelBlockHash.h

        Engines/Traversal/CPU/TwoVolumeTraversal_CPU_PlainVoxelArray.h
        Engines/Traversal/CPU/TwoVolumeTraversal_CPU_VoxelBlockHash.h
        Engines/Traversal/CPU/TwoVolumeTraversal_CPU_PVA_to_VBH.h

        Engines/Traversal/CPU/ThreeVolumeTraversal_CPU_PlainVoxelArray.h
        Engines/Traversal/CPU/ThreeVolumeTraversal_CPU_VoxelBlockHash.h

        ## CUDA
        Engines/Traversal/CUDA/ImageTraversal_CUDA.cuh
        Engines/Traversal/CUDA/ImageTraversal_CUDA_Kernels.cuh

        Engines/Traversal/CUDA/TwoImageTraversal_CUDA.h
        Engines/Traversal/CUDA/TwoImageTraversal_CUDA_Kernels.h

        Engines/Traversal/CUDA/MemoryBlockTraversal_CUDA.cuh

        Engines/Traversal/CUDA/RawArrayTraversal_CUDA.cuh
        Engines/Traversal/CUDA/RawArrayTraversal_CUDA_Kernels.cuh

        Engines/Traversal/CUDA/Regular2DSubGridArrayTraversal_CUDA.cuh
        Engines/Traversal/CUDA/Regular2DSubGridArrayTraversal_CUDA_Kernels.cuh

        Engines/Traversal/CUDA/HashTableTraversal_CUDA.cuh
        Engines/Traversal/CUDA/HashTableTraversal_CUDA_Kernels.cuh

        Engines/Traversal/CUDA/VolumeTraversal_CUDA_PlainVoxelArray.h
        Engines/Traversal/CUDA/VolumeTraversal_CUDA_PlainVoxelArray_Kernels.h
        Engines/Traversal/CUDA/VolumeTraversal_CUDA_VoxelBlockHash.h
        Engines/Traversal/CUDA/VolumeTraversal_CUDA_VoxelBlockHash_Kernels.h

        Engines/Traversal/CUDA/TwoVolumeTraversal_CUDA_PlainVoxelArray.h
        Engines/Traversal/CUDA/TwoVolumeTraversal_CUDA_PlainVoxelArray_Kernels.h
        Engines/Traversal/CUDA/TwoVolumeTraversal_CUDA_VoxelBlockHash.h
        Engines/Traversal/CUDA/TwoVolumeTraversal_CUDA_VoxelBlockHash_Kernels.h
        Engines/Traversal/CUDA/TwoVolumeTraversal_CUDA_PVA_to_VBH.h
        Engines/Traversal/CUDA/TwoVolumeTraversal_CUDA_PVA_to_VBH_Kernels.h

        Engines/Traversal/CUDA/ThreeVolumeTraversal_CUDA_PlainVoxelArray.h
        Engines/Traversal/CUDA/ThreeVolumeTraversal_CUDA_PlainVoxelArray_Kernels.h
        Engines/Traversal/CUDA/ThreeVolumeTraversal_CUDA_VoxelBlockHash.h
        Engines/Traversal/CUDA/ThreeVolumeTraversal_CUDA_VoxelBlockHash_Kernels.h
        ##
)

#====================================== REDUCTION ENGINES ======================================================
set(
        ITMLIB_ENGINES_REDUCTION_HEADERS

        ## Interface
        Engines/Reduction/Interface/VolumeReduction.h
        Engines/Reduction/Interface/BlockVolumeReduction.h

        ## Shared
        Engines/Reduction/Shared/ReductionResult.h

        ## CPU
        Engines/Reduction/CPU/VolumeReduction_CPU_PlainVoxelArray.h
        Engines/Reduction/CPU/VolumeReduction_CPU_VoxelBlockHash.h

        ## CUDA
        Engines/Reduction/CUDA/VolumeReduction_CUDA_PlainVoxelArray.h
        Engines/Reduction/CUDA/VolumeReduction_CUDA_PlainVoxelArray_Kernels.h
        Engines/Reduction/CUDA/VolumeReduction_CUDA_VoxelBlockHash.h
        Engines/Reduction/CUDA/VolumeReduction_CUDA_VoxelBlockHash_Kernels.h
)


#====================================== EDIT-AND-COPY ENGINES =====================================================
set(
        ITMLIB_ENGINES_EDIT_AND_COPY_HEADERS

        ## Upper-level
        Engines/EditAndCopy/EditAndCopyEngineFactory.h

        ## Interface
        Engines/EditAndCopy/Interface/EditAndCopyEngineInterface.h

        ## Shared
        Engines/EditAndCopy/Shared/EditAndCopyEngine_Shared.h
        Engines/EditAndCopy/Shared/EditAndCopyEngine_Functors.h

        # CPU
        Engines/EditAndCopy/CPU/EditAndCopyEngine_CPU.h

        # CUDA
        Engines/EditAndCopy/CUDA/EditAndCopyEngine_CUDA.h
        Engines/EditAndCopy/CUDA/EditAndCopyEngine_CUDA_VoxelBlockHash_Kernels.h
        Engines/EditAndCopy/CUDA/EditAndCopyEngine_CUDA_PlainVoxelArray_Kernels.h
)

set(
        ITMLIB_ENGINES_EDIT_AND_COPY_SOURCES

        ## CPU
        Engines/EditAndCopy/CPU/EditAndCopyEngine_CPU.cpp
        Engines/EditAndCopy/CPU/EditAndCopyEngine_CPU_VoxelBlockHash.tpp
        Engines/EditAndCopy/CPU/EditAndCopyEngine_CPU_PlainVoxelArray.tpp
        Engines/EditAndCopy/CPU/EditAndCopyEngine_CPU_VoxelBlockHash.cpp
        Engines/EditAndCopy/CPU/EditAndCopyEngine_CPU_PlainVoxelArray.cpp

        ## CUDA
        Engines/EditAndCopy/CUDA/EditAndCopyEngine_CUDA_VoxelBlockHash.tcu
        Engines/EditAndCopy/CUDA/EditAndCopyEngine_CUDA_PlainVoxelArray.tcu
        Engines/EditAndCopy/CUDA/EditAndCopyEngine_CUDA_VoxelBlockHash.cu
        Engines/EditAndCopy/CUDA/EditAndCopyEngine_CUDA_PlainVoxelArray.cu
)

#====================================== MESHING ENGINES ================================================================
##
set(
        ITMLIB_ENGINES_MESHING_HEADERS

        # Top-level
        Engines/Meshing/MeshingEngineFactory.h
        Engines/Meshing/MultiMeshingEngineFactory.h
        # Interface
        Engines/Meshing/Interface/MeshingEngine.h
        Engines/Meshing/Interface/MultiMeshingEngine.h
        # Shared
        Engines/Meshing/Shared/MeshingEngine_Shared.h
        Engines/Meshing/Shared/MultiMeshingEngine_Shared.h
        # CPU
        Engines/Meshing/CPU/MeshingEngine_CPU.h
        Engines/Meshing/CPU/MultiMeshingEngine_CPU.h
        # CUDA
        Engines/Meshing/CUDA/MeshingEngine_CUDA.h
        Engines/Meshing/CUDA/MultiMeshingEngine_CUDA.h
        # Multi-scene
        Engines/MultiScene/ActiveMapManager.h
        Engines/MultiScene/GlobalAdjustmentEngine.h
        Engines/MultiScene/VoxelMapGraphManager.h
)

##
set(
        ITMLIB_ENGINES_MESHING_SOURCES

        ## CPU
        Engines/Meshing/CPU/MeshingEngine_CPU.tpp
        Engines/Meshing/CPU/MeshingEngine_CPU_PlainVoxelArray.cpp
        Engines/Meshing/CPU/MeshingEngine_CPU_VoxelBlockHash.cpp
        Engines/Meshing/CPU/MultiMeshingEngine_CPU_PlainVoxelArray.cpp
        Engines/Meshing/CPU/MultiMeshingEngine_CPU_VoxelBlockHash.cpp
        Engines/Meshing/CPU/MultiMeshingEngine_CPU.tpp

        ## CUDA
        Engines/Meshing/CUDA/MeshingEngine_CUDA.tcu
        Engines/Meshing/CUDA/MeshingEngine_CUDA_PlainVoxelArray.cu
        Engines/Meshing/CUDA/MeshingEngine_CUDA_VoxelBlockHash.cu
        Engines/Meshing/CUDA/MultiMeshingEngine_CUDA.tcu
        Engines/Meshing/CUDA/MultiMeshingEngine_CUDA_PlainVoxelArray.cu
        Engines/Meshing/CUDA/MultiMeshingEngine_CUDA_VoxelBlockHash.cu

        # Multi-Scene
        Engines/MultiScene/ActiveMapManager.cpp
        Engines/MultiScene/GlobalAdjustmentEngine.cpp
        Engines/MultiScene/VoxelMapGraphManager.tpp
        Engines/MultiScene/VoxelMapGraphManager_PlainVoxelArray.cpp
        Engines/MultiScene/VoxelMapGraphManager_VoxelBlockHash.cpp
)

#====================================== INDEXING ENGINES ===============================================================
set(
        ITMLIB_ENGINES_INDEXING_HEADERS
        ## Top-level
        Engines/Indexing/IndexingEngineFactory.h
        Engines/Indexing/IndexingSettings.h

        ## Shared
        Engines/Indexing/Shared/IndexingEngine_DiagnosticData.h
        Engines/Indexing/Shared/IndexingEngine_Shared.h
        Engines/Indexing/Shared/IndexingEngine_RayMarching.h
        Engines/Indexing/Shared/IndexingEngine_DiagnosticRayMarching.h
        Engines/Indexing/Shared/IndexingEngine_Functors.h
        Engines/Indexing/Shared/IndexingEngine_ConcurrentBlockManagement.h

        ## PVA
        Engines/Indexing/PVA/IndexingEngine_PlainVoxelArray.h

        ## VBH
        Engines/Indexing/VBH/IndexingEngine_VoxelBlockHash.h
        Engines/Indexing/VBH/IndexingEngine_VoxelBlockHash_Internal.h
        Engines/Indexing/VBH/CPU/IndexingEngine_VoxelBlockHash_CPU.h
        Engines/Indexing/VBH/CUDA/IndexingEngine_VoxelBlockHash_CUDA_Kernels.h
        Engines/Indexing/VBH/IndexingEngine_VoxelBlockHash_DIAGNOSTIC.h
        Engines/Indexing/VBH/IndexingEngine_VoxelBlockHash_OPTIMIZED.h
)

set(
        ITMLIB_ENGINES_INDEXING_SOURCES

        ## Top-level
        Engines/Indexing/IndexingSettings.cpp
        ## Interface
        Engines/Indexing/Interface/IndexingEngine.h
        ## PVA
        Engines/Indexing/PVA/IndexingEngine_CUDA_PlainVoxelArray.cu
        Engines/Indexing/PVA/IndexingEngine_CPU_PlainVoxelArray.cpp
        Engines/Indexing/PVA/IndexingEngine_PlainVoxelArray.tpp
        ## VBH
        Engines/Indexing/VBH/IndexingEngine_VoxelBlockHash.tpp
        Engines/Indexing/VBH/IndexingEngine_VoxelBlockHash_CPU.cpp
        Engines/Indexing/VBH/IndexingEngine_VoxelBlockHash_CUDA.cu
        ## CPU/CUDA VBH
        Engines/Indexing/VBH/CPU/IndexingEngine_VoxelBlockHash_CPU.tpp
        Engines/Indexing/VBH/CUDA/IndexingEngine_VoxelBlockHash_CUDA.tcu
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
    Engines/DepthFusion/DepthFusionEngine.tpp
    Engines/DepthFusion/DepthFusionEngine_CPU_VoxelBlockHash.cpp
    Engines/DepthFusion/DepthFusionEngine_CPU_PlainVoxelArray.cpp
    Engines/DepthFusion/DepthFusionEngine_CUDA_PlainVoxelArray.cu
    Engines/DepthFusion/DepthFusionEngine_CUDA_VoxelBlockHash.cu
    Engines/DepthFusion/DepthFusionSettings.cpp)

set(ITMLIB_ENGINES_DEPTH_FUSION_HEADERS
    Engines/DepthFusion/DepthFusionEngineFactory.h
    Engines/DepthFusion/DepthFusionEngine_Shared.h
    Engines/DepthFusion/DepthFusionEngine.h
    Engines/DepthFusion/DepthFusionSettings.h)

## ======================================= VOLUME FUSION ENGINE ========================================================
set(ITMLIB_ENGINES_VOLUME_FUSION_SOURCES
    Engines/VolumeFusion/VolumeFusionEngine.tpp
    Engines/VolumeFusion/VolumeFusionEngine_CPU_PlainVoxelArray.cpp
    Engines/VolumeFusion/VolumeFusionEngine_CPU_VoxelBlockHash.cpp
    Engines/VolumeFusion/VolumeFusionEngine_CUDA_PlainVoxelArray.cu
    Engines/VolumeFusion/VolumeFusionEngine_CUDA_VoxelBlockHash.cu
    Engines/VolumeFusion/VolumeFusionSettings.cpp
    )

set(ITMLIB_ENGINES_VOLUME_FUSION_HEADERS
    Engines/VolumeFusion/VolumeFusionEngine.h
    Engines/VolumeFusion/VolumeFusionFunctors.h
    Engines/VolumeFusion/VolumeFusionEngineFactory.h
    Engines/VolumeFusion/VolumeFusionSettings.h
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
set(ITMLIB_ENGINES_VOLUME_FILE_IO_HEADERS
    Engines/VolumeFileIO/VolumeFileIOEngine.h
    )
set(ITMLIB_ENGINES_VOLUME_FILE_IO_SOURCES
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

set(
        ITMLIB_ENGINES_VIEW_BUILDER_HEADERS
        ## Top-level
        Engines/ViewBuilder/ViewBuilderFactory.h
        ## Interface
        Engines/ViewBuilder/Interface/ViewBuilder.h
        ## Shared
        Engines/ViewBuilder/Shared/ViewBuilder_Shared.h
        ## CPU
        Engines/ViewBuilder/CPU/ViewBuilder_CPU.h
        ## CUDA
        Engines/ViewBuilder/CUDA/ViewBuilder_CUDA.h

)

set(
        ITMLIB_ENGINES_VIEW_BUILDER_SOURCES

        ## Top-level
        Engines/ViewBuilder/ViewBuilderFactory.cpp
        ## CPU
        Engines/ViewBuilder/CPU/ViewBuilder_CPU.cpp
        ## CUDA
        Engines/ViewBuilder/CUDA/ViewBuilder_CUDA.cu
)

## ======================================== VISUALIZATION ENGINES ======================================================
##
set(
        ITMLIB_ENGINES_RENDERING_HEADERS

        ## Top-level
        Engines/Rendering/RenderingEngine.h
        Engines/Rendering/RenderingEngine_Specialized.h
        Engines/Rendering/RenderingEngineFactory.h
        Engines/Rendering/RenderingSettings.h
        Engines/Rendering/MultiVisualizationEngineFactory.h
        Engines/Rendering/SurfelVisualizationEngineFactory.h
        ## Interface
        Engines/Rendering/Interface/MultiVisualizationEngine.h
        Engines/Rendering/Interface/SurfelVisualizationEngine.h
        Engines/Rendering/Interface/RenderingEngineInterface.h
        ## Shared
        Engines/Rendering/Shared/RenderingBlock.h
        Engines/Rendering/Shared/RenderingEngine_Functors.h
        Engines/Rendering/Shared/SurfelVisualizationEngine_Settings.h
        Engines/Rendering/Shared/SurfelVisualizationEngine_Shared.h
        Engines/Rendering/Shared/RenderingEngine_Shared.h
        ## CPU
        Engines/Rendering/CPU/MultiVisualizationEngine_CPU.h
        Engines/Rendering/CPU/SurfelVisualizationEngine_CPU.h
        ## CUDA
        Engines/Rendering/CUDA/MultiVisualizationEngine_CUDA.h
        Engines/Rendering/CUDA/SurfelVisualizationEngine_CUDA.h
        Engines/Rendering/CUDA/VisualizationHelpers_CUDA.h
)

##
set(
        ITMLIB_ENGINES_RENDERING_SOURCES

        ## Top-level
        Engines/Rendering/RenderingEngine.tpp
        Engines/Rendering/RenderingEngine.cpp
        Engines/Rendering/RenderingEngine.cu
        Engines/Rendering/RenderingEngine_Specialized.tpp
        Engines/Rendering/RenderingSettings.cpp
        ## Interface
        Engines/Rendering/Interface/SurfelVisualizationEngine.tpp
        Engines/Rendering/Interface/SurfelVisualizationEngine.cpp
        Engines/Rendering/Interface/IRenderingEngine.cpp
        ## CPU
        Engines/Rendering/CPU/MultiVisualizationEngine_CPU.tpp
        Engines/Rendering/CPU/MultiVisualizationEngine_CPU_PlainVoxelArray.cpp
        Engines/Rendering/CPU/MultiVisualizationEngine_CPU_VoxelBlockHash.cpp
        Engines/Rendering/CPU/SurfelVisualizationEngine_CPU.tpp
        Engines/Rendering/CPU/SurfelVisualizationEngine_CPU.cpp
        ## CUDA
        Engines/Rendering/CUDA/MultiVisualizationEngine_CUDA.tcu
        Engines/Rendering/CUDA/MultiVisualizationEngine_CUDA_PlainVoxelArray.cu
        Engines/Rendering/CUDA/MultiVisualizationEngine_CUDA_VoxelBlockHash.cu
        Engines/Rendering/CUDA/SurfelVisualizationEngine_CUDA.tcu
        Engines/Rendering/CUDA/SurfelVisualizationEngine_CUDA.cu
        Engines/Rendering/CUDA/VisualizationHelpers_CUDA.cu
)


## ================================================ ANALYTICS ENGINES ==================================================

set(
        ITMLIB_ENGINES_ANALYTICS_HEADERS

        Engines/Analytics/AnalyticsEngineFactory.h
        Engines/Analytics/AnalyticsEngine.h
        Engines/Analytics/AnalyticsEngineInterface.h
        Engines/Analytics/AnalyticsEngine_Functors.h
        Engines/Analytics/AnalyticsLogging.h
)

set(
        ITMLIB_ENGINES_ANALYTICS_SOURCES

        Engines/Analytics/AnalyticsEngine.tpp
        Engines/Analytics/AnalyticsEngine_CPU_PlainVoxelArray.cpp
        Engines/Analytics/AnalyticsEngine_CPU_VoxelBlockHash.cpp
        Engines/Analytics/AnalyticsEngine_CUDA_PlainVoxelArray.cu
        Engines/Analytics/AnalyticsEngine_CUDA_VoxelBlockHash.cu
        Engines/Analytics/AnalyticsLogging.cpp
)

## ================================================== TELEMETRY ENGINES ================================================

set(
        ITMLIB_ENGINES_TELEMETRY_HEADERS

        Engines/Telemetry/TelemetryRecorder.h
        Engines/Telemetry/TelemetryRecorderFactory.h
)
set(
        ITMLIB_ENGINES_TELEMETRY_SOURCES

        ## Top-level
        Engines/Telemetry/TelemetryRecorder.tpp
        ## CPU
        Engines/Telemetry/CPU/TelemetryRecorder_CPU_PlainVoxelArray.cpp
        Engines/Telemetry/CPU/TelemetryRecorder_CPU_VoxelBlockHash.cpp
        ## CUDA
        Engines/Telemetry/CUDA/TelemetryRecorder_CUDA_PlainVoxelArray.cu
        Engines/Telemetry/CUDA/TelemetryRecorder_CUDA_VoxelBlockHash.cu
)


# ============================================== LEVEL SET ALIGNMENT ENGINES =================================================
set(
        ITMLIB_ENGINES_LEVEL_SET_ALIGNMENT_HEADERS

        ## Top-level
        Engines/LevelSetAlignment/LevelSetAlignmentEngineFactory.h
        ## Interface
        Engines/LevelSetAlignment/Interface/LevelSetAlignmentEngineInterface.h
        Engines/LevelSetAlignment/Interface/LevelSetAlignmentParameters.h
        Engines/LevelSetAlignment/Interface/LevelSetAlignmentEngine.h
        ## Functors
        Engines/LevelSetAlignment/Functors/WarpGradientFunctor.h
        Engines/LevelSetAlignment/Functors/WarpGradientFunctor_Diagnostic.h
        Engines/LevelSetAlignment/Functors/WarpGradientFunctor_Optimized.h
        ## Shared
        Engines/LevelSetAlignment/Shared/WarpHessian.h
        Engines/LevelSetAlignment/Shared/WarpGradientCommon.h
        Engines/LevelSetAlignment/Shared/LevelSetAlignmentSharedRoutines.h
        Engines/LevelSetAlignment/Shared/LevelSetAlignmentDiagnosticRoutines.h
        Engines/LevelSetAlignment/Shared/LevelSetAlignmentSharedFunctors.h
        Engines/LevelSetAlignment/Shared/WarpGradientRoutines.h
        Engines/LevelSetAlignment/Shared/WarpGradientAggregates.h
        Engines/LevelSetAlignment/Shared/DataTermSharedRoutines.h
        Engines/LevelSetAlignment/Shared/LevelSetTermSharedRoutines.h
        Engines/LevelSetAlignment/Shared/SmoothingTermSharedRoutines.h
        Engines/LevelSetAlignment/Shared/SdfGradientSharedRoutines.h Engines/LevelSetAlignment/Functors/WarpStatisticFunctors.h)

set(
        ITMLIB_ENGINES_LEVEL_SET_ALIGNMENT_SOURCES

        ## Interface
        Engines/LevelSetAlignment/Interface/LevelSetAlignmentEngine.tpp
        Engines/LevelSetAlignment/Interface/LevelSetAlignmentParameters.cpp
        ## CPU
        Engines/LevelSetAlignment/CPU/LevelSetAlignmentEngine_CPU_PlainVoxelArray.cpp
        Engines/LevelSetAlignment/CPU/LevelSetAlignmentEngine_CPU_VoxelBlockHash.cpp
        ## CUDA
        Engines/LevelSetAlignment/CUDA/LevelSetAlignmentEngine_CUDA_PlainVoxelArray.cu
        Engines/LevelSetAlignment/CUDA/LevelSetAlignmentEngine_CUDA_VoxelBlockHash.cu
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
    Objects/Camera/RGBD_CalibrationInformation.h
    )

##
set(ITMLIB_OBJECTS_MESHING_HEADERS
    Objects/Meshing/Mesh.h
    )

set(ITMLIB_OBJECTS_MESHING_SOURCES
    Objects/Meshing/Mesh.cpp
    )

##
set(ITMLIB_OBJECTS_MISC_HEADERS
    Objects/Misc/IMUCalibrator.h
    Objects/Misc/IMUMeasurement.h
    Objects/Misc/PointCloud.h
    )

##
set(ITMLIB_OBJECTS_RENDER_STATE_SOURCES
    Objects/RenderStates/SurfelRenderState.cpp
    )

##
set(ITMLIB_OBJECTS_RENDER_STATE_HEADERS
    Objects/RenderStates/RenderState.h
    Objects/RenderStates/RenderStateMultiScene.h
    Objects/RenderStates/SurfelRenderState.h
    )

##
set(ITMLIB_OBJECTS_VOLUME_SOURCES
    Objects/Volume/GlobalCache.tpp
    Objects/Volume/GlobalCache_PlainVoxelArray.cpp
    Objects/Volume/GlobalCache_VoxelBlockHash.cpp
    Objects/Volume/VoxelVolume.tpp
    Objects/Volume/VoxelVolume.cpp
    Objects/Volume/VoxelBlockHash.cpp
    Objects/Volume/PlainVoxelArray.cpp
    )

##
set(ITMLIB_OBJECTS_VOLUME_HEADERS
    Objects/Volume/GlobalCache.h
    Objects/Volume/LocalMap.h
    Objects/Volume/MultiSceneAccess.h
    Objects/Volume/PlainVoxelArray.h
    Objects/Volume/RepresentationAccess.h
    Objects/Volume/TrilinearInterpolation.h
    Objects/Volume/TrilinearDistribution.h
    Objects/Volume/VoxelVolume.h
    Objects/Volume/SurfelScene.h
    Objects/Volume/SurfelTypes.h
    Objects/Volume/VoxelBlockHash.h
    Objects/Volume/VoxelTypes.h
    )

##
set(ITMLIB_OBJECTS_TRACKING_HEADERS
    Objects/Tracking/DepthHierarchyLevel.h
    Objects/Tracking/ImageHierarchy.h
    Objects/Tracking/IntensityHierarchyLevel.h
    Objects/Tracking/RGBHierarchyLevel.h
    Objects/Tracking/VolumeHierarchyLevel.h
    Objects/Tracking/TemplatedHierarchyLevel.h
    Objects/Tracking/CameraTrackingState.h
    Objects/Tracking/ViewHierarchyLevel.h
    Objects/Tracking/TrackerIterationType.h
    )

##
set(ITMLIB_OBJECTS_VIEWS_HEADERS
    Objects/Views/View.h
    Objects/Views/ViewIMU.h
    )

set(ITMLIB_OBJECTS_VIEWS_SOURCES
    Objects/Views/View.cpp
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
    )

set(ITMLIB_CAMERA_TRACKERS_CPU_HEADERS
    CameraTrackers/CPU/ColorTracker_CPU.h
    CameraTrackers/CPU/DepthTracker_CPU.h
    CameraTrackers/CPU/ExtendedTracker_CPU.h
    )

##
set(ITMLIB_CAMERA_TRACKERS_CUDA_SOURCES
    CameraTrackers/CUDA/ColorTracker_CUDA.cu
    CameraTrackers/CUDA/DepthTracker_CUDA.cu
    CameraTrackers/CUDA/ExtendedTracker_CUDA.cu
    )

set(ITMLIB_CAMERA_TRACKERS_CUDA_HEADERS
    CameraTrackers/CUDA/ColorTracker_CUDA.h
    CameraTrackers/CUDA/DepthTracker_CUDA.h
    CameraTrackers/CUDA/ExtendedTracker_CUDA.h
    )

##
set(ITMLIB_CAMERA_TRACKERS_INTERFACE_SOURCES
    CameraTrackers/Interface/ColorTracker.cpp
    CameraTrackers/Interface/DepthTracker.cpp
    CameraTrackers/Interface/ExtendedTracker.cpp
    CameraTrackers/Interface/FileBasedTracker.cpp
    CameraTrackers/Interface/ForceFailTracker.cpp
    CameraTrackers/Interface/IMUTracker.cpp
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
    )

##
set(ITMLIB_CAMERA_TRACKERS_SHARED_HEADERS
    CameraTrackers/Shared/ColorTracker_Shared.h
    CameraTrackers/Shared/DepthTracker_Shared.h
    CameraTrackers/Shared/ExtendedTracker_Shared.h
    )

# =================================================== UTILITIES (MISC) =================================================
set(
        ITMLIB_UTILS_SOURCES

        Utils/Analytics/VoxelVolumeComparison/VoxelVolumeComparison_CPU.tpp
        Utils/Analytics/VoxelVolumeComparison/VoxelVolumeComparison_CPU.cpp
        Utils/Analytics/VoxelVolumeComparison/VoxelVolumeComparison_CUDA.tcu
        Utils/Analytics/VoxelVolumeComparison/VoxelVolumeComparison_CUDA.cu
        Utils/Analytics/NeighborVoxelIterationInfo.cpp
        Utils/Analytics/RawArrayComparison.cpp
        Utils/Analytics/RawArrayComparison.cu
        Utils/Analytics/BenchmarkUtilities.cpp

        Utils/Collections/NestedMap3DOfArrays.tpp
        Utils/Collections/NestedMap3DOfArrays.cpp
        Utils/Collections/NestedMap3D.tpp
        Utils/Collections/NestedMap3D.cpp
        Utils/Collections/OperationsOnSTLContainers.cpp

        Utils/Enums/IndexingMethod.cpp
        Utils/Enums/ExecutionMode.cpp

        Utils/Configuration/AutomaticRunSettings.cpp
        Utils/Configuration/Configuration.cpp
        Utils/Configuration/LoggingSettings.cpp
        Engines/Telemetry/TelemetrySettings.cpp

        Utils/FileIO/JSON_Utilities.cpp
        Utils/FileIO/CSV_Utilities.cpp
        Utils/FileIO/RecordHandling.cpp

        Utils/Geometry/CardinalAxesAndPlanes.cpp
        Utils/Geometry/Rodrigues.cpp

        Utils/Logging/LoggingUtilities.cpp
        Utils/Logging/Logging.cpp

        Utils/Metacoding/SerializableStruct_Impl.cpp
        Utils/Metacoding/DeferrableSerializableStruct_Impl.cpp
        Utils/Quaternions/QuaternionFromMatrix.cpp

        Utils/Telemetry/TelemetryUtilities.cpp

        Utils/Math.cpp

        Utils/VoxelVolumeParameters.cpp
        Utils/Analytics/Histogram.cpp)

set(
        ITMLIB_UTILS_HEADERS

        Utils/Analytics/BenchmarkUtilities.h
        Utils/Analytics/AlmostEqual.h
        Utils/Analytics/IsAltered.h
        Utils/Analytics/NeighborVoxelIterationInfo.h
        Utils/Analytics/RawArrayComparison.h
        Utils/Analytics/RawArrayComparison.cuh
        Utils/Analytics/RawArrayComparisonTemplateInstantiations.h
        Utils/Analytics/Statistics.h

        Utils/Collections/NestedMap3DOfArrays.h
        Utils/Collections/NestedMap3D.h
        Utils/Collections/OperationsOnSTLContainers.h

        Utils/Configuration/AutomaticRunSettings.h
        Utils/Configuration/Configuration.h
        Utils/Configuration/LoggingSettings.h
        Engines/Telemetry/TelemetrySettings.h

        Utils/Enums/IndexingMethod.h
        Utils/Enums/ExecutionMode.h
        Utils/Enums/HashBlockProperties.h
        Utils/Enums/VoxelFlags.h
        Utils/Enums/WarpType.h

        Utils/FileIO/JSON_Utilities.h
        Utils/FileIO/CSV_Utilities.h
        Utils/FileIO/RecordHandling.h

        Utils/Geometry/CardinalAxesAndPlanes.h
        Utils/Geometry/CheckBlockVisibility.h
        Utils/Geometry/FrustumTrigonometry.h
        Utils/Geometry/GeometryBooleanOperations.h
        Utils/Geometry/GeometryConversionOperations.h
        Utils/Geometry/IntersectionChecks.h
        Utils/Geometry/Rodrigues.h
        Utils/Geometry/Segment.h
        Utils/Geometry/SpatialIndexConversions.h

        Utils/Metacoding/DeferrableStructUtilities.h
        Utils/Metacoding/DeferrableSerializableStruct.h
        Utils/Metacoding/DeferrableSerializableStruct_Impl.h
        Utils/Metacoding/Metacoding.h
        Utils/Metacoding/MetacodingAuxiliaryUtilities.h
        Utils/Metacoding/PathlessSerializableStruct.h
        Utils/Metacoding/PathlessSerializableStruct_Impl.h
        Utils/Metacoding/SerializableStruct.h
        Utils/Metacoding/SerializableStruct_Impl.h
        Utils/Metacoding/SerializableEnum.h
        Utils/Metacoding/SerializableEnum_Impl.h
        Utils/Metacoding/SequenceLevel1Macros.h
        Utils/Metacoding/SequenceLevel2Macros.h
        Utils/Metacoding/PreprocessorNargs.h

        Utils/Quaternions/Quaternion.h
        Utils/Quaternions/QuaternionIO.h
        Utils/Quaternions/QuaternionUtils.h
        Utils/Quaternions/QuaternionFromMatrix.h

        Utils/Telemetry/TelemetryUtilities.h

        Utils/Logging/ConsolePrintColors.h
        Utils/Analytics/Histogram.h
        Utils/Logging/Logging.cpp
        Utils/Logging/LoggingUtilities.h
        Utils/Logging/PrettyPrinters.h
        Utils/Logging/PrettyPrinters.cpp

        Utils/CUDA/CUDAUtils.h
        Utils/ImageTypes.h

        Utils/Math.h
        Utils/Collections/MemoryBlock_StdContainer_Convertions.h
        Utils/MemoryBlockTypes.h
        Utils/PixelUtils.h
        Utils/ProjectionUtils.h

        Utils/VoxelVolumeParameters.h
        Utils/SurfelVolumeParameters.h

        Utils/Analytics/VoxelVolumeComparison/VoxelVolumeComparison.h
        Utils/Analytics/VoxelVolumeComparison/VoxelVolumeComparison_CPU.h
        Utils/Analytics/VoxelVolumeComparison/VoxelVolumeComparison_CUDA.h
        Utils/Analytics/VoxelVolumeComparison/VoxelVolumeComparison_Functors.h
)

#################################################################
# Collect the project files into sources, headers and templates #
#################################################################

set(sources
    ${ITMLIB_OBJECTS_VOLUME_SOURCES}
    ${ITMLIB_ENGINES_MAIN_SOURCES}
    ${ITMLIB_ENGINES_LOW_LEVEL_SOURCES}
    ${ITMLIB_ENGINES_EDIT_AND_COPY_SOURCES}
    ${ITMLIB_ENGINES_MESHING_SOURCES}
    ${ITMLIB_ENGINES_INDEXING_SOURCES}
    ${ITMLIB_ENGINES_RECONSTRUCTION_SOURCES}
    ${ITMLIB_ENGINES_DEPTH_FUSION_SOURCES}
    ${ITMLIB_ENGINES_VOLUME_FUSION_SOURCES}
    ${ITMLIB_ENGINES_WARP_VOLUME_SOURCES}
    ${ITMLIB_ENGINES_VOLUME_FILE_IO_SOURCES}
    ${ITMLIB_ENGINES_SWAPPING_CPU_SOURCES}
    ${ITMLIB_ENGINES_SWAPPING_CUDA_SOURCES}
    ${ITMLIB_ENGINES_VIEW_BUILDER_SOURCES}
    ${ITMLIB_ENGINES_RENDERING_SOURCES}
    ${ITMLIB_ENGINES_ANALYTICS_SOURCES}
    ${ITMLIB_ENGINES_TELEMETRY_SOURCES}
    ${ITMLIB_ENGINES_LEVEL_SET_ALIGNMENT_SOURCES}
    ${ITMLIB_OBJECTS_CAMERA_SOURCES}
    ${ITMLIB_OBJECTS_MESHING_SOURCES}
    ${ITMLIB_OBJECTS_RENDER_STATE_SOURCES}
    ${ITMLIB_CAMERA_TRACKERS_CPU_SOURCES}
    ${ITMLIB_CAMERA_TRACKERS_CUDA_SOURCES}
    ${ITMLIB_CAMERA_TRACKERS_INTERFACE_SOURCES}
    ${ITMLIB_UTILS_SOURCES}
    )

set(headers
    ${ITMLIB_TOPLEVEL_HEADERS}
    ${ITMLIB_ENGINES_MAIN_HEADERS}
    ${ITMLIB_ENGINES_COMMON_HEADERS}
    ${ITMLIB_ENGINES_LOW_LEVEL_HEADERS}
    ${ITMLIB_ENGINES_EDIT_AND_COPY_HEADERS}
    ${ITMLIB_ENGINES_TRAVERSAL_HEADERS}
    ${ITMLIB_ENGINES_REDUCTION_HEADERS}
    ${ITMLIB_ENGINES_MESHING_HEADERS}
    ${ITMLIB_ENGINES_INDEXING_HEADERS}
    ${ITMLIB_ENGINES_RECONSTRUCTION_HEADERS}
    ${ITMLIB_ENGINES_DEPTH_FUSION_HEADERS}
    ${ITMLIB_ENGINES_VOLUME_FUSION_HEADERS}
    ${ITMLIB_ENGINES_WARP_VOLUME_HEADERS}
    ${ITMLIB_ENGINES_VOLUME_FILE_IO_HEADERS}
    ${ITMLIB_ENGINES_SWAPPING_HEADERS}
    ${ITMLIB_ENGINES_SWAPPING_CPU_HEADERS}
    ${ITMLIB_ENGINES_SWAPPING_CUDA_HEADERS}
    ${ITMLIB_ENGINES_SWAPPING_INTERFACE_HEADERS}
    ${ITMLIB_ENGINES_SWAPPING_SHARED_HEADERS}
    ${ITMLIB_ENGINES_VIEW_BUILDER_HEADERS}
    ${ITMLIB_ENGINES_RENDERING_HEADERS}
    ${ITMLIB_ENGINES_ANALYTICS_HEADERS}
    ${ITMLIB_ENGINES_TELEMETRY_HEADERS}
    ${ITMLIB_ENGINES_LEVEL_SET_ALIGNMENT_HEADERS}
    ${ITMLIB_OBJECTS_CAMERA_HEADERS}
    ${ITMLIB_OBJECTS_MESHING_HEADERS}
    ${ITMLIB_OBJECTS_MISC_HEADERS}
    ${ITMLIB_OBJECTS_RENDER_STATE_HEADERS}
    ${ITMLIB_OBJECTS_VOLUME_SOURCES}
    ${ITMLIB_OBJECTS_VOLUME_HEADERS}
    ${ITMLIB_OBJECTS_TRACKING_HEADERS}
    ${ITMLIB_OBJECTS_VIEWS_HEADERS}
    ${ITMLIB_OBJECTS_VIEWS_SOURCES}
    ${ITMLIB_CAMERA_TRACKERS_HEADERS}
    ${ITMLIB_CAMERA_TRACKERS_CPU_HEADERS}
    ${ITMLIB_CAMERA_TRACKERS_CUDA_HEADERS}
    ${ITMLIB_CAMERA_TRACKERS_INTERFACE_HEADERS}
    ${ITMLIB_CAMERA_TRACKERS_SHARED_HEADERS}
    ${ITMLIB_UTILS_HEADERS})


source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${sources} ${headers})

add_library(ITMLib STATIC ${sources} ${headers})

target_link_libraries(ITMLib PUBLIC threadpool11 log4cplus tinyply MiniSlamGraphLib FernRelocLib ORUtils Boost::program_options Boost::filesystem)

if (WITH_OPENMP)
    target_link_libraries(ITMLib PUBLIC OpenMP::OpenMP_CXX)
endif ()

target_include_directories(ITMLib PRIVATE ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES})



