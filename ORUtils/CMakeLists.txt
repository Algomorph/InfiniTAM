##############################
# CMakeLists.txt for ORUtils #
##############################

###########################
# Specify the target name #
###########################

set(targetname ORUtils)

################################
# Specify the libraries to use #
################################

include(${PROJECT_SOURCE_DIR}/cmake/UseCUDA.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/UseOpenMP.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/UsePNG.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/UseBoost.cmake)

#############################
# Specify the project files #
#############################

set(
        sources

        FileUtils.cpp
        IStreamWrapper.cpp
        KeyValueConfig.cpp
        MemoryBlockPersistence.cpp
        OStreamWrapper.cpp
        SE3Pose.cpp
        TypeTraits.cpp
)

set(
        headers

        Cholesky.h
        CUDADefines.h
        FileUtils.h
        HomkerMap.h
        Image.h
        IStreamWrapper.h
        KeyValueConfig.h
        LexicalCast.h
        MathUtils.h
        Matrix.h
        MemoryBlock.h
        MemoryBlockPersistence.h
        MemoryBlockComparison.h
        MemoryDeviceType.h
        NVTimer.h
        OStreamWrapper.h
        PlatformIndependence.h
        PlatformIndependentAtomics.h
        PlatformIndependedParallelSum.h
        CrossPlatformMacros.h
        SE3Pose.h
        SVMClassifier.h
        Vector.h
        TypeTraits.h
)

#############################
# Specify the source groups #
#############################

source_group("" FILES ${sources} ${headers})

##########################################
# Specify the target and where to put it #
##########################################

include(${PROJECT_SOURCE_DIR}/cmake/SetCUDALibTarget.cmake)

##########################################
# Link external dependency libraries     #
##########################################

include(${PROJECT_SOURCE_DIR}/cmake/LinkPNG.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/LinkBoost.cmake)
target_link_libraries(${targetname} zstr)
