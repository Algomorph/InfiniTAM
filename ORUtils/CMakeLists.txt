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
        KeyValueConfig.cpp
        SE3Pose.cpp
)

set(
        headers

        Cholesky.h
        CUDADefines.h
        FileUtils.h
        HomkerMap.h
        Image.h
        KeyValueConfig.h
        LexicalCast.h
        MathUtils.h
        Matrix.h
        MemoryBlock.h
        MemoryBlockPersister.h
        MemoryBlockComparison.h
        MemoryDeviceType.h
        NVTimer.h
        PlatformIndependence.h
        PlatformIndependentAtomics.h
        SE3Pose.h
        SVMClassifier.h
        Vector.h
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
