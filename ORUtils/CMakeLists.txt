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
include(${PROJECT_SOURCE_DIR}/cmake/UseBoost.cmake)

###############################
# Handle resources            #
###############################
set(RESOURCE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})

file(GLOB_RECURSE RESOURCE_FILES
     "${CMAKE_CURRENT_SOURCE_DIR}/*.ttf"
     # add other resource file patters here
     # e.g. organize under ORUtils/images, ORUtils/icons, etc.
     )

foreach (RESOURCE_FILE ${RESOURCE_FILES})
    string(REPLACE ${CMAKE_CURRENT_SOURCE_DIR} ${RESOURCE_DIRECTORY} NEW_FILE ${RESOURCE_FILE})
    get_filename_component(TARGET_DIR ${NEW_FILE} DIRECTORY)
    file(MAKE_DIRECTORY ${TARGET_DIR})
    configure_file(
            ${RESOURCE_FILE}
            ${NEW_FILE}
            COPYONLY
    )
endforeach (RESOURCE_FILE)

###############################
# Compile config file         #
###############################
configure_file(ORUtilsConfig.h.in ORUtilsConfig.h @ONLY)

#############################
# Specify the project files #
#############################

set(
        sources

        DrawText.cpp
        FileUtils.cpp
        IStreamWrapper.cpp
        KeyValueConfig.cpp
        VectorAndMatrixPersistence.cpp
        MemoryBlockPersistence.cpp
        OStreamWrapper.cpp
        SE3Pose.cpp
        TypeTraits.cpp
)

set(
        headers

        ${CMAKE_CURRENT_BINARY_DIR}/ORUtilsConfig.h
        Cholesky.h
        CUDADefines.h
        DrawText.h
        FileUtils.h
        HomkerMap.h
        Image.h
        IStreamWrapper.h
        KeyValueConfig.h
        LexicalCast.h
        MathUtils.h
        Matrix.h
        VectorAndMatrixPersistence.h
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

include(${PROJECT_SOURCE_DIR}/cmake/LinkBoost.cmake)
target_link_libraries(${targetname} PUBLIC zstr png )
target_link_libraries(${targetname} PRIVATE wft freetype)
target_include_directories(${targetname} PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
