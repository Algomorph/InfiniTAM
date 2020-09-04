###################################
# CMakeLists.txt for FernRelocLib #
###################################

###########################
# Specify the target name #
###########################

set(targetname FernRelocLib)

################################
# Specify the libraries to use #
################################

include(${PROJECT_SOURCE_DIR}/cmake/UseBoost.cmake)

#############################
# Specify the project files #
#############################

set(
        sources

        FernConservatory.cpp
        PoseDatabase.cpp
        RelocDatabase.cpp
)

set(
        headers

        FernConservatory.h
        PixelUtils.h
        PoseDatabase.h
        RelocDatabase.h
        Relocaliser.h
)

source_group("" FILES ${sources} ${headers})
add_library(${targetname} ${sources} ${headers})
target_include_directories(${targetname} PRIVATE ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES})

include(${PROJECT_SOURCE_DIR}/cmake/LinkBoost.cmake)