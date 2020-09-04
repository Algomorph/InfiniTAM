###################################
# CMakeLists.txt for FernRelocLib #
###################################

set(targetname FernRelocLib)

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