###################################
# CMakeLists.txt for FernRelocLib #
###################################
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
add_library(FernRelocLib ${sources} ${headers})
target_include_directories(FernRelocLib PRIVATE ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES})