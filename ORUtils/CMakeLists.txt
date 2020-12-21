##############################
# CMakeLists.txt for ORUtils #
##############################

# *** handle resource files & config file ***

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

configure_file(ORUtilsConfig.h.in ORUtilsConfig.h @ONLY)
# ***

set(
        sources

        DrawText.cpp
        FileUtils.cpp
        ImageCombination.cpp
        IStreamWrapper.cpp
        KeyValueConfig.cpp

        OStreamWrapper.cpp
        SE3Pose.cpp
        TypeTraits.cpp

        MathTypePersistence/MathTypePersistence_Matrix4.cpp
        MathTypePersistence/MathTypePersistence_Matrix3.cpp
        MathTypePersistence/MathTypePersistence_Vector4.cpp
        MathTypePersistence/MathTypePersistence_Vector6.cpp
        MathTypePersistence/MathTypePersistence_Vector3.cpp
        MathTypePersistence/MathTypePersistence_Vector2.cpp

        MemoryBlockPersistence/MemoryBlockPersistence.cpp
        MemoryBlockPersistence/MemoryBlockPersistenceOperators.cpp
        MemoryBlockPersistence/ImagePersistenceOperators.cpp
        MemoryBlockPersistence/MemoryBlockPersistenceOperators.tpp
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
        ImageCombination.h
        IStreamWrapper.h
        KeyValueConfig.h
        LexicalCast.h
        MathUtils.h
        Matrix.h
        MathTypePersistence/MathTypePersistence.h
        MathTypePersistence/MathTypePersistence_Internal.h
        MemoryBlockPersistence/MemoryBlockPersistence.h
        MemoryBlockPersistence/MemoryBlockPersistenceOperators.h
        MemoryBlock.h
        MemoryBlockComparison.h
        MemoryDeviceType.h
        NVTimer.h
        OStreamWrapper.h
        PlatformIndependence.h
        PlatformIndependentAtomics.h
        PlatformIndependentParallelSum.h
        CrossPlatformMacros.h
        SE3Pose.h
        SVMClassifier.h
        Vector.h
        TypeTraits.h
        CompileTimeStringConcatenation.h
        )

source_group("" FILES ${sources} ${headers})
add_library(ORUtils STATIC ${sources} ${headers})

# *** dependencies ***
if (WITH_OPENMP)
    target_link_libraries(ORUtils PUBLIC OpenMP::OpenMP_CXX)
endif ()
target_link_libraries(ORUtils PUBLIC zstr png Boost::filesystem)
target_link_libraries(ORUtils PRIVATE wft freetype)
target_include_directories(ORUtils PUBLIC ${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES})
