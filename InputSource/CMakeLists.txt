##################################
# CMakeLists.txt for InputSource #
##################################

# *** locate dependency packages ***
set(InputSource_DEPENDENCIES)
if(WITH_FFMPEG)
    list(APPEND InputSource_DEPENDENCIES ffmpeg)
endif()

# ** UVC **
option(WITH_UVC "Build with libuvc support?" OFF)

if(WITH_UVC)
    find_package(libuvc REQUIRED)
    if (libuvc_FOUND)
        if (NOT TARGET libuvc::libuvc)
            add_library(libuvc::libuvc UNKNOWN IMPORTED)
        endif ()
        set_target_properties(
                libuvc::libuvc PROPERTIES
                IMPORTED_LOCATION "${libuvc_LIBRARIES}"
                INTERFACE_INCLUDE_DIRECTORIES "${libuvc_INCLUDE_DIRS}"
                INTERFACE_COMPILE_DEFINITIONS "COMPILE_WITH_LibUVC"
        )
        list(APPEND InputSource_DEPENDENCIES libuvc::libuvc)
    endif()
endif()

# ** OpenNI **
option(WITH_OPENNI "Build with OpenNI support?" OFF)
if(WITH_OPENNI)
    find_package(OpenNI REQUIRED)
    if(OpenNI_FOUND)
        target_compile_definitions(OpenNI::OpenNI INTERFACE WITH_OPENNI)
        list(APPEND InputSource_DEPENDENCIES OpenNI::OpenNI)
    endif()
endif()

# ** OpenNI2 **
option(WITH_OPENNI2 "Build with OpenNI 2 support?" OFF)
if(WITH_OPENNI2)
    find_package(OpenNI2 REQUIRED)
    if(OpenNI2_FOUND)
        target_compile_definitions(OpenNI2::OpenNI2 INTERFACE WITH_OPENNI2)
        list(APPEND InputSource_DEPENDENCIES OpenNI2::OpenNI2)
    endif()
endif()

# ** Kinect SDK v2 **
option(WITH_KINECTSDK2 "Build with Kinect SDK v2 support?" OFF)
if(WITH_KINECTSDK2)
    find_package(KinectSDK2 REQUIRED)
    if(KinectSDK2_FOUND)
        target_compile_definitions(KinectSDK2::KinectSDK2 INTERFACE WITH_KINECTSDK2)
        list(APPEND InputSource_DEPENDENCIES KinectSDK2::KinectSDK2)
    endif()
endif()

# ** LibRoyale **
option(WITH_LIBROYALE "Build with LibRoyale support?" OFF)
if(WITH_LIBROYALE)
    find_package(LibRoyale REQUIRED)
    if(LibRoyale_FOUND)
        target_compile_definitions(LibRoyale::LibRoyale INTERFACE COMPILE_WITH_LIBROYALE)
        list(APPEND InputSource_DEPENDENCIES LibRoyale::LibRoyale)
    endif()
endif()

# ** Realsense **
option(WITH_REALSENSE2 "Build with RealSense2 support?" OFF)
if(WITH_REALSENSE2)
    find_package(realsense2 REQUIRED)
    if(realsense2_FOUND)
        target_compile_definitions(realsense2::realsense2 INTERFACE COMPILE_WITH_REALSENSE22)
        list(APPEND InputSource_DEPENDENCIES realsense2::realsense2)
    endif()
endif()


# *** set up sources & target ***

set(
        sources

        CompositeImageSourceEngine.cpp
        FFMPEGReader.cpp
        FFMPEGWriter.cpp
        ImageSourceEngine.cpp
        IMUSourceEngine.cpp
        Kinect2Engine.cpp
        LibUVCEngine.cpp
        OpenNI2Engine.cpp
        PicoFlexxEngine.cpp
        RealSenseEngine.cpp
)
set(
        headers

        CompositeImageSourceEngine.h
        FFMPEGReader.h
        FFMPEGWriter.h
        ImageSourceEngine.h
        IMUSourceEngine.h
        Kinect2Engine.h
        LibUVCEngine.h
        OpenNI2Engine.h
        PicoFlexxEngine.h
        RealSenseEngine.h
)
source_group("" FILES ${sources} ${headers})

add_library(InputSource ${sources} ${headers})
target_link_libraries(InputSource ${InputSource_DEPENDENCIES})

target_include_directories(InputSource PRIVATE ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES})


