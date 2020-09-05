#######################################################################################################################
# FindKinectSDK2.cmake                                                                                                #
#######################################################################################################################
#                                                                                                                     #
# Find Kinect for Windows SDK v2 (Kinect SDK v2)
#
# Use this module by invoking find_package with the form:
#
#   find_package( KinectSDK2 [[COMPONENTS] [Face] [Fusion] [VisualGestureBuilder]] [QUIET] [REQUIRED] )
#
# This script will define the following targets if successful:
#
#   KinectSDK2::KinectSDK2              The main Kinect v2 SDK library
#
# These component targets may also be defined (which automatically link to the main Kinect v2 SDK library):
#
#   KinectSDK2::Face                    Face or HDFace features of Kinect v2 SDK library.
#   KinectSDK2::Fusion                  Fusion features of Kinect v2 SDK.
#   KinectSDK2::VisualGestureBuilder    Visual Gesture Builder features of Kinect v2 SDK.
#
# For each successfully-discovered component target, a command for post-build copying of required SDK files will be
# stored in the following variables:
#
#   KinectSDK2_Face_COPY_COMMAND
#   KinectSDK2_Fusion_COPY_COMMAND
#   KinectSDK2_VisualGestureBuilder_COPY_COMMAND
#                                                                                                                     #
#######################################################################################################################
#                                                                                                                     #
# Example usage:
#
#   # ....
#   add_executable( my_target main.cpp)
#   # ....
#
#   if(MSVC)
#       find_package(KinectSDK2 COMPONENTS Face REQUIRED)
#
#       target_link_libraries(my_target PUBLIC KinectSDK2::Face)
#
#       add_custom_command(TARGET my_target POST_BUILD ${KinectSDK2_Face_COPY_COMMAND})
#   endif()
#                                                                                                                     #
#######################################################################################################################


# Target Platform
set(_KinectSDK2_target_platform)
if (NOT CMAKE_CL_64)
    set(_KinectSDK2_target_platform x86)
else ()
    set(_KinectSDK2_target_platform x64)
endif ()


if (MSVC_VERSION LESS 1700)
    message(FATAL_ERROR "Kinect for Windows SDK v2 supported Visual Studio 2012 or later.")
    set(KinectSDK2_FOUND FALSE)
endif ()

# Root Directory
find_path(KinectSDK2_DIR $ENV{KINECTSDK20_DIR} DOC "Kinect for Windows SDK v2 Install Path.")

# Include Directory
find_path(KinectSDK2_INCLUDE_DIR ${KinectSDK2_DIR}/inc)

# Library Directory
find_path(KinectSDK2_LIBRARY_DIR ${KinectSDK2_DIR}/Lib/${_KinectSDK2_target_platform})

# Libraries
find_library(KinectSDK2_LIBRARY ${KinectSDK2_LIBRARY_DIR}/Kinect20)

# Component discovery
function(_kinectsdk2_find component redist_folder)
    find_library("KinectSDK2_${component}_LIBRARY" "${KinectSDK2_LIBRARY_DIR}/Kinect20.${component}")
    find_path("KinectSDK2_${component}_REDIST_DIR" ${KinectSDK2_DIR}/Redist/${redist_folder}/${_KinectSDK2_target_platform})
    mark_as_advanced("KinectSDK2_${component}_LIBRARY")
    mark_as_advanced("KinectSDK2_${component}_REDIST_DIR")
    if (KinectSDK2_${component}_LIBRARY AND KinectSDK2_${component}_REDIST_DIR)
        if (NOT TARGET "KinectSDK2::${component}")
            add_library("KinectSDK2::${component}" UNKNOWN IMPORTED)
            set_target_properties("KinectSDK2::${component}" PROPERTIES
                                  IMPORTED_LOCATION "${KinectSDK2_${component}_LIBRARY}"
                                  INTERFACE_INCLUDE_DIRECTORIES "${KinectSDK2_INCLUDE_DIR}"
                                  IMPORTED_LINK_INTERFACE_LIBRARIES ${KinectSDK2_LIBRARY})
        endif ()
        set(KinectSDK2_${component}_COPY_COMMAND COMMAND xcopy ${KinectSDK2_${_KinectSDK2_component}_REDIST_DIR} "$(OutDir)" /e /y /i /r > NUL)
        set(KinectSDK2_${component}_FOUND 1 PARENT_SCOPE)
    endif ()
endfunction()

_kinectsdk2_find(Face Face)
_kinectsdk2_find(Fusion Fusion)
_kinectsdk2_find(VisualGestureBuilder VGB)

set(KinectSDK2_LIBRARIES ${KinectSDK2_LIBRARY})
set(_KinectSDK2_required_component_vars)
foreach (_KinectSDK2_component in LISTS KinectSDK2_FIND_COMPONENTS)
    if (TARGET "KinectSDK2::${_KinectSDK2_component}")
        list(APPEND KinectSDK2_LIBRARIES ${KinectSDK2_${_KinectSDK2_component}_LIBRARY})
        if (KinectSDK2_FIND_REQUIRED_${_KinectSDK2_component})
            list(APPEND _KinectSDK2_required_component_vars
                 ${KinectSDK2_${_KinectSDK2_component}_LIBRARY}
                 ${KinectSDK2_${_KinectSDK2_component}_REDIST_DIR})
        endif ()
    endif ()
endforeach ()
unset(_KinectSDK2_component)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(KinectSDK2
                                  REQUIRED_VARS KinectSDK2_INCLUDE_DIR KinectSDK2_LIBRARY ${_KinectSDK2_required_component_vars}
                                  HANDLE_COMPONENTS)
unset(_KinectSDK2_required_component_vars)
unset(_KinectSDK2_target_platform)

if (KinectSDK2_FOUND)
    if (NOT TARGET KinectSDK2::KinectSDK2)
        add_library(KinectSDK2::KinectSDK2 UNKNOWN IMPORTED)
    endif ()
    set_target_properties("KinectSDK2::KinectSDK2" PROPERTIES
                          IMPORTED_LOCATION "${KinectSDK2_LIBRARY}"
                          INTERFACE_INCLUDE_DIRECTORIES "${KinectSDK2_INCLUDE_DIR}")
endif ()

mark_as_advanced(
    KinectSDK2_INCLUDE_DIR
    KinectSDK2_LIBRARY_DIR
    KinectSDK2_LIBRARY
)