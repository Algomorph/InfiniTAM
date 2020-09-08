###############################################################################
# Find OpenNI2
#
# This sets the following variables:
# OpenNI2_FOUND - True if OPENNI was found.
# OpenNI2_INCLUDE_DIRS - Directories containing the OPENNI include files.
# OpenNI2_LIBRARIES - Libraries needed to use OPENNI.
#
# File forked from Pangolin, project of Steven Lovegrove
# (https://github.com/stevelovegrove/Pangolin).

find_package(PkgConfig)
pkg_check_modules(PC_OPENNI QUIET openni2-dev)

set(OpenNI2_DEFINITIONS ${PC_OPENNI_CFLAGS_OTHER})

#add a hint so that it can find it without the pkg-config
find_path(OpenNI2_INCLUDE_DIR OpenNI.h
          HINTS
            ${PC_OPENNI_INCLUDEDIR}
            ${PC_OPENNI_INCLUDE_DIRS}
          PATHS
            "${PROGRAM_FILES}/OpenNI2/Include"
            "${CMAKE_SOURCE_DIR}/../OpenNI2/Include"
            /usr/include
            /user/include
          PATH_SUFFIXES openni2 ni2
)

if(${CMAKE_CL_64})
    set(OPENNI_PATH_SUFFIXES lib64 lib)
else()
    set(OPENNI_PATH_SUFFIXES lib)
endif()

#add a hint so that it can find it without the pkg-config
find_library(OpenNI2_LIBRARY
             NAMES OpenNI2
             HINTS
               ${PC_OPENNI_LIBDIR}
               ${PC_OPENNI_LIBRARY_DIRS}
             PATHS
               "${PROGRAM_FILES}/OpenNI2/Redist"
               "${PROGRAM_FILES}/OpenNI2"
               "${CMAKE_SOURCE_DIR}/../OpenNI2/Bin/x64-Release"
               /usr/lib
               /user/lib
             PATH_SUFFIXES ${OPENNI_PATH_SUFFIXES}
)

set(OpenNI2_INCLUDE_DIRS ${OpenNI2_INCLUDE_DIR})
set(OpenNI2_LIBRARIES ${OpenNI2_LIBRARY})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(OpenNI2 DEFAULT_MSG OpenNI2_LIBRARY OpenNI2_INCLUDE_DIR)

if (OpenNI2_FOUND)
    if (NOT TARGET OpenNI2::OpenNI2)
        add_library(OpenNI2::OpenNI2 UNKNOWN IMPORTED)
    endif ()
    set_target_properties(
            OpenNI2::OpenNI2 PROPERTIES
            IMPORTED_LOCATION "${OpenNI2_LIBRARY}"
            INTERFACE_INCLUDE_DIRECTORIES "${OpenNI2_INCLUDE_DIR}"
    )
endif ()

mark_as_advanced(OpenNI2_LIBRARY OpenNI2_INCLUDE_DIR)
