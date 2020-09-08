# -*- mode: cmake; -*-
###############################################################################
# Find OpenNI
#
# This sets the following variables:
# OpenNI_FOUND - True if OpenNI was found.
# OpenNI_INCLUDE_DIRS - Directories containing the OpenNI include files.
# OpenNI_LIBRARIES - Libraries needed to use OpenNI.
# OpenNI_DEFINITIONS - Compiler flags for OpenNI.
#
# File forked from Pangolin, project of Steven Lovegrove
# (https://github.com/stevelovegrove/Pangolin).

find_package(PkgConfig)
pkg_check_modules(PC_OpenNI QUIET openni-dev)

set(OpenNI_DEFINITIONS ${PC_OpenNI_CFLAGS_OTHER})

#using the 64bit version of OpenNi if generating for 64bit
if (CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(PROGRAMFILES_ "$ENV{PROGRAMW6432}")
    set(OpenNI_SUFFIX "64")
else (CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(PROGRAMFILES_ "$ENV{PROGRAMFILES}")
    set(OpenNI_SUFFIX "")
endif (CMAKE_SIZEOF_VOID_P EQUAL 8)

#add a hint so that it can find it without the pkg-config
find_path(OpenNI_INCLUDE_DIR XnStatus.h
          HINTS ${PC_OpenNI_INCLUDEDIR} ${PC_OpenNI_INCLUDE_DIRS} /usr/include/ni /usr/include/openni
          "${PROGRAMFILES_}/OpenNI/Include"
          PATH_SUFFIXES openni)
#add a hint so that it can find it without the pkg-config
find_library(OpenNI_LIBRARY
             NAMES OpenNI64 OpenNI
             HINTS ${PC_OpenNI_LIBDIR} ${PC_OpenNI_LIBRARY_DIRS} /usr/lib "${PROGRAMFILES_}/OpenNI/Lib${OpenNI_SUFFIX}")

set(OpenNI_INCLUDE_DIRS ${OpenNI_INCLUDE_DIR})
set(OpenNI_LIBRARIES ${OpenNI_LIBRARY})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(OpenNI DEFAULT_MSG OpenNI_LIBRARY OpenNI_INCLUDE_DIR)

if (OpenNI_FOUND)
    if (NOT TARGET OpenNI::OpenNI)
        add_library(OpenNI::OpenNI UNKNOWN IMPORTED)
    endif ()
    set_target_properties(
            OpenNI::OpenNI PROPERTIES
            IMPORTED_LOCATION "${OpenNI_LIBRARY}"
            INTERFACE_INCLUDE_DIRECTORIES "${OpenNI_INCLUDE_DIR}"
    )
endif ()

mark_as_advanced(OpenNI_LIBRARY OpenNI_INCLUDE_DIR)

