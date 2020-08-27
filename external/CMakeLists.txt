##########################################################
# external/CMakeLists.txt:
# Automates dependency handling to a reasonable extent.
##########################################################

include(ExternalProject)
include(SetUpDependency)
include(ProvideDependencySourceOption)


######################
# Dependency options
######################
set(STANDARD_PACKAGE_SOURCE_OPTIONS
    ${FIND_LOCAL_SOURCE}
    ${BUILD_EXTERNAL_SOURCE}
    ${BUILD_PACKAGED_SOURCE})

set(NO_PACKAGED_SOURCE_OPTIONS
    ${FIND_LOCAL_SOURCE}
    ${BUILD_EXTERNAL_SOURCE})

set(DOWNLOAD_PREBUILT_SOURCE_OPTIONS
    ${FIND_LOCAL_SOURCE}
    ${DOWNLOAD_PREBUILT_SOURCE})

# log4cplus
provide_dependency_source_option(log4cplus DEFAULT_INDEX 1 OPTIONS ${STANDARD_PACKAGE_SOURCE_OPTIONS})

# zlib
if (MSVC)
    set(ZLIB_DEFAULT_SOURCE_OPTION_INDEX 1)
else ()
    set(ZLIB_DEFAULT_SOURCE_OPTION_INDEX 0)
endif ()
provide_dependency_source_option(zlib DEFAULT_INDEX ${ZLIB_DEFAULT_SOURCE_OPTION_INDEX} OPTIONS ${STANDARD_PACKAGE_SOURCE_OPTIONS})

# libpng
if (MSVC)
    set(PNG_DEFAULT_SOURCE_OPTION_INDEX 1)
else ()
    set(PNG_DEFAULT_SOURCE_OPTION_INDEX 0)
endif ()
provide_dependency_source_option(png DEFAULT_INDEX ${PNG_DEFAULT_SOURCE_OPTION_INDEX} OPTIONS ${NO_PACKAGED_SOURCE_OPTIONS})


# glut
if (MSVC)
    set(GLUT_DEFAULT_SOURCE_OPTION_INDEX 1)
else ()
    set(GLUT_DEFAULT_SOURCE_OPTION_INDEX 0)
endif ()
provide_dependency_source_option(glut DEFAULT_INDEX ${GLUT_DEFAULT_SOURCE_OPTION_INDEX} OPTIONS ${STANDARD_PACKAGE_SOURCE_OPTIONS})

# ffmpeg
option(WITH_FFMPEG "Build with FFmpeg support?" ON)
if (WITH_FFMPEG)
    if (MSVC)
        provide_dependency_source_option(ffmpeg DEFAULT_INDEX ${GLUT_DEFAULT_SOURCE_OPTION_INDEX} OPTIONS ${DOWNLOAD_PREBUILT_SOURCE_OPTIONS})
    else ()
        provide_dependency_source_option(ffmpeg DEFAULT_INDEX ${GLUT_DEFAULT_SOURCE_OPTION_INDEX} OPTIONS ${FIND_LOCAL_SOURCE})
    endif ()
endif ()

# freetype
if (MSVC)
    set(FREETYPE_DEFAULT_SOURCE_OPTION_INDEX 1)
else ()
    set(FREETYPE_DEFAULT_SOURCE_OPTION_INDEX 0)
endif ()
provide_dependency_source_option(freetype DEFAULT_INDEX ${FREETYPE_DEFAULT_SOURCE_OPTION_INDEX} OPTIONS ${NO_PACKAGED_SOURCE_OPTIONS})


#######################
# Retrieve dependencies
#######################

# log4cplus
set_up_dependency(NAME log4cplus
                  PREFERRED_SOURCE ${LOG4CPLUS_PREFERRED_SOURCE}
                  SOURCE_OPTIONS ${LOG4CPLUS_SOURCE_OPTIONS}
                  FORCE_PREFERRED_SOURCE ${LOG4CPLUS_FORCE_PREFERRED_SOURCE}
                  GIT_REPOSITORY https://github.com/log4cplus/log4cplus.git
                  GIT_TAG 3b9d98fb5fd72298745cf21620ace8f2b6d770ad
                  CMAKE_ARGS -DWITH_UNIT_TESTS=OFF -DBUILD_SHARED_LIBS=OFF -DUNICODE=OFF -DLOG4CPLUS_BUILD_TESTING=OFF -DLOG4CPLUS_BUILD_LOGGINGSERVER=OFF
                  PACKAGED
                  INCLUDE_TARGET include
                  LIBRARY_TARGET_MSVC_DEBUG lib/log4cplusSD.lib
                  LIBRARY_TARGET_MSVC_RELEASE lib/log4cplusS.lib
                  LIBRARY_TARGET_MSVC_INTERFACE_LINK_LIBRARIES "ws2_32;advapi32"
                  LIBRARY_TARGET_POSIX lib/liblog4cplusS.a
                  )

# zlib
set_up_dependency(NAME zlib
                  PREFERRED_SOURCE ${ZLIB_PREFERRED_SOURCE}
                  SOURCE_OPTIONS ${ZLIB_SOURCE_OPTIONS}
                  FORCE_PREFERRED_SOURCE ${LOG4CPLUS_FORCE_PREFERRED_SOURCE}
                  GIT_REPOSITORY https://github.com/madler/zlib.git
                  GIT_TAG cacf7f1d4e3d44d871b605da3b647f07d718623f
                  PACKAGED
                  INCLUDE_TARGET include
                  LIBRARY_TARGET_MSVC_DEBUG lib/zlibstaticd.lib
                  LIBRARY_TARGET_MSVC_RELEASE lib/zlibstatic.lib
                  LIBRARY_TARGET_POSIX lib/libz.a
                  ALTERNATIVE_LOCAL_NAME ZLIB
                  )

# libPNG
if (ZLIB_SOURCE STREQUAL FIND_LOCAL_SOURCE)
    set(PNG_ZLIB_CMAKE_ARGS)
else ()
    get_target_property(ZLIB_INCLUDE_DIR zlib INTERFACE_INCLUDE_DIRECTORIES)
    get_target_property(ZLIB_LIBRARY_DEBUG zlib IMPORTED_LOCATION_DEBUG)
    get_target_property(ZLIB_LIBRARY_RELEASE zlib IMPORTED_LOCATION_RELEASE)
    set(PNG_ZLIB_CMAKE_ARGS -DZLIB_LIBRARY_DEBUG=${ZLIB_LIBRARY_DEBUG} -DZLIB_LIBRARY_RELEASE=${ZLIB_LIBRARY_RELEASE} -DZLIB_INCLUDE_DIR=${ZLIB_INCLUDE_DIR})
endif ()

set_up_dependency(NAME png
                  PREFERRED_SOURCE ${PNG_PREFERRED_SOURCE}
                  SOURCE_OPTIONS ${PNG_SOURCE_OPTIONS}
                  FORCE_PREFERRED_SOURCE ${PNG_FORCE_PREFERRED_SOURCE}
                  GIT_REPOSITORY https://github.com/glennrp/libpng.git
                  GIT_TAG dbe3e0c43e549a1602286144d94b0666549b18e6
                  CMAKE_ARGS ${PNG_ZLIB_CMAKE_ARGS} -DPNG_SHARED=OFF -DPNG_STATIC=ON -DPNG_EXECUTABLES=OFF -DPNG_TESTS=OFF
                  INCLUDE_TARGET include
                  LIBRARY_TARGET_MSVC_DEBUG lib/libpng16_staticd.lib
                  LIBRARY_TARGET_MSVC_RELEASE lib/libpng16_static.lib
                  LIBRARY_TARGET_POSIX lib/libpng16.a
                  ALTERNATIVE_LOCAL_NAME PNG
                  )
if (NOT PNG_SOURCE STREQUAL FIND_LOCAL_SOURCE AND NOT ZLIB_SOURCE STREQUAL FIND_LOCAL_SOURCE)
    add_dependencies(__png __zlib)
endif ()
set_target_properties(png PROPERTIES INTERFACE_COMPILE_DEFINITIONS USE_LIBPNG)

# glut
set_up_dependency(NAME glut
                  PREFERRED_SOURCE ${GLUT_PREFERRED_SOURCE}
                  SOURCE_OPTIONS ${GLUT_SOURCE_OPTIONS}
                  FORCE_PREFERRED_SOURCE ${GLUT_FORCE_PREFERRED_SOURCE}
                  GIT_REPOSITORY https://github.com/dcnieho/FreeGLUT.git
                  GIT_TAG 349a23dcc1264a76deb79962d1c90462ad0c6f50
                  SOURCE_SUBDIR freeglut/freeglut
                  CMAKE_ARGS -DFREEGLUT_BUILD_STATIC_LIBS=ON -DFREEGLUT_BUILD_SHARED_LIBS=OFF -DFREEGLUT_BUILD_DEMOS=OFF -DFREEGLUT_REPLACE_GLUT=OFF
                  PACKAGED
                  INCLUDE_TARGET include
                  LIBRARY_TARGET_MSVC_DEBUG lib/freeglut_staticd.lib
                  LIBRARY_TARGET_MSVC_RELEASE lib/freeglut_static.lib
                  LIBRARY_TARGET_POSIX lib/freeglut.a
                  ALTERNATIVE_LOCAL_NAME GLUT
                  )
if (NOT GLUT_SOURCE STREQUAL FIND_LOCAL_SOURCE)
    set_target_properties(glut PROPERTIES INTERFACE_COMPILE_DEFINITIONS FREEGLUT_STATIC)
endif ()

# ffmpeg
if (WITH_FFMPEG)
    set_up_dependency(NAME ffmpeg
                      PREFERRED_SOURCE ${FFMPEG_PREFERRED_SOURCE}
                      SOURCE_OPTIONS ${FFMPEG_SOURCE_OPTIONS}
                      FORCE_PREFERRED_SOURCE ${FFMPEG_FORCE_PREFERRED_SOURCE}
                      ALTERNATIVE_LOCAL_NAME FFMPEG
                      COMPONENTS avcodec;avformat;avfilter;avutil
                      OPTIONAL_COMPONENTS avresample;avdevice;swresample;swscale)
    set_target_properties(ffmpeg PROPERTIES INTERFACE_COMPILE_DEFINITIONS COMPILE_WITH_FFMPEG)
endif ()

# log4cplus
if (PNG_SOURCE STREQUAL FIND_LOCAL_SOURCE)
    set(FREETYPE_PNG_CMAKE_ARGS)
    set(FREETYPE_ZLIB_CMAKE_ARGS)
else ()
    get_target_property(PNG_INCLUDE_DIR png INTERFACE_INCLUDE_DIRECTORIES)
    get_target_property(PNG_LIBRARY_DEBUG png IMPORTED_LOCATION_DEBUG)
    get_target_property(PNG_LIBRARY_RELEASE png IMPORTED_LOCATION_RELEASE)
    set(FREETYPE_PNG_CMAPE_ARGS -DPNG_LIBRARY=${PNG_LIBRARY_RELEASE} -DPNG_LIBRARY_DEBUG=${PNG_LIBRARY_DEBUG} -DPNG_PNG_INCLUDE_DIR=${PNG_INCLUDE_DIR})

    get_target_property(ZLIB_INCLUDE_DIR zlib INTERFACE_INCLUDE_DIRECTORIES)
    get_target_property(ZLIB_LIBRARY_DEBUG zlib IMPORTED_LOCATION_DEBUG)
    get_target_property(ZLIB_LIBRARY_RELEASE zlib IMPORTED_LOCATION_RELEASE)
    set(FREETYPE_ZLIB_CMAKE_ARGS -DZLIB_LIBRARY_DEBUG=${ZLIB_LIBRARY_DEBUG} -DZLIB_LIBRARY=${ZLIB_LIBRARY_RELEASE} -DZLIB_INCLUDE_DIR=${ZLIB_INCLUDE_DIR})
endif ()
set_up_dependency(NAME freetype
                  PREFERRED_SOURCE ${FREETYPE_PREFERRED_SOURCE}
                  SOURCE_OPTIONS ${FREETYPE_SOURCE_OPTIONS}
                  FORCE_PREFERRED_SOURCE ${FREETYPE_FORCE_PREFERRED_SOURCE}
                  ALTERNATIVE_LOCAL_NAME Freetype
                  GIT_REPOSITORY git://git.sv.nongnu.org/freetype/freetype2.git
                  GIT_TAG 132f19b779828b194b3fede187cee719785db4d8
                  CMAKE_ARGS ${FREETYPE_PNG_CMAPE_ARGS} ${FREETYPE_ZLIB_CMAKE_ARGS} -DFT_WITH_PNG=ON -DFT_WITH_ZLIB=ON -DFT_WITH_BZIP2=OFF
                  PACKAGED
                  INCLUDE_TARGET include/freetype2
                  LIBRARY_TARGET_MSVC_DEBUG lib/freetyped.lib
                  LIBRARY_TARGET_MSVC_RELEASE lib/freetype.lib
                  # LIBRARY_TARGET_MSVC_INTERFACE_LINK_LIBRARIES "ws2_32;advapi32"
                  LIBRARY_TARGET_POSIX lib/libfreetype.a
                  LIBRARY_TARGET_POSIX_DEBUG lib/libfreetyped.a
                  )

# freetype
if (NOT FREETYPE_SOURCE STREQUAL FIND_LOCAL_SOURCE)
    if (NOT ZLIB_SOURCE STREQUAL FIND_LOCAL_SOURCE)
        add_dependencies(__freetype __zlib)
    endif ()
    if (NOT PNG_SOURCE STREQUAL FIND_LOCAL_SOURCE)
        add_dependencies(__freetype __png)
    endif ()
endif ()


# tinyply (for .ply mesh input/output)
add_library(tinyply packaged/tinyply/tinyply.h packaged/tinyply/tinyply.cpp)
target_include_directories(tinyply INTERFACE packaged)

# zstr (for compressed input/output file streams)
add_library(zstr packaged/zstr/zstr.hpp packaged/zstr/strict_fstream.hpp
            packaged/zstr/zstr.cpp)
target_link_libraries(zstr zlib)
target_include_directories(zstr INTERFACE packaged)

# wft (freetype C++ wrapper, modified by yours truly)
add_library(wft packaged/wft/wft.hpp packaged/wft/wft.cpp)
target_link_libraries(wft freetype)

#threadpool11 (https://github.com/tghosgor/threadpool11) fast thread pooling for C++
# 39fcc1fc51671989d4f3c02903f9d39c6323b685
find_package(Threads REQUIRED)
find_package(Boost REQUIRED)
add_library(threadpool11 STATIC
            packaged/threadpool11/pool.cpp
            packaged/threadpool11/pool.hpp
            packaged/threadpool11/threadpool11.hpp
            packaged/threadpool11/work.hpp)

if(CMAKE_VERSION VERSION_LESS 3.15)
    set(BOOST_TARGET Boost::boost)
else()
    set(BOOST_TARGET Boost::headers)
endif()

target_link_libraries(threadpool11 Threads::Threads ${BOOST_TARGET})