include(ExternalProject)
include(${CMAKE_CURRENT_SOURCE_DIR}/utils.cmake)

######################
# Dependency options
######################
# to avoid magic strings
set(FIND_LOCAL_SOURCE FIND_LOCAL)
set(BUILD_EXTERNAL_SOURCE BUILD_EXTERNAL)
set(BUILD_PACKAGED_SOURCE BUILD_PACKAGED)

set(ALL_PACKAGE_SOURCE_OPTIONS
    ${FIND_LOCAL_SOURCE}
    ${BUILD_EXTERNAL_SOURCE}
    ${BUILD_PACKAGED_SOURCE})

set(LARGE_PACKAGE_SOURCE_OPTIONS
    ${FIND_LOCAL_SOURCE}
    ${BUILD_EXTERNAL_SOURCE})

set(BAD_REPO_PACKAGE_SOURCE_OPTIONS
    ${FIND_LOCAL_SOURCE}
    ${BUILD_PACKAGED_SOURCE})

# log4cplus
provide_dependency_source_option(log4cplus DEFAULT_INDEX 1 OPTIONS ${ALL_PACKAGE_SOURCE_OPTIONS})

# zlib
if(MSVC)
    set(ZLIB_DEFAULT_SOURCE_OPTION_INDEX 1)
else()
    set(ZLIB_DEFAULT_SOURCE_OPTION_INDEX 0)
endif()
provide_dependency_source_option(zlib DEFAULT_INDEX ${ZLIB_DEFAULT_SOURCE_OPTION_INDEX} OPTIONS ${ALL_PACKAGE_SOURCE_OPTIONS})

# libpng
option(WITH_PNG "Build with libpng support?" ON)
if(MSVC)
    set(PNG_DEFAULT_SOURCE_OPTION_INDEX 1)
else()
    set(PNG_DEFAULT_SOURCE_OPTION_INDEX 0)
endif()
if(WITH_PNG)
    provide_dependency_source_option(png DEFAULT_INDEX ${PNG_DEFAULT_SOURCE_OPTION_INDEX} OPTIONS ${LARGE_PACKAGE_SOURCE_OPTIONS})
endif()

# glut
if(MSVC)
    set(GLUT_DEFAULT_SOURCE_OPTION_INDEX 1)
else()
    set(GLUT_DEFAULT_SOURCE_OPTION_INDEX 0)
endif()
provide_dependency_source_option(glut DEFAULT_INDEX ${GLUT_DEFAULT_SOURCE_OPTION_INDEX} OPTIONS ${ALL_PACKAGE_SOURCE_OPTIONS})


#######################
# Retrieve dependencies
#######################

# log4cplus
handle_external_package(NAME log4cplus
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
handle_external_package(NAME zlib
                        PREFERRED_SOURCE ${ZLIB_PREFERRED_SOURCE}
                        SOURCE_OPTIONS ${ZLIB_SOURCE_OPTIONS}
                        FORCE_PREFERRED_SOURCE ${LOG4CPLUS_FORCE_PREFERRED_SOURCE}
                        GIT_REPOSITORY https://github.com/madler/zlib.git
                        GIT_TAG cacf7f1d4e3d44d871b605da3b647f07d718623f
                        PACKAGED
                        INCLUDE_TARGET include
                        LIBRARY_TARGET_MSVC_DEBUG lib/zlibd.lib
                        LIBRARY_TARGET_MSVC_RELEASE lib/zlib.lib
                        LIBRARY_TARGET_POSIX lib/libz.a
                        ALTERNATIVE_LOCAL_NAME ZLIB
                        )

if(WITH_PNG)
    if(ZLIB_SOURCE STREQUAL FIND_LOCAL_SOURCE)
        set(PNG_ZLIB_CMAKE_ARGS)
    else()
        get_target_property(ZLIB_INCLUDE_DIR zlib INTERFACE_INCLUDE_DIRECTORIES)
        get_target_property(ZLIB_LIBRARY_DEBUG zlib IMPORTED_LOCATION_DEBUG)    
        get_target_property(ZLIB_LIBRARY_RELEASE zlib IMPORTED_LOCATION_RELEASE)
        set(PNG_ZLIB_CMAKE_ARGS -DZLIB_LIBRARY_DEBUG=${ZLIB_LIBRARY_DEBUG} -DZLIB_LIBRARY_RELEASE=${ZLIB_LIBRARY_RELEASE} -DZLIB_INCLUDE_DIR=${ZLIB_INCLUDE_DIR})
    endif()

    # libpng
    handle_external_package(NAME png
                           PREFERRED_SOURCE ${PNG_PREFERRED_SOURCE}
                           SOURCE_OPTIONS ${PNG_SOURCE_OPTIONS}
                           FORCE_PREFERRED_SOURCE ${PNG_FORCE_PREFERRED_SOURCE}
                           GIT_REPOSITORY git@github.com:glennrp/libpng.git
                           GIT_TAG dbe3e0c43e549a1602286144d94b0666549b18e6
                           CMAKE_ARGS ${PNG_ZLIB_CMAKE_ARGS} -DPNG_SHARED=OFF -DPNG_STATIC=ON -DPNG_EXECUTABLES=OFF -DPNG_TESTS=OFF
                           INCLUDE_TARGET include
                           LIBRARY_TARGET_MSVC_DEBUG lib/libpng16_staticd.lib
                           LIBRARY_TARGET_MSVC_RELEASE lib/libpng_static16.lib
                           LIBRARY_TARGET_POSIX lib/libpng16.a
                           ALTERNATIVE_LOCAL_NAME PNG
                           )
    if(NOT PNG_SOURCE STREQUAL FIND_LOCAL_SOURCE AND NOT ZLIB_SOURCE STREQUAL FIND_LOCAL_SOURCE)
        add_dependencies(__png __zlib)
    endif()
endif()



handle_external_package(NAME glut
                        PREFERRED_SOURCE ${GLUT_PREFERRED_SOURCE}
                        SOURCE_OPTIONS ${GLUT_SOURCE_OPTIONS}
                        FORCE_PREFERRED_SOURCE ${GLUT_FORCE_PREFERRED_SOURCE}
                        GIT_REPOSITORY git@github.com:dcnieho/FreeGLUT.git
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

if(NOT GLUT_SOURCE STREQUAL FIND_LOCAL_SOURCE)
    set_target_properties(glut PROPERTIES INTERFACE_COMPILE_DEFINITIONS FREEGLUT_STATIC)
endif()

# tinyply (for .ply mesh input/output)
add_library(tinyply ${CMAKE_CURRENT_SOURCE_DIR}/packaged/tinyply/tinyply.h ${CMAKE_CURRENT_SOURCE_DIR}/packaged/tinyply/tinyply.cpp)
target_include_directories(tinyply INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/packaged)

# zstr (for compressed input/output file streams)
add_library(zstr ${CMAKE_CURRENT_SOURCE_DIR}/packaged/zstr/zstr.hpp ${CMAKE_CURRENT_SOURCE_DIR}/packaged/zstr/strict_fstream.hpp
            ${CMAKE_CURRENT_SOURCE_DIR}/packaged/zstr/zstr.cpp)
target_link_libraries(zstr zlib)
target_include_directories(zstr INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/packaged)