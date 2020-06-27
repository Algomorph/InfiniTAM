include(ExternalProject)
include(${CMAKE_CURRENT_SOURCE_DIR}/utils.cmake)

######################
# Dependency options
######################
set(ALL_PACKAGE_SOURCE_OPTIONS
    FIND_LOCAL
    BUILD_EXTERNAL
    BUILD_PACKAGED)

# log4cplus
provide_dependency_source_option(log4cplus DEFAULT_INDEX 1 OPTIONS ${ALL_PACKAGE_SOURCE_OPTIONS})

# zlib
if(MSVC)
    set(ZLIB_DEFAULT_SOURCE_OPTION_INDEX 1)
else()
    set(ZLIB_DEFAULT_SOURCE_OPTION_INDEX 0)
endif()
provide_dependency_source_option(zlib DEFAULT_INDEX ${ZLIB_DEFAULT_SOURCE_OPTION_INDEX} OPTIONS ${ALL_PACKAGE_SOURCE_OPTIONS})

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
                        SOURCE_OPTIONS ${ZLIB_PREFERRED_SOURCE}
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
#TODO
#handle_external_package(NAME png
#                        SOURCE ${PNG_SOURCE}
#                        GIT_REPOSITORY https://github.com/madler/zlib.git
#                        GIT_TAG cacf7f1d4e3d44d871b605da3b647f07d718623f
#                        PACKAGED
#                        INCLUDE_TARGET include
#                        LIBRARY_TARGET_MSVC_DEBUG lib/zlibd.lib
#                        LIBRARY_TARGET_MSVC_RELEASE lib/zlib.lib
#                        LIBRARY_TARGET_POSIX lib/libz.a
#                        ALTERNATIVE_LOCAL_NAME ZLIB
#                        )

# tinyply (for .ply mesh input/output)
add_library(tinyply ${CMAKE_CURRENT_SOURCE_DIR}/packaged/tinyply/tinyply.h ${CMAKE_CURRENT_SOURCE_DIR}/packaged/tinyply/tinyply.cpp)
target_include_directories(tinyply INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/packaged)

# zstr (for compressed input/output file streams)
add_library(zstr ${CMAKE_CURRENT_SOURCE_DIR}/packaged/zstr/zstr.hpp ${CMAKE_CURRENT_SOURCE_DIR}/packaged/zstr/strict_fstream.hpp
            ${CMAKE_CURRENT_SOURCE_DIR}/packaged/zstr/zstr.cpp)
target_link_libraries(zstr zlib)
target_include_directories(zstr INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/packaged)