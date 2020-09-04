cmake_minimum_required(VERSION 3.18.2 FATAL_ERROR)

# *** OpenMP ****
option(WITH_OPENMP "Enable OpenMP support?" ON)
if(WITH_OPENMP)
    find_package(OpenMP REQUIRED QUIET)
    target_compile_definitions(OpenMP::OpenMP_CXX INTERFACE WITH_OPENMP)
endif()


# *** OpenGL ***
find_package(OpenGL REQUIRED QUIET)

# *** Boost ***
set(Boost_FIND_REQUIRED TRUE)
set(Boost_FIND_QUIETLY TRUE)
set(Boost_USE_MULTITHREADED TRUE)
if (CMAKE_CXX_COMPILER_ID MATCHES MSVC)
    set(Boost_USE_STATIC_LIBS TRUE)
    set(Boost_USE_STATIC_RUNTIME ON)
endif ()
find_package(Boost REQUIRED COMPONENTS filesystem program_options)

if(NOT TARGET Boost::filesystem)
    add_library(Boost::filesystem IMPORTED INTERFACE)
    set_property(TARGET Boost::filesystem PROPERTY
                 INTERFACE_INCLUDE_DIRECTORIES ${Boost_INCLUDE_DIR})
    set_property(TARGET Boost::filesystem PROPERTY
                 INTERFACE_LINK_LIBRARIES ${Boost_LIBRARIES})
endif()

if(NOT TARGET Boost::program_options)
    add_library(Boost::program_options IMPORTED INTERFACE)
    set_property(TARGET Boost::program_options PROPERTY
                 INTERFACE_INCLUDE_DIRECTORIES ${Boost_INCLUDE_DIR})
    set_property(TARGET Boost::program_options PROPERTY
                 INTERFACE_LINK_LIBRARIES ${Boost_LIBRARIES})
endif()

if (CMAKE_CXX_COMPILER_ID MATCHES MSVC)
    target_compile_definitions(Boost::filesystem INTERFACE BOOST_ALL_NO_LIB BOOST_SYSTEM_NO_DEPRECATED)
    target_compile_definitions(Boost::program_options INTERFACE BOOST_ALL_NO_LIB BOOST_SYSTEM_NO_DEPRECATED)
    # disable warnings about too few arguments for function-like macro invocation -- these stem from branches in the preprocessor code that are never taken
    target_compile_options(Boost::filesystem INTERFACE $<$<COMPILE_LANGUAGE:CXX>:/wd4003> )
    target_compile_options(Boost::program_options INTERFACE $<$<COMPILE_LANGUAGE:CXX>:/wd4003> )
endif()

