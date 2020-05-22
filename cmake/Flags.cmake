###############
# Flags.cmake #
###############

# If on Mac OS X:
if (${CMAKE_SYSTEM} MATCHES "Darwin")
    # Make sure that C++11 warnings are disabled.
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-c++11-extensions")

    # Make sure that the template depth is sufficient.
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ftemplate-depth=512")

    if (${CMAKE_SYSTEM} MATCHES "Darwin-13.")
        # If on Mac OS X 10.9 (Mavericks), use the libstdc++ implementation of the C++ Standard Library and prevent C++11 code from being compiled.
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libstdc++")
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -stdlib=libstdc++")
        add_definitions(-DNO_CPP11)
    else ()
        # Otherwise, use the libc++ implementation of the C++ Standard Library, and enable C++11 support.
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++ -std=c++11")
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -stdlib=libc++ -std=c++11")
    endif ()
endif ()

# If on Linux, make sure that C++11 support is enabled.
if (${CMAKE_SYSTEM} MATCHES "Linux")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")
endif ()

# If on Windows and using Visual Studio:
if (MSVC_IDE)
    # Disable the annoying warnings about using secure CRT functions (they're Microsoft-specific, so we can't use them portably).
    add_definitions(-D_CRT_SECURE_NO_WARNINGS)

    # Prevent the definitions of min and max when including windows.h.
    add_definitions(-DNOMINMAX)

    # Make sure that the maths constants are defined.
    add_definitions(-D_USE_MATH_DEFINES)

    # Define a macro needed when using Boost.ASIO.
    add_definitions(-D_WIN32_WINNT=0x0501)
endif ()

# CUDA-specific
if (WITH_CUDA)
    if (${CMAKE_VERSION} VERSION_LESS 3.8 OR (MSVC_IDE AND ${CMAKE_VERSION} VERSION_LESS 3.9))
        #TODO: for CMake backward-compatibility, figure out how to add --extended-lambda to the CUDA compile flags here
        message( FATAL_ERROR "CMake 3.9 or prior compatibility, figure out how to add \"--extended-lambda\" (without quotes) to the CUDA compile flags here." )
    else ()
        set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} --extended-lambda")
    endif ()
endif ()