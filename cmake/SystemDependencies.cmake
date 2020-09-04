cmake_minimum_required(VERSION 3.18.2 FATAL_ERROR)

# *** OpenMP ****
option(WITH_OPENMP "Enable OpenMP support?" ON)
if(WITH_OPENMP)
    find_package(OpenMP REQUIRED QUIET)
    add_definitions(-DWITH_OPENMP)
endif()

# *** OpenGL ***
find_package(OpenGL REQUIRED QUIET)



