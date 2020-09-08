#####################################
# CMakeLists.txt for Apps/InfiniTAM #
#####################################
set(
        sources

        InfiniTAM.cpp
        UIEngine.cpp
        UIEngine_GlutFunctions.cpp
        CreateDefaultImageSource.cpp
)

set(
        headers

        UIEngine.h
        prettyprint.hpp
        CreateDefaultImageSource.h
)

source_group("" FILES ${sources} ${headers})
add_executable(InfiniTAM ${sources} ${headers})
target_link_libraries(InfiniTAM PUBLIC glut InputSource ITMLib OpenGL::GL Boost::program_options)
