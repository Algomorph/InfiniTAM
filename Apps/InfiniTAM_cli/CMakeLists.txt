#####################################
# CMakeLists.txt for Apps/InfiniTAM #
#####################################

set(sources
    InfiniTAM_cli.cpp
    CLIEngine.cpp
    CreateDefaultImageSource.cpp
)

set(headers
    CLIEngine.h
    CreateDefaultImageSource.h
)

source_group("" FILES ${sources} ${headers})
add_library(InfiniTAM_cli ${sources} ${headers})
target_link_libraries(InfiniTAM_cli PRIVATE MiniSlamGraphLib ORUtils FernRelocLib ITMLib InputSource OpenGL::GL Boost::program_options)