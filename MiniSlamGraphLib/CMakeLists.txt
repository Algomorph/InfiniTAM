#######################################
# CMakeLists.txt for MiniSlamGraphLib #
#######################################

set(MiniSlamGraphLib_DEPENDENCIES ORUtils)

option(WITH_CSPARSE "Compile with Suite Sparse (CSparse) support?" OFF)

if (WITH_CSPARSE)
    find_package(CSparse REQUIRED)
    if(CSparce_FOUND)
        target_compile_definitions(CSparse::CSparse INTERFACE COMPILE_WITH_CSPARSE)
        list(APPEND MiniSlamGraphLib_DEPENDENCIES CSparse::CSparse)
    endif()
endif ()

set(
        sources

        GraphEdge.cpp
        GraphEdgeSE3.cpp
        LevenbergMarquardtMethod.cpp
        MatrixWrapper.cpp
        PoseGraph.cpp
        SlamGraph.cpp
        SlamGraphErrorFunction.cpp
        QuaternionHelpers.cpp
)

set(
        headers

        GraphEdge.h
        GraphEdgeSE3.h
        GraphNode.h
        GraphNodeSE3.h
        LevenbergMarquardtMethod.h
        Matrix_CSparse.h
        MatrixWrapper.h
        ParameterIndex.h
        PoseGraph.h
        QuaternionHelpers.h
        SlamGraph.h
        SlamGraphErrorFunction.h
        SparseBlockMatrix.h
        SparseRegularBlockMatrix.h
        VariableLengthVector.h
)

source_group("" FILES ${sources} ${headers})
add_library(MiniSlamGraphLib ${sources} ${headers})
target_link_libraries(MiniSlamGraphLib PUBLIC ${MiniSlamGraphLib_DEPENDENCIES})
