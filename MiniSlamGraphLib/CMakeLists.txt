#######################################
# CMakeLists.txt for MiniSlamGraphLib #
#######################################

set(targetname MiniSlamGraphLib)

if (WITH_CSPARSE)
    include_directories(${CSPARSE_INCLUDE_DIRS})
    add_definitions(-DCOMPILE_WITH_CSPARSE)
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
add_library(${targetname} ${sources} ${headers})
target_link_libraries(${targetname} ORUtils ${CSparse_LIBRARIES})
