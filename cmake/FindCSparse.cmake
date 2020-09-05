# FindCSparse.cmake
# This module defines target CSparse::CSparce
# based on CSparse_LIBRARY and CSparse_INCLUDE_DIR variables.

set(CSparse_ROOT "/usr" CACHE FILEPATH "Root directory of CSparse")

find_library(CSparse_LIBRARY NAMES cxsparse csparse PATHS "${CSparse_ROOT}/Lib" "C:/Program Files (x86)/CSparse/Lib" "C:/Program Files/CSparse/Lib" "${CSparse_ROOT}/Bin/x64-Release/" ${CMAKE_LIB_PATH})

find_path(CSparse_INCLUDE_DIR cs.h PATH "${CSparse_ROOT}/include/suitesparse" "${CSparse_ROOT}/include")

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(CSparse DEFAULT_MSG CSparse_LIBRARY CSparse_INCLUDE_DIR)

if(CSparse_FOUND)
  set(CSparse_LIBRARIES ${CSparse_LIBRARY})
  if (NOT TARGET CSparse::CSparse)
    add_library(CSparse::CSparse UNKNOWN IMPORTED)
  endif ()
  set_target_properties(
          CSparse::CSparse PROPERTIES
          IMPORTED_LOCATION "${CSparse_LIBRARY}"
          INTERFACE_INCLUDE_DIRECTORIES "${CSparse_INCLUDE_DIR}"
  )
endif()

mark_as_advanced(CSparse_LIBRARY
                 CSparse_INCLUDE_DIR)

