#########################
# FindLibRoyale.cmake   #
#########################

find_path(LibRoyale_ROOT royale_license.txt
	PATHS ${LibRoyale_ROOT} "C:/Program Files/royale/2.3.0.92" "/usr/local" "/opt")

find_library(LibRoyale_LIBRARY
	NAMES royale
	PATHS "${LibRoyale_ROOT}/lib" {CMAKE_LIB_PATH}
)

find_path(LibRoyale_INCLUDE_DIR royale.hpp 
	PATHS "${LibRoyale_ROOT}/include"
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(OpenNI2 DEFAULT_MSG LibRoyale_LIBRARY LibRoyale_INCLUDE_DIR)

if(LibRoyale_FOUND)
	if (NOT TARGET LibRoyale::LibRoyale)
		add_library(LibRoyale::LibRoyale UNKNOWN IMPORTED)
	endif ()
	set_target_properties(
			LibRoyale::LibRoyale PROPERTIES
			IMPORTED_LOCATION "${LibRoyale_LIBRARY}"
			INTERFACE_INCLUDE_DIRECTORIES "${LibRoyale_INCLUDE_DIR}"
	)
endif()