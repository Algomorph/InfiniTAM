include(HandleLocalPackage)

function(download_prebuilt_package)
		set(options)
    	set(oneValueArgs NAME ALTERNATIVE_LOCAL_NAME REQUIRE_FOUND HASH_ALGORITHM EXPECTED_HASH DOWNLOAD_URL ARCHIVE_FILENAME ROOT_VARIABLE)
    	set(multiValueArgs COMPONENTS OPTIONAL_COMPONENTS)
    	cmake_parse_arguments(DPP "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

		set(INSTALL_FOLDER ${CMAKE_CURRENT_BINARY_DIR}/${DPP_NAME})
		file(MAKE_DIRECTORY ${INSTALL_FOLDER})

		set(ARCHICE_FILE ${INSTALL_FOLDER}/${DPP_ARCHIVE_FILENAME})

		file(DOWNLOAD ${DPP_DOWNLOAD_URL} ${ARCHICE_FILE}
			INACTIVITY_TIMEOUT 4
			EXPECTED_HASH ${DPP_HASH_ALGORITHM}=${DPP_EXPECTED_HASH})
		
		if(CMAKE_VERSION VERSION_LESS 3.18)
    		execute_process(COMMAND ${CMAKE_COMMAND} -E tar -xf ${ARCHICE_FILE} WORKING_DIRECTORY ${INSTALL_FOLDER})
  		else()
    		file(ARCHIVE_EXTRACT INPUT ${ARCHICE_FILE} DESTINATION ${INSTALL_FOLDER})
  		endif()

  		if(DPP_ROOT_VARIABLE)
  			get_filename_component(EXTRACTED_FOLDER_NAME ${DPP_ARCHIVE_FILENAME} NAME_WLE)
  			set(${DPP_ROOT_VARIABLE} ${INSTALL_FOLDER}/${EXTRACTED_FOLDER_NAME})
  		endif()

		handle_local_package(
            NAME ${DPP_NAME}
            ALTERNATIVE_LOCAL_NAME ${DPP_ALTERNATIVE_LOCAL_NAME}
            REQUIRE_LOCAL_FOUND ${DPP_REQUIRE_FOUND}
            COMPONENTS ${DPP_COMPONENTS}
            OPTIONAL_COMPONENTS ${DPP_OPTIONAL_COMPONENTS}
        )
        if(PACKAGE_READY)
        	set(PACKAGE_READY TRUE PARENT_SCOPE)
        endif()
endfunction()

if(MSVC)
	function(download_prebuilt_package_msvc)
		set(options)
    	set(oneValueArgs NAME ALTERNATIVE_LOCAL_NAME REQUIRE_FOUND)
    	set(multiValueArgs COMPONENTS OPTIONAL_COMPONENTS)
    	cmake_parse_arguments(DPPM "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

		if(DPPM_NAME STREQUAL ffmpeg)
			download_prebuilt_package(
				NAME ffmpeg
				ALTERNATIVE_LOCAL_NAME ${DPPM_ALTERNATIVE_LOCAL_NAME}
				REQUIRE_FOUND ${DPPM_REQUIRE_FOUND}
				COMPONENTS ${DPPM_COMPONENTS}
				OPTIONAL_COMPONENTS ${DPPM_OPTIONAL_COMPONTENTS}
				HASH_ALGORITHM SHA256
				EXPECTED_HASH 019a81f79ef88f8465a807f8fa2a416836810d7775a933cbde519e2eff69ead9
				DOWNLOAD_URL https://ffmpeg.zeranoe.com/builds/win64/dev/ffmpeg-4.3-win64-dev.zip
				ARCHIVE_FILENAME ffmpeg-4.3-win64-dev.zip
				ROOT_VARIABLE "FFMPEG_ROOT"
			)
		else()
			message(FATAL_ERROR "Downloading of prebuilt package for dependency '${package_name}' is not supported.")
		endif()
		if(PACKAGE_READY)
        	set(PACKAGE_READY TRUE PARENT_SCOPE)
        endif()
	endfunction()
endif()