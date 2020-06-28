if(MSVC)
	function(download_prebuilt_ffmpeg_msvc)
		set(NAME ffmpeg)
		set(INSTALL_FOLDER ${CMAKE_CURRENT_BINARY_DIR}/${NAME})
		set(DOWNLOAD_URL https://ffmpeg.zeranoe.com/builds/win64/dev/ffmpeg-4.3-win64-dev.zip)
		set(EXPECTED_SHA256 "33dcf2e4edd73d26cd8fb6d6decfe315d73ad81cd71cf849502395844b9e0a1b")

		set(LIBRARY_LOCATION ${INSTALL_FOLDER}/ffmpeg-4.3-win64-dev/lib)
		set(IMPORTED_LOCATION
			${LIBRARY_LOCATION}/avcodec.lib
			${LIBRARY_LOCATION}/avdevice.lib
			${LIBRARY_LOCATION}/avfilter.lib
			${LIBRARY_LOCATION}/avformat.lib
			${LIBRARY_LOCATION}/avutil.lib
			${LIBRARY_LOCATION}/swresample.lib
			${LIBRARY_LOCATION}/swscale.lib
			)
		set(INTERFACE_INCLUDE_DIRECTORIES ${INSTALL_FOLDER}/ffmpeg-4.3-win64-dev/include)

		file(MAKE_DIRECTORY ${INSTALL_FOLDER})
		set(ARCHICE_FILE ${INSTALL_FOLDER}/ffmpeg-4.3-win64-dev.zip)
		file(DOWNLOAD ${DOWNLOAD_URL} ${ARCHICE_FILE}
			SHOW_PROGRESS INACTIVITY_TIMEOUT 4
			EXPECTED_HASH SHA256=${EXPECTED_SHA256})
		
		if(CMAKE_VERSION VERSION_LESS 3.18)
    		execute_process(COMMAND ${CMAKE_COMMAND} -E tar -xf ${ARCHICE_FILE} WORKING_DIRECTORY ${INSTALL_FOLDER})
  		else()
    		file(ARCHIVE_EXTRACT INPUT ${ARCHICE_FILE} DESTINATION ${INSTALL_FOLDER})
  		endif()
  		
  		add_library(${NAME} STATIC IMPORTED GLOBAL)
  		set_target_properties(${NAME} PROPERTIES
                              IMPORTED_LINK_INTERFACE_LANGUAGES_DEBUG "CXX;RC"
                              IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX;RC"
                              INTERFACE_INCLUDE_DIRECTORIES ${INTERFACE_INCLUDE_DIRECTORIES}
                              IMPORTED_LOCATION_RELWITHDEBINFO ${IMPORTED_LOCATION}
                              IMPORTED_LOCATION_RELEASE ${IMPORTED_LOCATION}
                              IMPORTED_LOCATION_DEBUG ${IMPORTED_LOCATION})
	endfunction()

	function(download_prebuilt_msvc PACKAGE_NAME)
		if(PACKAGE_NAME STREQUAL ffmpeg)
			download_prebuilt_ffmpeg_msvc()
		else()
			message(FATAL_ERROR "Downloading of prebuilt package for dependency '${package_name}' is not supported.")
		endif()
	endfunction()
endif()