####################
# LinkBoost.cmake #
####################

if(Boost_FOUND)
    target_compile_definitions(${targetname} PUBLIC WITH_BOOST)
    target_link_libraries(${targetname} PUBLIC ${Boost_LIBRARIES})
    target_include_directories(${targetname} PUBLIC ${Boost_INCLUDE_DIRS})
    # windows-specific stuff
    if (CMAKE_CXX_COMPILER_ID MATCHES MSVC)
        target_compile_definitions(${targetname} PUBLIC -DBOOST_ALL_NO_LIB -DBOOST_SYSTEM_NO_DEPRECATED)
        link_directories(${Boost_LIBRARY_DIRS})
        # disable warnings about too few arguments for function-like macro invocation -- these stem from branches in the preprocessor code that are never taken
        target_compile_options(${targetname} PUBLIC $<$<COMPILE_LANGUAGE:CXX>:/wd4003>)
    endif ()
endif()
