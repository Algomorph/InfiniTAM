## https://stackoverflow.com/questions/32183975/how-to-print-all-the-properties-of-a-target-in-cmake/56738858#56738858
## https://stackoverflow.com/a/56738858/3743145

## Get all properties that CMake supports
execute_process(COMMAND cmake --help-property-list OUTPUT_VARIABLE CMAKE_PROPERTY_LIST)
## Convert command output into a CMake list
string(REGEX REPLACE ";" "\\\\;" CMAKE_PROPERTY_LIST "${CMAKE_PROPERTY_LIST}")
string(REGEX REPLACE "\n" ";" CMAKE_PROPERTY_LIST "${CMAKE_PROPERTY_LIST}")

list(REMOVE_DUPLICATES CMAKE_PROPERTY_LIST)

## Provided for potential debugging purposes on CMake targets
function(print_target_properties tgt)
    if (NOT TARGET ${tgt})
        message("There is no target named '${tgt}'")
        return()
    endif ()

    if (MSVC)
        foreach (prop ${CMAKE_PROPERTY_LIST})
            set(propval)
            if (prop MATCHES "^.*_<CONFIG>$")
                string(REPLACE "<CONFIG>" "Debug" prop_DEBUG ${prop})
                get_target_property(propval ${tgt} ${prop_DEBUG})
                if (propval)
                    message("${tgt} ${prop_DEBUG} = ${propval}")
                endif ()
                string(REPLACE "<CONFIG>" "Release" prop_RELEASE ${prop})
                get_target_property(propval ${tgt} ${prop_RELEASE})
                if (propval)
                    message("${tgt} ${prop_RELEASE} = ${propval}")
                endif ()
            else ()
                get_target_property(propval ${tgt} ${prop})
                if (propval)
                    message("${tgt} ${prop} = ${propval}")
                endif ()
            endif ()
        endforeach (prop)
    else ()
        foreach (prop ${CMAKE_PROPERTY_LIST})
            string(REPLACE "<CONFIG>" "${CMAKE_BUILD_TYPE}" prop ${prop})
            get_target_property(propval ${tgt} ${prop})
            if (propval)
                message("${tgt} ${prop} = ${propval}")
            endif ()
        endforeach (prop)
    endif ()
endfunction(print_target_properties)