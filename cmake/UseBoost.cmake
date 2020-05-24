###################
# UseBoost.cmake #
###################

set(Boost_FIND_REQUIRED TRUE)
set(Boost_FIND_QUIETLY TRUE)
set(Boost_USE_MULTITHREADED TRUE)
if (CMAKE_CXX_COMPILER_ID MATCHES MSVC)
    set(Boost_USE_STATIC_LIBS TRUE)
    set(Boost_USE_STATIC_RUNTIME OFF)
endif ()
find_package(Boost REQUIRED COMPONENTS filesystem program_options)
