#################
# LinkPNG.cmake #
#################

if(WITH_PNG)
  target_link_libraries(${targetname} png)
  target_compile_definitions(${targetname} PUBLIC -DUSE_LIBPNG)
endif()
