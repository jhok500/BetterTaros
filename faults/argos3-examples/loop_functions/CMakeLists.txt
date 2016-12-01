include_directories(${CMAKE_SOURCE_DIR})

add_subdirectory(epuck_omega_algorithm_loop_functions)

# If Qt+OpenGL dependencies were found, descend into these directories
if(ARGOS_COMPILE_QTOPENGL)
  add_subdirectory(epuck_id_loop_functions)
endif(ARGOS_COMPILE_QTOPENGL)