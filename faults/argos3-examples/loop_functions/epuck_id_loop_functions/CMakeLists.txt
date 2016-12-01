add_library(epuck_id_loop_functions MODULE 
  epuck_id_qtuser_functions.h
  epuck_id_qtuser_functions.cpp)

target_link_libraries(epuck_id_loop_functions
  epuck_omega_algorithm
  argos3core_simulator
  argos3plugin_simulator_entities
  argos3plugin_simulator_epuck
  argos3plugin_simulator_qtopengl
  ${QT_LIBRARIES} ${GLUT_LIBRARY} ${OPENGL_LIBRARY})
