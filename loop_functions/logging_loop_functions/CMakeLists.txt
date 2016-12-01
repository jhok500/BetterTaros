link_directories(${CMAKE_BINARY_DIR}/controllers/footbot_diffusion)
set(logging_loop_functions_SOURCES logging_loop_functions.cpp)

add_library(logging_loop_functions MODULE ${logging_loop_functions_SOURCES} ${Boost_LIBRARIES} )
target_link_libraries(logging_loop_functions
        footbot_diffusion
        argos3core_simulator
        argos3plugin_simulator_dynamics2d
        argos3plugin_simulator_entities
        argos3plugin_simulator_footbot
        argos3plugin_simulator_genericrobot)