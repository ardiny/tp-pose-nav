add_library(marXbot MODULE Consts.cpp  Map.cpp marXbot.cpp parameters.cpp)
add_library(offlinemap MODULE Consts.cpp  Map.cpp offmarXbot.cpp) 
add_library(marXbottest MODULE marXbot_test.cpp) 
target_link_libraries(marXbot
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot)
target_link_libraries(marXbottest
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot)
target_link_libraries(offlinemap
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot
 )

