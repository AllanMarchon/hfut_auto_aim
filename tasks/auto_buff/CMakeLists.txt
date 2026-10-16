add_library(auto_buff STATIC
  buff_config.cpp
  hfut_buff_runtime.cpp
  detectors/njust_rune_detector.cpp
  detectors/climber_rune_detector.cpp
  rune_energy_fitter.cpp
  models/njust_rune_model.cpp
  models/sp_rune_model.cpp
  models/sp_rune_solver.cpp
  models/sp_rune_tracker.cpp
  rune_predictor.cpp
  aimers/njust_rune_aimer.cpp
  aimers/climber_rune_aimer.cpp
  planners/mpc_buff_tracking.cpp
  planners/mpc_buff_planner.cpp
  planners/njust_buff_planner.cpp)
target_link_libraries(auto_buff
  PUBLIC ${OpenCV_LIBS} yaml-cpp tools auto_aim
  PRIVATE ${HFUT_FMT_TARGET} openvino::runtime)
