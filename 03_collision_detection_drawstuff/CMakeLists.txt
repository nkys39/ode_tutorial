add_executable(03_collision_detection_drawstuff main.cpp ../common/drawstuff_viewer.cpp)
target_include_directories(03_collision_detection_drawstuff PRIVATE ${CMAKE_SOURCE_DIR}/common)
target_link_libraries(03_collision_detection_drawstuff ${ODE_LIBRARY} drawstuff X11)
