add_executable(02_falling_sphere_drawstuff main.cpp ../common/drawstuff_viewer.cpp)
target_include_directories(02_falling_sphere_drawstuff PRIVATE ${CMAKE_SOURCE_DIR}/common)
target_link_libraries(02_falling_sphere_drawstuff ${ODE_LIBRARY} drawstuff X11)
