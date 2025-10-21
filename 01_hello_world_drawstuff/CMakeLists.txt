add_executable(01_hello_world_drawstuff main.cpp ../common/drawstuff_viewer.cpp)
target_include_directories(01_hello_world_drawstuff PRIVATE ${CMAKE_SOURCE_DIR}/common)
target_link_libraries(01_hello_world_drawstuff ${ODE_LIBRARY} drawstuff X11)
