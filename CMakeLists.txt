cmake_minimum_required(VERSION 3.0.0)
project(pid_controller VERSION 0.1.0)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_GENERATOR "Unix Makefiles" CACHE INTERNAL "" FORCE)

add_subdirectory(models)
add_subdirectory(gnuplot-iostream)
add_subdirectory(gnuplot_utils)

configure_file(plot_pid_controller.zsh ${CMAKE_BINARY_DIR} COPYONLY)
configure_file(Makefile ${CMAKE_BINARY_DIR} COPYONLY)

add_executable(pid_controller main.cpp)
target_link_libraries(pid_controller
    models
    gnuplot_iostream
    )


