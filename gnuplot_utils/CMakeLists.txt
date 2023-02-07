cmake_minimum_required(VERSION 3.0.0)
project(pid_controller VERSION 0.1.0)
set(CMAKE_CXX_STANDARD 20)

add_library(
    gnuplot_utils
    gp_data_writer.hpp
)

set_target_properties(gnuplot_utils PROPERTIES LINKER_LANGUAGE CXX)

target_compile_definitions(gnuplot_utils PUBLIC MODELS=1)