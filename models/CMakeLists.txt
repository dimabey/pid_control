cmake_minimum_required(VERSION 3.0.0)
project(pid_controller VERSION 0.1.0)
set(CMAKE_CXX_STANDARD 20)
set (CMAKE_GENERATOR "Unix Makefiles" CACHE INTERNAL "" FORCE)

add_library(
    models
    pid_controller.hpp
    plant.hpp
    rate_limiter.hpp
)

set_target_properties(models PROPERTIES LINKER_LANGUAGE CXX)

target_compile_definitions(models PUBLIC MODELS=1)