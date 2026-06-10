cmake_minimum_required(VERSION 3.16)
project(gravity_sim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Use a system GLFW if one is installed; otherwise download and build it.
find_package(glfw3 3.3 QUIET)
if(NOT glfw3_FOUND)
    include(FetchContent)
    set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
    set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    FetchContent_Declare(
        glfw
        GIT_REPOSITORY https://github.com/glfw/glfw.git
        GIT_TAG 3.4
    )
    FetchContent_MakeAvailable(glfw)
endif()

find_package(OpenGL REQUIRED)

add_executable(gravity_sim
    src/main.cpp
    src/Simulation.cpp
)

target_include_directories(gravity_sim PRIVATE include)
target_link_libraries(gravity_sim PRIVATE glfw OpenGL::GL)
target_compile_options(gravity_sim PRIVATE -Wall -Wextra)
