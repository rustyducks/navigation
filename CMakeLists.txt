cmake_minimum_required(VERSION 3.0.0)

option(CROSSCOMPILE_ARM "Cross compiling for arm architecture" OFF)

if(CROSSCOMPILE_ARM)
    if(DEFINED ENV{LINUX_ARM_TOOLCHAIN_PATH})
        set (LINUX_ARM_TOOLCHAIN_PATH $ENV{LINUX_ARM_TOOLCHAIN_PATH})
    else()
        set (LINUX_ARM_TOOLCHAIN_PATH /usr/lib/ccache)
    endif()
    set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/arm_build.cmake)
endif()

project(navigation VERSION 0.1.0)

if(NOT CMAKE_CROSSCOMPILING)
    include(CTest)
    enable_testing()
endif()

add_compile_options(-Wall -Wextra -pedantic -Werror)
set (CMAKE_CXX_STANDARD 11)

find_package(Eigen3 REQUIRED)

include_directories(
    include
    ${Eigen3_INCLUDE_DIRS}
)

add_library(geometry_tools src/Geometry/Point.cpp src/Geometry/Trajectory.cpp src/Geometry/Speed.cpp)

add_executable(navigation tests/navigation.cpp src/PurePursuitControl.cpp src/RotationControl.cpp src/LinearControl.cpp)
target_link_libraries(navigation geometry_tools)


set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
