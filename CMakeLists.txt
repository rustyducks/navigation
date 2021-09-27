cmake_minimum_required(VERSION 3.14)

option(CROSSCOMPILE_ARM "Cross compiling for arm architecture" OFF)
option(ENABLE_TESTS "Enable tests compiling" OFF)

if(CROSSCOMPILE_ARM)
    if(DEFINED ENV{LINUX_ARM_TOOLCHAIN_PATH})
        set (LINUX_ARM_TOOLCHAIN_PATH $ENV{LINUX_ARM_TOOLCHAIN_PATH})
    else()
        set (LINUX_ARM_TOOLCHAIN_PATH /usr/lib/ccache)
    endif()
    set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/arm_build.cmake)
endif()

project(navigation VERSION 0.1.0)

add_compile_options(-Wall -Wextra -pedantic -Werror)
set (CMAKE_CXX_STANDARD 11)

find_package(Eigen3 REQUIRED)

include_directories(
    include
    ${Eigen3_INCLUDE_DIRS}
)

add_library(geometry_tools src/Geometry/Point.cpp src/Geometry/Trajectory.cpp src/Geometry/Speed.cpp)

add_executable(navigation test/navigation.cpp src/PurePursuitControl.cpp src/RotationControl.cpp src/LinearControl.cpp)
target_link_libraries(navigation geometry_tools)



if(NOT CMAKE_CROSSCOMPILING AND ENABLE_TESTS)
    include(FetchContent)
    FetchContent_Declare(
      googletest
      URL https://github.com/google/googletest/archive/609281088cfefc76f9d0ce82e1ff6c30cc3591e5.zip
    )
    FetchContent_MakeAvailable(googletest)

    enable_testing()

    add_executable(
    geometry_tests
    test/geometry_tests.cpp
    )
    target_link_libraries(
    geometry_tests
    gtest_main
    geometry_tools
    )
    include(GoogleTest)
    gtest_discover_tests(geometry_tests)
endif()


set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
