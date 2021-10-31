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

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/")
if (NOT CMAKE_CROSSCOMPILING)
    find_package(Ivy REQUIRED)
endif()
find_package(GeometryTools REQUIRED)

if (CMAKE_CROSSCOMPILING)
    include_directories(
        include
    )
else()
    include_directories(
        include
        ${Ivy_INCLUDE_DIRS}
    )
endif()

add_library(navigation src/PurePursuitControl.cpp src/RotationControl.cpp src/LinearControl.cpp)
target_link_libraries(navigation GeometryTools::GeometryTools)



if((NOT CMAKE_CROSSCOMPILING) AND ENABLE_TESTS)
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
    GeometryTools::GeometryTools
    )
    
    add_executable(navigation_tests test/navigation_tests.cpp src/Communication/Ivy.cpp)
    target_link_libraries(navigation_tests gtest_main navigation ${Ivy_LIBRARY_OPTIMIZED} GeometryTools::GeometryTools)
    include(GoogleTest)
    gtest_discover_tests(geometry_tests)
    gtest_discover_tests(navigation_tests)
endif()


set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
