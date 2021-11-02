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

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")
if (NOT CMAKE_CROSSCOMPILING)
    find_package(Ivy REQUIRED)
endif()

include(FetchContent)
FetchContent_Declare(
  GeometryTools
  GIT_REPOSITORY https://github.com/rustyducks/geometry_tools.git
  GIT_TAG main
)
FetchContent_MakeAvailable(GeometryTools)

add_library(Navigation STATIC src/PurePursuitControl.cpp src/RotationControl.cpp src/LinearControl.cpp)
target_link_libraries(Navigation PUBLIC GeometryTools)
target_include_directories(Navigation PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>  
    $<INSTALL_INTERFACE:include>
)
set_target_properties(Navigation PROPERTIES PUBLIC_HEADER 
"include/Navigation/LinearControl.h;include/Navigation/Parameters.h;include/Navigation/PositionControlBase.h;include/Navigation/PurePursuitControl.h;include/Navigation/RotationControl.h")



### Tests
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
    GeometryTools
    )
    
    add_executable(navigation_tests test/navigation_tests.cpp src/Communication/Ivy.cpp)
    target_link_libraries(navigation_tests gtest_main Navigation ${Ivy_LIBRARY_OPTIMIZED} GeometryTools)
    include(GoogleTest)
    gtest_discover_tests(geometry_tests)
    gtest_discover_tests(navigation_tests)
endif()

### Pack
set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)


### Install
install(TARGETS Navigation
        EXPORT NavigationTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        INCLUDES DESTINATION include
        PUBLIC_HEADER DESTINATION include/Navigation

)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    NavigationConfigVersion.cmake
    VERSION ${PACKAGE_VERSION}
    COMPATIBILITY AnyNewerVersion
)

install(EXPORT NavigationTargets
        FILE NavigationTargets.cmake
        NAMESPACE Navigation::
        DESTINATION lib/cmake/Navigation
)

configure_file(cmake/NavigationConfig.cmake.in NavigationConfig.cmake @ONLY)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/NavigationConfig.cmake"
              "${CMAKE_CURRENT_BINARY_DIR}/NavigationConfigVersion.cmake"
        DESTINATION lib/cmake/Navigation
        )