cmake_minimum_required(VERSION 3.20)
project(bad_day_in_orbit LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

option(BDIO_BUILD_TESTS "Build unit tests" ON)
option(BDIO_SANITIZE "Build with address and undefined behaviour sanitizers" OFF)

add_library(bdio_warnings INTERFACE)
target_compile_options(bdio_warnings INTERFACE
  -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wno-sign-conversion -Werror)

if(BDIO_SANITIZE)
  add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer)
  add_link_options(-fsanitize=address,undefined)
endif()

add_library(bdio
  src/packet.cpp
  src/spacecraft.cpp
  src/faults.cpp
  src/scheduler.cpp
  src/naive_controller.cpp
  src/hardened_controller.cpp
  src/harness.cpp
)
target_include_directories(bdio PUBLIC include)
target_link_libraries(bdio PRIVATE bdio_warnings)

foreach(app bdio_run bdio_campaign bdio_bench)
  add_executable(${app} apps/${app}.cpp)
  target_link_libraries(${app} PRIVATE bdio bdio_warnings)
endforeach()

if(BDIO_BUILD_TESTS)
  include(FetchContent)
  FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.15.2.tar.gz
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  )
  set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)

  enable_testing()
  add_executable(bdio_tests
    tests/test_crc_packet.cpp
    tests/test_tmr.cpp
    tests/test_hardened.cpp
    tests/test_naive.cpp
    tests/test_harness.cpp
    tests/test_scheduler.cpp
    tests/test_no_alloc.cpp
  )
  target_link_libraries(bdio_tests PRIVATE bdio bdio_warnings GTest::gtest_main)
  include(GoogleTest)
  gtest_discover_tests(bdio_tests)
endif()
