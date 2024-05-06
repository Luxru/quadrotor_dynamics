cmake_minimum_required(VERSION 3.0)
project(quadrotor)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_DEBUG_FLAGS "-g")
set(CMAKE_CXX_RELEASE_FLAGS "-O3 -fPIC")
# Debug mode,set build type to Debug
set(CMAKE_BUILD_TYPE RELEASE)

include(FetchContent)

FetchContent_Declare(
  yaml-cpp
  GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
  GIT_TAG yaml-cpp-0.7.0 # Can be a tag (yaml-cpp-x.x.x), a commit hash, or a branch name (master)
)
FetchContent_GetProperties(yaml-cpp)

FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

if(NOT yaml-cpp_POPULATED)
  message(STATUS "Fetching yaml-cpp...")
  FetchContent_Populate(yaml-cpp)
  add_subdirectory(${yaml-cpp_SOURCE_DIR} ${yaml-cpp_BINARY_DIR})
endif()

# find all cpp files in the src directory
file(GLOB SOURCES "src/*/*.cpp")

add_library(quadrotor SHARED
    ${SOURCES} 
)

target_include_directories(quadrotor PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(quadrotor PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../third_party) # Include the yaml-cpp headers
target_link_libraries(quadrotor PUBLIC yaml-cpp::yaml-cpp) # The library or executable that require yaml-cpp library

add_executable(quadrotor_test main.cpp)
target_link_libraries(quadrotor_test quadrotor)

enable_testing()
# Add test executable
# Test files are in the test directory
file(GLOB TEST_SOURCES "tests/*/*.cpp")

add_executable(
  quad_test
  ${TEST_SOURCES}
)

target_link_libraries(
  quad_test
  GTest::gtest_main
  quadrotor
)

include(GoogleTest)
gtest_discover_tests(quad_test)