cmake_minimum_required(VERSION 3.16.0)

find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES CXX)

find_package(threepp CONFIG REQUIRED)

# Plugins must be shared libraries, and the plugin links this
if(NOT DEFINED BUILD_SHARED_LIBS)
  set(BUILD_SHARED_LIBS ON)
endif()
if(WIN32)
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
  add_compile_definitions(NOMINMAX)
  add_compile_options($<$<CXX_COMPILER_ID:MSVC>:/bigobj>)
endif()

option(TESSERACT_VISUALIZATION_THREEPP_ENABLE_TESTING "Build the unit tests" OFF)

find_package(tesseract REQUIRED COMPONENTS common geometry scene_graph state_solver collision kinematics environment
                                           visualization)
find_package(Threads REQUIRED)

# threepp needs C++20, but only inside this library. The public header stays C++17.
add_library(tesseract_visualization_threepp src/conversions.cpp src/threepp_visualization.cpp)
target_compile_features(tesseract_visualization_threepp PUBLIC cxx_std_17)
target_compile_options(tesseract_visualization_threepp PRIVATE $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall -Wextra>)
target_link_libraries(tesseract_visualization_threepp
                      PUBLIC tesseract::visualization tesseract::environment tesseract::scene_graph tesseract::geometry tesseract::common
                      PRIVATE threepp::threepp Threads::Threads)
target_include_directories(tesseract_visualization_threepp PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                                  "$<INSTALL_INTERFACE:include>")

add_library(tesseract_visualization_threepp_plugin SHARED src/threepp_visualization_plugin.cpp)
target_compile_options(tesseract_visualization_threepp_plugin PRIVATE $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall -Wextra>)
target_link_libraries(tesseract_visualization_threepp_plugin PRIVATE tesseract_visualization_threepp)

add_executable(tesseract_visualization_threepp_abb_irb2400_viewer examples/abb_irb2400_viewer.cpp)
target_link_libraries(tesseract_visualization_threepp_abb_irb2400_viewer PRIVATE tesseract_visualization_threepp)

configure_package(NAMESPACE tesseract
                  TARGETS tesseract_visualization_threepp tesseract_visualization_threepp_plugin
                          tesseract_visualization_threepp_abb_irb2400_viewer
                  DEPENDENCIES "tesseract COMPONENTS common geometry scene_graph state_solver collision kinematics environment visualization")
install(DIRECTORY include/ DESTINATION include FILES_MATCHING PATTERN "*.h")

if(TESSERACT_VISUALIZATION_THREEPP_ENABLE_TESTING)
  enable_testing()
  find_package(GTest REQUIRED)

  # tesseract installs its support data, which the tests load as package://tesseract/support
  get_filename_component(TESSERACT_SHARE_DIR "${tesseract_DIR}/../../../share" ABSOLUTE)

  # conversions.h exposes threepp, unlike the viewer's header
  add_executable(tesseract_visualization_threepp_conversions_unit test/conversions_unit.cpp)
  target_link_libraries(tesseract_visualization_threepp_conversions_unit PRIVATE tesseract_visualization_threepp threepp::threepp GTest::gtest_main)
  add_test(NAME tesseract_visualization_threepp_conversions_unit COMMAND tesseract_visualization_threepp_conversions_unit)
  set_tests_properties(tesseract_visualization_threepp_conversions_unit PROPERTIES ENVIRONMENT "TESSERACT_RESOURCE_PATH=${TESSERACT_SHARE_DIR}")

  # These render, so they need EGL, which is Linux only
  if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    add_executable(tesseract_visualization_threepp_unit test/threepp_unit.cpp)
    target_link_libraries(tesseract_visualization_threepp_unit PRIVATE tesseract_visualization_threepp GTest::gtest_main)
    add_test(NAME tesseract_visualization_threepp_unit COMMAND tesseract_visualization_threepp_unit)
    set_tests_properties(tesseract_visualization_threepp_unit PROPERTIES
      ENVIRONMENT "TESSERACT_VISUALIZATION_PLUGIN_DIRECTORIES=$<TARGET_FILE_DIR:tesseract_visualization_threepp_plugin>;TESSERACT_VISUALIZATION_PLUGINS=tesseract_visualization_threepp_plugin")
  endif()
endif()
