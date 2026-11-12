cmake_minimum_required(VERSION 3.16.0)

find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES CXX)

find_package(threepp CONFIG REQUIRED)

if(NOT DEFINED BUILD_SHARED_LIBS)
  set(BUILD_SHARED_LIBS ON)
endif()
if(WIN32)
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
  add_compile_definitions(NOMINMAX)
  add_compile_options($<$<CXX_COMPILER_ID:MSVC>:/bigobj>)
endif()

option(TESSERACT_THREEPP_ENABLE_TESTING "Build the unit tests" OFF)

find_package(tesseract REQUIRED COMPONENTS common geometry scene_graph state_solver collision kinematics environment
                                           visualization)

# The headers expose threepp, so this library and its users need C++20
add_library(tesseract_threepp src/conversions.cpp src/tesseract_scene.cpp)
target_compile_features(tesseract_threepp PUBLIC cxx_std_20)
target_compile_options(tesseract_threepp PRIVATE $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall -Wextra>)
target_link_libraries(tesseract_threepp PUBLIC threepp::threepp tesseract::visualization tesseract::environment
                                               tesseract::scene_graph tesseract::geometry tesseract::common)
target_include_directories(tesseract_threepp PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                    "$<INSTALL_INTERFACE:include>")

configure_package(NAMESPACE tesseract TARGETS tesseract_threepp
                  DEPENDENCIES threepp
                  "tesseract COMPONENTS common geometry scene_graph state_solver collision kinematics environment visualization")
install(DIRECTORY include/ DESTINATION include FILES_MATCHING PATTERN "*.h")

if(TESSERACT_THREEPP_ENABLE_TESTING)
  enable_testing()
  find_package(GTest REQUIRED)

  # tesseract installs its support data, which the tests load as package://tesseract/support
  get_filename_component(TESSERACT_SHARE_DIR "${tesseract_DIR}/../../../share" ABSOLUTE)

  # Neither needs OpenGL, so they run on every platform
  foreach(test IN ITEMS conversions_unit tesseract_scene_unit)
    add_executable(tesseract_threepp_${test} test/${test}.cpp)
    target_link_libraries(tesseract_threepp_${test} PRIVATE tesseract_threepp GTest::gtest_main)
    add_test(NAME tesseract_threepp_${test} COMMAND tesseract_threepp_${test})
    set_tests_properties(tesseract_threepp_${test} PROPERTIES ENVIRONMENT "TESSERACT_RESOURCE_PATH=${TESSERACT_SHARE_DIR}")
  endforeach()
endif()
