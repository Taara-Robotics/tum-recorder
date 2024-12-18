cmake_minimum_required(VERSION 3.1)
project(tum-recorder LANGUAGES CXX)

# ----- Find dependencies -----

# filesystem
set(filesystem_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/3rd/filesystem/include)

# popl
set(popl_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/3rd/popl/include)

# Orbbec SDK
find_package(OrbbecSDK REQUIRED)

# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# ----- Build example executables -----

set(EXECUTABLE_TARGETS "")

add_executable(record_orbbec src/record_orbbec.cc)
target_link_libraries(record_orbbec PRIVATE OrbbecSDK::OrbbecSDK)
list(APPEND EXECUTABLE_TARGETS record_orbbec)

foreach(EXECUTABLE_TARGET IN LISTS EXECUTABLE_TARGETS)
    # Set output directory for executables
    set_target_properties(${EXECUTABLE_TARGET} PROPERTIES
                          RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}"
                          RUNTIME_OUTPUT_DIRECTORY_DEBUG "${PROJECT_BINARY_DIR}"
                          RUNTIME_OUTPUT_DIRECTORY_RELEASE "${PROJECT_BINARY_DIR}"
                          RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL "${PROJECT_BINARY_DIR}"
                          RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO "${PROJECT_BINARY_DIR}")

    # Link OpenCV (for waitKey)
    target_link_libraries(${EXECUTABLE_TARGET} PRIVATE opencv_core opencv_imgproc opencv_imgcodecs opencv_highgui)

    # include 3rd party library headers
    target_include_directories(${EXECUTABLE_TARGET}
                               PRIVATE
                               $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/3rd/popl/include>
                               $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/3rd/filesystem/include>)
endforeach()
