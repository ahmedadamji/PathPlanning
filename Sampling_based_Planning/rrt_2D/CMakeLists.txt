cmake_minimum_required(VERSION 3.12)
project(SamplingBasedPlanning2D)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(SOURCE_DIR "${PROJECT_SOURCE_DIR}/src")
set(INCLUDE_DIR "${PROJECT_SOURCE_DIR}/include")

# Include OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# Add the path to the directory containing Env.h and the source files
set(SOURCES
    ${SOURCE_DIR}/main.cpp
    ${SOURCE_DIR}/Env.cpp
    ${SOURCE_DIR}/Plotting.cpp
    ${SOURCE_DIR}/AStar.cpp
    )

# Include header files
include_directories(${INCLUDE_DIR})

# Create an executable target for the main.cpp file
add_executable(${PROJECT_NAME} ${SOURCES})

# Link the OpenCV libraries
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} Threads::Threads)
