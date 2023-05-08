cmake_minimum_required(VERSION 3.1)

# Set project name
project(MyProject)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

# Find OpenCV package
find_package(OpenCV REQUIRED)

# Find Eigen package
find_package(Eigen3 REQUIRED)


# Find Python package
find_package(PythonLibs REQUIRED)

# Add executable target for main.cpp
add_executable(speedProfile main.cpp)

# Add include directories for the lib folder, OpenCV and Eigen
include_directories(lib ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${PYTHON_INCLUDE_DIRS})

# Add library target for all cpp files in the lib folder
file(GLOB_RECURSE LIB_SOURCES "lib/*.cpp")
add_library(MyLib ${LIB_SOURCES})

# Link MyLib with OpenCV and Eigen
target_link_libraries(MyLib ${OpenCV_LIBS} Eigen3::Eigen ${PYTHON_LIBRARIES})

# Link the executable target with the VisionLib library, OpenCV and Eigen
target_link_libraries(speedProfile MyLib ${OpenCV_LIBS} Eigen3::Eigen ${PYTHON_LIBRARIES})

