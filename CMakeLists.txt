cmake_minimum_required(VERSION 3.15) # Adjust based on your CMake version

project(MuJoCoExample LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17) # Or your preferred C++ standard
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

# MuJoCo Installation Path (Adjust this to your MuJoCo installation)
set(MUJOCO_PATH "/home/dell/Software/mujoco")

if (NOT EXISTS "${MUJOCO_PATH}")
    message(FATAL_ERROR "MuJoCo path not found: ${MUJOCO_PATH}")
endif()

# Include directories
include_directories("${MUJOCO_PATH}/include")

# Link libraries
link_directories("${MUJOCO_PATH}/lib")

# Source files
add_executable(LiCAS_A1_Control Main.cpp)

# Link MuJoCo libraries
target_link_libraries(LiCAS_A1_Control glfw mujoco)

# Optional: Copy model files to the build directory (if needed)

# Example for copying a model file named "model.xml"
# configure_file(model.xml ${CMAKE_BINARY_DIR}/model.xml COPYONLY)

# Example of copying all xml files in a directory named "models"
# file(GLOB MODEL_FILES "models/*.xml")
# foreach(MODEL_FILE ${MODEL_FILES})
#     get_filename_component(MODEL_NAME ${MODEL_FILE} NAME)
#     configure_file(${MODEL_FILE} ${CMAKE_BINARY_DIR}/${MODEL_NAME} COPYONLY)
# endforeach()

