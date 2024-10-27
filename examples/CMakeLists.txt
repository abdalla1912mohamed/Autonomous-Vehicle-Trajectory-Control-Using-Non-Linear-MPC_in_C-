# Set the minimum required version of CMake
cmake_minimum_required(VERSION 3.10)  # Updated minimum version

# Set the project name
project(examples)

# Enable C++20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)  # Ensure the standard is required

# Enable optimization
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

# Disable eigen stack allocation warning
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DEIGEN_STACK_ALLOCATION_LIMIT=0")

# Enable OpenMP (uncomment if OpenMP is needed)
# find_package(OpenMP REQUIRED)
# if(OpenMP_CXX_FOUND)
#     set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
# endif()

# Find the mpc++ package
find_package(mpc++ CONFIG REQUIRED)

# Include the mpc++ headers
include_directories(${mpc++_INCLUDE_DIRS})

# Find Eigen
find_package(Eigen3 3.3 REQUIRED NO_MODULE)  # Adjust version if needed
include_directories(${EIGEN3_INCLUDE_DIR})

# Get all the .cpp files in the directory
file(GLOB CPP_FILES "*.cpp")

# Put the executables in the bin directory in the build directory
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# Create a target for each .cpp file
foreach(CPP_FILE ${CPP_FILES})
    # Get the file name without extension
    get_filename_component(TARGET_NAME ${CPP_FILE} NAME_WE)

    # Write the file name to the console
    message(STATUS "Adding target for ${CPP_FILE} -> ${TARGET_NAME}")

    # Add the target
    add_executable(${TARGET_NAME} ${CPP_FILE})
    target_link_libraries(${TARGET_NAME} mpc++)
endforeach()
