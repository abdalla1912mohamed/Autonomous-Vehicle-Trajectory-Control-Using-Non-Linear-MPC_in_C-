# libmpc++
**libmpc++** is a C++ library to solve linear and non-linear MPC. The library is written in modern **C++20** and it
is tested to work on Linux, macOS and Windows.
### Requirements
* gcc version (>= 10.3.0)

The libmpc++ website can be found at the following link: https://altairlab.gitlab.io/optcontrol/libmpc/

Starting from the version **0.5.0** the library is also available as a Python package (https://pypi.org/project/pympcxx/). The Python package is available on PyPI and can be installed using pip:
```
pip install pympcxx
```

## Dependecies
The library depends on the following external libraries which must be installed on the machine before using libmpc++

* *Eigen3* header-only linear algebra library (https://eigen.tuxfamily.org/index.php?title=Main_Page)
* *NLopt* set of solvers for nonlinear programming (https://nlopt.readthedocs.io/en/latest/)
* *OSQP* solver for convex quadratic programming (https://osqp.org)

If you are a developer, to setup the debug environment you also need to install:
- `boost` for stacktrace debug (https://www.boost.org)
- `Catch2` test suite (https://github.com/catchorg/Catch2)
- `gcovr`, `lcov` GCC code coverage

## Example
There is also a collection of examples that guide you through the most important features of **libmpc++** in the `examples` folder

* `quadrotor_ex.cpp`: example of a linear MPC for a quadrotor regulation problem
* `ugv_ex.cpp`: example of a non-linear MPC for a UGV tracking problem with obstacles
* `vanderpol_ex.cpp`: example of a non-linear MPC for a Van der Pol oscillator regulation problem
* `networked_oscillator_ex.cpp`: example of a non-linear MPC for a set of coupled Van der Pol oscillator regulation problem

The examples are written in C++ and can be compiled using CMake:

```
mkdir build
cd build
cmake ..
make
```

## Usage
The latest version of libmpc++ is available from GitHub https://github.com/nicolapiccinelli/libmpc/releases and does not require any
installation process other than the one required by its dependecies.

### System wide usage on an Ubuntu Linux
If you're a developer run the `configure.sh` script with superuser privileges to install the external dependencies
```
sudo ./configure.sh
```
or `configure.sh --disable-test` if you do not need the test-suite
```
sudo ./configure.sh --disable-test
```
Then, after installing all the dependencies configure and compile the library using CMake. From the folder containing
the CMakeLists.txt file compile the library by doing
```
mkdir build
cd build
cmake ..
make
```
Finally, install the library with superuser privileges
```
sudo make install
```
This will allow you to install the MPC++ library on your system. Using the include 
```
#include <mpc/LMPC.hpp> 
```
in your package will be enough to add this library to your project.

### CMakelists.txt example
This is an example of a CMakeLists.txt file to use libmpc++ in your project.

```cmake
cmake_minimum_required(VERSION 3.0)
project(your_project_name)

# set the C++ standard to C++ 20
set(CMAKE_CXX_STANDARD 20)
# set the C++ compiler to use O3
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

find_package(mpc++ CONFIG REQUIRED)

# Declare a C++ library
include_directories(${mpc++_INCLUDE_DIRS})
add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries(${PROJECT_NAME} mpc++)
```

## Development Container
Docker is helpful for creating a stable localized development environment. Install Docker with the convenience script:
```console
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
```

You can build the development image with the command below. Note that the image will be tagged with the name specified after the '-t' option.
```console
docker build -t dev_image .
```



###readme_abdalla

READ_ME_FILE 
Abdalla Mohamed 
////// install the dependencies and set up  the MPC main Library /////////////

extract  MAIN_DIRECTORY 
cd lib 
cd libmpc 

sudo ./configure.sh   //  run the configure.sh script with superuser privileges to install the external dependencies 
or configure.sh --disable-test if you do not need the test-suite

sudo ./configure.sh --disable-test
Then, after installing all the dependencies configure and compile the library using CMake. From the folder containing the CMakeLists.txt file compile the library by doing

mkdir build
cd build
cmake ..
make
Finally, install the library with superuser privileges

sudo make install
This will allow you to install the MPC++ library on your system. Using the include
/////////////////////////////////////////////////////////////////////////////////////////////// // if it doesnt work git clone https://github.com/nicolapiccinelli/libmpc
#include <mpc/LMPC.hpp> 


└── main_directory
    └── lib
        └── libmpc
            └── examples

cd MPC_ISA
cd MAIN_DIRECTORY
cd lib
cd libmpc
cd examples
mkdir build && cd build // make a build folder then change directory to build
cmake .. 
make  
cd bin 
ls
./MPC_CHANGING_LANES_VARYING_VELOCITY    or  ./MPC_SOLVER_LANE_CHANGE_CONSTANT_VELOCITY or  ./MPC_solver_trajectory_const_vel    ./ MPC_TRAJECTORY                                                      run any of the four testing scripts 

1) MPC_CHANGING_LANES_VARYING_VELOCITY  -- > the lane changes the lane and varies its velocity doing both longitidunal and lateral control sucessfully 
after solving a csv file : MPC_CHANGING_LANES_VARYING_VELOCITY.csv" is generated in the bin folder ( same folder as the executable) 



2) MPC_SOLVER_LANE_CHANGE_CONSTANT_VELOCITY ---> changes the lane while keeping constant velocity 
a csv file  "MPC_CHANGING_LANES_constant_VELOCITY.csv" containing all the model paramters is generated 

3) MPC_solver_trajectory_const_vel  --> a constant velocity is initialized with a trajectory track of a polynorimal of n = 3 
a csv file is generated "trajectory.csv" . 

4) ./ MPC_TRAJECTORY ----> a trajectory polynoimal control as well as a reference velocity is supplied to the model 
a csv file of all data is generated trajectory.csv


You can plot by the matlibplot scripts supplied within the zip folder . 

:: reference library : https://github.com/nicolapiccinelli/libmpc



// old repo --> contains an obj oriented model and mpc solver implemented using singleon patterns and trying to solve using ipopt solver 
however the ipopt solver must have an HSL lisence ( wasted about 2 days time searching for those ) however it is fully implemented . 
