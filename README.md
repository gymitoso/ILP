# ILP

This work proposes to implement a solver for Integer Linear Programming problems,
applying the branch-and-bound technique with the Simplex algorithm.
- Comments in code are in Portuguese

## What You'll Need
- Eigen library: http://eigen.tuxfamily.org
- Cmake: https://cmake.org
- C and C++ compiler
- For Cmake to work set CC and CXX environment variables to point for the compilers C and C++, respectively.

## How to run
1. Add Eigen path to CMakeLists.txt
- Run: ``` cmake . ``` or ```cmake -G "MinGW Makefiles" .``` (in my case). Check: https://cmake.org/documentation/
- Run: ```make```
- Run: ```ILP path/to/inputFile```
