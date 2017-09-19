cmake_minimum_required(VERSION 3.8)
project(ILP)

include_directories(path/to/Eigen)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES
	headers/BranchBound.h
	headers/Exception.h
	headers/Interpreter.h
	headers/Problem.h
	headers/Simplex.h
	sources/BranchBound.cpp
	sources/Exception.cpp
	sources/Interpreter.cpp
	sources/Problem.cpp
	sources/Simplex.cpp
	main.cpp)

add_executable(ILP ${SOURCE_FILES})
