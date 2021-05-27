cmake_minimum_required(VERSION 3.05)
project(RoboticsAlgorithmLib C)

set(CMAKE_C_STANDARD 11)

aux_source_directory(./ DIR_SRCS)
add_executable(RoboticsAlgorithmLib ${DIR_SRCS})