cmake_minimum_required(VERSION 3.10)

project(main CXX)

set(CMAKE_CXX_FLAGS "-pthread -Wall -Wextra -Wpedantic -std=c++17")

include_directories("include")
file(GLOB SOURCES "src/*.cpp")

add_executable(${PROJECT_NAME} ${SOURCES})