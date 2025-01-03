cmake_minimum_required(VERSION 3.20.0)

project("buddy-recorder")

find_package(yaml-cpp 0.6.2 REQUIRED)

add_executable(${PROJECT_NAME} main.cpp)

target_link_libraries(${PROJECT_NAME} PRIVATE yaml-cpp)