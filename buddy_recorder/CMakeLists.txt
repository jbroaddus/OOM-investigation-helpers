cmake_minimum_required(VERSION 3.20.0)

project("buddy-recorder")

if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
endif()

find_package(Threads REQUIRED)
find_package(Boost 1.71 REQUIRED COMPONENTS program_options)
find_package(yaml-cpp 0.6.2 REQUIRED)

add_executable(${PROJECT_NAME} main.cpp)

target_link_libraries(${PROJECT_NAME} 
    PRIVATE 
    Boost::program_options
    yaml-cpp    
)

target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)