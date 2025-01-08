cmake_minimum_required(VERSION 3.20.0)

project("buddy-recorder")

if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
endif()

option(DEBUG_LOGS "Turns on debugging logs" OFF)

find_package(Threads REQUIRED)
find_package(Boost 1.71 REQUIRED COMPONENTS program_options)
find_package(yaml-cpp 0.6.2 REQUIRED)

add_executable(${PROJECT_NAME}
  ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/thread_safe_event.cpp
)

target_link_libraries(${PROJECT_NAME}
    PRIVATE
    Boost::program_options
    yaml-cpp
    Threads::Threads
)

target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)

if(DEBUG_LOGS)
  target_compile_definitions(${PROJECT_NAME} PRIVATE DEBUG_LOGS)
endif()
