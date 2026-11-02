cmake_minimum_required(VERSION 3.16)
project(traffic_simulator LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(traffic_model
    src/Objects/Object/object.cpp
    src/Objects/Point/point.cpp
    src/Objects/Road/road.cpp
    src/Objects/Lane/lane.cpp
    src/Objects/Car/car.cpp
    src/Objects/SpawnPoint/spawn_point.cpp
    src/Objects/Simulation/simulation.cpp
)
target_include_directories(traffic_model PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)

if(WIN32)
    add_executable(traffic_simulator WIN32 src/main_win32.cpp)
    target_compile_definitions(traffic_simulator PRIVATE UNICODE _UNICODE NOMINMAX)
    target_link_libraries(traffic_simulator PRIVATE traffic_model comctl32)
    if(MINGW)
        target_link_options(traffic_simulator PRIVATE -municode)
    endif()
endif()

enable_testing()
add_executable(simulation_tests tests/simulation_tests.cpp)
target_link_libraries(simulation_tests PRIVATE traffic_model)
add_test(NAME simulation_tests COMMAND simulation_tests)
