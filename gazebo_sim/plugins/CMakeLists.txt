cmake_minimum_required(VERSION 3.16)
project(cuadc_gazebo_plugins LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(gz-cmake3 REQUIRED)
find_package(gz-sim7 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-transport12 REQUIRED)

foreach(plugin
    CuadcHandLaunchSystem
    CuadcPayloadReleaseSystem
    CuadcAnemometerSystem
    CuadcTwinMotorSystem
    CuadcScenarioStateSystem
    CuadcAirframeDiagnosticsSystem)
  add_library(${plugin} SHARED ${plugin}.cc)
  target_link_libraries(${plugin} PRIVATE
    gz-sim7::gz-sim7
    gz-plugin2::gz-plugin2
    gz-transport12::gz-transport12)
endforeach()
