cmake_minimum_required(VERSION 3.14)
project(automotive_telematics LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Threads REQUIRED)
find_package(OpenSSL 1.1.1 REQUIRED)

add_library(telematics
  src/obd2.cpp
  src/gps.cpp
  src/vehicle_state.cpp
  src/event_detector.cpp
  src/trip_tracker.cpp
  src/store_forward.cpp
  src/can_bus.cpp
  src/telematics_unit.cpp
  src/net/tcp_socket.cpp
  src/net/tls.cpp
  src/net/mqtt_codec.cpp
  src/net/mqtt_client.cpp
  src/net/http_client.cpp
  src/net/transports.cpp
)
target_include_directories(telematics PUBLIC include)
target_link_libraries(telematics PUBLIC Threads::Threads OpenSSL::SSL OpenSSL::Crypto)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(telematics PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_executable(tcu_app app/main.cpp)
target_link_libraries(tcu_app PRIVATE telematics)

enable_testing()
add_executable(tcu_tests tests/test_main.cpp)
target_link_libraries(tcu_tests PRIVATE telematics)
add_test(NAME tcu_tests COMMAND tcu_tests)

add_executable(tcu_net_tests tests/test_net.cpp)
target_link_libraries(tcu_net_tests PRIVATE telematics)
add_test(NAME tcu_net_tests COMMAND tcu_net_tests)
