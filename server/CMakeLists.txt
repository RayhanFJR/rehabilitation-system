cmake_minimum_required(VERSION 3.10)
project(RehabControlServer)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ====== FIND LIBRARIES ======
find_package(PkgConfig REQUIRED)
pkg_check_modules(MODBUS REQUIRED libmodbus)

find_package(Boost 1.66 REQUIRED COMPONENTS system)
find_package(Threads REQUIRED)

# ====== INCLUDE DIRECTORIES ======
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${MODBUS_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

# ====== SOURCE FILES ======
set(SOURCES
    src/main_server.cpp
    src/trajectory/TrajectoryManager.cpp
    src/modbus/ModbusServer.cpp
    src/modbus/DataHandler.cpp
    src/serial/SerialPort.cpp
    src/state_machine/StateMachine.cpp
)

# ====== BUILD EXECUTABLE ======
add_executable(rehab_server ${SOURCES})

# ====== LINK LIBRARIES ======
target_link_libraries(rehab_server PRIVATE
    ${MODBUS_LIBRARIES}
    Boost::system
    Threads::Threads
)

# ====== WARNINGS ======
if(NOT MSVC)
    target_compile_options(rehab_server PRIVATE -Wall -Wextra -Wpedantic)
endif()
