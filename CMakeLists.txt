cmake_minimum_required(VERSION 3.10)


project(elmo_drive)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wfatal-errors -std=c++14 -Wall -Wno-unknown-pragmas")



#
configure_file(${CMAKE_SOURCE_DIR}/.in/io_map.py.in ${CMAKE_SOURCE_DIR}/script/io_map.py)
configure_file(${CMAKE_SOURCE_DIR}/.in/dir.h.in ${CMAKE_SOURCE_DIR}/include/dir.h)


# configure_file(${CMAKE_SOURCE_DIR}/include/configDir.h.in ${CMAKE_SOURCE_DIR}/include/configDir.h)
# configure_file(${CMAKE_SOURCE_DIR}/script/io_map.py.in ${CMAKE_SOURCE_DIR}/script/io_map.py)

find_package(ct_rbd)

include_directories(${ct_rbd_INCLUDE_DIRS})

include_directories(/usr/local/include/soem/)
include_directories(/usr/include/eigen3/)
include_directories(include/)



add_custom_command(OUTPUT ${CMAKE_SOURCE_DIR}/src/io.cpp ${CMAKE_SOURCE_DIR}/include/io.h
        DEPENDS ${CMAKE_SOURCE_DIR}/config/io
        DEPENDS ${CMAKE_SOURCE_DIR}/.in/io_map.py.in
        COMMAND env python3 ${CMAKE_SOURCE_DIR}/script/io_map.py
        )


include_directories(/usr/local/include/soem/)
include_directories(/usr/include/eigen3/)
include_directories(include/)


set(target_src
        src/motor.cpp
        src/joint.cpp
        src/transform.cpp
        src/io.cpp
        include/soem.h include/Leg.h)

include_directories(/usr/local/include/soem/)
include_directories(${CMAKE_SOURCE_DIR}/include/pegasus2)
include_directories(/usr/include/eigen3/)
add_executable(elmo.exe  src/elmo.cpp ${target_src})
target_link_libraries(elmo.exe soem pthread)
target_compile_definitions(elmo.exe PUBLIC EIGEN_NO_DEBUG)



