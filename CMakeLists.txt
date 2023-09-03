cmake_minimum_required(VERSION 3.22)


set(CMAKE_C_STANDARD 11)

option(BUILD_SHARED_LIBS "Build using shared libraries" ON)

project(vrEmu6502)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/)

if (NOT BUILD_SHARED_LIBS)
 add_definitions(-DVR_6502_EMU_STATIC)
endif()

include(CTest)

add_subdirectory(src)
add_subdirectory(test)
