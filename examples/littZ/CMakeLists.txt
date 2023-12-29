cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE}) # required pre-project
project(littZ LANGUAGES C CXX)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 14)

option(STL "With C++ STL" OFF)

target_sources(app PRIVATE src/main.cpp src/zb_zcl_opentherm.c)
target_include_directories(app PRIVATE ${CMAKE_SOURCE_DIR}/../../include)

if (STL)
  set(CONFIG_LIB_CPLUSPLUS y CACHE STRING "Enable Zephyr Kconfig STL configuration option" FORCE)
  target_compile_definitions(app PUBLIC LITT_HAVE_STL)
endif()