cmake_minimum_required(VERSION 3.16)

file (STRINGS version.txt BUILD_NUMBER)
set(PROJECT_VER ${BUILD_NUMBER})
add_compile_definitions(BUILD_NUMBER="${BUILD_NUMBER}")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# PicoTTS needs this
add_compile_options(-Wmaybe-uninitialized)

project(flatsphere)
