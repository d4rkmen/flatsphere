cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# PicoTTS needs this
add_compile_options(-Wmaybe-uninitialized)

project(flatsphere)
