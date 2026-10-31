cmake_minimum_required(VERSION 3.16)

file (STRINGS version.txt BUILD_NUMBER)
set(PROJECT_VER ${BUILD_NUMBER})
add_compile_definitions(BUILD_NUMBER="${BUILD_NUMBER}")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(flatsphere)

# esptool --chip esp32s3 merge_bin --output build/<project>_full.bin
#   0x0 build/bootloader/bootloader.bin
#   0x8000 build/partition_table/partition-table.bin
#   0x9000 build/nvs.bin
#   0x10000 build/<project>.bin
#   0x410000 build/espeak.bin
idf_build_get_property(python PYTHON)
idf_build_get_property(idf_path IDF_PATH)
set(esptool ${python})
if(NOT "$ENV{ESPTOOL_WRAPPER}" STREQUAL "")
    list(APPEND esptool "$ENV{ESPTOOL_WRAPPER}")
endif()
list(APPEND esptool "${idf_path}/components/esptool_py/esptool/esptool.py" --chip esp32s3)
set(full_bin "${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}_full.bin")
add_custom_command(
    OUTPUT "${full_bin}"
    COMMAND ${esptool} merge_bin --output "${CMAKE_PROJECT_NAME}_full.bin"
        0x0 bootloader/bootloader.bin
        0x8000 partition_table/partition-table.bin
        0x9000 nvs.bin
        0x10000 ${CMAKE_PROJECT_NAME}.bin
        0x410000 espeak.bin
    DEPENDS gen_project_binary bootloader partition_table_bin nvs_nvs_bin spiffs_espeak_bin
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
    COMMENT "Merging ${CMAKE_PROJECT_NAME}_full.bin"
    VERBATIM
)
add_custom_target(merge-full ALL DEPENDS "${full_bin}")
set_property(DIRECTORY APPEND PROPERTY ADDITIONAL_CLEAN_FILES "${full_bin}")
