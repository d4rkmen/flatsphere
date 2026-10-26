# All neural sources and voice contracts are local. No source-download fallback.
idf_component_register(INCLUDE_DIRS "include" REQUIRES esp_timer spiffs espeak_ng
    LDFRAGMENTS "linker.lf")
if(CMAKE_BUILD_EARLY_EXPANSION)
    return()
endif()
if(NOT CONFIG_IDF_TARGET_ESP32S3 OR NOT CONFIG_SPIRAM)
    message(FATAL_ERROR "FlatSphere sanoTTS requires ESP32-S3 with PSRAM")
endif()
include("${CMAKE_CURRENT_LIST_DIR}/cmake/voice.cmake")
sanotts_select_voice("${CMAKE_CURRENT_LIST_DIR}")
include("${CMAKE_CURRENT_LIST_DIR}/cmake/runtime.cmake")
set(GENERATED "${CMAKE_CURRENT_BINARY_DIR}/generated")
sanotts_prepare_runtime("${CMAKE_CURRENT_LIST_DIR}" "${GENERATED}")
include("${CMAKE_CURRENT_LIST_DIR}/cmake/g2p.cmake")
sanotts_prepare_g2p("${CMAKE_CURRENT_LIST_DIR}" "${GENERATED}" "${SANOTTS_LANGUAGE}")

# The port layer (kernels + dual-core runner) lives in its own archive so the
# PSRAM .bss mapping below cannot move the internal-SRAM activations the PIE
# instructions must read. Dependency direction: runtime -> port.
add_library(flatsphere_sano_kernels STATIC
    "port/snt_kernels_esp32s3.c" "port/sn_matvec_s8_esp32s3.S"
    "upstream/src/snt_kernels_ref.c" "sano_port.c")
target_include_directories(flatsphere_sano_kernels PRIVATE "include" "upstream/include")
target_compile_options(flatsphere_sano_kernels PRIVATE
    "$<$<COMPILE_LANGUAGE:C>:-std=gnu11>" -O2 -fstack-usage)
target_link_libraries(flatsphere_sano_kernels PUBLIC idf::esp_timer idf::esp_hw_support idf::freertos idf::heap)
add_library(flatsphere_sano_runtime STATIC
    "${GENERATED}/snt_tts.c" "${GENERATED}/esp_g2p.c" "sanotts.cpp")
# The scalar reference kernels stay byte-identical; rename them so the S3
# dispatcher can fall back to them for non-internal operands.
set_source_files_properties("upstream/src/snt_kernels_ref.c" PROPERTIES COMPILE_DEFINITIONS
    "snt_dot_s8=snt_ref_dot_s8;snt_matvec_s8=snt_ref_matvec_s8;snt_dot_s16s8=snt_ref_dot_s16s8;snt_matvec_s16s8=snt_ref_matvec_s16s8;snt_weights_resident=snt_ref_weights_resident")
target_include_directories(flatsphere_sano_runtime PRIVATE
    "${VOICE_DIR}" "include" "upstream/include" "upstream/g2p")
if(SANOTTS_EXPERIMENTAL)
    set(voice_experimental 1)
else()
    set(voice_experimental 0)
endif()
target_compile_definitions(flatsphere_sano_runtime PRIVATE FSD_FAST_MATH SNT_PORT_RES_ARENA
    "SANOTTS_VOICE_NAME=\"${SANOTTS_VOICE}\"" SANOTTS_VOICE_EXPERIMENTAL=${voice_experimental})
target_compile_options(flatsphere_sano_runtime PRIVATE
    "$<$<COMPILE_LANGUAGE:C>:-std=gnu11>" -O2 -fstack-usage)
target_link_libraries(flatsphere_sano_runtime PUBLIC idf::esp_timer idf::spiffs idf::espeak_ng idf::esp_hw_support idf::freertos idf::heap)
target_link_libraries(${COMPONENT_LIB} INTERFACE flatsphere_sano_runtime flatsphere_sano_kernels)

# Float fields require aligned flash-mapped bases, including after voice switches.
configure_file(model_blobs.S.in "${GENERATED}/model_blobs.S" @ONLY)
set_source_files_properties("${GENERATED}/model_blobs.S" PROPERTIES
    OBJECT_DEPENDS "${VOICE_DIR}/front_q8.bin;${VOICE_DIR}/model_q8.bin")
target_sources(flatsphere_sano_runtime PRIVATE "${GENERATED}/model_blobs.S")
