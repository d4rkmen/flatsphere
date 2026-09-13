idf_component_register(INCLUDE_DIRS "include" REQUIRES esp_timer spiffs)

if(CONFIG_FLATSPHERE_TTS_SANO)
    if(POLICY CMP0135)
        cmake_policy(SET CMP0135 NEW)
    endif()
    if(POLICY CMP0169)
        cmake_policy(SET CMP0169 OLD)
    endif()
    include(FetchContent)
    FetchContent_Declare(sano_source
        URL https://codeload.github.com/Ampixa/sanoTTS/tar.gz/939d982b9faa54cbcf5d24cc878f5cd514b2646e
        URL_HASH SHA256=68a9bcc82c568889a6d656c7efa85ba2565ba5b5681c2336229bdc364f67c048)
    FetchContent_Declare(espeak_source
        URL https://codeload.github.com/espeak-ng/espeak-ng/tar.gz/4870adfa25b1a32b4361592f1be8a40337c58d6c
        URL_HASH SHA256=cd83f84c4e495f281ac14e919aecf2834306ec1ea1b498de8ef6000f3a0f90de)
    # These are source/data packages, not nested CMake projects.
    FetchContent_GetProperties(sano_source)
    if(NOT sano_source_POPULATED)
        FetchContent_Populate(sano_source)
    endif()
    FetchContent_GetProperties(espeak_source)
    if(NOT espeak_source_POPULATED)
        FetchContent_Populate(espeak_source)
    endif()
    set(MCU "${sano_source_SOURCE_DIR}/mcu")
    set(PORT "${MCU}/ports/esp32s3/firmware")
    set(ESPEAK "${espeak_source_SOURCE_DIR}/src")
    set(GENERATED "${CMAKE_CURRENT_BINARY_DIR}/generated")
    file(MAKE_DIRECTORY "${GENERATED}")

    # Patch only build-local copies; pinned upstream trees remain unchanged.
    # Fail closed if an upstream revision ever changes a patch anchor.
    function(patch_exact var old new)
        string(FIND "${${var}}" "${old}" found)
        if(found EQUAL -1)
            message(FATAL_ERROR "sanoTTS patch anchor missing: ${old}")
        endif()
        string(REPLACE "${old}" "${new}" patched "${${var}}")
        set(${var} "${patched}" PARENT_SCOPE)
    endfunction()

    file(READ "${ESPEAK}/libespeak-ng/speech.c" speech)
    patch_exact(speech "return GetFileLength(path_home) == -EISDIR;"
        "if (GetFileLength(path_home) == -EISDIR) return 1;\n\tchar probe[sizeof(path_home)+16];\n\tsnprintf(probe, sizeof(probe), \"%s/phontab\", path_home);\n\tFILE *fp = fopen(probe, \"rb\");\n\tif (!fp) return 0;\n\tfclose(fp);\n\treturn 1;")
    file(WRITE "${GENERATED}/speech.c" "${speech}")

    file(READ "${MCU}/src/snt_tts.c" runtime)
    patch_exact(runtime "#include <math.h>" "#include <math.h>\n#include <setjmp.h>\nstatic jmp_buf allocation_failure;")
    patch_exact(runtime "abort();" "longjmp(allocation_failure, 1);")
    patch_exact(runtime "    SnScratch *S = SCR();\n    g_front ="
        "    if (!cfg || !cfg->arena || cfg->arena_size < 16 || !cfg->front_blob || !cfg->dec_blob || !phoneme_ids || !cb || n_ids < 1 || n_ids > 256) return -1;\n    for (int i = 0; i < n_ids; ++i) if (phoneme_ids[i] < 0 || phoneme_ids[i] >= FRONT_AC_VOCAB) return -1;\n    if (setjmp(allocation_failure)) return -2;\n    SnScratch *S = SCR();\n    g_front =")
    patch_exact(runtime "for (int t = 0; t < n_tokens; t++) T += durs[t];"
        "for (int t = 0; t < n_tokens; t++) { if (durs[t] < 1 || durs[t] > 640 - T) return -3; T += durs[t]; }")
    file(WRITE "${GENERATED}/snt_tts.c" "${runtime}")

    file(READ "${PORT}/main/esp_g2p.c" g2p)
    patch_exact(g2p "if (!g_ready) return -1;" "if (!g_ready || !text || !out || cap < 3) return -1;")
    patch_exact(g2p "if (id >= 0 && n + 2 <= cap)" "if (id >= 0 && n + 3 > cap) return -1;\n                if (id >= 0)")
    patch_exact(g2p "if (sid >= 0 && n + 2 <= cap)" "if (sid >= 0 && n + 3 > cap) return -1; if (sid >= 0)")
    patch_exact(g2p "for (const char *q = p1 - 1; q >= p0; q--) {\n            char c = *q;"
        "for (const char *q = p1; q > p0;) {\n            char c = *--q;")
    patch_exact(g2p "if (n + 1 <= cap) out[n++] = ID_EOS;"
        "if (tp && *(const char *)tp) return -1;\n    out[n++] = ID_EOS;")
    if(CONFIG_FLATSPHERE_TTS_VOICE_UK)
        # espeak voice "uk" drives the Ukrainian dictionary and voice definition.
        patch_exact(g2p "espeak_ng_SetVoiceByName(\"en\")"
            "espeak_ng_SetVoiceByName(\"uk\")")
    endif()
    file(WRITE "${GENERATED}/esp_g2p.c" "${g2p}")

    set(ESPEAK_FILES common mnemonics error ieee80 compiledata compiledict
        dictionary encoding intonation langopts numbers phoneme phonemelist
        readclause setlengths soundicon spect ssml synthdata synthesize
        tr_languages translate translateword voices wavegen espeak_api)
    set(sources "${GENERATED}/speech.c" "${GENERATED}/esp_g2p.c")
    foreach(name IN LISTS ESPEAK_FILES)
        list(APPEND sources "${ESPEAK}/libespeak-ng/${name}.c")
    endforeach()
    foreach(name case categories ctype proplist scripts tostring)
        list(APPEND sources "${ESPEAK}/ucd-tools/src/${name}.c")
    endforeach()
    # Match upstream's legacy translator port warning policy, scoped to its
    # sources only. Keep project adapter and neural runtime diagnostics enabled.
    set_source_files_properties(${sources} PROPERTIES COMPILE_OPTIONS "-w")
    # The component starts as an interface library for the Pico-only build.
    add_library(flatsphere_sano_runtime STATIC ${sources}
        "${GENERATED}/snt_tts.c" "${MCU}/src/snt_kernels_ref.c"
        "${CMAKE_CURRENT_LIST_DIR}/sano_port.c"
        "${CMAKE_CURRENT_LIST_DIR}/sanotts.cpp")
    target_include_directories(flatsphere_sano_runtime PRIVATE
        "include" "${MCU}/include" "${MCU}/src" "${PORT}/main"
        "${PORT}/components/espeak-ng" "${ESPEAK}/include"
        "${ESPEAK}/libespeak-ng" "${ESPEAK}/ucd-tools/src/include")
    target_compile_definitions(flatsphere_sano_runtime PRIVATE HAVE_CONFIG_H FSD_FAST_MATH)
    target_compile_options(flatsphere_sano_runtime PRIVATE
        "$<$<COMPILE_LANGUAGE:C>:-std=gnu11>" -O2)
    target_link_libraries(flatsphere_sano_runtime PUBLIC idf::esp_timer idf::spiffs)
    target_link_libraries(${COMPONENT_LIB} INTERFACE flatsphere_sano_runtime)
    if(CONFIG_FLATSPHERE_TTS_VOICE_UK)
        # Ukrainian model blobs and matching meta headers (lang/ua/export_int8.py).
        # BEFORE is required: ${MCU}/src also ships model/*.h (English); the
        # runtime's #include "model/front_q8_meta.h" must resolve to the voice
        # selected here, not the English headers.
        set(VOICE_DIR "${CMAKE_CURRENT_LIST_DIR}/voices/uk")
        if(CONFIG_FLATSPHERE_TTS_UK_MYKYTA)
            set(VOICE_DIR "${CMAKE_CURRENT_LIST_DIR}/voices/uk-mykyta")
            if(CONFIG_FLATSPHERE_TTS_MYKYTA_DEVICE_TEST)
                get_filename_component(VOICE_DIR "${CONFIG_FLATSPHERE_TTS_MYKYTA_TEST_EXPORT}"
                    ABSOLUTE BASE_DIR "${PROJECT_DIR}")
                message(WARNING "Mykyta DEVICE TEST: parity not accepted; not a release voice")
            elseif(NOT EXISTS "${VOICE_DIR}/acceptance.json")
                message(FATAL_ERROR "Install an accepted Mykyta bundle or explicitly enable device-test mode")
            endif()
            file(READ "${VOICE_DIR}/export_manifest.json" voice_manifest)
            string(JSON speaker GET "${voice_manifest}" speaker)
            if(NOT speaker STREQUAL "mykyta")
                message(FATAL_ERROR "Selected export is not Mykyta")
            endif()
            string(JSON artifact_count LENGTH "${voice_manifest}" artifacts)
            math(EXPR artifact_last "${artifact_count} - 1")
            foreach(i RANGE 0 ${artifact_last})
                string(JSON artifact MEMBER "${voice_manifest}" artifacts ${i})
                string(JSON expected GET "${voice_manifest}" artifacts "${artifact}" sha256)
                file(SHA256 "${VOICE_DIR}/${artifact}" actual)
                if(NOT actual STREQUAL expected)
                    message(FATAL_ERROR "Mykyta artifact hash mismatch: ${artifact}")
                endif()
            endforeach()
        endif()
        target_include_directories(flatsphere_sano_runtime BEFORE PRIVATE "${VOICE_DIR}")
        target_add_binary_data(flatsphere_sano_runtime "${VOICE_DIR}/front_q8.bin" BINARY)
        target_add_binary_data(flatsphere_sano_runtime "${VOICE_DIR}/model_q8.bin" BINARY)
    else()
        set(VOICE_DIR "${MCU}/test/fixtures/en_us_r7")
        target_add_binary_data(flatsphere_sano_runtime "${VOICE_DIR}/front_q8.bin" BINARY)
        target_add_binary_data(flatsphere_sano_runtime "${VOICE_DIR}/model_q8.bin" BINARY)
    endif()
    # espeak data image: English minimal set from the port, plus Ukrainian
    # dictionary/voice when the Ukrainian voice is selected.
    set(ESPEAK_DATA_DIR "${CMAKE_CURRENT_BINARY_DIR}/espeak-ng-data")
    file(MAKE_DIRECTORY "${ESPEAK_DATA_DIR}")
    file(COPY "${PORT}/espeak-ng-data/" DESTINATION "${ESPEAK_DATA_DIR}")
    if(CONFIG_FLATSPHERE_TTS_VOICE_UK)
        file(COPY "${CMAKE_CURRENT_LIST_DIR}/espeak-uk/" DESTINATION "${ESPEAK_DATA_DIR}")
    endif()
    spiffs_create_partition_image(espeak "${ESPEAK_DATA_DIR}" FLASH_IN_PROJECT)
endif()