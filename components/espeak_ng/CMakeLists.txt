# Local G2P-only eSpeak 1.52.0. Model-specific IPA-to-ID mapping belongs to sanoTTS.
set(espeak_files common mnemonics error ieee80 compiledata compiledict dictionary
    encoding intonation langopts numbers phoneme phonemelist readclause setlengths
    soundicon spect ssml synthdata synthesize tr_languages translate translateword
    voices wavegen espeak_api)
set(sources)
foreach(name IN LISTS espeak_files)
    list(APPEND sources "upstream/src/libespeak-ng/${name}.c")
endforeach()
foreach(name case categories ctype proplist scripts tostring)
    list(APPEND sources "upstream/src/ucd-tools/src/${name}.c")
endforeach()
idf_component_register(SRCS ${sources}
    INCLUDE_DIRS "upstream/src/include"
    PRIV_INCLUDE_DIRS "port" "upstream/src/libespeak-ng" "upstream/src/ucd-tools/src/include"
    REQUIRES spiffs
    LDFRAGMENTS "linker.lf")
if(CMAKE_BUILD_EARLY_EXPANSION)
    return()
endif()
include("${CMAKE_CURRENT_LIST_DIR}/cmake/prepare.cmake")
espeak_prepare("${CMAKE_CURRENT_LIST_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/generated")
target_sources(${COMPONENT_LIB} PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/generated/speech.c")
target_compile_definitions(${COMPONENT_LIB} PRIVATE HAVE_CONFIG_H)
# Legacy upstream translator warning policy, not applied to project adapters.
target_compile_options(${COMPONENT_LIB} PRIVATE -std=gnu11 -O2 -w -fstack-usage)
# Both languages are a single immutable compatible G2P dataset. No stale overlay
# can survive switching voice; no dependency on a Python installation or cache.
spiffs_create_partition_image(espeak "${CMAKE_CURRENT_LIST_DIR}/data" FLASH_IN_PROJECT)