# Exact safety patches against unchanged upstream source. Shared by host tests.
function(sanotts_prepare_runtime component generated)
    file(READ "${component}/upstream/provenance.json" provenance)
    string(JSON count LENGTH "${provenance}" files)
    math(EXPR last "${count} - 1")
    foreach(i RANGE 0 ${last})
        string(JSON name MEMBER "${provenance}" files ${i})
        string(JSON expected GET "${provenance}" files "${name}" sha256)
        file(SHA256 "${component}/${name}" actual)
        if(NOT actual STREQUAL expected)
            message(FATAL_ERROR "sanoTTS source provenance mismatch: ${name}")
        endif()
        set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${component}/${name}")
    endforeach()
    file(MAKE_DIRECTORY "${generated}")
    file(READ "${component}/upstream/src/snt_tts.c" runtime)
    macro(sanotts_patch old new)
        string(FIND "${runtime}" "${old}" found)
        if(found EQUAL -1)
            message(FATAL_ERROR "sanoTTS patch anchor missing: ${old}")
        endif()
        string(REPLACE "${old}" "${new}" runtime "${runtime}")
    endmacro()
    sanotts_patch("#include <math.h>"
        "#include <math.h>\n#include <setjmp.h>\nstatic jmp_buf allocation_failure;")
    sanotts_patch("abort();" "longjmp(allocation_failure, 1);")
    sanotts_patch("    SnScratch *S = SCR();\n    g_front ="
        "    if (!cfg || !cfg->arena || cfg->arena_size < 16 || !cfg->front_blob || !cfg->dec_blob || !phoneme_ids || !cb || n_ids < 1 || n_ids > 256) return -1;\n    for (int i = 0; i < n_ids; ++i) if (phoneme_ids[i] < 0 || phoneme_ids[i] >= FRONT_AC_VOCAB) return -1;\n    if (setjmp(allocation_failure)) return -2;\n    SnScratch *S = SCR();\n    g_front =")
    sanotts_patch("for (int t = 0; t < n_tokens; t++) T += durs[t];"
        "for (int t = 0; t < n_tokens; t++) { if (durs[t] < 1 || durs[t] > 640 - T) return -3; T += durs[t]; }")
    # Weight residency copies (aa_try) go to a separate internal-SRAM arena on
    # ESP so PIE SIMD can read them; the main arena stays in PSRAM. Marks are
    # mirrored so phase releases free both. Host builds are unchanged.
    sanotts_patch("/* arena alloc that returns NULL instead of aborting when out of space */\nstatic void *aa_try(size_t n) {\n"
        "#ifdef SNT_PORT_RES_ARENA\nextern void *snt_port_res_arena;\nextern size_t snt_port_res_cap;\n#endif\nstatic size_t g_res_top;\n/* arena alloc that returns NULL instead of aborting when out of space */\nstatic void *aa_try(size_t n) {\n#ifdef SNT_PORT_RES_ARENA\n    if (snt_port_res_arena) {\n        size_t rtop = (g_res_top + 15) & ~(size_t)15;\n        if (rtop + n <= snt_port_res_cap) { g_res_top = rtop + n; return (unsigned char *)snt_port_res_arena + rtop; }\n    }\n#endif\n")
    sanotts_patch("    g_arena_top = 0;\n" "    g_arena_top = 0;\n    g_res_top = 0;\n")
    sanotts_patch("    size_t mark_dur = g_arena_top;\n" "    size_t mark_dur = g_arena_top, res_mark_dur = g_res_top;\n")
    sanotts_patch("    g_arena_top = mark_dur;\n" "    g_arena_top = mark_dur;\n    g_res_top = res_mark_dur;\n")
    sanotts_patch("    size_t mark_front = g_arena_top;\n" "    size_t mark_front = g_arena_top, res_mark_front = g_res_top;\n")
    sanotts_patch("    g_arena_top = mark_front;" "    g_res_top = res_mark_front;\n    g_arena_top = mark_front;")
    # A real signal's DC and Nyquist bins are real. torch.istft (the reference)
    # drops their imaginary parts; the runtime's hermitian pack does not, which
    # turns the head's phase at bin N/2 into a steady 11.025 kHz tone (+72 dB
    # over the reference, 13% of the golden error on a GAN-trained decoder).
    sanotts_patch("        par_run(spec_range, FSD_BINS, &sctx);\n"
        "        par_run(spec_range, FSD_BINS, &sctx);\n        fim[0] = 0.0f;\n        fim[NFFT / 2] = 0.0f;\n")
    file(WRITE "${generated}/snt_tts.c" "${runtime}")
endfunction()