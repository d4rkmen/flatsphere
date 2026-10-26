include("${ROOT}/components/espeak_ng/cmake/prepare.cmake")
espeak_prepare("${ROOT}/components/espeak_ng" "${GENERATED}")
include("${ROOT}/components/sanotts/cmake/g2p.cmake")
sanotts_prepare_g2p("${ROOT}/components/sanotts" "${GENERATED}" "${LANGUAGE}")