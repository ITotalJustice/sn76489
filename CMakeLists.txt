cmake_minimum_required(VERSION 3.13)


option(SN76489_CXX
    "build with Blip_Buffer.cpp, gives access to bass and treble control"
    OFF
)


project(sn76489
    VERSION 1.0.0
    LANGUAGES C
)

# check to see if we have math.h
include(CheckIncludeFile)
CHECK_INCLUDE_FILE(math.h HAS_MATH_H)

if (HAS_MATH_H)
    message(STATUS "found math.h!")
else()
    if (SN76489_CXX)
        message(WARNING "force disabling SN76489_CXX as math.h not found!")
        set(SN76489_CXX OFF)
    endif()
endif()

add_library(sn76489 sn76489.c)
target_include_directories(sn76489 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_definitions(sn76489 PRIVATE
    SN76489_CXX=$<BOOL:${SN76489_CXX}>
)

set_target_properties(sn76489 PROPERTIES C_STANDARD 99)

if (SN76489_CXX)
    message(STATUS "sn76489 built with Blip_Buffer (CXX)")
    enable_language(CXX)
    set_target_properties(sn76489 PROPERTIES CXX_STANDARD 98)
    target_sources(sn76489 PRIVATE blargg/blip_wrap.cpp blargg/Blip_Buffer.cpp)
else()
    message(STATUS "sn76489 built with blip_buf (C)")
    target_sources(sn76489 PRIVATE blargg/blip_wrap.c blargg/blip_buf.c)
endif()

if (HAS_MATH_H)
    # only link with math.h if found as some platforms don't require linking against it
    find_library(MATH_LIB NAMES m)
    if (MATH_LIB)
        target_link_libraries(sn76489 PUBLIC ${MATH_LIB})
    endif()
endif()

# enable warnings
target_compile_options(sn76489 PRIVATE
    $<$<OR:$<C_COMPILER_ID:Clang>,$<C_COMPILER_ID:AppleClang>,$<C_COMPILER_ID:GNU>>:
        -Wall
        -Wextra
    >
    $<$<C_COMPILER_ID:MSVC>:
        /W4
    >
)
