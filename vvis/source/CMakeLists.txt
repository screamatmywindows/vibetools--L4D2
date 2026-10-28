cmake_minimum_required(VERSION 3.16)
project(vvis_argon LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(vvis_argon
    vvis_main.cpp
)

if(MSVC)
    # Visual Studio 2022 (MSVC v143) optimization flags for Intel Alder Lake (i5-12450H)
    # Strictly targets AVX2 + FMA3 + BMI2; never emits AVX-512 instructions!
    target_compile_options(vvis_argon PRIVATE
        /O2 /Oi /Ot /GL /arch:AVX2 /fp:fast /EHsc /W4
    )
    target_link_options(vvis_argon PRIVATE /LTCG)
else()
    # GCC / Clang x64 flags for AVX2 + FMA3 + BMI2 + POPCNT
    target_compile_options(vvis_argon PRIVATE
        -O3 -mavx2 -mfma -mbmi2 -mpopcnt -ffast-math -pthread -Wall -Wextra
    )
    target_link_libraries(vvis_argon PRIVATE pthread)
endif()
