@echo off
REM ===========================================================================
REM VVIS++ Argon Edition — Visual Studio 2022 (MSVC v143) x64 Build Script
REM Target CPU: Intel Core i5-12450H (Alder Lake 4P+4E, AVX2 + FMA3 + BMI2)
REM ===========================================================================

echo Building VVIS++ Argon Edition (x64 Release - AVX2/FMA3/BMI2)...
cl.exe /nologo /std:c++20 /O2 /Oi /Ot /GL /arch:AVX2 /fp:fast /EHsc /DNDEBUG ^
    vvis_main.cpp /Fe:vvis++.exe /link /LTCG

if %ERRORLEVEL% EQU 0 (
    echo [SUCCESS] Built vvis++.exe successfully!
) else (
    echo [ERROR] Build failed. Ensure you are running inside "x64 Native Tools Command Prompt for VS 2022".
)
