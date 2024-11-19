@echo off
SETLOCAL

REM include all files from main project except main.cpp, include only the correct platform layer
for %%a in (../*.cpp) do (
    for /f "tokens=1 delims=_" %%n in ("%%~na") do (
        if "%%n"=="platform" (
            REM include only platform_win32.cpp
            for /f "tokens=2 delims=_" %%n in ("%%~na") do (
                if "%%n"=="win32" (
                    call SET files=%%files%% ../%%a
                )
            )
        ) else (
            REM include all other files except main.cpp
            if NOT "%%a"=="main.cpp" (
                call SET files=%%files%% ../%%a
            )
        )
    )
)


SET fixVCRT=-D_MT -D_DLL -lmsvcrt -lucrt -lmsvcprt -lvcruntime -Xlinker /NODEFAULTLIB
SET libs=-lkernel32 -luser32 -lgdi32 -lbenchmark.lib -lshlwapi.lib -lole32.lib -loleaut32.lib
SET dirs=-Llib -Iinclude -I../include/ -I../
SET warns=-Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers
SET opts=-m64 -O3 -march=native -Xlinker /LTCG
SET misc=-fno-exceptions -fno-rtti -D_CRT_SECURE_NO_WARNINGS -DBENCHMARK_STATIC_DEFINE -D_ENABLE_EXTENDED_ALIGNED_STORAGE -Xclang -flto-visibility-public-std

clang++ -std=c++20 %opts% %dirs% %libs% %warns% %misc% %fixVCRT% -o bench_clang.exe %files%

ENDLOCAL