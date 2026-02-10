@echo off
REM Patches LVGL, esp_lvgl_port, and esp_websocket_client cmake/source files
REM Run this after managed_components are downloaded (idf.py reconfigure)

setlocal EnableDelayedExpansion

REM Patch LVGL - needs to be in REQUIRES (public) since lv_conf.h is included from public headers
set "LVGL_CMAKE=managed_components\lvgl__lvgl\env_support\cmake\esp.cmake"
if exist "%LVGL_CMAKE%" (
    findstr /C:"REQUIRES ${IDF_COMPONENTS} main)" "%LVGL_CMAKE%" >nul 2>&1
    if !errorlevel! equ 0 (
        echo LVGL: Already patched
    ) else (
        powershell -Command "(Get-Content '%LVGL_CMAKE%') -replace 'PRIV_REQUIRES \$\{IDF_COMPONENTS\}\)', 'REQUIRES ${IDF_COMPONENTS} main)' | Set-Content '%LVGL_CMAKE%'"
        findstr /C:"REQUIRES ${IDF_COMPONENTS} main)" "%LVGL_CMAKE%" >nul 2>&1
        if !errorlevel! equ 0 (
            echo LVGL: Patch applied successfully
        ) else (
            echo LVGL: Patch failed - please manually edit %LVGL_CMAKE%
            echo Change line with "PRIV_REQUIRES ${IDF_COMPONENTS})" to "REQUIRES ${IDF_COMPONENTS} main)"
        )
    )
) else (
    echo LVGL: %LVGL_CMAKE% not found
)

REM Patch esp_websocket_client - disable redirect support (not available in ESP-IDF 5.5)
set "WS_CLIENT=managed_components\espressif__esp_websocket_client\esp_websocket_client.c"
if exist "%WS_CLIENT%" (
    findstr /C:"#define WS_TRANSPORT_REDIRECT_HEADER_SUPPORT    0" "%WS_CLIENT%" >nul 2>&1
    if !errorlevel! equ 0 (
        echo esp_websocket_client: Already patched
    ) else (
        powershell -Command "(Get-Content '%WS_CLIENT%') -replace '#define WS_TRANSPORT_REDIRECT_HEADER_SUPPORT    1', '#define WS_TRANSPORT_REDIRECT_HEADER_SUPPORT    0' | Set-Content '%WS_CLIENT%'"
        echo esp_websocket_client: Patched ^(redirect disabled^)
    )
) else (
    echo esp_websocket_client: %WS_CLIENT% not found
)

echo.
echo Done. Now run: idf.py build
