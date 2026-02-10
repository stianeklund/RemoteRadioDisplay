#!/bin/bash
# Patches LVGL, esp_lvgl_port, and esp_websocket_client cmake/source files
# Run this after managed_components are downloaded (idf.py reconfigure)

# Patch LVGL - needs to be in REQUIRES (public) since lv_conf.h is included from public headers
LVGL_CMAKE="managed_components/lvgl__lvgl/env_support/cmake/esp.cmake"
if [ -f "$LVGL_CMAKE" ]; then
    # Check if already patched with REQUIRES (public)
    if grep -q "REQUIRES \${IDF_COMPONENTS} main)" "$LVGL_CMAKE"; then
        echo "LVGL: Already patched (REQUIRES)"
    else
        # First, check if PRIV_REQUIRES was patched previously (wrong) and fix it
        if grep -q "PRIV_REQUIRES \${IDF_COMPONENTS} main)" "$LVGL_CMAKE"; then
            # Change PRIV_REQUIRES to REQUIRES
            sed -i 's/PRIV_REQUIRES \${IDF_COMPONENTS} main)/REQUIRES ${IDF_COMPONENTS} main)/g' "$LVGL_CMAKE"
            echo "LVGL: Fixed PRIV_REQUIRES -> REQUIRES"
        elif grep -q "PRIV_REQUIRES \${IDF_COMPONENTS})" "$LVGL_CMAKE"; then
            # Change PRIV_REQUIRES to REQUIRES and add main
            sed -i 's/PRIV_REQUIRES \${IDF_COMPONENTS})/REQUIRES ${IDF_COMPONENTS} main)/g' "$LVGL_CMAKE"
            if grep -q "REQUIRES \${IDF_COMPONENTS} main)" "$LVGL_CMAKE"; then
                echo "LVGL: Patch applied successfully (REQUIRES)"
            else
                echo "LVGL: Patch failed"
            fi
        else
            echo "LVGL: Pattern not found"
        fi
    fi
else
    echo "LVGL: $LVGL_CMAKE not found"
fi

# Patch esp_lvgl_port - needs main to access lv_conf.h
PORT_CMAKE="managed_components/espressif__esp_lvgl_port/CMakeLists.txt"
if [ -f "$PORT_CMAKE" ]; then
    if grep -q 'REQUIRES "esp_lcd" main)' "$PORT_CMAKE"; then
        echo "esp_lvgl_port: Already patched"
    elif grep -q 'REQUIRES "esp_lcd")' "$PORT_CMAKE"; then
        sed -i 's/REQUIRES "esp_lcd")/REQUIRES "esp_lcd" main)/g' "$PORT_CMAKE"
        if grep -q 'REQUIRES "esp_lcd" main)' "$PORT_CMAKE"; then
            echo "esp_lvgl_port: Patch applied successfully"
        else
            echo "esp_lvgl_port: Patch failed"
        fi
    else
        echo "esp_lvgl_port: Pattern not found (may already be patched differently)"
    fi
else
    echo "esp_lvgl_port: $PORT_CMAKE not found"
fi

# Patch esp_websocket_client - disable redirect support (not available in ESP-IDF 5.5)
WS_CLIENT="managed_components/espressif__esp_websocket_client/esp_websocket_client.c"
if [ -f "$WS_CLIENT" ]; then
    if grep -q '#define WS_TRANSPORT_REDIRECT_HEADER_SUPPORT    0' "$WS_CLIENT"; then
        echo "esp_websocket_client: Already patched (redirect disabled)"
    elif grep -q '#define WS_TRANSPORT_REDIRECT_HEADER_SUPPORT    1' "$WS_CLIENT"; then
        sed -i 's/#define WS_TRANSPORT_REDIRECT_HEADER_SUPPORT    1/#define WS_TRANSPORT_REDIRECT_HEADER_SUPPORT    0/g' "$WS_CLIENT"
        echo "esp_websocket_client: Patched (redirect disabled)"
    else
        echo "esp_websocket_client: Redirect pattern not found"
    fi
else
    echo "esp_websocket_client: $WS_CLIENT not found"
fi
