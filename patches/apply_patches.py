#!/usr/bin/env python3
"""Apply IDF 6.0 compatibility patches to managed components.

Run after `idf.py reconfigure` or fresh clone to re-apply patches that fix
managed components for ESP-IDF 6.0 API changes.

Usage:
    python patches/apply_patches.py
"""

import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
MANAGED = PROJECT_ROOT / "managed_components"


def apply_replacements(component_dir: Path, replacements: list) -> bool:
    """Apply a list of (file, old_str, new_str) replacements."""
    all_ok = True
    for rel_path, old_str, new_str in replacements:
        target = component_dir / rel_path
        if not target.exists():
            print(f"  SKIP: {rel_path} not found")
            all_ok = False
            continue

        content = target.read_text(encoding="utf-8")

        if old_str not in content:
            if new_str in content:
                print(f"  OK:   {rel_path} (already patched)")
                continue
            print(f"  FAIL: {rel_path} (original text not found)")
            all_ok = False
            continue

        content = content.replace(old_str, new_str, 1)
        target.write_text(content, encoding="utf-8")
        print(f"  OK:   {rel_path}")

    return all_ok


# ── waveshare/esp_lcd_hx8394 v1.0.3 ────────────────────────────────────────
# IDF 6.0: esp_lcd_panel_dev_config_t.color_space renamed to .rgb_ele_order
HX8394_REPLACEMENTS = [
    (
        "esp_lcd_hx8394.c",
        "switch (panel_dev_config->color_space)",
        "switch (panel_dev_config->rgb_ele_order)",
    ),
]

# ── espressif/esp_lvgl_port v2.7.x ─────────────────────────────────────────
# IDF 6.0: color_space_t, COLOR_TYPE_ID(), COLOR_SPACE_*, COLOR_PIXEL_*
# replaced with ppa_srm_color_mode_t and PPA_SRM_COLOR_MODE_* enums
ESP_LVGL_PORT_REPLACEMENTS = [
    # lcd_ppa.h: config struct fields
    (
        "src/common/ppa/lcd_ppa.h",
        "    color_space_t   color_space;  /*!< Color space of input/output data */\n"
        "    uint32_t        pixel_format; /*!< Pixel format of input/output data */",
        "    ppa_srm_color_mode_t srm_cm;  /*!< PPA SRM color mode (e.g. PPA_SRM_COLOR_MODE_RGB565) */",
    ),
    # lcd_ppa.c: COLOR_TYPE_ID() call
    (
        "src/common/ppa/lcd_ppa.c",
        "ppa_ctx->color_type_id = COLOR_TYPE_ID(cfg->color_space, cfg->pixel_format);",
        "ppa_ctx->color_type_id = cfg->srm_cm;",
    ),
    # esp_lvgl_port_disp.c: pixel_format variable
    (
        "src/lvgl9/esp_lvgl_port_disp.c",
        "        uint32_t pixel_format = COLOR_PIXEL_RGB565;\n"
        "        if (disp_cfg->color_format == LV_COLOR_FORMAT_RGB888) {\n"
        "            pixel_format = COLOR_PIXEL_RGB888;\n"
        "        }",
        "        ppa_srm_color_mode_t srm_cm = PPA_SRM_COLOR_MODE_RGB565;\n"
        "        if (disp_cfg->color_format == LV_COLOR_FORMAT_RGB888) {\n"
        "            srm_cm = PPA_SRM_COLOR_MODE_RGB888;\n"
        "        }",
    ),
    # esp_lvgl_port_disp.c: PPA config struct fields
    (
        "src/lvgl9/esp_lvgl_port_disp.c",
        "            .color_space = COLOR_SPACE_RGB,\n"
        "            .pixel_format = pixel_format,",
        "            .srm_cm = srm_cm,",
    ),
]

PATCHES = [
    (
        "waveshare__esp_lcd_hx8394",
        HX8394_REPLACEMENTS,
        "IDF 6.0: color_space -> rgb_ele_order",
    ),
    (
        "espressif__esp_lvgl_port",
        ESP_LVGL_PORT_REPLACEMENTS,
        "IDF 6.0: color_space_t/COLOR_TYPE_ID -> ppa_srm_color_mode_t",
    ),
]


def main():
    all_ok = True
    print("Applying IDF 6.0 compatibility patches...")
    print(f"Project root: {PROJECT_ROOT}\n")

    for component_name, replacements, description in PATCHES:
        component_dir = MANAGED / component_name
        print(f"[{component_name}] {description}")

        if not component_dir.exists():
            print("  SKIP: component not found (not needed for this target?)\n")
            continue

        if not apply_replacements(component_dir, replacements):
            all_ok = False
        print()

    if all_ok:
        print("All patches applied successfully.")
    else:
        print("Some patches failed - check output above.", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
