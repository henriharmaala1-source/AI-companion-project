/*
 * Host shim for M5GFX: the REAL LovyanGFX drawing code (fonts, text width,
 * fills) drawing into a 240x135 RGB565 canvas instead of the ST7789. ui.cpp
 * compiles against this unchanged.
 *
 * On the Cardputer the panel is 135x240 and rotation 1 makes it 240x135
 * landscape; the canvas is created already landscape, so setRotation() is a
 * no-op here. Brightness has no meaning in a PNG.
 */
#pragma once
#include <lgfx/v1/LGFXBase.hpp>
#include <lgfx/v1/LGFX_Sprite.hpp>
#include "sim.h"

class M5GFX : public lgfx::LGFX_Sprite {
public:
    bool init(void);
    void setRotation(uint_fast8_t r) { (void)r; }
    void setBrightness(uint8_t b) { (void)b; }
    void waitDMA(void) { sim_display_flushed(); }
};
