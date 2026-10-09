/*
 * sim_display.cpp - the Cardputer LCD as an in-memory canvas.
 *
 * ui.cpp draws through the real LovyanGFX code into a 240x135 RGB565 sprite
 * (see shim/M5GFX.h). The harness reads pixels back and writes PNGs.
 */
#include <M5GFX.h>
#include <zlib.h>

#include <cstdio>
#include <cstring>
#include <vector>

#include "sim.h"

/* ASSUMPTION: one ui_update() pushes roughly 10-25 k pixels over SPI at
 * 40 MHz, i.e. a few milliseconds. The virtual clock is charged this much per
 * waitDMA() so the time between capture slices is not free. */
static constexpr int64_t FLUSH_US = 6000;

static M5GFX *s_panel;

bool M5GFX::init(void)
{
    setColorDepth(16);
    if (!createSprite(240, 135)) {
        return false;
    }
    s_panel = this;
    return true;
}

extern "C" void sim_display_flushed(void) { sim_advance_us(FLUSH_US); }

extern "C" int sim_display_ready(void) { return s_panel != nullptr; }

extern "C" uint16_t sim_display_pixel(int x, int y)
{
    return s_panel ? (uint16_t)s_panel->readPixel(x, y) : 0;
}

/* ------------------------------------------------------------ PNG */

static void be32(std::vector<uint8_t> &v, uint32_t x)
{
    for (int s = 24; s >= 0; s -= 8) {
        v.push_back((uint8_t)(x >> s));
    }
}

static void chunk(FILE *f, const char *type, const std::vector<uint8_t> &data)
{
    std::vector<uint8_t> c;
    be32(c, (uint32_t)data.size());
    c.insert(c.end(), type, type + 4);
    c.insert(c.end(), data.begin(), data.end());
    const uLong crc = crc32(0, c.data() + 4, (uInt)(c.size() - 4));
    be32(c, (uint32_t)crc);
    fwrite(c.data(), 1, c.size(), f);
}

/* Written at 2x so the 6x8 font is readable on a laptop screen. */
extern "C" int sim_display_save_png(const char *path)
{
    if (!s_panel) {
        return -1;
    }
    const int scale = 2, w = 240 * scale, h = 135 * scale;
    std::vector<uint8_t> raw;
    raw.reserve((size_t)(w * 3 + 1) * h);
    for (int y = 0; y < h; y++) {
        raw.push_back(0); /* filter: none */
        for (int x = 0; x < w; x++) {
            const uint16_t c = sim_display_pixel(x / scale, y / scale);
            raw.push_back((uint8_t)(((c >> 11) & 31) * 255 / 31));
            raw.push_back((uint8_t)(((c >> 5) & 63) * 255 / 63));
            raw.push_back((uint8_t)((c & 31) * 255 / 31));
        }
    }
    uLongf zlen = compressBound((uLong)raw.size());
    std::vector<uint8_t> z(zlen);
    if (compress2(z.data(), &zlen, raw.data(), (uLong)raw.size(), 9) != Z_OK) {
        return -1;
    }
    z.resize(zlen);

    FILE *f = std::fopen(path, "wb");
    if (!f) {
        return -1;
    }
    static const uint8_t sig[8] = { 0x89, 'P', 'N', 'G', '\r', '\n', 0x1a, '\n' };
    std::fwrite(sig, 1, sizeof(sig), f);
    std::vector<uint8_t> ihdr;
    be32(ihdr, (uint32_t)w);
    be32(ihdr, (uint32_t)h);
    ihdr.insert(ihdr.end(), { 8, 2, 0, 0, 0 }); /* 8-bit RGB */
    chunk(f, "IHDR", ihdr);
    chunk(f, "IDAT", z);
    chunk(f, "IEND", {});
    std::fclose(f);
    return 0;
}
