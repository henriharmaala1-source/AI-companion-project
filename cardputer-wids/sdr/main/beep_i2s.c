/*
 * beep_i2s.c - the Cardputer speaker through the ESP-IDF I2S driver.
 *
 * Pins and port are M5Stack's own (M5Unified, src/M5Unified.inl, case
 * board_M5Cardputer): BCLK GPIO41, WS GPIO43, DOUT GPIO42, I2S port 1. The
 * ADV is listed there with the same values but also runs an ES8311 codec
 * init over I2C, which this file does not do, so the speaker is expected to
 * stay silent on an ADV (the keyboard driver does not support it either).
 *
 * The channel exists only while a pattern plays: nothing is allocated or
 * driven the rest of the time, and the heap here is small (about 12 KB). If
 * anything fails, the pattern is skipped and the detector carries on.
 */
#include "beep.h"

#include <math.h>
#include <stdbool.h>

#include "driver/gpio.h"
#include "driver/i2s_std.h"

#define SPK_BCLK   GPIO_NUM_41
#define SPK_WS     GPIO_NUM_43
#define SPK_DOUT   GPIO_NUM_42
#define SPK_PORT   I2S_NUM_1
#define RATE_HZ    16000u
#define CHUNK      64u        /* stereo frames per write: 256 bytes of stack */
#define RAMP_MS    5u         /* fade in/out: no click at the tone edges */
#define TWO_PI_F   6.28318530717958647692f

static i2s_chan_handle_t s_tx;
static float s_phase;         /* continues across tones: no click either */

int beep_hw_open(void)
{
    if (s_tx != NULL) {
        return 1;
    }
    i2s_chan_config_t chan = I2S_CHANNEL_DEFAULT_CONFIG(SPK_PORT, I2S_ROLE_MASTER);
    chan.dma_desc_num = 2;                /* the driver's minimum */
    chan.dma_frame_num = CHUNK * 2;       /* 2 x 128 x 4 B = 1 KB */
    if (i2s_new_channel(&chan, &s_tx, NULL) != ESP_OK) {
        s_tx = NULL;
        return 0;
    }
    i2s_std_config_t cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(RATE_HZ),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = SPK_BCLK,
            .ws = SPK_WS,
            .dout = SPK_DOUT,
            .din = I2S_GPIO_UNUSED,
        },
    };
    if (i2s_channel_init_std_mode(s_tx, &cfg) != ESP_OK || i2s_channel_enable(s_tx) != ESP_OK) {
        (void)i2s_del_channel(s_tx);
        s_tx = NULL;
        return 0;
    }
    s_phase = 0.0f;
    return 1;
}

/* Hand n stereo frames to the driver; a failed write only drops that chunk. */
static void write_chunk(const int16_t *frames, size_t n)
{
    size_t written = 0;
    (void)i2s_channel_write(s_tx, frames, n * 2u * sizeof(int16_t), &written, 200);
}

void beep_hw_tone(unsigned hz, unsigned ms, int amp)
{
    if (s_tx == NULL) {
        return;
    }
    const unsigned total = RATE_HZ * ms / 1000u;
    const unsigned ramp = RATE_HZ * RAMP_MS / 1000u;
    const float step = TWO_PI_F * (float)hz / (float)RATE_HZ;
    int16_t buf[CHUNK * 2];
    unsigned done = 0;
    while (done < total) {
        const unsigned n = total - done < CHUNK ? total - done : CHUNK;
        for (unsigned k = 0; k < n; k++) {
            const unsigned at = done + k;
            float env = 1.0f;
            if (at < ramp) {
                env = (float)at / (float)ramp;
            } else if (total - at < ramp) {
                env = (float)(total - at) / (float)ramp;
            }
            const int16_t v = hz ? (int16_t)(env * (float)amp * sinf(s_phase)) : 0;
            buf[2 * k] = v;       /* left and right the same */
            buf[2 * k + 1] = v;
            s_phase += step;
            if (s_phase > TWO_PI_F) {
                s_phase -= TWO_PI_F;
            }
        }
        write_chunk(buf, n);
        done += n;
    }
}

void beep_hw_close(void)
{
    if (s_tx == NULL) {
        return;
    }
    /* write() returns once the data is queued, not played: send enough
     * silence to push the last samples out, then stop. */
    int16_t zeros[CHUNK * 2] = { 0 };
    for (unsigned k = 0; k < 4; k++) {
        write_chunk(zeros, CHUNK);
    }
    (void)i2s_channel_disable(s_tx);
    (void)i2s_del_channel(s_tx);
    s_tx = NULL;
}
