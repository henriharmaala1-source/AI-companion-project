/* cp_keyboard.c - GPIO side of cp_keyboard.h. */
#include "cp_keyboard.h"

#include "driver/gpio.h"
#include "esp_rom_sys.h"

static const gpio_num_t k_select[3] = { GPIO_NUM_8, GPIO_NUM_9, GPIO_NUM_11 };
static const gpio_num_t k_sense[7] = { GPIO_NUM_13, GPIO_NUM_15, GPIO_NUM_3, GPIO_NUM_4,
                                       GPIO_NUM_5, GPIO_NUM_6, GPIO_NUM_7 };

static uint8_t s_prev[8];   /* sense bits per select state, previous scan */
static uint8_t s_stable[8]; /* debounced state */

void cp_kb_init(void)
{
    for (unsigned i = 0; i < 3; i++) {
        gpio_reset_pin(k_select[i]);
        gpio_set_direction(k_select[i], GPIO_MODE_OUTPUT);
        gpio_set_level(k_select[i], 0);
    }
    for (unsigned i = 0; i < 7; i++) {
        gpio_reset_pin(k_sense[i]);
        gpio_set_direction(k_sense[i], GPIO_MODE_INPUT);
        gpio_set_pull_mode(k_sense[i], GPIO_PULLUP_ONLY);
    }
}

static uint8_t read_state(unsigned select)
{
    for (unsigned b = 0; b < 3; b++) {
        gpio_set_level(k_select[b], (select >> b) & 1u);
    }
    esp_rom_delay_us(10); /* let the select lines settle before sampling */
    uint8_t bits = 0;
    for (unsigned j = 0; j < 7; j++) {
        if (gpio_get_level(k_sense[j]) == 0) { /* active low */
            bits |= (uint8_t)(1u << j);
        }
    }
    return bits;
}

uint8_t cp_kb_poll(void)
{
    uint8_t pressed = CP_KEY_NONE;
    for (unsigned i = 0; i < 8; i++) {
        const uint8_t now = read_state(i);
        const uint8_t stable = now & s_prev[i];          /* seen twice */
        const uint8_t newly = stable & (uint8_t)~s_stable[i];
        s_stable[i] = (uint8_t)((s_stable[i] & now) | stable);
        s_prev[i] = now;
        for (unsigned j = 0; j < 7 && pressed == CP_KEY_NONE; j++) {
            unsigned row, col;
            if ((newly & (1u << j)) && cp_kb_position(i, j, &row, &col)) {
                pressed = cp_kb_char(row, col);
            }
        }
    }
    return pressed;
}
