/*
 * cp_keyboard.h - Cardputer (original, GPIO-matrix) keyboard for ESP-IDF.
 *
 * Replaces M5Cardputer's Arduino-only driver. The matrix and the key map are
 * ported from M5Stack's M5Cardputer library (MIT licence, (c) M5Stack
 * Technology Co Ltd): IOMatrix.h / IOMatrix.cpp / Keyboard.h.
 *
 *   select lines  GPIO 8, 9, 11   (3-bit code, 8 states)
 *   sense lines   GPIO 13, 15, 3, 4, 5, 6, 7  (active low, pulled up)
 *   8 states x 7 lines = 56 keys = the 4 x 14 layout.
 *
 * The Cardputer ADV uses a TCA8418 I2C controller instead; this driver does
 * NOT support it (cp_kb_init() cannot tell the boards apart - check yours).
 *
 * Only unshifted characters are reported. Modifier keys come back as the
 * CP_KEY_* codes below so the app can see them if it wants to.
 */
#ifndef CP_KEYBOARD_H
#define CP_KEYBOARD_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CP_KB_ROWS 4u
#define CP_KB_COLS 14u

enum {
    CP_KEY_NONE = 0x00,
    CP_KEY_BACKSPACE = 0x08,
    CP_KEY_TAB = 0x09,
    CP_KEY_ENTER = 0x0d,
    CP_KEY_FN = 0x80,
    CP_KEY_SHIFT = 0x81,
    CP_KEY_CTRL = 0x82,
    CP_KEY_OPT = 0x83,
    CP_KEY_ALT = 0x84,
};

/* Pure helpers (no hardware), unit-tested on the host. */

/* Map one matrix contact - select state 0..7, sense line 0..6 - to a key
 * position. Returns false for out-of-range input. */
bool cp_kb_position(unsigned select, unsigned sense, unsigned *row, unsigned *col);

/* Unshifted character at a position, or a CP_KEY_* code. */
uint8_t cp_kb_char(unsigned row, unsigned col);

/* Hardware (ESP-IDF only). */
void cp_kb_init(void);

/* Scan the matrix once (about 8 x 10 us). Returns the first newly pressed
 * key since the previous scan, or CP_KEY_NONE. Debounced by requiring the
 * key to be seen on two consecutive scans. */
uint8_t cp_kb_poll(void);

#ifdef __cplusplus
}
#endif
#endif /* CP_KEYBOARD_H */
