/* cp_keymap.c - pure parts of cp_keyboard.h. Ported from M5Cardputer (MIT). */
#include "cp_keyboard.h"

/* First layer of M5Cardputer's _key_value_map[4][14], same order. */
static const uint8_t k_map[CP_KB_ROWS][CP_KB_COLS] = {
    { '`', '1', '2', '3', '4', '5', '6', '7', '8', '9', '0', '-', '=', CP_KEY_BACKSPACE },
    { CP_KEY_TAB, 'q', 'w', 'e', 'r', 't', 'y', 'u', 'i', 'o', 'p', '[', ']', '\\' },
    { CP_KEY_FN, CP_KEY_SHIFT, 'a', 's', 'd', 'f', 'g', 'h', 'j', 'k', 'l', ';', '\'', CP_KEY_ENTER },
    { CP_KEY_CTRL, CP_KEY_OPT, CP_KEY_ALT, 'z', 'x', 'c', 'v', 'b', 'n', 'm', ',', '.', '/', ' ' },
};

/* IOMatrix.h X_map_chart: sense line j feeds column x_1 in select states
 * 4..7 and column x_2 in states 0..3. */
static const uint8_t k_col_hi[7] = { 0, 2, 4, 6, 8, 10, 12 };
static const uint8_t k_col_lo[7] = { 1, 3, 5, 7, 9, 11, 13 };

bool cp_kb_position(unsigned select, unsigned sense, unsigned *row, unsigned *col)
{
    if (select > 7 || sense > 6 || row == 0 || col == 0) {
        return false;
    }
    /* IOMatrix.cpp: y = (i > 3) ? i - 4 : i, then flipped as 3 - y. */
    const unsigned y = select > 3 ? select - 4 : select;
    *row = 3u - y;
    *col = select > 3 ? k_col_hi[sense] : k_col_lo[sense];
    return true;
}

uint8_t cp_kb_char(unsigned row, unsigned col)
{
    if (row >= CP_KB_ROWS || col >= CP_KB_COLS) {
        return CP_KEY_NONE;
    }
    return k_map[row][col];
}
