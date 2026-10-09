/* Host test: the ported Cardputer matrix covers every key exactly once. */
#include "cp_keyboard.h"
#include <assert.h>
#include <stdio.h>

int main(void)
{
    unsigned seen[CP_KB_ROWS][CP_KB_COLS] = { { 0 } };
    for (unsigned s = 0; s < 8; s++) {
        for (unsigned j = 0; j < 7; j++) {
            unsigned r, c;
            assert(cp_kb_position(s, j, &r, &c));
            assert(r < CP_KB_ROWS && c < CP_KB_COLS);
            seen[r][c]++;
        }
    }
    for (unsigned r = 0; r < CP_KB_ROWS; r++) {
        for (unsigned c = 0; c < CP_KB_COLS; c++) {
            assert(seen[r][c] == 1);
        }
    }
    unsigned r, c;
    assert(!cp_kb_position(8, 0, &r, &c) && !cp_kb_position(0, 7, &r, &c));
    assert(cp_kb_char(9, 0) == CP_KEY_NONE);
    assert(cp_kb_char(3, 13) == ' ' && cp_kb_char(0, 13) == CP_KEY_BACKSPACE && cp_kb_char(1, 1) == 'q');
    printf("KEYMAP TEST PASSED: 56 contacts -> 56 distinct keys\n");
    return 0;
}
