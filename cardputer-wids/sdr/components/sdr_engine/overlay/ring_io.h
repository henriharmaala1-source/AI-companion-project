/* SPDX-License-Identifier: GPL-3.0-or-later
 *
 * ring_io.h - replacement for esp-sdr's main/common/ring_io.h.
 *
 * Upstream's version moves bytes to the USB Serial/JTAG FIFO or UART0. This
 * one moves them to a RAM sink so the Cardputer can analyse spectra itself.
 * It is copied next to a build-time copy of ring_capture.c (see this
 * component's CMakeLists.txt), because a same-directory "#include" always
 * beats -I paths. The function names and contracts match upstream exactly:
 *   ring_input_available() - true when the run should stop;
 *   ring_read_byte()       - consume one stop byte ('\n');
 *   ring_write()           - non-blocking, returns bytes accepted.
 */
#include <stdbool.h>
#include <stdint.h>
#include "sdr_sink.h"

static bool ring_input_available(void) { return sdr_sink_stop_requested(); }

static int ring_read_byte(uint8_t *b)
{
    if (!sdr_sink_stop_requested()) {
        return 0;
    }
    sdr_sink_take_stop();
    *b = '\n';
    return 1;
}

static int ring_write(const uint8_t *p, unsigned n) { return sdr_sink_write(p, n); }
