/*
 * sim_platform.c - the "board" under the simulated firmware.
 *
 * One thread, one virtual clock in microseconds. Time moves only when the
 * code under test would spend it: capture slices (sim_engine.c), screen
 * transfers (sim_display.cpp), processing, and vTaskDelay(). The firmware's
 * main loop never returns, so the run ends by longjmp() out of vTaskDelay()
 * once the scenario's time is up.
 */
#include "sim_platform.h"

#include <setjmp.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cp_keyboard.h"
#include "elrs_detect.h"
#include "sim.h"

static int64_t s_now_us;
static int64_t s_end_us;
static jmp_buf s_end_jmp;
static FILE   *s_log_file;
static sim_log_t s_log;
static char    s_line[1024];
static size_t  s_line_len;
static const sim_key_t *s_keys;
static unsigned s_n_keys, s_next_key;
static void   (*s_tick)(void);

/* ------------------------------------------------------------ clock */

int64_t sim_now_us(void) { return s_now_us; }

void sim_advance_us(int64_t us)
{
    if (us > 0) {
        s_now_us += us;
    }
}

void sim_task_delay_ms(uint32_t ms)
{
    if (s_tick) {
        s_tick();
    }
    sim_advance_us((int64_t)ms * 1000);
    if (s_now_us >= s_end_us) {
        longjmp(s_end_jmp, 1);
    }
}

void sim_fail(const char *why)
{
    fprintf(stderr, "SIM FAIL at t=%.3f s: %s\n", s_now_us / 1e6, why);
    exit(2);
}

/* ------------------------------------------------------------ console */

/* Value of "key": in a JSON line, copied as text. Good enough for the flat
 * objects wids_log writes; not a general JSON parser. */
static bool field(const char *line, const char *key, char *out, size_t cap)
{
    char pat[32];
    snprintf(pat, sizeof(pat), "\"%s\":", key);
    const char *p = strstr(line, pat);
    if (p == NULL || cap == 0) {
        return false;
    }
    p += strlen(pat);
    const bool quoted = *p == '"';
    p += quoted ? 1 : 0;
    size_t n = 0;
    while (p[n] && n + 1 < cap && (quoted ? p[n] != '"' : (p[n] != ',' && p[n] != '}'))) {
        out[n] = p[n];
        n++;
    }
    out[n] = '\0';
    return true;
}

static int state_from_name(const char *s)
{
    if (strcmp(s, elrs_state_name(ELRS_LIKELY)) == 0) {
        return ELRS_LIKELY;
    }
    return strcmp(s, elrs_state_name(ELRS_HOPPER)) == 0 ? ELRS_HOPPER : ELRS_QUIET;
}

static void parse_line(const char *line)
{
    char ev[32], val[64];
    s_log.lines++;
    const size_t len = strlen(line);
    if (len < 2 || line[0] != '{' || line[len - 1] != '}' || !field(line, "t", val, sizeof(val)) ||
        !field(line, "ev", ev, sizeof(ev))) {
        s_log.bad_json = true;
        return;
    }
    const int64_t t = strtoll(val, NULL, 10);
    if (strcmp(ev, "elrs") == 0 && field(line, "state", val, sizeof(val)) &&
        s_log.n_tr < SIM_MAX_TRANSITIONS) {
        s_log.tr[s_log.n_tr++] = (sim_transition_t){ .t_us = t, .state = state_from_name(val) };
    } else if (strcmp(ev, "elrs_config") == 0 && s_log.n_whys < SIM_MAX_WHYS) {
        if (field(line, "why", s_log.whys[s_log.n_whys], sizeof(s_log.whys[0]))) {
            s_log.n_whys++;
        }
    } else if (strcmp(ev, "sdr_stats") == 0) {
        if (field(line, "crc_bad", val, sizeof(val))) {
            s_log.crc_bad = (uint32_t)strtoul(val, NULL, 10);
        }
        if (field(line, "frames", val, sizeof(val))) {
            s_log.frames_last = (uint32_t)strtoul(val, NULL, 10);
        }
    } else if (strcmp(ev, "boot") == 0) {
        s_log.boots++;
    } else if (strcmp(ev, "error") == 0) {
        s_log.errors++;
    } else if (strcmp(ev, "sdr_error") == 0) {
        s_log.sdr_errors++;
    }
}

void sim_console_write(const char *data, size_t len)
{
    if (s_log_file) {
        fwrite(data, 1, len, s_log_file);
    }
    for (size_t i = 0; i < len; i++) {
        if (data[i] == '\n') {
            s_line[s_line_len] = '\0';
            parse_line(s_line);
            s_line_len = 0;
        } else if (s_line_len + 1 < sizeof(s_line)) {
            s_line[s_line_len++] = data[i];
        }
    }
}

/* ------------------------------------------------------------ keyboard */

void cp_kb_init(void) {}

uint8_t cp_kb_poll(void)
{
    if (s_next_key < s_n_keys && s_now_us >= (int64_t)(s_keys[s_next_key].t_s * 1e6)) {
        return s_keys[s_next_key++].key;
    }
    return 0;
}

/* ------------------------------------------------------------ run */

bool sim_platform_start(const char *log_path, double end_s, const sim_key_t *keys, unsigned n_keys,
                        void (*tick)(void))
{
    s_log_file = fopen(log_path, "w");
    memset(&s_log, 0, sizeof(s_log));
    s_now_us = 0;
    s_end_us = (int64_t)(end_s * 1e6);
    s_keys = keys;
    s_n_keys = n_keys;
    s_next_key = 0;
    s_tick = tick;
    s_line_len = 0;
    return s_log_file != NULL;
}

void sim_platform_run(void (*entry)(void))
{
    if (setjmp(s_end_jmp) == 0) {
        entry();
    }
    if (s_log_file) {
        fclose(s_log_file);
        s_log_file = NULL;
    }
}

const sim_log_t *sim_platform_log(void) { return &s_log; }

int sim_state_at(const sim_log_t *log, double t_s)
{
    int state = ELRS_QUIET;
    for (unsigned i = 0; i < log->n_tr && log->tr[i].t_us <= (int64_t)(t_s * 1e6); i++) {
        state = log->tr[i].state;
    }
    return state;
}
