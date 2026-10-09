/* sim_platform.h - virtual clock, console capture, scripted keys. */
#ifndef SIM_PLATFORM_H
#define SIM_PLATFORM_H

#include <stdbool.h>
#include <stdint.h>

#define SIM_MAX_TRANSITIONS 256
#define SIM_MAX_WHYS        32

typedef struct {
    double  t_s;
    uint8_t key;
} sim_key_t;

typedef struct {
    int64_t t_us;
    int     state;        /* elrs_state_t */
} sim_transition_t;

/* What the firmware said on its console, parsed as it was written. */
typedef struct {
    sim_transition_t tr[SIM_MAX_TRANSITIONS];
    unsigned n_tr;
    char     whys[SIM_MAX_WHYS][16];   /* "why" of every elrs_config event */
    unsigned n_whys;
    unsigned lines, boots, errors, sdr_errors;
    uint32_t crc_bad, frames_last;     /* from the last sdr_stats line */
    bool     bad_json;                 /* a line that does not look like NDJSON */
} sim_log_t;

/* Prepare a run: console goes to `log_path`, keys fire at their times, the
 * run ends at `end_s` of virtual time. `tick` is called once per main-loop
 * pass (at vTaskDelay), when the screen is in a settled state. */
bool sim_platform_start(const char *log_path, double end_s, const sim_key_t *keys, unsigned n_keys,
                        void (*tick)(void));

/* Call app_main(); returns when the virtual clock reaches end_s. */
void sim_platform_run(void (*entry)(void));

const sim_log_t *sim_platform_log(void);
int  sim_state_at(const sim_log_t *log, double t_s);   /* QUIET before any event */

#endif /* SIM_PLATFORM_H */
