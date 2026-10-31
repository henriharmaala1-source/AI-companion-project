/* sim_platform.h - virtual clock, console capture, scripted keys. */
#ifndef SIM_PLATFORM_H
#define SIM_PLATFORM_H

#include <stdbool.h>
#include <stdint.h>

#define SIM_MAX_TRANSITIONS 256
#define SIM_MAX_WHYS        32
#define SIM_MAX_TARGET_KINDS 8

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
    unsigned beeps_detected, beeps_lost, beeps_boot, beeps_test;   /* "beep" events by pattern */
    unsigned beeps_unplayed;           /* "beep" events with played:false (sound off) */
    unsigned sound_changes;            /* "sound" events ('m' key) */
    unsigned lora_fixed;               /* "lora_fixed" events with fixed:true */
    float    lora_delta_db;            /* from the last "lora" line */
    float    lora_snr_db;
    /* "target" events, per kind name (targets.c) */
    char     tgt_kind[SIM_MAX_TARGET_KINDS][16];
    unsigned tgt_ons[SIM_MAX_TARGET_KINDS];      /* times it turned on */
    int64_t  tgt_first_us[SIM_MAX_TARGET_KINDS]; /* first "on" */
    int64_t  tgt_last_off_us[SIM_MAX_TARGET_KINDS]; /* last "off", -1 = never */
    bool     tgt_now[SIM_MAX_TARGET_KINDS];      /* on at the end of the run */
    char     tgt_text[SIM_MAX_TARGET_KINDS][48]; /* text of the first "on" */
    unsigned n_tgt;
} sim_log_t;

/* Index of a target kind in the log, or -1 if it never appeared. */
int sim_target(const sim_log_t *log, const char *kind);

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
