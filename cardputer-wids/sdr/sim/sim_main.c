/*
 * sim_main.c - scenarios and checks for the ELRS watch simulator.
 *
 * Each scenario boots the REAL firmware (app_main.c, ui.cpp, the detector,
 * the SPC1 decoder, the sink, wids_log) on a virtual clock, with a modelled
 * radio scene and scripted key presses, then checks what a user would check:
 * the console log, the verdict timeline and the screen.
 *
 *   sim list              scenario names
 *   sim <name> [outdir]   one scenario
 *   sim all [outdir]      every scenario, each in its own process
 *
 * Output per scenario in outdir/<name>/: log.ndjson (the console), PNG
 * screenshots, summary.txt. Exit status is non-zero if any check failed.
 */
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>

#include "elrs_detect.h"
#include "rf_scene.h"
#include "sim.h"
#include "sim_engine.h"
#include "sim_platform.h"

void app_main(void);

/* ------------------------------------------------------------ scenarios */

typedef enum { SHOT_ANY, SHOT_DETECTED, SHOT_NOT_DETECTED, SHOT_DETAILS } shot_expect_t;

typedef struct {
    double t_s;
    const char *name;
    shot_expect_t expect;
} shot_t;

typedef struct {
    const char *what;
    double   duration_s;
    bool     inverted;
    rf_scene_t scene;
    sim_key_t keys[8];
    unsigned n_keys;
    shot_t   shots[8];
    unsigned n_shots;
    /* expectations */
    bool     expect_detect;      /* false: ELRS-LIKELY must never appear */
    double   elrs_on_s, elrs_off_s;
    double   detect_within_s;    /* after elrs_on_s (or after the last key) */
    double   detect_after_s;     /* no ELRS-LIKELY before this time */
    double   release_within_s;   /* after elrs_off_s */
    const char *need_whys[4];
} scenario_t;

static rf_emitter_t elrs(float level, double on, double off, float packet_ms, float air_ms,
                         unsigned pph, float bw_khz)
{
    return (rf_emitter_t){ .kind = RF_ELRS, .level_db = level, .on_s = on, .off_s = off,
                           .packet_ms = packet_ms, .airtime_ms = air_ms, .packets_per_hop = pph,
                           .bw_khz = bw_khz, .seed = 0x5eed };
}

/* ExpressLRS 2.4 GHz rates, same figures as test/test_elrs_detect.c. */
static rf_emitter_t elrs_500(float lvl, double on, double off) { return elrs(lvl, on, off, 2.0f, 1.1f, 4, 800); }
static rf_emitter_t elrs_250(float lvl, double on, double off) { return elrs(lvl, on, off, 4.0f, 2.2f, 4, 800); }
static rf_emitter_t elrs_50(float lvl, double on, double off) { return elrs(lvl, on, off, 20.0f, 8.0f, 2, 800); }
static rf_emitter_t elrs_f1000(float lvl, double on, double off) { return elrs(lvl, on, off, 1.0f, 0.25f, 2, 600); }

static rf_emitter_t wifi(unsigned ch, float lvl, float duty)
{
    return (rf_emitter_t){ .kind = RF_WIFI, .level_db = lvl, .on_s = 0, .off_s = 1e9,
                           .wifi_channel = ch, .duty = duty };
}

static rf_emitter_t ble(float lvl)
{
    return (rf_emitter_t){ .kind = RF_BLE, .level_db = lvl, .on_s = 0, .off_s = 1e9 };
}

/* Bluetooth Classic audio, AFH avoiding `wifi_ch`. */
static rf_emitter_t bt_audio(float lvl, unsigned wifi_ch)
{
    return (rf_emitter_t){ .kind = RF_BT_CLASSIC, .level_db = lvl, .on_s = 0, .off_s = 1e9,
                           .wifi_channel = wifi_ch };
}

static void add(scenario_t *s, rf_emitter_t e) { (void)rf_scene_add(&s->scene, &e); }

/* A typical flat: a busy access point on channel 6, a neighbour's on 1, a
 * phone and a watch doing BLE. */
static void home_background(scenario_t *s)
{
    add(s, wifi(6, 25.0f, 0.25f));
    add(s, wifi(1, 12.0f, 0.05f));
    add(s, ble(18.0f));
}

static void shot(scenario_t *s, double t, const char *name, shot_expect_t e)
{
    s->shots[s->n_shots++] = (shot_t){ t, name, e };
}

static void key(scenario_t *s, double t, uint8_t k)
{
    s->keys[s->n_keys++] = (sim_key_t){ t, k };
}

static void sc_home_no_elrs(scenario_t *s)
{
    s->what = "Wi-Fi, BLE and Bluetooth audio, no ELRS: must never say DETECTED";
    s->duration_s = 90;
    home_background(s);
    add(s, bt_audio(30.0f, 6));
    shot(s, 30, "idle", SHOT_NOT_DETECTED);
    shot(s, 89, "end", SHOT_NOT_DETECTED);
}

static void sc_bt_audio_no_afh(scenario_t *s)
{
    s->what = "Loud Bluetooth audio on all 79 channels (no AFH), the closest look-alike";
    s->duration_s = 60;
    add(s, bt_audio(40.0f, 0));
    shot(s, 59, "end", SHOT_NOT_DETECTED);
}

static void sc_elrs_500_arrives(scenario_t *s)
{
    s->what = "ELRS 500 Hz switched on at 10 s and off at 50 s, home background";
    s->duration_s = 90;
    home_background(s);
    add(s, elrs_500(25.0f, 10, 50));
    s->expect_detect = true;
    s->elrs_on_s = 10;
    s->elrs_off_s = 50;
    s->detect_within_s = 10;
    s->detect_after_s = 10;
    s->release_within_s = 20;
    shot(s, 5, "before", SHOT_NOT_DETECTED);
    shot(s, 35, "during", SHOT_DETECTED);
    shot(s, 85, "after", SHOT_NOT_DETECTED);
}

static void rate_case(scenario_t *s, const char *what, rf_emitter_t e)
{
    s->what = what;
    s->duration_s = 40;
    add(s, wifi(6, 25.0f, 0.25f));
    add(s, e);
    s->expect_detect = true;
    s->elrs_on_s = 2;
    s->elrs_off_s = 1e9;
    s->detect_within_s = 10;
    s->detect_after_s = 2;
    shot(s, 39, "end", SHOT_DETECTED);
}

static void sc_elrs_f1000(scenario_t *s) { rate_case(s, "ELRS F1000 (FLRC, 2 ms dwell)", elrs_f1000(25.0f, 2, 1e9)); }
static void sc_elrs_250(scenario_t *s) { rate_case(s, "ELRS 250 Hz (16 ms dwell)", elrs_250(25.0f, 2, 1e9)); }
static void sc_elrs_50(scenario_t *s) { rate_case(s, "ELRS 50 Hz (40 ms dwell, slowest)", elrs_50(25.0f, 2, 1e9)); }

static void sc_elrs_in_crowd(scenario_t *s)
{
    s->what = "ELRS 250 Hz alongside Bluetooth audio, BLE and busy Wi-Fi";
    s->duration_s = 40;
    home_background(s);
    add(s, bt_audio(30.0f, 6));
    add(s, elrs_250(25.0f, 2, 1e9));
    s->expect_detect = true;
    s->elrs_on_s = 2;
    s->elrs_off_s = 1e9;
    s->detect_within_s = 12;
    s->detect_after_s = 2;
    shot(s, 39, "end", SHOT_DETECTED);
}

static void sc_elrs_50_beside_bt(scenario_t *s)
{
    s->what = "Slowest ELRS (50 Hz, fewest bursts) beside loud full-band Bluetooth audio";
    s->duration_s = 40;
    add(s, bt_audio(40.0f, 0));
    add(s, elrs_50(25.0f, 2, 1e9));
    s->expect_detect = true;
    s->elrs_on_s = 2;
    s->elrs_off_s = 1e9;
    s->detect_within_s = 12;
    s->detect_after_s = 2;
    shot(s, 39, "end", SHOT_DETECTED);
}

static void sc_elrs_weak(scenario_t *s)
{
    s->what = "ELRS 6 dB above noise, under the 10 dB threshold: an honest miss";
    s->duration_s = 40;
    add(s, elrs_500(6.0f, 2, 1e9));
    shot(s, 39, "end", SHOT_NOT_DETECTED);
}

static void sc_inverted_then_i(scenario_t *s)
{
    s->what = "Hardware delivers a mirrored spectrum; pressing 'i' at 25 s must recover";
    s->duration_s = 60;
    s->inverted = true;
    add(s, wifi(6, 25.0f, 0.25f));
    add(s, elrs_500(25.0f, 2, 1e9));
    key(s, 25, 'i');
    key(s, 45, 'd');
    key(s, 50, 'd');
    s->expect_detect = true;
    s->elrs_on_s = 25;   /* the clock for "detect within" starts at the key */
    s->elrs_off_s = 1e9;
    s->detect_within_s = 12;
    s->detect_after_s = 0;
    s->need_whys[0] = "invert";
    shot(s, 20, "before_i", SHOT_ANY);
    shot(s, 40, "after_i", SHOT_DETECTED);
    shot(s, 47, "details", SHOT_DETAILS);
    shot(s, 55, "back", SHOT_DETECTED);
}

static void sc_keys(scenario_t *s)
{
    s->what = "Every key: + - t c d; log records each change, details view works";
    s->duration_s = 20;
    home_background(s);
    key(s, 3, '+');
    key(s, 4, '-');
    key(s, 5, 't');
    key(s, 6, 'c');
    key(s, 8, 'd');
    key(s, 12, 'd');
    s->need_whys[0] = "threshold";
    s->need_whys[1] = "clear";
    shot(s, 10, "details", SHOT_DETAILS);
    shot(s, 15, "main", SHOT_NOT_DETECTED);
}

typedef struct {
    const char *name;
    void (*build)(scenario_t *s);
} entry_t;

static const entry_t k_scenarios[] = {
    { "home_no_elrs", sc_home_no_elrs },
    { "bt_audio_no_afh", sc_bt_audio_no_afh },
    { "elrs_500_arrives", sc_elrs_500_arrives },
    { "elrs_f1000", sc_elrs_f1000 },
    { "elrs_250", sc_elrs_250 },
    { "elrs_50", sc_elrs_50 },
    { "elrs_in_crowd", sc_elrs_in_crowd },
    { "elrs_50_beside_bt", sc_elrs_50_beside_bt },
    { "elrs_weak", sc_elrs_weak },
    { "inverted_then_i", sc_inverted_then_i },
    { "keys", sc_keys },
};
#define N_SCENARIOS (sizeof(k_scenarios) / sizeof(k_scenarios[0]))

/* ------------------------------------------------------------ the screen */

/* What the main view is showing, read from two pixels ui.cpp always paints:
 * the red banner fill, or the corner of the green "NOT DETECTED" outline. */
static shot_expect_t screen_shows(void)
{
    if (sim_display_pixel(2, 16) == 0xF800) {
        return SHOT_DETECTED;
    }
    if (sim_display_pixel(4, 16) == 0x07E0) {
        return SHOT_NOT_DETECTED;
    }
    return SHOT_DETAILS;
}

static const char *shot_name(shot_expect_t e)
{
    switch (e) {
    case SHOT_DETECTED: return "DETECTED";
    case SHOT_NOT_DETECTED: return "NOT DETECTED";
    case SHOT_DETAILS: return "details/other";
    default: return "any";
    }
}

static const scenario_t *g_sc;
static char g_dir[512];
static shot_expect_t g_seen[8];
static bool g_taken[8];
static double g_next_film_s;

#define FILM_PERIOD_S 5.0

/* Called once per main-loop pass, after the screen has been drawn. */
static void tick(void)
{
    char path[640];
    const double t = sim_now_us() / 1e6;
    for (unsigned i = 0; i < g_sc->n_shots; i++) {
        if (!g_taken[i] && t >= g_sc->shots[i].t_s) {
            snprintf(path, sizeof(path), "%s/%s.png", g_dir, g_sc->shots[i].name);
            (void)sim_display_save_png(path);
            g_seen[i] = screen_shows();
            g_taken[i] = true;
        }
    }
    if (t >= g_next_film_s) {
        snprintf(path, sizeof(path), "%s/film_%03u.png", g_dir, (unsigned)(g_next_film_s + 0.5));
        (void)sim_display_save_png(path);
        g_next_film_s += FILM_PERIOD_S;
    }
}

/* ------------------------------------------------------------ checks */

static FILE *g_summary;
static unsigned g_fails;

static void report(bool ok, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
static void report(bool ok, const char *fmt, ...)
{
    char msg[256];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(msg, sizeof(msg), fmt, ap);
    va_end(ap);
    printf("    %s  %s\n", ok ? "pass" : "FAIL", msg);
    if (g_summary) {
        fprintf(g_summary, "%s  %s\n", ok ? "pass" : "FAIL", msg);
    }
    g_fails += ok ? 0u : 1u;
}

static void note(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void note(const char *fmt, ...)
{
    char msg[256];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(msg, sizeof(msg), fmt, ap);
    va_end(ap);
    printf("    info  %s\n", msg);
    if (g_summary) {
        fprintf(g_summary, "info  %s\n", msg);
    }
}

static double first_likely_s(const sim_log_t *log, double after_s)
{
    for (unsigned i = 0; i < log->n_tr; i++) {
        if (log->tr[i].state == ELRS_LIKELY && log->tr[i].t_us >= (int64_t)(after_s * 1e6)) {
            return log->tr[i].t_us / 1e6;
        }
    }
    return -1.0;
}

static bool has_why(const sim_log_t *log, const char *why)
{
    for (unsigned i = 0; i < log->n_whys; i++) {
        if (strcmp(log->whys[i], why) == 0) {
            return true;
        }
    }
    return false;
}

static void check_log_health(const sim_log_t *log)
{
    report(log->boots == 1, "one boot record (%u)", log->boots);
    report(!log->bad_json, "every console line is a JSON object (%u lines)", log->lines);
    report(log->errors == 0 && log->sdr_errors == 0, "no error / sdr_error records (%u / %u)",
           log->errors, log->sdr_errors);
    report(log->crc_bad == 0, "decoder saw no corrupt frames (crc_bad %u)", (unsigned)log->crc_bad);
}

static void check_verdicts(const scenario_t *sc, const sim_log_t *log)
{
    const double first_any = first_likely_s(log, 0.0);
    if (!sc->expect_detect) {
        report(first_any < 0, "never ELRS DETECTED (first at %.1f s)", first_any);
        return;
    }
    report(first_any < 0 || first_any >= sc->detect_after_s,
           "no ELRS DETECTED before %.0f s (first at %.1f s)", sc->detect_after_s, first_any);
    const double first = first_likely_s(log, sc->elrs_on_s);
    report(first >= 0 && first <= sc->elrs_on_s + sc->detect_within_s,
           "ELRS DETECTED within %.0f s of %.0f s (took %.1f s)", sc->detect_within_s, sc->elrs_on_s,
           first >= 0 ? first - sc->elrs_on_s : -1.0);
    if (sc->elrs_off_s < sc->duration_s) {
        const double by = sc->elrs_off_s + sc->release_within_s;
        report(sim_state_at(log, by) != ELRS_LIKELY && first_likely_s(log, by) < 0,
               "released within %.0f s of the link stopping at %.0f s", sc->release_within_s,
               sc->elrs_off_s);
    }
}

static void check_screen(const scenario_t *sc)
{
    for (unsigned i = 0; i < sc->n_shots; i++) {
        const shot_t *s = &sc->shots[i];
        if (!g_taken[i]) {
            report(false, "screenshot '%s' at %.0f s was never taken", s->name, s->t_s);
        } else if (s->expect == SHOT_ANY) {
            note("screen at %.0f s ('%s.png'): %s", s->t_s, s->name, shot_name(g_seen[i]));
        } else {
            report(g_seen[i] == s->expect, "screen at %.0f s shows %s ('%s.png': %s)", s->t_s,
                   shot_name(s->expect), s->name, shot_name(g_seen[i]));
        }
    }
}

static void engine_notes(const scenario_t *sc)
{
    const sim_engine_stats_t *e = sim_engine_stats();
    note("listening %.0f%% of the time over %llu slices", 100.0 * e->capture_us / (sc->duration_s * 1e6),
         (unsigned long long)e->slices);
    note("frames %llu, dropped %llu, mean %.2f ms, longest %.1f ms, %llu longer than 10 ms",
         (unsigned long long)e->frames, (unsigned long long)e->drops,
         e->frames ? e->frame_us_sum / e->frames / 1000.0 : 0.0, e->frame_us_max / 1000.0,
         (unsigned long long)e->frames_over_10ms);
    note("queue: %llu drains timed out, %llu bytes discarded at run start",
         (unsigned long long)e->drain_timeouts, (unsigned long long)e->txq_discarded);
}

/* ------------------------------------------------------------ running */

static int run_one(const entry_t *en, const char *outdir)
{
    static scenario_t sc;
    memset(&sc, 0, sizeof(sc));
    rf_scene_init(&sc.scene);
    en->build(&sc);

    snprintf(g_dir, sizeof(g_dir), "%s/%s", outdir, en->name);
    mkdir(outdir, 0755);
    mkdir(g_dir, 0755);
    char path[640];
    snprintf(path, sizeof(path), "%s/log.ndjson", g_dir);
    g_sc = &sc;
    g_next_film_s = 0.0;
    memset(g_taken, 0, sizeof(g_taken));

    printf("%s: %s\n", en->name, sc.what);
    sim_engine_attach(&sc.scene, sc.inverted);
    if (!sim_platform_start(path, sc.duration_s, sc.keys, sc.n_keys, tick)) {
        fprintf(stderr, "cannot write %s\n", path);
        return 1;
    }
    sim_platform_run(app_main);

    snprintf(path, sizeof(path), "%s/summary.txt", g_dir);
    g_summary = fopen(path, "w");
    if (g_summary) {
        fprintf(g_summary, "%s: %s\n", en->name, sc.what);
    }
    const sim_log_t *log = sim_platform_log();
    check_log_health(log);
    check_verdicts(&sc, log);
    check_screen(&sc);
    for (unsigned i = 0; i < 4 && sc.need_whys[i]; i++) {
        report(has_why(log, sc.need_whys[i]), "log records a '%s' config change", sc.need_whys[i]);
    }
    engine_notes(&sc);
    if (g_summary) {
        fclose(g_summary);
    }
    return g_fails ? 1 : 0;
}

static const entry_t *find(const char *name)
{
    for (size_t i = 0; i < N_SCENARIOS; i++) {
        if (strcmp(k_scenarios[i].name, name) == 0) {
            return &k_scenarios[i];
        }
    }
    return NULL;
}

/* The firmware keeps state in statics, so every scenario gets a fresh process. */
static int run_all(const char *outdir)
{
    unsigned failed = 0;
    for (size_t i = 0; i < N_SCENARIOS; i++) {
        fflush(stdout);
        const pid_t pid = fork();
        if (pid == 0) {
            exit(run_one(&k_scenarios[i], outdir));
        }
        int status = 0;
        if (pid < 0 || waitpid(pid, &status, 0) < 0 || !WIFEXITED(status) || WEXITSTATUS(status) != 0) {
            failed++;
            printf("  -> %s FAILED\n\n", k_scenarios[i].name);
        } else {
            printf("  -> %s ok\n\n", k_scenarios[i].name);
        }
    }
    printf("%u of %zu scenarios passed\n", (unsigned)(N_SCENARIOS - failed), N_SCENARIOS);
    return failed ? 1 : 0;
}

int main(int argc, char **argv)
{
    setvbuf(stdout, NULL, _IOLBF, 0);
    const char *outdir = argc > 2 ? argv[2] : "out";
    if (argc < 2 || strcmp(argv[1], "list") == 0) {
        for (size_t i = 0; i < N_SCENARIOS; i++) {
            printf("%-18s\n", k_scenarios[i].name);
        }
        return argc < 2 ? 2 : 0;
    }
    if (strcmp(argv[1], "all") == 0) {
        return run_all(outdir);
    }
    const entry_t *en = find(argv[1]);
    if (en == NULL) {
        fprintf(stderr, "unknown scenario '%s' (try: list)\n", argv[1]);
        return 2;
    }
    return run_one(en, outdir);
}
