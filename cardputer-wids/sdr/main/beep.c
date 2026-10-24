/* beep.c - patterns, volume and the log record. See beep.h. */
#include "beep.h"

#include <stddef.h>

#include "wids_log.h"

/* Peak sample value per level (of 32767). The speaker is small and the
 * amplifier gain unknown to us: start moderate, 'm' changes it. */
#define AMP_NORMAL 8000
#define AMP_QUIET  1800

typedef struct {
    unsigned hz, ms;   /* hz == 0: silence */
} note_t;

static const note_t k_boot[] = { { 1000, 40 } };
static const note_t k_detected[] = { { 880, 90 }, { 0, 40 }, { 1320, 150 } };
static const note_t k_lost[] = { { 660, 90 }, { 0, 40 }, { 440, 150 } };

static sound_level_t s_level = SOUND_NORMAL;

void beep_init(void) { s_level = SOUND_NORMAL; }

sound_level_t beep_level(void) { return s_level; }

const char *beep_level_name(sound_level_t level)
{
    return level == SOUND_NORMAL ? "normal" : level == SOUND_QUIET ? "quiet" : "off";
}

static const char *pattern_name(beep_pattern_t p)
{
    switch (p) {
    case BEEP_BOOT: return "boot";
    case BEEP_DETECTED: return "detected";
    case BEEP_LOST: return "lost";
    default: return "test";
    }
}

static int level_amp(sound_level_t level)
{
    return level == SOUND_NORMAL ? AMP_NORMAL : level == SOUND_QUIET ? AMP_QUIET : 0;
}

void beep_play(beep_pattern_t pattern)
{
    const note_t *notes = k_detected;
    size_t n = sizeof(k_detected) / sizeof(k_detected[0]);
    if (pattern == BEEP_BOOT) {
        notes = k_boot;
        n = sizeof(k_boot) / sizeof(k_boot[0]);
    } else if (pattern == BEEP_LOST) {
        notes = k_lost;
        n = sizeof(k_lost) / sizeof(k_lost[0]);
    }

    const int amp = level_amp(s_level);
    int played = 0;
    if (amp > 0 && beep_hw_open()) {
        for (size_t i = 0; i < n; i++) {
            beep_hw_tone(notes[i].hz, notes[i].ms, amp);
        }
        beep_hw_close();
        played = 1;
    }
    wids_log_event("beep", "\"pattern\":\"%s\",\"level\":\"%s\",\"played\":%s", pattern_name(pattern),
                   beep_level_name(s_level), played ? "true" : "false");
}

sound_level_t beep_next_level(void)
{
    s_level = s_level == SOUND_NORMAL ? SOUND_QUIET : s_level == SOUND_QUIET ? SOUND_OFF : SOUND_NORMAL;
    wids_log_event("sound", "\"level\":\"%s\"", beep_level_name(s_level));
    beep_play(BEEP_BOOT);   /* the tick lets the user hear the new volume */
    return s_level;
}
