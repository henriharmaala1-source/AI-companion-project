/*
 * beep.h - audible alerts on the Cardputer's speaker.
 *
 * The speaker is an I2S amplifier, not a piezo buzzer, so a sound is a stream
 * of samples. beep_play() BLOCKS for the length of the pattern (0.04-0.3 s),
 * so call it only between capture slices: the radio is not listening while it
 * plays, which is rare (a verdict change) and short. It also keeps the I2S
 * DMA from ever running during a slice, when interrupts are masked and the
 * DMA would replay stale buffers.
 *
 * Output only: this module never touches the radio.
 */
#ifndef BEEP_H
#define BEEP_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    BEEP_BOOT,       /* one short tick: proves the speaker works */
    BEEP_DETECTED,   /* two rising tones: ELRS verdict began */
    BEEP_LOST,       /* two falling tones: ELRS verdict ended */
    BEEP_TEST,       /* same as detected, from the test key */
} beep_pattern_t;

typedef enum { SOUND_NORMAL, SOUND_QUIET, SOUND_OFF } sound_level_t;

void           beep_init(void);
sound_level_t  beep_level(void);
const char    *beep_level_name(sound_level_t level);
/* normal -> quiet -> off -> normal; plays a tick at the new level. */
sound_level_t  beep_next_level(void);
/* Play a pattern at the current level; silent when off. Logs a "beep" event
 * either way. Returns after the sound has ended. */
void           beep_play(beep_pattern_t pattern);

/* ---- backend, one implementation per target --------------------------- */
/* Open the output; false if it cannot be (no sound, no harm). */
int  beep_hw_open(void);
/* Play a tone (hz == 0: silence) of `ms`; blocks until it has been handed to
 * the hardware. amp is the peak sample value, 0..32767. */
void beep_hw_tone(unsigned hz, unsigned ms, int amp);
/* Let the last samples finish, then release the output and its memory. */
void beep_hw_close(void);

#ifdef __cplusplus
}
#endif

#endif /* BEEP_H */
