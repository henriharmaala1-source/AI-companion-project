/*
 * targets.h - everything ELRS watch recognises besides ELRS itself.
 *
 * ELRS keeps the big red banner. Everything else the detectors report at the
 * same time becomes one short line on the screen and an on/off event in the
 * log ("target"). PRESENCE ONLY, 2.4 GHz ONLY: each line names a KIND of
 * signal and where it is, never a device or a person.
 */
#ifndef TARGETS_H
#define TARGETS_H

#include <stdbool.h>
#include <stdint.h>

#include "band_watch.h"
#include "chirp_watch.h"
#include "elrs_detect.h"
#include "ofdm_detect.h"
#include "ui.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Most important first: the list is built in this order. */
typedef enum {
    TGT_LORA_FIXED = 0,  /* LoRa chirps on one channel: not an ELRS link */
    TGT_DJI,             /* LTE-like OFDM: DJI-style drone video link */
    TGT_HOPPER,          /* narrowband hopping off the ELRS grid */
    TGT_WIDE,            /* busy wide signal, no OFDM timing found */
    TGT_MICROWAVE,       /* mains-locked oven pulses */
    TGT_WIFI,            /* Wi-Fi OFDM */
    TGT_KINDS
} target_kind_t;

typedef struct {
    const elrs_status_t  *elrs;    /* grid detector */
    bool                  elrs_shown;  /* the banner says ELRS DETECTED */
    const chirp_status_t *chirp;
    const band_watch_t   *band;
    const band_mw_t      *mw;
    const ofdm_watch_t   *ofdm;
    uint64_t              now_us;
} target_inputs_t;

/* Fill `out` (up to max lines), most important first. Returns the count. */
unsigned targets_build(const target_inputs_t *in, ui_target_t *out, unsigned max);

const char *target_kind_name(target_kind_t k);

#ifdef __cplusplus
}
#endif
#endif /* TARGETS_H */
