/* sim_engine.h - harness side of the simulated capture engine. */
#ifndef SIM_ENGINE_H
#define SIM_ENGINE_H

#include <stdbool.h>
#include <stdint.h>
#include "rf_scene.h"

typedef struct {
    uint64_t slices;
    double   capture_us;       /* time the receiver was actually listening */
    uint64_t frames, drops;
    double   frame_us_sum, frame_us_max;
    uint64_t frames_over_10ms; /* frames that smear several ELRS hops together */
    uint64_t drain_timeouts;   /* runs whose output never fully left the queue */
    uint64_t txq_discarded;    /* bytes thrown away when the next run reset the queue */
} sim_engine_stats_t;

/* The scene the "antenna" sees; `inverted` mirrors the spectrum, standing in
 * for the unknown orientation of the real hardware (plan M0). */
void sim_engine_attach(const rf_scene_t *scene, bool inverted);
const sim_engine_stats_t *sim_engine_stats(void);

#endif /* SIM_ENGINE_H */
