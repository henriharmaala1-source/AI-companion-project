"""Pair the C5VRX receive channel with the relay VTX channel.

Flow (disarmed, on the ground):
  1. read the FC's VTX config and vtxtable over MSP
  2. pick the receive channel (given, or the best video found by a survey
     that skips channels near the VTX's current frequency, so the C5 does
     not lock onto its own VTX)
  3. tune the C5 there and plan the VTX frequency furthest away
  4. set the VTX over MSP and wait until the FC reports it

In flight Betaflight will not retune the VTX, so watch() keeps the C5 on
channels compatible with the fixed VTX frequency instead.
"""

import time
from dataclasses import dataclass
from typing import Iterable, List, Optional, Tuple

from . import channels
from .c5vrx import C5vrx, Status
from .msp import VTXDEV_UNKNOWN, VTXDEV_UNSUPPORTED, Msp
from .plan import DEFAULT_MIN_SEPARATION_MHZ, EU_SRD_RANGE, VtxPlan, plan_vtx, rx_compatible

# Standard 40-channel set offered when the FC has no vtxtable.
_FALLBACK = tuple((c.freq, 0, 0) for c in channels.ALL if c.band != channels.BAND_LETTERS.index("L"))


class PairError(Exception):
    pass


@dataclass(frozen=True)
class PairResult:
    rx: channels.Channel
    rx_status: Status
    vtx: VtxPlan


def vtx_candidates(fc: Msp, cfg) -> Iterable[Tuple[int, int, int]]:
    if cfg.table_available and cfg.table_bands:
        return [(f, b, c) for (b, c), f in sorted(fc.vtx_table(cfg.table_bands).items())]
    return _FALLBACK


def survey(c5: C5vrx, avoid_freq: int, min_separation: int, log=print) -> List[Status]:
    """Visit every channel far enough from avoid_freq and record what C5VRX sees."""
    results = []
    for ch in channels.ALL:  # firmware order keeps the number of key presses low
        if not rx_compatible(ch.freq, avoid_freq, min_separation):
            continue
        c5.goto(ch)
        st = c5.measure()
        log(f"survey {ch.name} {ch.freq} MHz video={st.detected or '-'} lock={int(st.afc_locked)} signal={st.signal}")
        results.append(st)
    return results


def best_video(results: Iterable[Status]) -> Optional[Status]:
    with_video = [s for s in results if s.has_video]
    if not with_video:
        return None
    return max(with_video, key=lambda s: (s.afc_locked, s.signal))


def pair(
    c5: C5vrx,
    fc: Msp,
    rx: Optional[channels.Channel] = None,
    min_separation: int = DEFAULT_MIN_SEPARATION_MHZ,
    allowed: Optional[Tuple[int, int]] = EU_SRD_RANGE,
    save: bool = False,
    log=print,
) -> PairResult:
    if fc.is_armed():
        raise PairError("FC is armed: Betaflight only retunes the VTX while disarmed")
    cfg = fc.vtx_config()
    if cfg.device_type in (VTXDEV_UNKNOWN, VTXDEV_UNSUPPORTED):
        raise PairError("FC reports no controllable VTX (check SmartAudio/Tramp wiring and vtxtable)")
    candidates = list(vtx_candidates(fc, cfg))

    if rx is None:
        found = best_video(survey(c5, cfg.freq, min_separation, log))
        if found is None:
            raise PairError(
                f"no analog video found outside +-{min_separation} MHz of the VTX ({cfg.freq} MHz); "
                f"give the source channel explicitly"
            )
        rx = found.channel

    plan = plan_vtx(rx.freq, candidates, min_separation, allowed)
    c5.goto(rx)
    log(f"RX {rx.name} {rx.freq} MHz -> VTX {plan.freq} MHz ({plan.separation} MHz apart)")

    if cfg.freq != plan.freq:
        fc.set_vtx_channel(cfg, plan.freq, plan.band, plan.channel)
        deadline = time.monotonic() + 5.0
        while fc.vtx_config().freq != plan.freq:
            if time.monotonic() > deadline:
                raise PairError(f"FC did not move the VTX to {plan.freq} MHz")
            time.sleep(0.2)
        if save:
            fc.save()
    status = c5.measure()
    if not status.has_video:
        log(f"warning: no video detected on {rx.name} yet")
    return PairResult(rx, status, plan)


def watch(c5: C5vrx, fc: Msp, result: PairResult, min_separation: int = DEFAULT_MIN_SEPARATION_MHZ,
          allowed: Optional[Tuple[int, int]] = EU_SRD_RANGE, log=print, stop=lambda: False) -> None:
    """Keep the pairing valid. Someone may change the C5 channel with its BOOT
    button; if that lands near the VTX, move the C5 back (armed) or re-pair
    the VTX (disarmed, VTX changes are allowed)."""
    rx = result.rx
    vtx_freq = result.vtx.freq
    last = None
    while not stop():
        st = c5.status()
        if st.channel != rx:
            if rx_compatible(st.channel.freq, vtx_freq, min_separation):
                log(f"C5 moved to {st.channel.name}, still {abs(st.channel.freq - vtx_freq)} MHz from the VTX: keeping it")
                rx = st.channel
            elif fc.is_armed():
                log(f"C5 moved to {st.channel.name}, too close to the VTX ({vtx_freq} MHz) while armed: moving back to {rx.name}")
                c5.goto(rx)
            else:
                log(f"C5 moved to {st.channel.name}: re-pairing the VTX")
                result = pair(c5, fc, st.channel, min_separation, allowed, log=log)
                rx, vtx_freq = result.rx, result.vtx.freq
        summary = (st.channel.name, st.detected, st.afc_locked)
        if summary != last:
            log(f"RX {st.channel.name} video={st.detected or '-'} lock={int(st.afc_locked)} signal={st.signal}")
            last = summary
