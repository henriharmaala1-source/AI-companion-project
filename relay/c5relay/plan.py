"""Choose the relay VTX frequency for a given receive channel.

The relay's own VTX sits centimetres from the C5 antenna, so the only
in-band defence that costs nothing is frequency separation: pick the VTX
frequency furthest from the receive channel. C5VRX captures +-20 MHz
around its channel and analog FM video is ~20-30 MHz wide, so 37 MHz
(adjacent Raceband) is not enough; 60 MHz is the default floor.
"""

from dataclasses import dataclass
from typing import Iterable, Optional, Tuple

DEFAULT_MIN_SEPARATION_MHZ = 60

# EU short-range-device band (ERC Rec 70-03): licence-free analog video in
# Finland is 5725-5875 MHz at 25 mW e.i.r.p. Wider needs an amateur licence.
EU_SRD_RANGE = (5725, 5875)


class PlanError(Exception):
    pass


@dataclass(frozen=True)
class VtxPlan:
    freq: int
    separation: int
    band: int = 0     # Betaflight vtxtable band (1-based), 0 = raw frequency
    channel: int = 0  # Betaflight vtxtable channel (1-based)


def plan_vtx(
    rx_freq: int,
    candidates: Iterable[Tuple[int, int, int]],
    min_separation: int = DEFAULT_MIN_SEPARATION_MHZ,
    allowed: Optional[Tuple[int, int]] = EU_SRD_RANGE,
) -> VtxPlan:
    """Pick the candidate (freq, band, channel) furthest from rx_freq.

    Candidates normally come from the FC's vtxtable, so the result is a
    channel the VTX can actually tune. Ties go to the lower table entry so
    the result does not depend on table order.
    """
    best = None
    for freq, band, channel in candidates:
        if freq <= 0:
            continue  # empty vtxtable slot
        if allowed and not allowed[0] <= freq <= allowed[1]:
            continue
        sep = abs(freq - rx_freq)
        key = (sep, -band, -channel)
        if best is None or key > best[0]:
            best = (key, VtxPlan(freq, sep, band, channel))
    if best is None:
        raise PlanError(f"no VTX frequency inside {allowed}")
    plan = best[1]
    if plan.separation < min_separation:
        raise PlanError(
            f"best VTX frequency {plan.freq} MHz is only {plan.separation} MHz from the "
            f"receive channel ({rx_freq} MHz), need {min_separation}: move the source "
            f"drone to the other end of the band"
        )
    return plan


def rx_compatible(rx_freq: int, vtx_freq: int, min_separation: int = DEFAULT_MIN_SEPARATION_MHZ) -> bool:
    return abs(rx_freq - vtx_freq) >= min_separation
