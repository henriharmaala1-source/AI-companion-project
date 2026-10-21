"""Analog 5.8 GHz FPV channel table, in the order C5VRX-5 steps through it.

C5VRX keeps its bands in the order R, A, B, E, F, L (main/rf.h) and its
`b` key moves to the same slot in the next band, `c` to the next slot in
the same band. Navigation in c5vrx.py depends on this order, so keep it
identical to the firmware's table.
"""

from dataclasses import dataclass

BAND_LETTERS = "RABEFL"

_FREQS = (
    (5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917),  # R  Raceband
    (5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725),  # A  Boscam A
    (5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866),  # B  Boscam B
    (5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945),  # E  Boscam E
    (5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880),  # F  Fatshark
    (5362, 5399, 5436, 5473, 5510, 5547, 5584, 5621),  # L  Lowband
)

# ESP32-C5 documented 5 GHz range; C5VRX tunes up to 5945 MHz but marks
# channels above 5885 MHz as not characterised.
C5_SPEC_MIN_MHZ = 5180
C5_SPEC_MAX_MHZ = 5885


@dataclass(frozen=True)
class Channel:
    band: int  # index into BAND_LETTERS
    slot: int  # 0..7
    freq: int  # MHz

    @property
    def name(self) -> str:
        return f"{BAND_LETTERS[self.band]}{self.slot + 1}"

    @property
    def index(self) -> int:
        return self.band * 8 + self.slot

    @property
    def in_c5_spec(self) -> bool:
        return C5_SPEC_MIN_MHZ <= self.freq <= C5_SPEC_MAX_MHZ


ALL = tuple(Channel(b, s, f) for b, row in enumerate(_FREQS) for s, f in enumerate(row))
_BY_NAME = {c.name: c for c in ALL}


def by_name(name: str) -> Channel:
    try:
        return _BY_NAME[name.strip().upper()]
    except KeyError:
        raise ValueError(f"unknown channel {name!r}, expected e.g. R1, A8, L3") from None
