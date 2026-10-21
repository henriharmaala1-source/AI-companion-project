"""Client for the C5VRX-5 USB serial console (115200, one byte per command).

C5VRX has no "go to channel X" command, only `b` (next band, same slot)
and `c` (next channel in the band). Each retune prints
`[CHANNEL] <name> <freq> MHz`, and the firmware prints a STATUS line every
2 s, e.g.

    STATUS channel=R1 freq=5658 afc_locked=1 standard=AUTO detected=PAL
           signal=73 menu=0 mode=DEFAULT ... busy=0 fw=...

Every retune also saves the channel to NVS, so avoid stepping through
channels in a tight loop.
"""

import time
from dataclasses import dataclass
from typing import Dict, Optional

from . import channels


class C5Error(Exception):
    pass


def parse_fields(line: str) -> Dict[str, str]:
    return dict(tok.split("=", 1) for tok in line.split()[1:] if "=" in tok)


@dataclass(frozen=True)
class Status:
    channel: channels.Channel
    afc_locked: bool
    detected: Optional[str]  # "PAL", "NTSC" or None
    signal: int              # 0..100
    menu_open: bool
    busy: bool

    @property
    def has_video(self) -> bool:
        return self.detected is not None

    @classmethod
    def parse(cls, line: str) -> "Status":
        f = parse_fields(line)
        detected = f.get("detected", "none")
        return cls(
            channel=channels.by_name(f["channel"]),
            afc_locked=f.get("afc_locked") == "1",
            detected=None if detected == "none" else detected,
            signal=int(f.get("signal", "0")),
            menu_open=f.get("menu") == "1",
            busy=f.get("busy") == "1",
        )


class C5vrx:
    """port: pyserial-like object with write(bytes) and readline() honouring a timeout."""

    STATUS_PERIOD_S = 2.0

    def __init__(self, port, log=print, settle_s: float = 2.5):
        self.port = port
        self.log = log
        self.settle_s = settle_s

    def _lines(self, timeout: float):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            raw = self.port.readline()
            if raw:
                yield raw.decode("ascii", "replace").strip()

    def status(self, timeout: float = 3 * STATUS_PERIOD_S) -> Status:
        for line in self._lines(timeout):
            if line.startswith("STATUS "):
                return Status.parse(line)
        raise C5Error("no STATUS line from C5VRX (wrong port, or firmware older than C5VRX-5?)")

    def _press(self, key: str, timeout: float = 3.0) -> Optional[channels.Channel]:
        """Send one retune key and return the channel C5VRX reports."""
        self.port.write(key.encode("ascii"))
        for line in self._lines(timeout):
            if line.startswith("[CHANNEL] "):
                return channels.by_name(line.split()[1])
            if line.startswith(f"COMMAND {key} refused=busy"):
                return None
        raise C5Error(f"C5VRX did not answer key {key!r}")

    def _wait_idle(self) -> Status:
        deadline = time.monotonic() + 600  # first-boot calibration can take up to ~8 min
        while time.monotonic() < deadline:
            st = self.status()
            if not st.busy:
                return st
            self.log("C5VRX busy (calibrating), waiting")
        raise C5Error("C5VRX stayed busy")

    def goto(self, target: channels.Channel, max_presses: int = 24) -> channels.Channel:
        st = self._wait_idle()
        if st.menu_open:
            self.port.write(b"e")  # close the on-screen menu; b/c would act on it otherwise
        current = st.channel
        presses = 0
        while current != target:
            if presses >= max_presses:
                raise C5Error(f"could not reach {target.name}, stuck on {current.name}")
            # rf_next_band keeps the slot, so fix the band first, then the slot.
            key = "b" if current.band != target.band else "c"
            reported = self._press(key)
            presses += 1
            if reported is None:
                current = self._wait_idle().channel
            else:
                current = reported
        return current

    def measure(self) -> Status:
        """Let the receiver lock on the current channel, then return a fresh STATUS."""
        deadline = time.monotonic() + self.settle_s
        st = self.status()
        while time.monotonic() < deadline:
            st = self.status()
        return st
