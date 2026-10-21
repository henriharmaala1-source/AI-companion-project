"""Minimal MSP v1 client for Betaflight VTX control.

Message layouts follow Betaflight src/main/msp/msp.c (MSP_VTX_CONFIG,
MSP_SET_VTX_CONFIG with the API 1.42 band/channel/frequency extension,
MSP_VTXTABLE_BAND, MSP_STATUS).

Betaflight applies VTX band/channel/frequency changes only while disarmed
(io/vtx.c, vtxProcessBandAndChannel / vtxProcessFrequency); power changes
also apply armed. Callers must pair channels before arming.
"""

import struct
import time
from dataclasses import dataclass
from typing import Dict, Tuple

MSP_VTX_CONFIG = 88
MSP_SET_VTX_CONFIG = 89
MSP_STATUS = 101
MSP_VTXTABLE_BAND = 137
MSP_EEPROM_WRITE = 250

VTXDEV_UNSUPPORTED = 0
VTXDEV_UNKNOWN = 0xFF


class MspError(Exception):
    pass


def encode(cmd: int, payload: bytes = b"") -> bytes:
    if len(payload) > 255:
        raise ValueError("MSP v1 payload is limited to 255 bytes")
    checksum = len(payload) ^ cmd
    for b in payload:
        checksum ^= b
    return b"$M<" + bytes((len(payload), cmd)) + payload + bytes((checksum,))


@dataclass(frozen=True)
class VtxConfig:
    device_type: int
    band: int
    channel: int
    power: int
    pit_mode: bool
    freq: int
    ready: bool
    low_power_disarm: int
    pit_mode_freq: int
    table_available: bool
    table_bands: int
    table_channels: int

    @classmethod
    def parse(cls, p: bytes) -> "VtxConfig":
        if len(p) < 15:
            raise MspError(f"MSP_VTX_CONFIG reply too short ({len(p)} bytes), need Betaflight 4.1+")
        dev, band, channel, power, pit, freq, ready, lpd, pit_freq, table, bands, chans = struct.unpack_from(
            "<BBBBBHBBHBBB", p
        )
        return cls(dev, band, channel, power, bool(pit), freq, bool(ready), lpd, pit_freq, bool(table), bands, chans)


class Msp:
    """MSP v1 over a pyserial-like port (write(bytes), read(n) with timeout)."""

    def __init__(self, port, timeout: float = 1.0):
        self.port = port
        self.timeout = timeout

    def request(self, cmd: int, payload: bytes = b"") -> bytes:
        self.port.write(encode(cmd, payload))
        return self._read_reply(cmd)

    def _read_exact(self, n: int, deadline: float) -> bytes:
        buf = b""
        while len(buf) < n:
            if time.monotonic() > deadline:
                raise MspError("timeout waiting for FC reply")
            buf += self.port.read(n - len(buf))
        return buf

    def _read_reply(self, cmd: int) -> bytes:
        deadline = time.monotonic() + self.timeout
        while True:
            # Resynchronise on '$M' so stray bytes (e.g. a displayport stream)
            # cannot shift the frame.
            if self._read_exact(1, deadline) != b"$":
                continue
            if self._read_exact(1, deadline) != b"M":
                continue
            direction = self._read_exact(1, deadline)
            size, rcmd = self._read_exact(2, deadline)
            payload = self._read_exact(size, deadline)
            (checksum,) = self._read_exact(1, deadline)
            expected = size ^ rcmd
            for b in payload:
                expected ^= b
            if checksum != expected:
                raise MspError(f"bad checksum on reply to {rcmd}")
            if rcmd != cmd:
                continue  # reply to someone else's request
            if direction == b"!":
                raise MspError(f"FC rejected MSP command {cmd}")
            if direction != b">":
                continue
            return payload

    def is_armed(self) -> bool:
        p = self.request(MSP_STATUS)
        if len(p) < 10:
            raise MspError("MSP_STATUS reply too short")
        (flags,) = struct.unpack_from("<I", p, 6)
        return bool(flags & 1)  # BOXARM is always the first active box

    def vtx_config(self) -> VtxConfig:
        return VtxConfig.parse(self.request(MSP_VTX_CONFIG))

    def vtx_table(self, bands: int) -> Dict[Tuple[int, int], int]:
        """Return {(band, channel): freq}, 1-based like the Betaflight CLI."""
        table = {}
        for band in range(1, bands + 1):
            p = self.request(MSP_VTXTABLE_BAND, bytes((band,)))
            name_len = p[1]
            count = p[2 + name_len + 2]
            offset = 2 + name_len + 3
            for ch in range(count):
                (freq,) = struct.unpack_from("<H", p, offset + 2 * ch)
                table[(band, ch + 1)] = freq
        return table

    def set_vtx_channel(self, current: VtxConfig, freq: int, band: int = 0, channel: int = 0) -> None:
        """Change only the VTX channel; power, pit mode and the rest stay as read.

        With band > 0 Betaflight looks the frequency up in its vtxtable;
        band 0 sets a raw frequency (needs a VTX that supports it).
        """
        payload = struct.pack(
            "<HBBBHBBH",
            freq,  # > 63, so Betaflight reads it as MHz; overridden below
            current.power,
            int(current.pit_mode),
            current.low_power_disarm,
            current.pit_mode_freq,
            band,
            channel,
            freq,
        )
        self.request(MSP_SET_VTX_CONFIG, payload)

    def save(self) -> None:
        self.request(MSP_EEPROM_WRITE)
