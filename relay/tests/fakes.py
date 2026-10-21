"""Hardware stand-ins: a C5VRX-5 console and a Betaflight FC speaking MSP.

FakeC5 prints the same STATUS / [CHANNEL] / COMMAND lines as the firmware
(main/control.c). FakeFC applies MSP_SET_VTX_CONFIG the way
src/main/msp/msp.c does, including the API 1.42 extension fields.
"""

import struct

from c5relay import channels, msp

# Betaflight's common 5.8 GHz vtxtable (bands A, B, E, F, R).
BF_TABLE = {
    1: (5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725),
    2: (5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866),
    3: (5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945),
    4: (5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880),
    5: (5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917),
}


class FakeFC:
    def __init__(self, vtx_freq=5865, armed=False, device_type=4, table=True):
        self.armed = armed
        self.device_type = device_type
        self.table = table
        self.band, self.channel, self.freq = 0, 0, vtx_freq
        self.power, self.pit, self.lpd, self.pit_freq = 2, 0, 1, 0
        self.saved = 0
        self.out = b""
        self.sets = []

    # pyserial-like interface
    def write(self, data: bytes):
        assert data[:3] == b"$M<"
        size, cmd = data[3], data[4]
        payload = data[5:5 + size]
        reply = self.handle(cmd, payload)
        if reply is None:
            self.out += b"$M!" + bytes((0, cmd)) + bytes((cmd,))
        else:
            self.out += msp.encode(cmd, reply).replace(b"$M<", b"$M>", 1)

    def read(self, n: int) -> bytes:
        data, self.out = self.out[:n], self.out[n:]
        return data

    def handle(self, cmd, p):
        if cmd == msp.MSP_STATUS:
            return struct.pack("<HHHIBH", 125, 0, 0x21, 1 if self.armed else 0, 0, 10) + b"\x00\x00\x00"
        if cmd == msp.MSP_VTX_CONFIG:
            return struct.pack("<BBBBBHBBHBBBB", self.device_type, self.band, self.channel, self.power, self.pit,
                               self.freq, 1, self.lpd, self.pit_freq, int(self.table),
                               len(BF_TABLE) if self.table else 0, 8 if self.table else 0, 5)
        if cmd == msp.MSP_VTXTABLE_BAND:
            band = p[0]
            if band not in BF_TABLE:
                return None
            name = b"BAND_" + bytes((ord("0") + band,)) + b"  "
            return bytes((band, len(name))) + name + b"X" + b"\x01" + bytes((8,)) + struct.pack("<8H", *BF_TABLE[band])
        if cmd == msp.MSP_SET_VTX_CONFIG:
            self.sets.append(p)
            first = struct.unpack_from("<H", p)[0]
            if first <= 63:
                self.band, self.channel = first // 8 + 1, first % 8 + 1
                self.freq = BF_TABLE[self.band][self.channel - 1]
            else:
                self.band, self.freq = 0, first
            rest = p[2:]
            if len(rest) >= 2:
                self.power, self.pit = rest[0], rest[1]
                rest = rest[2:]
            if rest:
                self.lpd, rest = rest[0], rest[1:]
            if len(rest) >= 2:
                self.pit_freq, rest = struct.unpack_from("<H", rest)[0], rest[2:]
            if len(rest) >= 4:
                band, channel, freq = struct.unpack_from("<BBH", rest)
                if band:
                    freq = BF_TABLE[band][channel - 1]
                self.band, self.channel, self.freq = band, channel, freq
            return b""
        if cmd == msp.MSP_EEPROM_WRITE:
            if self.armed:
                return None
            self.saved += 1
            return b""
        return None


class FakeC5:
    """sources: {freq_mhz: signal}; the FC's VTX (if given) leaks into every
    channel within 20 MHz of it, like a co-located transmitter would."""

    def __init__(self, start="A1", sources=None, fc=None, busy_lines=0):
        self.ch = channels.by_name(start)
        self.sources = dict(sources or {})
        self.fc = fc
        self.busy_lines = busy_lines
        self.queue = []
        self.presses = []

    def _video(self):
        transmitters = dict(self.sources)
        if self.fc is not None:
            transmitters[self.fc.freq] = 100
        best = 0
        for freq, signal in transmitters.items():
            if abs(freq - self.ch.freq) <= 20:
                best = max(best, signal if abs(freq - self.ch.freq) <= 3 else signal // 2)
        return best

    def write(self, data: bytes):
        for key in data.decode():
            if key not in "bc":
                continue
            self.presses.append(key)
            if self.busy_lines:
                self.queue.append(f"COMMAND {key} refused=busy (a calibration owns the picture)")
                continue
            b, s = self.ch.band, self.ch.slot
            if key == "b":
                b = (b + 1) % len(channels.BAND_LETTERS)
            else:
                s = (s + 1) % 8
            self.ch = channels.ALL[b * 8 + s]
            self.queue.append(f"[CHANNEL] {self.ch.name} {self.ch.freq} MHz")

    def readline(self) -> bytes:
        if self.queue:
            return (self.queue.pop(0) + "\n").encode()
        busy = 1 if self.busy_lines else 0
        self.busy_lines = max(0, self.busy_lines - 1)
        signal = self._video()
        detected = "PAL" if signal >= 20 else "none"
        return (f"STATUS channel={self.ch.name} freq={self.ch.freq} afc_locked={int(signal >= 40)} standard=AUTO "
                f"detected={detected} signal={signal} menu=0 mode=DEFAULT kp_auto=1 output=1.0x live=1 "
                f"busy={busy} fw=test\n").encode()
