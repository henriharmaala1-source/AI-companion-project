"""python -m c5relay --c5 /dev/ttyACM0 --fc /dev/ttyAMA0 [--rx R1] [--watch]"""

import argparse
import sys

from . import channels
from .c5vrx import C5vrx
from .controller import PairError, pair, watch
from .msp import Msp, MspError
from .plan import DEFAULT_MIN_SEPARATION_MHZ, EU_SRD_RANGE, PlanError


def channel(text: str):
    try:
        return channels.by_name(text)
    except ValueError as e:
        raise argparse.ArgumentTypeError(str(e)) from None


def freq_range(text: str):
    if text.lower() == "any":
        return None
    lo, hi = (int(v) for v in text.split("-"))
    return lo, hi


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="c5relay", description="Pair a C5VRX receiver with the relay VTX")
    ap.add_argument("--c5", required=True, help="C5VRX USB console, e.g. /dev/ttyACM0")
    ap.add_argument("--fc", required=True, help="FC MSP port, e.g. /dev/ttyAMA0")
    ap.add_argument("--fc-baud", type=int, default=115200)
    ap.add_argument("--rx", type=channel, help="source channel (e.g. R1); default: survey")
    ap.add_argument("--min-sep", type=int, default=DEFAULT_MIN_SEPARATION_MHZ,
                    help="minimum RX/VTX separation in MHz (default %(default)s)")
    ap.add_argument("--vtx-range", type=freq_range, default=EU_SRD_RANGE,
                    help="allowed VTX frequencies, LO-HI MHz or 'any' (default 5725-5875: EU licence-free; "
                         "wider needs an amateur licence)")
    ap.add_argument("--save", action="store_true", help="write the VTX channel to the FC's EEPROM")
    ap.add_argument("--watch", action="store_true", help="keep running and guard the pairing")
    args = ap.parse_args(argv)

    import serial  # pyserial; imported here so the library and tests work without it

    with serial.Serial(args.c5, 115200, timeout=0.5) as c5_port, \
            serial.Serial(args.fc, args.fc_baud, timeout=0.2) as fc_port:
        c5, fc = C5vrx(c5_port), Msp(fc_port)
        try:
            result = pair(c5, fc, args.rx, args.min_sep, args.vtx_range, args.save)
            if args.watch:
                watch(c5, fc, result, args.min_sep, args.vtx_range)
        except (PairError, PlanError, MspError) as e:
            print(f"error: {e}", file=sys.stderr)
            return 1
        except KeyboardInterrupt:
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
