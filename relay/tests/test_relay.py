import struct
import unittest

from c5relay import channels, msp
from c5relay.c5vrx import C5vrx, Status
from c5relay.controller import PairError, pair, watch
from c5relay.plan import EU_SRD_RANGE, PlanError, plan_vtx

from .fakes import FakeC5, FakeFC

quiet = lambda *a: None


class ChannelTests(unittest.TestCase):
    def test_order_matches_c5vrx_table(self):
        self.assertEqual(channels.by_name("R1").index, 0)
        self.assertEqual(channels.by_name("A1").index, 8)
        self.assertEqual(channels.by_name("l8").freq, 5621)
        self.assertEqual(len(channels.ALL), 48)

    def test_spec_range(self):
        self.assertTrue(channels.by_name("E5").in_c5_spec)    # 5885
        self.assertFalse(channels.by_name("E8").in_c5_spec)   # 5945

    def test_unknown_name(self):
        with self.assertRaises(ValueError):
            channels.by_name("Z9")


class PlanTests(unittest.TestCase):
    CANDIDATES = [(5658, 5, 1), (5725, 1, 8), (5865, 1, 1), (5866, 2, 8), (5917, 5, 8), (0, 3, 1)]

    def test_furthest_inside_allowed_range(self):
        p = plan_vtx(5658, self.CANDIDATES, allowed=EU_SRD_RANGE)
        self.assertEqual((p.freq, p.band, p.channel, p.separation), (5866, 2, 8, 208))

    def test_any_range_takes_the_far_end(self):
        self.assertEqual(plan_vtx(5658, self.CANDIDATES, allowed=None).freq, 5917)

    def test_too_close_is_refused(self):
        with self.assertRaises(PlanError):
            plan_vtx(5800, [(5845, 1, 2)], min_separation=60)

    def test_tie_prefers_lower_table_entry(self):
        p = plan_vtx(5800, [(5860, 4, 7), (5740, 4, 1)], allowed=None)
        self.assertEqual((p.band, p.channel), (4, 1))


class MspTests(unittest.TestCase):
    def test_encode(self):
        self.assertEqual(msp.encode(msp.MSP_STATUS), b"$M<\x00\x65\x65")

    def test_reply_after_junk_and_foreign_frames(self):
        fc = FakeFC()
        client = msp.Msp(fc)
        fc.out = b"\x00garbage$" + msp.encode(7, b"\x01").replace(b"<", b">", 1)
        self.assertFalse(client.is_armed())

    def test_error_frame(self):
        fc = FakeFC(armed=True)
        with self.assertRaises(msp.MspError):
            msp.Msp(fc).save()

    def test_set_channel_keeps_power_and_pit(self):
        fc = FakeFC()
        fc.power, fc.pit, fc.pit_freq = 3, 1, 5584
        client = msp.Msp(fc)
        cfg = client.vtx_config()
        client.set_vtx_channel(cfg, 5866, band=2, channel=8)
        self.assertEqual((fc.band, fc.channel, fc.freq), (2, 8, 5866))
        self.assertEqual((fc.power, fc.pit, fc.pit_freq), (3, 1, 5584))
        self.assertEqual(len(fc.sets[0]), 11)

    def test_raw_frequency(self):
        fc = FakeFC()
        client = msp.Msp(fc)
        client.set_vtx_channel(client.vtx_config(), 5800)
        self.assertEqual((fc.band, fc.freq), (0, 5800))

    def test_vtx_table(self):
        client = msp.Msp(FakeFC())
        table = client.vtx_table(5)
        self.assertEqual(table[(2, 8)], 5866)
        self.assertEqual(table[(5, 1)], 5658)
        self.assertEqual(len(table), 40)


class C5Tests(unittest.TestCase):
    def test_status_line(self):
        st = Status.parse("STATUS channel=R1 freq=5658 afc_locked=1 standard=AUTO detected=PAL signal=73 menu=0 "
                          "mode=DEFAULT kp_auto=1 output=1.0x live=1 busy=0 fw=5.0.1+abc")
        self.assertEqual(st.channel.name, "R1")
        self.assertTrue(st.afc_locked and st.has_video)
        self.assertEqual(st.signal, 73)

    def test_goto_band_then_slot(self):
        c5 = FakeC5(start="A1")
        reached = C5vrx(c5, log=quiet, settle_s=0).goto(channels.by_name("L3"))
        self.assertEqual(reached.name, "L3")
        self.assertEqual(c5.presses, ["b"] * 4 + ["c"] * 2)  # A->B->E->F->L, then 1->3

    def test_goto_waits_out_calibration(self):
        c5 = FakeC5(start="R1", busy_lines=2)
        C5vrx(c5, log=quiet, settle_s=0).goto(channels.by_name("R2"))
        self.assertEqual(c5.ch.name, "R2")


class PairTests(unittest.TestCase):
    def test_explicit_rx(self):
        fc = FakeFC(vtx_freq=5658)
        c5 = FakeC5(start="A1", sources={5658: 80}, fc=fc)
        r = pair(C5vrx(c5, log=quiet, settle_s=0), msp.Msp(fc), channels.by_name("R1"), log=quiet)
        self.assertEqual((c5.ch.name, fc.freq, fc.band, fc.channel), ("R1", 5866, 2, 8))
        self.assertEqual(r.vtx.separation, 208)

    def test_refuses_when_armed(self):
        fc = FakeFC(armed=True)
        with self.assertRaises(PairError):
            pair(C5vrx(FakeC5(fc=fc), log=quiet, settle_s=0), msp.Msp(fc), channels.by_name("R1"), log=quiet)
        self.assertEqual(fc.sets, [])

    def test_survey_skips_own_vtx(self):
        # The relay's own VTX on A1 (5865) is the strongest thing the C5 can
        # hear; the survey must find the weaker source drone on R1 instead.
        fc = FakeFC(vtx_freq=5865)
        c5 = FakeC5(start="A1", sources={5658: 45}, fc=fc)
        r = pair(C5vrx(c5, log=quiet, settle_s=0), msp.Msp(fc), log=quiet)
        self.assertEqual(r.rx.name, "R1")
        self.assertEqual(fc.freq, 5866)

    def test_no_controllable_vtx(self):
        fc = FakeFC(device_type=msp.VTXDEV_UNKNOWN)
        with self.assertRaises(PairError):
            pair(C5vrx(FakeC5(fc=fc), log=quiet, settle_s=0), msp.Msp(fc), channels.by_name("R1"), log=quiet)


class WatchTests(unittest.TestCase):
    def run_watch(self, fc, c5, result, rounds):
        n = iter(range(rounds))
        watch(C5vrx(c5, log=quiet, settle_s=0), msp.Msp(fc), result, log=quiet,
              stop=lambda: next(n, None) is None)

    def paired(self):
        fc = FakeFC(vtx_freq=5658)
        c5 = FakeC5(start="R1", sources={5658: 80}, fc=fc)
        r = pair(C5vrx(c5, log=quiet, settle_s=0), msp.Msp(fc), channels.by_name("R1"), log=quiet)
        return fc, c5, r

    def test_armed_moves_c5_back(self):
        fc, c5, r = self.paired()
        fc.armed = True
        c5.ch = channels.by_name("B8")  # pilot pressed BOOT: right on top of the VTX
        self.run_watch(fc, c5, r, 2)
        self.assertEqual(c5.ch.name, "R1")
        self.assertEqual(fc.freq, 5866)

    def test_disarmed_repairs_vtx(self):
        fc, c5, r = self.paired()
        c5.ch = channels.by_name("A1")  # 5865, next to the VTX on 5866
        self.run_watch(fc, c5, r, 2)
        self.assertEqual(c5.ch.name, "A1")
        self.assertEqual(fc.freq, 5725)  # furthest EU channel from 5865

    def test_compatible_move_is_kept(self):
        fc, c5, r = self.paired()
        fc.armed = True
        c5.ch = channels.by_name("R2")  # 5695, still far from 5866
        self.run_watch(fc, c5, r, 2)
        self.assertEqual(c5.ch.name, "R2")


if __name__ == "__main__":
    unittest.main()
