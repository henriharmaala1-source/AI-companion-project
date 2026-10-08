# Back-of-envelope numbers for the ELRS-on-Cardputer plan. Inputs marked ASSUME are guesses.
import math
f = 2.44e9
def fspl(d): return 20*math.log10(d) + 20*math.log10(f) - 147.55
eirp = 20 + 2            # 100 mW + ~2 dBi (EU 2.4 GHz EIRP cap is 100 mW)
rx_gain = -3             # ASSUME: small PCB antenna in a plastic case
nf = 10                  # ASSUME: S3 receive noise figure
bin_hz = 80e6/256        # 80 MS/s, 256-bin FFT
floor = -174 + 10*math.log10(bin_hz) + nf
sig_bins = 0.8e6/bin_hz  # LoRa 800 kHz spreads over ~2.6 bins
print(f"noise floor per {bin_hz/1e3:.0f} kHz bin: {floor:.1f} dBm (NF {nf} dB)")
for d in (50, 200, 500, 1000, 3000):
    p = eirp - fspl(d) + rx_gain
    per_bin = p - 10*math.log10(sig_bins)
    for wall in (0, 15):
        print(f"d={d:5d} m  wall={wall:2d} dB  rx={p-wall:6.1f} dBm  SNR/bin={per_bin-wall-floor:5.1f} dB")
print()
# Dwell per channel = hop interval (packets) x packet interval, from ExpressLRS common.cpp 2.4 GHz rows
rows = {"FLRC 1000Hz":(2,1000),"FLRC 500Hz":(2,2000),"LoRa 500Hz":(4,2000),"LoRa 333Hz":(4,3003),
        "LoRa 250Hz":(4,4000),"LoRa 150Hz":(4,6666),"LoRa 100Hz":(4,10000),"LoRa 50Hz":(2,20000)}
for k,(hop,us) in rows.items():
    dwell = hop*us/1000
    print(f"{k:12s} dwell {dwell:5.1f} ms  80-hop block {80*dwell/1000:5.2f} s  full 240-hop sequence {240*dwell/1000:5.2f} s")
print()
# FFT sampling: 80 MS/s, 256 bins, esp-sdr S3 dual profile stride 10 -> one analysed FFT every 32 us
fft_us = 256/80e6*1e6; gap = fft_us*10
print(f"FFT window {fft_us:.1f} us, one analysed every ~{gap:.0f} us -> a 0.2 ms FLRC packet gets ~{200/gap:.0f} looks")
