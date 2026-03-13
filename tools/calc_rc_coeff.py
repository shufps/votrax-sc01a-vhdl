#!/usr/bin/env python3
"""
calc_rc_coeff.py  –  IIR Koeffizienten für RC-Tiefpassfilter

Berechnet k für:  y[n] = k·x[n] + (1-k)·y[n-1]

Methode: Impulse Invariance
  Analoger Pol bei s = -1/(R·C)  →  z = exp(-1/(R·C·fs))
  k = 1 - exp(-1/(R·C·fs))

Usage:
  python3 calc_rc_coeff.py                         # Bally Astrocade defaults
  python3 calc_rc_coeff.py 110e3 560e-12 48000     # R C fs
"""

import math
import sys


def calc(R: float, C: float, fs: float, frac_bits: int = 15) -> dict:
    tau = R * C
    fc  = 1.0 / (2.0 * math.pi * tau)
    k   = 1.0 - math.exp(-1.0 / (tau * fs))
    scale  = 1 << frac_bits
    k_q    = round(k * scale)
    km_q   = scale - k_q   # exact complement, ensures k_q + km_q = scale

    # -3 dB of 4 cascaded stages: fc_4 = fc * sqrt(2^(1/4) - 1) ≈ fc * 0.4349
    fc4 = fc * math.sqrt(2 ** (1/4) - 1)

    return dict(R=R, C=C, fs=fs, tau=tau, fc=fc, fc4=fc4, k=k,
                frac_bits=frac_bits, scale=scale, k_q=k_q, km_q=km_q)


def print_result(d: dict) -> None:
    print(f"\nRC Tiefpassfilter  (1st-order IIR, Impulse Invariance)")
    print(f"  R        = {d['R']/1e3:.1f} kΩ")
    print(f"  C        = {d['C']*1e12:.0f} pF")
    print(f"  fs       = {d['fs']:.0f} Hz")
    print(f"  τ = R·C  = {d['tau']*1e6:.3f} µs")
    print(f"  fc (1 Stufe)   = {d['fc']:.1f} Hz")
    print(f"  fc (-3dB, 4×)  = {d['fc4']:.1f} Hz")
    print(f"  k              = {d['k']:.8f}")
    print(f"  1-k            = {1-d['k']:.8f}")
    print()
    print(f"  Q{d['frac_bits']} Koeffizienten (Summe = {d['scale']}):")
    print(f"    RC_K   = {d['k_q']:6d}   (k   × 2^{d['frac_bits']})")
    print(f"    RC_KM  = {d['km_q']:6d}   ((1-k) × 2^{d['frac_bits']})")
    print(f"    check: {d['k_q']} + {d['km_q']} = {d['k_q'] + d['km_q']}  "
          f"({'OK' if d['k_q'] + d['km_q'] == d['scale'] else 'FEHLER'})")
    print()
    print(f"  VHDL constants:")
    print(f"    constant RC_K  : integer := {d['k_q']};")
    print(f"    constant RC_KM : integer := {d['scale']} - RC_K;  -- = {d['km_q']}")


if __name__ == "__main__":
    if len(sys.argv) == 4:
        R  = float(sys.argv[1])
        C  = float(sys.argv[2])
        fs = float(sys.argv[3])
    elif len(sys.argv) == 1:
        # Bally Astrocade: 4× 110kΩ / 560pF, Ausgang nach sc01a_resamp (48kHz)
        R, C, fs = 110e3, 560e-12, 48000.0
    else:
        print(__doc__)
        sys.exit(1)

    print_result(calc(R, C, fs))
