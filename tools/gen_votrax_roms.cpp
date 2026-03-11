#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cfloat>
#include <functional>
#include <initializer_list>
#include <complex>
#include <unistd.h>

// sclock/cclock = (main/18)/(main/36) = 2 (fixed, clock-independent)
static constexpr double SCLOCK_NORM = 2.0;

double m_sclock = 0.0;
double m_cclock = 0.0;

static void set_clocks_from_dac(int dac_i) {
  if (dac_i < 0x40)
    dac_i = 0x40;
  double sc_hz = 950000.0 + (dac_i - 0xA0) * 5500.0;
  m_sclock = sc_hz / 18.0;
  m_cclock = sc_hz / 36.0;
}

// ============================================================
// Fixed-point format: s(2.15) = 18-bit signed
// range: -4.0 ... +3.99997
// default scale factor: 2^15 = 32768
// ============================================================
static constexpr int    FP_BITS  = 18;
static constexpr int    FP_FRAC  = 15;
static constexpr double FP_SCALE = (1 << FP_FRAC); // 32768.0
static constexpr int    FP_MAX   =  (1 << (FP_BITS-1)) - 1; //  131071
static constexpr int    FP_MIN   = -(1 << (FP_BITS-1));     // -131072

int to_fixed(double v)
{
    int r = (int)round(v * FP_SCALE);
    if (r > FP_MAX) { fprintf(stderr, "OVERFLOW: %f -> %d\n", v, r); r = FP_MAX; }
    if (r < FP_MIN) { fprintf(stderr, "UNDERFLOW: %f -> %d\n", v, r); r = FP_MIN; }
    return r;
}

int to_fixed_s(double v, double scale)
{
    int r = (int)round(v * scale);
    if (r > FP_MAX) { fprintf(stderr, "OVERFLOW: %f -> %d\n", v, r); r = FP_MAX; }
    if (r < FP_MIN) { fprintf(stderr, "UNDERFLOW: %f -> %d\n", v, r); r = FP_MIN; }
    return r;
}

// ============================================================
// ROM layout per filter setting (8 slots, coeff_idx = lower 3 bits of addr):
//   idx 0 → b0 = 1.0 always (stored anyway for uniformity, never read)
//   idx 1 → a0
//   idx 2 → a1
//   idx 3 → a2
//   idx 4 → a3
//   idx 5 → b1
//   idx 6 → b2
//   idx 7 → b3

struct Coeffs {
    double b0, a0, a1, a2, a3, b1, b2, b3;
    double operator[](int i) const {
        const double *p = &b0;
        return p[i];
    }
};

// ============================================================
// Per-filter optimal scale computation
//
// ROM layout (addr_for_tap in iir_filter_slow):
//   A group: ROM idx 1..N_X       (feedforward: a0..a(N_X-1))
//   B group: ROM idx 5..4+N_Y-1   (feedback:    b1..b(N_Y-1))
//   ROM idx 0 = b0 = 1.0, never read by iir_filter_slow
//
// Scale = largest power of 2 such that max_abs * scale <= FP_MAX
// ============================================================
struct FilterScales {
    int frac_a = FP_FRAC;
    int frac_b = FP_FRAC;
    double scale_a() const { return std::ldexp(1.0, frac_a); }
    double scale_b() const { return std::ldexp(1.0, frac_b); }
};

FilterScales compute_scales(int n_settings, int N_X, int N_Y,
                             std::function<Coeffs(int)> get_coeffs);

// Fill 8 ROM slots using per-group scales.
// Split point: idx < 5 → A scale, idx >= 5 → B scale.
// (b0 at idx 0 is stored with A scale but never used in computation.)
void fill_scaled(int32_t *dst, const Coeffs &c, const FilterScales &sc)
{
    dst[0] = 0;  // b0=1.0 at idx 0 is never read by iir_filter_slow (addr_for_tap skips it)
    for (int i = 1; i < 8; i++) {
        double scale = (i >= 5) ? sc.scale_b() : sc.scale_a();
        dst[i] = to_fixed_s(c[i], scale);
    }
}

double bits_to_caps(uint32_t value, std::initializer_list<double> caps)
{
    double total = 0.0;
    for (double c : caps) {
        if (value & 1) total += c;
        value >>= 1;
    }
    return total;
}

// ============================================================
// Filter builders (clock-free, normalized by b0)
// ============================================================

Coeffs build_standard(double c1t, double c1b, double c2t, double c2b,
                       double c3, double c4)
{
	double k0 = c1t / (m_cclock * c1b);
	double k1 = c4 * c2t / (m_cclock * c1b * c3);
	double k2 = c4 * c2b / (m_cclock * m_cclock * c1b * c3);

	double fpeak = sqrt(fabs(k0*k1 - k2))/(2*M_PI*k2);
	double zc = 2*M_PI*fpeak/tan(M_PI*fpeak / m_sclock);

	double m0 = zc*k0;
	double m1 = zc*k1;
	double m2 = zc*zc*k2;

    double b0 = 1+m1+m2;
    return {
        1.0,
        (1+m0)/b0, (3+m0)/b0, (3-m0)/b0, (1-m0)/b0,
        (3+m1-m2)/b0, (3-m1-m2)/b0, (1-m1+m2)/b0
    };
}

Coeffs build_lowpass(double c1t, double c1b)
{
	double k = c1b / (m_cclock * c1t) * (150.0/4000.0);
	double fpeak = 1/(2*M_PI*k);
	double zc = 2*M_PI*fpeak/tan(M_PI*fpeak / m_sclock);
	double m = zc*k;
    double b0 = 1.0 + m;
    return { 1.0, 1.0/b0, 0, 0, 0, (1.0-m)/b0, 0, 0 };
}

Coeffs build_noise_shaper(double c1, double c2t, double c2b,
                           double c3, double c4)
{
	double k0 = c2t*c3*c2b/c4;
	double k1 = c2t*(m_cclock * c2b);
	double k2 = c1*c2t*c3/(m_cclock * c4);

	double fpeak = sqrt(1/k2)/(2*M_PI);
	double zc = 2*M_PI*fpeak/tan(M_PI*fpeak / m_sclock);

	double m0 = zc*k0;
	double m1 = zc*k1;
	double m2 = zc*zc*k2;

    double b0 = 1+m1+m2;
    return { 1.0, m0/b0, 0, -m0/b0, 0, (2-2*m2)/b0, (1-m1+m2)/b0, 0 };
}

Coeffs build_injection(double c1b, double c2t, double c2b,
                        double c3, double /* c4 */)
{
    static constexpr double T = 2.0 * SCLOCK_NORM;

    double k0_n = c2t;
    double k1_n = c1b * c3 / c2t - c2t;
    double m_n  = T * c2b;

    double b0_raw = (k1_n >= 0.0) ? (k1_n + m_n) : (k1_n - m_n);
    double b1_raw = (k1_n >= 0.0) ? (k1_n - m_n) : (k1_n + m_n);
    double a_pos  = k0_n + m_n;
    double a_neg  = k0_n - m_n;

    if (fabs(b0_raw) < 1e-12) {
        fprintf(stderr, "build_injection: b0≈0 at c2t=%.0f c3=%.0f, using pass-through\n",
                c2t, c3);
        return { 1.0, 1.0, 0, 0, 0, 0, 0, 0 };
    }

    return { 1.0, a_pos/b0_raw, a_neg/b0_raw, 0, 0, b1_raw/b0_raw, 0, 0 };
}

// ============================================================
// compute_scales implementation (after Coeffs is defined)
// ============================================================
FilterScales compute_scales(int n_settings, int N_X, int N_Y,
                             std::function<Coeffs(int)> get_coeffs)
{
    double max_a = 0.0, max_b = 0.0;
    int N_B = N_Y - 1;
    for (int s = 0; s < n_settings; s++) {
        Coeffs c = get_coeffs(s);
        // A group: ROM idx 1..N_X  (a0..a(N_X-1))
        for (int i = 1; i <= N_X; i++)
            max_a = std::max(max_a, std::fabs(c[i]));
        // B group: ROM idx 5..4+N_B  (b1..b(N_Y-1))
        for (int i = 5; i < 5 + N_B; i++)
            max_b = std::max(max_b, std::fabs(c[i]));
    }
    int fa = (max_a > 0.0) ? (int)std::floor(std::log2((double)FP_MAX / max_a)) : FP_FRAC;
    int fb = (max_b > 0.0) ? (int)std::floor(std::log2((double)FP_MAX / max_b)) : FP_FRAC;
    return {fa, fb};
}

// ============================================================
// Compute scales for all filters at once
// ============================================================
struct AllFilterScales {
    FilterScales f1, f2v, f3, f4, fx, fn, f2n;
};

AllFilterScales compute_all_scales()
{
    AllFilterScales s;

    s.f1 = compute_scales(16, 4, 4, [](int b) {
        return build_standard(11247, 11797, 949, 52067,
               2280 + bits_to_caps(b, {2546, 4973, 9861, 19724}), 166272);
    });

    s.f2v = compute_scales(512, 4, 4, [](int idx) {
        int qb = idx / 32, fb = idx % 32;
        double c2t = 829 + bits_to_caps(qb, {1390, 2965, 5875, 11297});
        double c3  = 2352 + bits_to_caps(fb, {833, 1663, 3164, 6327, 12654});
        return build_standard(24840, 29154, c2t, 38180, c3, 34270);
    });

    s.f3 = compute_scales(16, 4, 4, [](int b) {
        return build_standard(0, 17594, 868, 18828,
               8480 + bits_to_caps(b, {2226, 4485, 9056, 18111}), 50019);
    });

    s.f4 = compute_scales(1, 4, 4, [](int) {
        return build_standard(0, 28810, 1165, 21457, 8558, 7289);
    });

    s.fx = compute_scales(1, 2, 2, [](int) {
        return build_lowpass(1122, 23131);
    });

    s.fn = compute_scales(1, 4, 4, [](int) {
        return build_noise_shaper(15500, 14854, 8450, 9523, 14083);
    });

    s.f2n = compute_scales(512, 2, 2, [](int idx) {
        int qb = idx / 32, fb = idx % 32;
        double c2t = 829 + bits_to_caps(qb, {1390, 2965, 5875, 11297});
        double c3  = 2352 + bits_to_caps(fb, {833, 1663, 3164, 6327, 12654});
        return build_injection(29154, c2t, 38180, c3, 34270);
    });

    return s;
}

// ============================================================
// Pole analysis (stability check)
// ============================================================

static std::complex<double> poly_eval(const double *p, int n, std::complex<double> z)
{
    std::complex<double> r(1.0, 0.0);
    for (int i = 0; i < n; i++)
        r = r * z + p[i];
    return r;
}

static void find_roots_dkr(const double *p, int n, std::complex<double> *roots)
{
    const std::complex<double> w(0.4, 0.9);
    for (int i = 0; i < n; i++)
        roots[i] = std::pow(w, i);

    for (int iter = 0; iter < 500; iter++) {
        double err = 0.0;
        for (int i = 0; i < n; i++) {
            std::complex<double> val = poly_eval(p, n, roots[i]);
            std::complex<double> den(1.0, 0.0);
            for (int j = 0; j < n; j++)
                if (j != i) den *= (roots[i] - roots[j]);
            std::complex<double> delta = val / den;
            roots[i] -= delta;
            err = std::max(err, std::abs(delta));
        }
        if (err < 1e-12) break;
    }
}

static int get_poles(const Coeffs &c, std::complex<double> poles[3],
                     bool *nyquist_factored = nullptr)
{
    if (nyquist_factored) *nyquist_factored = false;

    int order;
    double p[3];
    if (std::fabs(c.b3) > 1e-9) {
        p[0] = c.b1; p[1] = c.b2; p[2] = c.b3; order = 3;
    } else if (std::fabs(c.b2) > 1e-9) {
        p[0] = c.b1; p[1] = c.b2; order = 2;
    } else {
        order = 1;
    }

    if (order == 3 && std::fabs(c.b2 - (c.b1 + c.b3 - 1.0)) < 1e-6) {
        double quad[2] = { c.b1 - 1.0, c.b3 };
        find_roots_dkr(quad, 2, poles);
        if (nyquist_factored) *nyquist_factored = true;
        return 2;
    }

    if (order == 1) {
        poles[0] = -c.b1;
    } else {
        find_roots_dkr(p, order, poles);
    }
    return order;
}

// ============================================================
// Sanity check: all coefficients fit in s(2.15)?
// ============================================================
void sanity_check()
{
    double gmax = -DBL_MAX, gmin = DBL_MAX;
    auto check = [&](const Coeffs &c) {
        for (int i = 0; i < 8; i++) {
            if (c[i] > gmax) gmax = c[i];
            if (c[i] < gmin) gmin = c[i];
        }
    };

    for (uint32_t b = 0; b < 16; b++)
        check(build_standard(11247, 11797, 949, 52067,
              2280 + bits_to_caps(b, {2546, 4973, 9861, 19724}), 166272));

    for (uint32_t qb = 0; qb < 16; qb++) {
        double c2t = 829 + bits_to_caps(qb, {1390, 2965, 5875, 11297});
        for (uint32_t fb = 0; fb < 32; fb++)
            check(build_standard(24840, 29154, c2t, 38180,
                  2352 + bits_to_caps(fb, {833, 1663, 3164, 6327, 12654}), 34270));
    }

    for (uint32_t b = 0; b < 16; b++)
        check(build_standard(0, 17594, 868, 18828,
              8480 + bits_to_caps(b, {2226, 4485, 9056, 18111}), 50019));

    check(build_standard(0, 28810, 1165, 21457, 8558, 7289));
    check(build_lowpass(1122, 23131));
    check(build_noise_shaper(15500, 14854, 8450, 9523, 14083));

    for (uint32_t qb = 0; qb < 16; qb++) {
        double c2t = 829 + bits_to_caps(qb, {1390, 2965, 5875, 11297});
        for (uint32_t fb = 0; fb < 32; fb++) {
            double c3 = 2352 + bits_to_caps(fb, {833, 1663, 3164, 6327, 12654});
            check(build_injection(29154, c2t, 38180, c3, 34270));
        }
    }

    fprintf(stderr, "Global coeff range: [%f, %f]\n", gmin, gmax);
    fprintf(stderr, "s(2.15) range:      [%f, %f]\n",
            (double)FP_MIN/FP_SCALE, (double)FP_MAX/FP_SCALE);
    fprintf(stderr, "Fits: %s\n\n",
            (gmin >= (double)FP_MIN/FP_SCALE && gmax <= (double)FP_MAX/FP_SCALE)
            ? "YES" : "NO - OVERFLOW!");
}

// ============================================================
// Stability check: pole radii for all filter settings
// ============================================================
void stability_check()
{
    const double NEAR_LO = 0.99;
    const double NEAR_HI = 1.01;

    struct Stats {
        const char *name;
        double max_r = 0.0, min_r = 1e9;
        int n_unstable = 0, n_near = 0;
        char worst[64] = "";
    };

    auto process = [&](Stats &s, const Coeffs &c, const char *label = nullptr) {
        std::complex<double> poles[3];
        int n = get_poles(c, poles);
        for (int i = 0; i < n; i++) {
            double r = std::abs(poles[i]);
            if (r > s.max_r) {
                s.max_r = r;
                if (label) snprintf(s.worst, sizeof(s.worst), "%s", label);
            }
            if (r < s.min_r) s.min_r = r;
            if (r > 1.0 + 1e-6)                             s.n_unstable++;
            else if (r >= NEAR_LO && r <= 1.0 + 1e-6)       s.n_near++;
        }
    };

    Stats f1{"F1"}, f2v{"F2V"}, f3{"F3"}, f4{"F4"}, fx{"FX"}, fn{"FN"}, f2n{"F2N"};
    char lbl[64];

    for (uint32_t b = 0; b < 16; b++) {
        snprintf(lbl, sizeof(lbl), "b=%u", b);
        process(f1, build_standard(11247, 11797, 949, 52067,
                2280 + bits_to_caps(b, {2546, 4973, 9861, 19724}), 166272), lbl);
    }

    for (uint32_t qb = 0; qb < 16; qb++) {
        double c2t = 829 + bits_to_caps(qb, {1390, 2965, 5875, 11297});
        for (uint32_t fb = 0; fb < 32; fb++) {
            snprintf(lbl, sizeof(lbl), "q=%u,f=%u", qb, fb);
            process(f2v, build_standard(24840, 29154, c2t, 38180,
                    2352 + bits_to_caps(fb, {833, 1663, 3164, 6327, 12654}), 34270), lbl);
        }
    }

    for (uint32_t b = 0; b < 16; b++) {
        snprintf(lbl, sizeof(lbl), "b=%u", b);
        process(f3, build_standard(0, 17594, 868, 18828,
                8480 + bits_to_caps(b, {2226, 4485, 9056, 18111}), 50019), lbl);
    }

    process(f4,  build_standard(0, 28810, 1165, 21457, 8558, 7289),  "fixed");
    process(fx,  build_lowpass(1122, 23131),                          "fixed");
    process(fn,  build_noise_shaper(15500, 14854, 8450, 9523, 14083), "fixed");

    for (uint32_t qb = 0; qb < 16; qb++) {
        double c2t = 829 + bits_to_caps(qb, {1390, 2965, 5875, 11297});
        for (uint32_t fb = 0; fb < 32; fb++) {
            double c3 = 2352 + bits_to_caps(fb, {833, 1663, 3164, 6327, 12654});
            snprintf(lbl, sizeof(lbl), "q=%u,f=%u", qb, fb);
            process(f2n, build_injection(29154, c2t, 38180, c3, 34270), lbl);
        }
    }

    fprintf(stderr, "=== Stability check (|z| of poles) ===\n");
    fprintf(stderr, "  Note: |z| < 1 = stable; |z| >= 0.99 = near unit circle; |z| >= 1 = UNSTABLE\n\n");

    auto report = [&](const Stats &s) {
        fprintf(stderr, "  %-6s: |z| in [%.6f, %.6f]  worst=%s",
                s.name, s.min_r, s.max_r, s.worst[0] ? s.worst : "?");
        if (s.n_unstable) fprintf(stderr, "  *** UNSTABLE: %d poles ***", s.n_unstable);
        if (s.n_near)     fprintf(stderr, "  [%d pole(s) near unit circle (>=%.2f)]", s.n_near, NEAR_LO);
        fprintf(stderr, "\n");
    };

    report(f1); report(f2v); report(f3); report(f4);
    report(fx);  report(fn);  report(f2n);
    fprintf(stderr, "\n");
}

// ============================================================
// C header output (for MAME fixed-point test)
// ============================================================
void gen_c_header(FILE *f, const AllFilterScales &scales)
{
    fprintf(f, "// Auto-generated by gen_votrax_roms.cpp\n");
    fprintf(f, "// SC01A filter coefficient ROMs\n");
    fprintf(f, "// Per-filter optimal fixed-point scaling (power of 2)\n");
    fprintf(f, "//\n");
    fprintf(f, "// A coefficients: ROM idx 1..N_X  (feedforward)\n");
    fprintf(f, "// B coefficients: ROM idx 5..7    (feedback)\n");
    fprintf(f, "// ROM idx 0 = b0 = 1.0 (stored but never read)\n");
    fprintf(f, "\n");
    fprintf(f, "#pragma once\n");
    fprintf(f, "#include <cstdint>\n\n");

    // Emit per-filter scale constants
    fprintf(f, "// Per-filter scale factors (scale = 2^FP_FRAC_x)\n");
    auto emit_scales = [&](const char *name, const FilterScales &s) {
        fprintf(f, "static constexpr int %s_FP_FRAC_A = %d;  // scale = %.0f\n",
                name, s.frac_a, s.scale_a());
        fprintf(f, "static constexpr int %s_FP_FRAC_B = %d;  // scale = %.0f\n",
                name, s.frac_b, s.scale_b());
    };
    emit_scales("F1",  scales.f1);
    emit_scales("F2V", scales.f2v);
    emit_scales("F3",  scales.f3);
    emit_scales("F4",  scales.f4);
    emit_scales("FX",  scales.fx);
    emit_scales("FN",  scales.fn);
    emit_scales("F2N", scales.f2n);
    fprintf(f, "\n");

    auto write_rom = [&](const char *name, const char *comment,
                         int entries, int32_t *data) {
        fprintf(f, "// %s\n", comment);
        fprintf(f, "static const int32_t %s[%d] = {\n", name, entries * 8);
        for (int e = 0; e < entries; e++) {
            fprintf(f, "    ");
            for (int i = 0; i < 8; i++) {
                fprintf(f, "%7d", data[e*8+i]);
                if (e*8+i < entries*8-1) fprintf(f, ",");
            }
            fprintf(f, "  // [%d]\n", e);
        }
        fprintf(f, "};\n\n");
    };

    // F1
    {
        int32_t data[16*8];
        for (uint32_t b = 0; b < 16; b++) {
            double c3 = 2280 + bits_to_caps(b, {2546, 4973, 9861, 19724});
            fill_scaled(&data[b*8],
                        build_standard(11247, 11797, 949, 52067, c3, 166272),
                        scales.f1);
        }
        fprintf(f, "// F1: 16 settings x 8 coeffs, FP_FRAC_A=%d FP_FRAC_B=%d\n",
                scales.f1.frac_a, scales.f1.frac_b);
        fprintf(f, "// addr = (filt_f1 << 3) | coeff_idx\n");
        fprintf(f, "static const int32_t f1_rom[128] = {\n");
        for (int i = 0; i < 128; i++)
            fprintf(f, "    %7d%s  // [%d] idx%d\n",
                    data[i], i<127?",":"", i/8, i%8);
        fprintf(f, "};\n\n");
    }

    // F2V
    {
        fprintf(f, "// F2V: 512 settings x 8 coeffs = 4096 entries, FP_FRAC_A=%d FP_FRAC_B=%d\n",
                scales.f2v.frac_a, scales.f2v.frac_b);
        fprintf(f, "// addr = (filt_f2q << 8) | (filt_f2 << 3) | coeff_idx\n");
        fprintf(f, "static const int32_t f2v_rom[4096] = {\n");
        for (uint32_t qb = 0; qb < 16; qb++) {
            double c2t = 829 + bits_to_caps(qb, {1390, 2965, 5875, 11297});
            for (uint32_t fb = 0; fb < 32; fb++) {
                double c3 = 2352 + bits_to_caps(fb, {833, 1663, 3164, 6327, 12654});
                int32_t entry[8];
                fill_scaled(entry,
                            build_standard(24840, 29154, c2t, 38180, c3, 34270),
                            scales.f2v);
                int base = (qb*32+fb)*8;
                fprintf(f, "    ");
                for (int i = 0; i < 8; i++)
                    fprintf(f, "%7d%s", entry[i], (base+i < 4095) ? "," : " ");
                fprintf(f, "  // q=%u f=%u\n", qb, fb);
            }
        }
        fprintf(f, "};\n\n");
    }

    // F3
    {
        fprintf(f, "// F3: 16 settings x 8 coeffs, FP_FRAC_A=%d FP_FRAC_B=%d\n",
                scales.f3.frac_a, scales.f3.frac_b);
        fprintf(f, "// addr = (filt_f3 << 3) | coeff_idx\n");
        fprintf(f, "static const int32_t f3_rom[128] = {\n");
        for (uint32_t b = 0; b < 16; b++) {
            double c3 = 8480 + bits_to_caps(b, {2226, 4485, 9056, 18111});
            int32_t entry[8];
            fill_scaled(entry,
                        build_standard(0, 17594, 868, 18828, c3, 50019),
                        scales.f3);
            fprintf(f, "    ");
            for (int i = 0; i < 8; i++)
                fprintf(f, "%7d%s", entry[i], (b*8+i < 127) ? "," : " ");
            fprintf(f, "  // [%u]\n", b);
        }
        fprintf(f, "};\n\n");
    }

    // F4, FX, FN
    auto write_const = [&](const char *name, const char *comment,
                           const Coeffs &c, const FilterScales &sc) {
        int32_t entry[8];
        fill_scaled(entry, c, sc);
        fprintf(f, "// %s  FP_FRAC_A=%d FP_FRAC_B=%d\n", comment, sc.frac_a, sc.frac_b);
        fprintf(f, "static const int32_t %s[8] = {", name);
        for (int i = 0; i < 8; i++)
            fprintf(f, " %7d%s", entry[i], i<7?",":"");
        fprintf(f, " };\n\n");
    };

    write_const("f4_rom", "F4: constant",
                build_standard(0, 28810, 1165, 21457, 8558, 7289), scales.f4);
    write_const("fx_rom", "FX lowpass: constant",
                build_lowpass(1122, 23131), scales.fx);
    write_const("fn_rom", "FN noise shaper: constant",
                build_noise_shaper(15500, 14854, 8450, 9523, 14083), scales.fn);

    // F2N
    {
        fprintf(f, "// F2N: 512 settings x 8 coeffs = 4096 entries, FP_FRAC_A=%d FP_FRAC_B=%d\n",
                scales.f2n.frac_a, scales.f2n.frac_b);
        fprintf(f, "// addr = (filt_f2q << 8) | (filt_f2 << 3) | coeff_idx\n");
        fprintf(f, "static const int32_t f2n_rom[4096] = {\n");
        for (uint32_t qb = 0; qb < 16; qb++) {
            double c2t = 829 + bits_to_caps(qb, {1390, 2965, 5875, 11297});
            for (uint32_t fb = 0; fb < 32; fb++) {
                double c3 = 2352 + bits_to_caps(fb, {833, 1663, 3164, 6327, 12654});
                int32_t entry[8];
                fill_scaled(entry,
                            build_injection(29154, c2t, 38180, c3, 34270),
                            scales.f2n);
                int base = (qb*32+fb)*8;
                fprintf(f, "    ");
                for (int i = 0; i < 8; i++)
                    fprintf(f, "%7d%s", entry[i], (base+i < 4095) ? "," : " ");
                fprintf(f, "  // q=%u f=%u\n", qb, fb);
            }
        }
        fprintf(f, "};\n\n");
    }
}

// ============================================================
// VHDL ROM entity generator
// ============================================================
void gen_vhdl_rom_entity(FILE *f, const char *name, int addr_bits,
                          int depth, const int32_t *data,
                          const FilterScales &scales)
{
    fprintf(f, "-- Auto-generated by gen_votrax_roms.cpp\n");
    fprintf(f, "-- %s: %d entries x 18-bit\n", name, depth);
    fprintf(f, "-- A coefficients (idx 1..N_X, feedforward): scale = 2^%d = %.0f\n",
            scales.frac_a, scales.scale_a());
    fprintf(f, "-- B coefficients (idx 5..7,  feedback):     scale = 2^%d = %.0f\n",
            scales.frac_b, scales.scale_b());
    fprintf(f, "-- addr(2:0) = coeff_idx: 0=b0(unused) 1=a0 2=a1 3=a2 4=a3 5=b1 6=b2 7=b3\n");
    fprintf(f, "-- Synchronous read: 1-cycle latency, infers as BRAM\n");
    fprintf(f, "\n");
    fprintf(f, "library ieee;\n");
    fprintf(f, "use ieee.std_logic_1164.all;\n");
    fprintf(f, "use ieee.numeric_std.all;\n");
    fprintf(f, "\n");
    fprintf(f, "entity %s is\n", name);
    fprintf(f, "    port (\n");
    fprintf(f, "        clk  : in  std_logic;\n");
    fprintf(f, "        addr : in  unsigned(%d downto 0);\n", addr_bits - 1);
    fprintf(f, "        data : out signed(17 downto 0)\n");
    fprintf(f, "    );\n");
    fprintf(f, "end entity;\n");
    fprintf(f, "\n");
    fprintf(f, "architecture rtl of %s is\n", name);
    fprintf(f, "\n");
    fprintf(f, "    -- Scaling constants (for iir_filter_slow generics FP_FRAC_A / FP_FRAC_B)\n");
    fprintf(f, "    constant FP_FRAC_A : integer := %d;  -- A coefficients: scale = 2^%d\n",
            scales.frac_a, scales.frac_a);
    fprintf(f, "    constant FP_FRAC_B : integer := %d;  -- B coefficients: scale = 2^%d\n",
            scales.frac_b, scales.frac_b);
    fprintf(f, "\n");
    fprintf(f, "    type rom_t is array(0 to %d) of signed(17 downto 0);\n", depth - 1);
    fprintf(f, "    constant ROM : rom_t := (\n");

    for (int i = 0; i < depth; i++) {
        int v = data[i];
        bool last = (i == depth - 1);
        int idx = i % 8;
        double scale = (idx >= 5) ? scales.scale_b() : scales.scale_a();
        const char *coeff_names[8] = {"b0","a0","a1","a2","a3","b1","b2","b3"};
        fprintf(f, "        to_signed(%7d, 18)%s  -- [%4d] %s = %+.6f\n",
                v, last ? " " : ",",
                i, coeff_names[idx], v / scale);
    }

    fprintf(f, "    );\n");
    fprintf(f, "\n");
    fprintf(f, "begin\n");
    fprintf(f, "\n");
    fprintf(f, "    process(clk)\n");
    fprintf(f, "    begin\n");
    fprintf(f, "        if rising_edge(clk) then\n");
    fprintf(f, "            data <= ROM(to_integer(addr));\n");
    fprintf(f, "        end if;\n");
    fprintf(f, "    end process;\n");
    fprintf(f, "\n");
    fprintf(f, "end architecture;\n");
}

// ============================================================
// VHDL package with all scale constants
// ============================================================
void gen_vhdl_scales_package(FILE *f, const AllFilterScales &scales)
{
    fprintf(f, "-- Auto-generated by gen_votrax_roms.cpp\n");
    fprintf(f, "-- SC01A filter coefficient scale constants\n");
    fprintf(f, "--\n");
    fprintf(f, "-- A coefficients: ROM idx 1..N_X  (feedforward, addr_for_tap: 1+tap)\n");
    fprintf(f, "-- B coefficients: ROM idx 5..4+NB (feedback,    addr_for_tap: 5+(tap-N_X))\n");
    fprintf(f, "-- Scale = 2^FP_FRAC_x  (optimal: largest power of 2 fitting 18-bit signed)\n");
    fprintf(f, "\n");
    fprintf(f, "library ieee;\n");
    fprintf(f, "use ieee.std_logic_1164.all;\n");
    fprintf(f, "\n");
    fprintf(f, "package sc01a_coeff_scales is\n");
    fprintf(f, "\n");

    auto emit = [&](const char *name, const FilterScales &s) {
        fprintf(f, "    -- %s\n", name);
        fprintf(f, "    constant %s_FP_FRAC_A : integer := %d;  -- scale = %.0f\n",
                name, s.frac_a, s.scale_a());
        fprintf(f, "    constant %s_FP_FRAC_B : integer := %d;  -- scale = %.0f\n",
                name, s.frac_b, s.scale_b());
        fprintf(f, "\n");
    };

    emit("F1",  scales.f1);
    emit("F2V", scales.f2v);
    emit("F3",  scales.f3);
    emit("F4",  scales.f4);
    emit("FX",  scales.fx);
    emit("FN",  scales.fn);
    emit("F2N", scales.f2n);

    fprintf(f, "end package;\n");
}

// ============================================================
void gen_vhdl_rom_entities(const char *outdir, const AllFilterScales &scales)
{
    char path[256];

    // ---- F1: 128 entries, 7-bit addr ----
    {
        int32_t data[128];
        for (uint32_t b = 0; b < 16; b++) {
            double c3 = 2280 + bits_to_caps(b, {2546, 4973, 9861, 19724});
            fill_scaled(&data[b*8],
                        build_standard(11247, 11797, 949, 52067, c3, 166272),
                        scales.f1);
        }
        snprintf(path, sizeof(path), "%s/f1_rom.vhd", outdir);
        FILE *f = fopen(path, "w");
        gen_vhdl_rom_entity(f, "f1_rom", 7, 128, data, scales.f1);
        fclose(f);
        fprintf(stderr, "Written: %s  (FP_FRAC_A=%d FP_FRAC_B=%d)\n",
                path, scales.f1.frac_a, scales.f1.frac_b);
    }

    // ---- F2V: 4096 entries, 12-bit addr ----
    {
        int32_t data[4096];
        for (uint32_t qb = 0; qb < 16; qb++) {
            double c2t = 829 + bits_to_caps(qb, {1390, 2965, 5875, 11297});
            for (uint32_t fb = 0; fb < 32; fb++) {
                double c3 = 2352 + bits_to_caps(fb, {833, 1663, 3164, 6327, 12654});
                fill_scaled(&data[(qb*32+fb)*8],
                            build_standard(24840, 29154, c2t, 38180, c3, 34270),
                            scales.f2v);
            }
        }
        snprintf(path, sizeof(path), "%s/f2v_rom.vhd", outdir);
        FILE *f = fopen(path, "w");
        gen_vhdl_rom_entity(f, "f2v_rom", 12, 4096, data, scales.f2v);
        fclose(f);
        fprintf(stderr, "Written: %s  (FP_FRAC_A=%d FP_FRAC_B=%d)\n",
                path, scales.f2v.frac_a, scales.f2v.frac_b);
    }

    // ---- F3: 128 entries, 7-bit addr ----
    {
        int32_t data[128];
        for (uint32_t b = 0; b < 16; b++) {
            double c3 = 8480 + bits_to_caps(b, {2226, 4485, 9056, 18111});
            fill_scaled(&data[b*8],
                        build_standard(0, 17594, 868, 18828, c3, 50019),
                        scales.f3);
        }
        snprintf(path, sizeof(path), "%s/f3_rom.vhd", outdir);
        FILE *f = fopen(path, "w");
        gen_vhdl_rom_entity(f, "f3_rom", 7, 128, data, scales.f3);
        fclose(f);
        fprintf(stderr, "Written: %s  (FP_FRAC_A=%d FP_FRAC_B=%d)\n",
                path, scales.f3.frac_a, scales.f3.frac_b);
    }

    // ---- F4: 8 entries, 3-bit addr ----
    {
        int32_t data[8];
        fill_scaled(data, build_standard(0, 28810, 1165, 21457, 8558, 7289), scales.f4);
        snprintf(path, sizeof(path), "%s/f4_rom.vhd", outdir);
        FILE *f = fopen(path, "w");
        gen_vhdl_rom_entity(f, "f4_rom", 3, 8, data, scales.f4);
        fclose(f);
        fprintf(stderr, "Written: %s  (FP_FRAC_A=%d FP_FRAC_B=%d)\n",
                path, scales.f4.frac_a, scales.f4.frac_b);
    }

    // ---- FX: 8 entries, 3-bit addr ----
    {
        int32_t data[8];
        fill_scaled(data, build_lowpass(1122, 23131), scales.fx);
        snprintf(path, sizeof(path), "%s/fx_rom.vhd", outdir);
        FILE *f = fopen(path, "w");
        gen_vhdl_rom_entity(f, "fx_rom", 3, 8, data, scales.fx);
        fclose(f);
        fprintf(stderr, "Written: %s  (FP_FRAC_A=%d FP_FRAC_B=%d)\n",
                path, scales.fx.frac_a, scales.fx.frac_b);
    }

    // ---- FN: 8 entries, 3-bit addr ----
    {
        int32_t data[8];
        fill_scaled(data, build_noise_shaper(15500, 14854, 8450, 9523, 14083), scales.fn);
        snprintf(path, sizeof(path), "%s/fn_rom.vhd", outdir);
        FILE *f = fopen(path, "w");
        gen_vhdl_rom_entity(f, "fn_rom", 3, 8, data, scales.fn);
        fclose(f);
        fprintf(stderr, "Written: %s  (FP_FRAC_A=%d FP_FRAC_B=%d)\n",
                path, scales.fn.frac_a, scales.fn.frac_b);
    }

    // ---- F2N: 4096 entries, 12-bit addr ----
    {
        int32_t data[4096];
        for (uint32_t qb = 0; qb < 16; qb++) {
            double c2t = 829 + bits_to_caps(qb, {1390, 2965, 5875, 11297});
            for (uint32_t fb = 0; fb < 32; fb++) {
                double c3 = 2352 + bits_to_caps(fb, {833, 1663, 3164, 6327, 12654});
                fill_scaled(&data[(qb*32+fb)*8],
                            build_injection(29154, c2t, 38180, c3, 34270),
                            scales.f2n);
            }
        }
        snprintf(path, sizeof(path), "%s/f2n_rom.vhd", outdir);
        FILE *f = fopen(path, "w");
        gen_vhdl_rom_entity(f, "f2n_rom", 12, 4096, data, scales.f2n);
        fclose(f);
        fprintf(stderr, "Written: %s  (FP_FRAC_A=%d FP_FRAC_B=%d)\n",
                path, scales.f2n.frac_a, scales.f2n.frac_b);
    }

    // ---- VHDL package with all scale constants ----
    {
        snprintf(path, sizeof(path), "%s/sc01a_coeff_scales_pkg.vhd", outdir);
        FILE *f = fopen(path, "w");
        gen_vhdl_scales_package(f, scales);
        fclose(f);
        fprintf(stderr, "Written: %s\n", path);
    }
}

// ============================================================
int main()
{
    set_clocks_from_dac(0x80);
    sanity_check();
    stability_check();

    AllFilterScales scales = compute_all_scales();

    fprintf(stderr, "=== Optimal scales (frac_a / frac_b) ===\n");
    auto print_scale = [&](const char *name, const FilterScales &s) {
        fprintf(stderr, "  %-6s: FP_FRAC_A=%d (%.0f)  FP_FRAC_B=%d (%.0f)\n",
                name, s.frac_a, s.scale_a(), s.frac_b, s.scale_b());
    };
    print_scale("F1",  scales.f1);
    print_scale("F2V", scales.f2v);
    print_scale("F3",  scales.f3);
    print_scale("F4",  scales.f4);
    print_scale("FX",  scales.fx);
    print_scale("FN",  scales.fn);
    print_scale("F2N", scales.f2n);
    fprintf(stderr, "\n");

    FILE *f = fopen("/tmp/votrax_rom_tables.h", "w");
    gen_c_header(f, scales);
    fclose(f);
    fprintf(stderr, "C header: /tmp/votrax_rom_tables.h\n");

    gen_vhdl_rom_entities("/tmp", scales);

    return 0;
}
