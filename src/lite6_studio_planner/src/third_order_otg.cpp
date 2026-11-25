#include "lite6_studio_planner/third_order_otg.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <utility>

namespace lite6_studio_planner
{
namespace otg
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kEps = 1e-12;
constexpr double kTimeTol = 1e-7;
constexpr double kPosCheckTol = 1e-6;
constexpr double kMaxInternalDur = 300.0;
constexpr int kMaxBoundDepth = 3;

inline void integrate_step(
  double dt, double q0, double v0, double a0, double j,
  double & q1, double & v1, double & a1)
{
  q1 = q0 + dt * (v0 + dt * (0.5 * a0 + dt * (j / 6.0)));
  v1 = v0 + dt * (a0 + 0.5 * j * dt);
  a1 = a0 + j * dt;
}

void update_phase_extrema(
  double t_off, double dt, double q0, double v0, double a0, double j,
  PositionExtrema & ext)
{
  auto record = [&](double tau, double val) {
    if (val < ext.min) {
      ext.min = val;
      ext.t_min = t_off + tau;
    }
    if (val > ext.max) {
      ext.max = val;
      ext.t_max = t_off + tau;
    }
  };
  auto eval_q = [&](double tau) {
    return q0 + tau * (v0 + tau * (0.5 * a0 + tau * (j / 6.0)));
  };

  record(0.0, q0);
  if (dt <= 0.0) {
    return;
  }
  record(dt, eval_q(dt));

  if (std::abs(j) > 1e-14) {
    const double disc = a0 * a0 - 2.0 * j * v0;
    if (std::abs(disc) <= 1e-16) {
      const double tau = -a0 / j;
      if (tau > 0.0 && tau < dt) {
        record(tau, eval_q(tau));
      }
    } else if (disc > 0.0) {
      const double sd = std::sqrt(disc);
      const double r1 = (-a0 - sd) / j;
      const double r2 = (-a0 + sd) / j;
      if (r1 > 0.0 && r1 < dt) {
        record(r1, eval_q(r1));
      }
      if (r2 > 0.0 && r2 < dt) {
        record(r2, eval_q(r2));
      }
    }
  } else if (std::abs(a0) > 1e-14) {
    const double tau = -v0 / a0;
    if (tau > 0.0 && tau < dt) {
      record(tau, eval_q(tau));
    }
  }
}

// ---------------- Fixed-Degree Polynomial Algebra & Exact Root-Finders ----------------

template <int MaxDeg>
struct Poly
{
  std::array<double, MaxDeg + 1> c{};

  Poly() = default;
  explicit Poly(double c0) { c[0] = c0; }
  Poly(double c0, double c1)
  {
    c[0] = c0;
    if constexpr (MaxDeg >= 1) {
      c[1] = c1;
    }
  }
  Poly(double c0, double c1, double c2)
  {
    c[0] = c0;
    if constexpr (MaxDeg >= 1) {
      c[1] = c1;
    }
    if constexpr (MaxDeg >= 2) {
      c[2] = c2;
    }
  }

  double eval(double x) const
  {
    double acc = c[MaxDeg];
    for (int i = MaxDeg - 1; i >= 0; --i) {
      acc = acc * x + c[i];
    }
    return acc;
  }

  double eval_deriv(double x) const
  {
    double acc = static_cast<double>(MaxDeg) * c[MaxDeg];
    for (int i = MaxDeg - 1; i >= 1; --i) {
      acc = acc * x + static_cast<double>(i) * c[i];
    }
    return acc;
  }

  Poly & operator+=(const Poly & o)
  {
    for (int i = 0; i <= MaxDeg; ++i) {
      c[i] += o.c[i];
    }
    return *this;
  }
  Poly & operator-=(const Poly & o)
  {
    for (int i = 0; i <= MaxDeg; ++i) {
      c[i] -= o.c[i];
    }
    return *this;
  }
};

template <int D>
Poly<D> operator+(Poly<D> a, const Poly<D> & b)
{
  a += b;
  return a;
}

template <int D>
Poly<D> operator-(Poly<D> a, const Poly<D> & b)
{
  a -= b;
  return a;
}

template <int D>
Poly<D> operator*(double s, Poly<D> a)
{
  for (int i = 0; i <= D; ++i) {
    a.c[i] *= s;
  }
  return a;
}

template <int D>
Poly<D> operator*(const Poly<D> & a, const Poly<D> & b)
{
  Poly<D> out;
  for (int i = 0; i <= D; ++i) {
    if (a.c[i] == 0.0) {
      continue;
    }
    for (int j = 0; j + i <= D; ++j) {
      out.c[i + j] += a.c[i] * b.c[j];
    }
  }
  return out;
}

struct RootList
{
  std::array<double, 12> r{};
  int n = 0;

  void push(double x)
  {
    if (!std::isfinite(x)) {
      return;
    }
    for (int i = 0; i < n; ++i) {
      if (std::abs(r[i] - x) <= 1e-10 * (1.0 + std::abs(x))) {
        return;
      }
    }
    if (n < static_cast<int>(r.size())) {
      r[n++] = x;
    }
  }
};

void solve_quadratic(double a, double b, double c, RootList & out)
{
  if (std::abs(a) <= 1e-15 * (1.0 + std::abs(b) + std::abs(c))) {
    if (std::abs(b) > 1e-15 * (1.0 + std::abs(c))) {
      out.push(-c / b);
    }
    return;
  }
  const double disc = b * b - 4.0 * a * c;
  const double scale = b * b + std::abs(4.0 * a * c);
  if (disc < -1e-12 * scale) {
    return;
  }
  if (disc <= 0.0) {
    out.push(-0.5 * b / a);
    return;
  }
  const double sd = std::sqrt(disc);
  const double q = -0.5 * (b + (b >= 0.0 ? sd : -sd));
  if (std::abs(q) > 1e-30) {
    out.push(q / a);
    out.push(c / q);
  } else {
    out.push(-0.5 * b / a);
  }
}

void solve_cubic_monic(double a, double b, double c, RootList & out)
{
  const double a3 = a / 3.0;
  const double p = b - a * a3;
  const double q = (2.0 * a * a * a - 9.0 * a * b + 27.0 * c) / 27.0;
  const double p3 = p / 3.0;
  const double q2 = 0.5 * q;
  const double disc = q2 * q2 + p3 * p3 * p3;

  auto polish = [&](double x) {
    for (int it = 0; it < 4; ++it) {
      const double f = ((x + a) * x + b) * x + c;
      const double df = (3.0 * x + 2.0 * a) * x + b;
      if (std::abs(df) <= 1e-14) {
        break;
      }
      const double dx = f / df;
      x -= dx;
      if (std::abs(dx) <= 1e-14 * (1.0 + std::abs(x))) {
        break;
      }
    }
    out.push(x);
  };

  if (disc > 1e-14 * (q2 * q2 + std::abs(p3 * p3 * p3) + 1e-30)) {
    const double sd = std::sqrt(disc);
    const double u = std::cbrt(-q2 + sd);
    const double v = std::cbrt(-q2 - sd);
    polish(u + v - a3);
  } else if (p < -1e-15) {
    const double r = 2.0 * std::sqrt(-p3);
    const double arg = std::clamp(-q2 / std::sqrt(-p3 * p3 * p3), -1.0, 1.0);
    const double phi = std::acos(arg) / 3.0;
    polish(r * std::cos(phi) - a3);
    polish(r * std::cos(phi - 2.0 * kPi / 3.0) - a3);
    polish(r * std::cos(phi + 2.0 * kPi / 3.0) - a3);
  } else {
    polish(-std::cbrt(q) - a3);
  }
}

void solve_cubic(double a, double b, double c, double d, RootList & out)
{
  const double scale = 1.0 + std::abs(b) + std::abs(c) + std::abs(d);
  if (std::abs(a) <= 1e-14 * scale) {
    solve_quadratic(b, c, d, out);
    return;
  }
  solve_cubic_monic(b / a, c / a, d / a, out);
}

void solve_quartic_monic(double a, double b, double c, double d, RootList & out)
{
  auto polish_and_add = [&](double x) {
    for (int it = 0; it < 6; ++it) {
      const double f = (((x + a) * x + b) * x + c) * x + d;
      const double df = ((4.0 * x + 3.0 * a) * x + 2.0 * b) * x + c;
      if (std::abs(df) <= 1e-14) {
        break;
      }
      const double dx = f / df;
      x -= dx;
      if (std::abs(dx) <= 1e-14 * (1.0 + std::abs(x))) {
        break;
      }
    }
    const double f = (((x + a) * x + b) * x + c) * x + d;
    const double mag = 1.0 + std::abs(d) + std::abs(x) * (
      std::abs(c) + std::abs(x) * (std::abs(b) + std::abs(x) * (std::abs(a) + std::abs(x))));
    if (std::abs(f) <= 1e-6 * mag) {
      out.push(x);
    }
  };

  const double shift = -0.25 * a;
  const double a2 = a * a;
  const double p = b - 0.375 * a2;
  const double q = 0.125 * a2 * a - 0.5 * a * b + c;
  const double r = -3.0 / 256.0 * a2 * a2 + 0.0625 * a2 * b - 0.25 * a * c + d;

  const double q_scale = 1.0 + std::abs(p) * std::sqrt(std::abs(p)) + std::abs(r);
  if (std::abs(q) <= 1e-13 * q_scale) {
    RootList y2_roots;
    solve_quadratic(1.0, p, r, y2_roots);
    for (int i = 0; i < y2_roots.n; ++i) {
      if (y2_roots.r[i] >= -1e-12) {
        const double sy = std::sqrt(std::max(0.0, y2_roots.r[i]));
        polish_and_add(sy + shift);
        polish_and_add(-sy + shift);
      }
    }
  } else {
    RootList z_roots;
    solve_cubic_monic(2.0 * p, p * p - 4.0 * r, -q * q, z_roots);
    double z = -1.0;
    for (int i = 0; i < z_roots.n; ++i) {
      if (z_roots.r[i] > z) {
        z = z_roots.r[i];
      }
    }
    if (z > 1e-16) {
      const double s = std::sqrt(z);
      const double alpha = 0.5 * (p + z - q / s);
      const double beta = 0.5 * (p + z + q / s);
      RootList y1, y2;
      solve_quadratic(1.0, s, alpha, y1);
      solve_quadratic(1.0, -s, beta, y2);
      for (int i = 0; i < y1.n; ++i) {
        polish_and_add(y1.r[i] + shift);
      }
      for (int i = 0; i < y2.n; ++i) {
        polish_and_add(y2.r[i] + shift);
      }
    }
  }

  // Also test critical points of P(x) to capture near-double / tangent roots reliably.
  RootList crits;
  solve_cubic(4.0, 3.0 * a, 2.0 * b, c, crits);
  for (int i = 0; i < crits.n; ++i) {
    const double xc = crits.r[i];
    const double fc = (((xc + a) * xc + b) * xc + c) * xc + d;
    const double mag = 1.0 + std::abs(d) + std::abs(xc) * (
      std::abs(c) + std::abs(xc) * (std::abs(b) + std::abs(xc) * (std::abs(a) + std::abs(xc))));
    if (std::abs(fc) <= 1e-8 * mag) {
      out.push(xc);
    }
  }
}

void solve_quartic(double c4, double c3, double c2, double c1, double c0, RootList & out)
{
  const double scale = 1.0 + std::abs(c3) + std::abs(c2) + std::abs(c1) + std::abs(c0);
  if (std::abs(c4) <= 1e-14 * scale) {
    solve_cubic(c3, c2, c1, c0, out);
    return;
  }
  solve_quartic_monic(c3 / c4, c2 / c4, c1 / c4, c0 / c4, out);
}

// Brent root-finder for a continuous 1D function on [lo, hi] with f(lo) * f(hi) <= 0.
template <typename Fn>
double brent_root(Fn && f, double a, double b, double fa, double fb, double tol = 1e-12)
{
  if (std::abs(fa) <= 1e-15) {
    return a;
  }
  if (std::abs(fb) <= 1e-15) {
    return b;
  }
  double c = a, fc = fa;
  double d = b - a, e = d;
  for (int iter = 0; iter < 60; ++iter) {
    if ((fb > 0.0 && fc > 0.0) || (fb < 0.0 && fc < 0.0)) {
      c = a;
      fc = fa;
      d = e = b - a;
    }
    if (std::abs(fc) < std::abs(fb)) {
      a = b; b = c; c = a;
      fa = fb; fb = fc; fc = fa;
    }
    const double tol1 = 2.0 * std::numeric_limits<double>::epsilon() * std::abs(b) + 0.5 * tol;
    const double xm = 0.5 * (c - b);
    if (std::abs(xm) <= tol1 || std::abs(fb) <= 1e-15) {
      return b;
    }
    if (std::abs(e) >= tol1 && std::abs(fa) > std::abs(fb)) {
      const double s = fb / fa;
      double p = 0.0, q = 0.0;
      if (a == c) {
        p = 2.0 * xm * s;
        q = 1.0 - s;
      } else {
        q = fa / fc;
        const double r = fb / fc;
        p = s * (2.0 * xm * q * (q - r) - (b - a) * (r - 1.0));
        q = (q - 1.0) * (r - 1.0) * (s - 1.0);
      }
      if (p > 0.0) {
        q = -q;
      }
      p = std::abs(p);
      if (2.0 * p < std::min(3.0 * xm * q - std::abs(tol1 * q), std::abs(e * q))) {
        e = d;
        d = p / q;
      } else {
        d = xm;
        e = d;
      }
    } else {
      d = xm;
      e = d;
    }
    a = b;
    fa = fb;
    if (std::abs(d) > tol1) {
      b += d;
    } else {
      b += (xm > 0.0 ? tol1 : -tol1);
    }
    fb = f(b);
  }
  return b;
}

// Rolle-isolated exact real root-finder on [lo, hi] for a polynomial of degree <= 6.
template <int MaxDeg>
void solve_poly_on_interval(const Poly<MaxDeg> & poly, double lo, double hi, RootList & out)
{
  if (!(hi >= lo)) {
    return;
  }
  // Find effective degree.
  double max_c = 0.0;
  for (int i = 0; i <= MaxDeg; ++i) {
    max_c = std::max(max_c, std::abs(poly.c[i]));
  }
  if (max_c <= 1e-30) {
    return;
  }
  int deg = MaxDeg;
  while (deg > 0 && std::abs(poly.c[deg]) <= 1e-14 * max_c) {
    --deg;
  }
  if (deg <= 4) {
    RootList all;
    solve_quartic(
      deg >= 4 ? poly.c[4] : 0.0,
      deg >= 3 ? poly.c[3] : 0.0,
      deg >= 2 ? poly.c[2] : 0.0,
      deg >= 1 ? poly.c[1] : 0.0,
      poly.c[0], all);
    for (int i = 0; i < all.n; ++i) {
      if (all.r[i] >= lo - 1e-9 && all.r[i] <= hi + 1e-9) {
        out.push(std::clamp(all.r[i], lo, hi));
      }
    }
    return;
  }

  // Build derivative polynomial (degree <= 5) and isolate monotone brackets recursively.
  Poly<MaxDeg - 1> dpoly;
  for (int i = 1; i <= MaxDeg; ++i) {
    dpoly.c[i - 1] = static_cast<double>(i) * poly.c[i];
  }
  RootList crits;
  if constexpr (MaxDeg > 1) {
    solve_poly_on_interval<MaxDeg - 1>(dpoly, lo, hi, crits);
  }
  std::array<double, 14> pts{};
  int np = 0;
  pts[np++] = lo;
  for (int i = 0; i < crits.n; ++i) {
    if (crits.r[i] > lo && crits.r[i] < hi) {
      pts[np++] = crits.r[i];
    }
  }
  pts[np++] = hi;
  std::sort(pts.begin(), pts.begin() + np);

  for (int i = 0; i < np; ++i) {
    if (std::abs(poly.eval(pts[i])) <= 1e-10 * max_c) {
      out.push(pts[i]);
    }
  }
  for (int i = 0; i + 1 < np; ++i) {
    const double x0 = pts[i], x1 = pts[i + 1];
    if (x1 - x0 <= 1e-14) {
      continue;
    }
    const double f0 = poly.eval(x0), f1 = poly.eval(x1);
    if ((f0 < 0.0 && f1 > 0.0) || (f0 > 0.0 && f1 < 0.0)) {
      const double xr = brent_root([&](double x) { return poly.eval(x); }, x0, x1, f0, f1);
      out.push(xr);
    }
  }
}

// ---------------- Profile Construction, Symmetry, & Strict Self-Validation ----------------

// Negates problem (q, v, a) -> (-q, -v, -a) and swaps (v_min, v_max), (a_min, a_max).
void negate_problem(
  const AxisState & s0, const AxisState & sf, const AxisLimits & lim,
  AxisState & o0, AxisState & of, AxisLimits & olim)
{
  o0 = {-s0.q, -s0.v, -s0.a};
  of = {-sf.q, -sf.v, -sf.a};
  olim.q_min = std::isfinite(lim.q_max) ? -lim.q_max : -std::numeric_limits<double>::infinity();
  olim.q_max = std::isfinite(lim.q_min) ? -lim.q_min : std::numeric_limits<double>::infinity();
  olim.v_min = -lim.v_max;
  olim.v_max = -lim.v_min;
  olim.a_min = -lim.a_max;
  olim.a_max = -lim.a_min;
  olim.j_max = lim.j_max;
}

void negate_profile(AxisProfile & prof)
{
  for (auto & ph : prof.phases) {
    ph.q0 = -ph.q0;
    ph.v0 = -ph.v0;
    ph.a0 = -ph.a0;
    ph.j = -ph.j;
  }
  prof.qf = -prof.qf;
  prof.vf = -prof.vf;
  prof.af = -prof.af;
  prof.finalize();
}

// Time-reversal + negation: r(s) = -q(T - s).
// Maps (p0, v0, a0) -> (pf, vf, af) into (-pf, vf, -af) -> (-p0, v0, -a0),
// keeping [v_min, v_max] unchanged, swapping [a_min, a_max] -> [-a_max, -a_min],
// and preserving UDDU jerk signs (+j, 0, -j, 0, -j, 0, +j).
void time_reverse_negate_problem(
  const AxisState & s0, const AxisState & sf, const AxisLimits & lim,
  AxisState & o0, AxisState & of, AxisLimits & olim)
{
  o0 = {-sf.q, sf.v, -sf.a};
  of = {-s0.q, s0.v, -s0.a};
  olim.q_min = std::isfinite(lim.q_max) ? -lim.q_max : -std::numeric_limits<double>::infinity();
  olim.q_max = std::isfinite(lim.q_min) ? -lim.q_min : std::numeric_limits<double>::infinity();
  olim.v_min = lim.v_min;
  olim.v_max = lim.v_max;
  olim.a_min = -lim.a_max;
  olim.a_max = -lim.a_min;
  olim.j_max = lim.j_max;
}

AxisProfile time_reverse_negate_profile(const AxisProfile & rev)
{
  AxisProfile out;
  out.phases.reserve(rev.phases.size());
  for (size_t k = rev.phases.size(); k-- > 0;) {
    const auto & ph = rev.phases[k];
    double q1 = 0.0, v1 = 0.0, a1 = 0.0;
    integrate_step(ph.dt, ph.q0, ph.v0, ph.a0, ph.j, q1, v1, a1);
    // For q(t) = -r(T - t): q_start = -q1, v_start = v1, a_start = -a1, j = +ph.j.
    out.append_phase(ph.dt, -q1, v1, -a1, ph.j);
  }
  out.finalize();
  return out;
}

// Builds and strictly validates a candidate 7-phase profile from durations t[0..6] and jerks j[0..6].
bool build_and_check_7phase(
  const AxisState & s0,
  const AxisState & sf,
  const AxisLimits & lim,
  const std::array<double, 7> & dt_in,
  const std::array<double, 7> & j_in,
  std::optional<double> expected_T,
  AxisProfile & out_prof)
{
  constexpr double kNegDurTol = -1e-11;
  double total_T = 0.0;
  std::array<double, 7> dt{};
  for (size_t k = 0; k < 7; ++k) {
    if (!std::isfinite(dt_in[k]) || !std::isfinite(j_in[k])) {
      return false;
    }
    if (dt_in[k] < kNegDurTol) {
      return false;
    }
    dt[k] = std::max(0.0, dt_in[k]);
    total_T += dt[k];
    if (dt[k] > 1e-12 && std::abs(j_in[k]) > lim.j_max * (1.0 + 1e-6) + 1e-6) {
      return false;
    }
  }
  if (!std::isfinite(total_T) || total_T > kMaxInternalDur) {
    return false;
  }
  if (expected_T.has_value() && std::abs(total_T - *expected_T) > 1e-6 * (1.0 + *expected_T)) {
    return false;
  }

  const double v_tol = 1e-6 * (1.0 + std::max(std::abs(lim.v_min), std::abs(lim.v_max)));
  const double a_tol = 1e-6 * (1.0 + std::max(std::abs(lim.a_min), std::abs(lim.a_max)));

  double q = s0.q, v = s0.v, a = s0.a;
  out_prof.clear();
  out_prof.phases.reserve(7);

  for (size_t k = 0; k < 7; ++k) {
    const double h = dt[k];
    if (h <= 1e-13) {
      continue;
    }
    const double j = j_in[k];
    double q1 = 0.0, v1 = 0.0, a1 = 0.0;
    integrate_step(h, q, v, a, j, q1, v1, a1);

    if (a1 < lim.a_min - a_tol || a1 > lim.a_max + a_tol) {
      return false;
    }
    if (v1 < lim.v_min - v_tol || v1 > lim.v_max + v_tol) {
      return false;
    }
    // Check interior velocity extremum where a(tau) == 0.
    if (std::abs(j) > 1e-14 && ((a > 0.0 && a1 < 0.0) || (a < 0.0 && a1 > 0.0))) {
      const double tau_z = -a / j;
      if (tau_z > 0.0 && tau_z < h) {
        const double v_ext = v + tau_z * (a + 0.5 * j * tau_z);
        if (v_ext < lim.v_min - v_tol || v_ext > lim.v_max + v_tol) {
          return false;
        }
      }
    }
    out_prof.append_phase(h, q, v, a, j);
    q = q1;
    v = v1;
    a = a1;
  }

  const double p_tol = 1e-6 * (1.0 + std::abs(s0.q) + std::abs(sf.q) + total_T * std::max(std::abs(lim.v_min), std::abs(lim.v_max)));
  const double vf_tol = 1e-6 * (1.0 + std::abs(s0.v) + std::abs(sf.v));
  const double af_tol = 1e-6 * (1.0 + std::abs(s0.a) + std::abs(sf.a));
  if (std::abs(q - sf.q) > p_tol || std::abs(v - sf.v) > vf_tol || std::abs(a - sf.a) > af_tol) {
    return false;
  }

  if (out_prof.phases.empty()) {
    out_prof.append_phase(0.0, s0.q, s0.v, s0.a, 0.0);
  }
  out_prof.finalize();
  return true;
}

double profile_peak_jerk(const AxisProfile & prof)
{
  double mj = 0.0;
  for (const auto & ph : prof.phases) {
    if (ph.dt > 1e-12) {
      mj = std::max(mj, std::abs(ph.j));
    }
  }
  return mj;
}

bool profile_within_pos(const AxisProfile & prof, const AxisLimits & lim, double tol = kPosCheckTol)
{
  return prof.extrema.min >= lim.q_min - tol && prof.extrema.max <= lim.q_max + tol;
}

double profile_pos_overshoot(const AxisProfile & prof, const AxisLimits & lim)
{
  double over = 0.0;
  if (std::isfinite(lim.q_min)) {
    over = std::max(over, lim.q_min - prof.extrema.min);
  }
  if (std::isfinite(lim.q_max)) {
    over = std::max(over, prof.extrema.max - lim.q_max);
  }
  return over;
}

// ---------------- Exact 1-DOF Velocity-Transition Solver (2nd-order in v, a) ----------------

// Minimum-time 3-phase ramp (v0, a0) -> (vf, af) under a in [a_min, a_max], |j| <= j_max.
struct VelRamp3
{
  bool ok = false;
  double duration = 0.0;
  double dq = 0.0;
  double v_min_seen = 0.0;
  double v_max_seen = 0.0;
  std::array<double, 3> dt{};
  std::array<double, 3> j{};
};

VelRamp3 compute_vel_ramp_3phase(
  double v0, double a0, double vf, double af,
  double a_min, double a_max, double j_max)
{
  VelRamp3 r;
  const double dv = vf - v0;
  const double dv_sw = 0.5 * std::abs(af - a0) * (af + a0) / j_max;

  if (dv >= dv_sw - 1e-14) {
    // UP-HOLD-DOWN: (+j, 0 at a_max, -j)
    const double disc = 0.5 * (a0 * a0 + af * af) + j_max * std::max(0.0, dv - dv_sw + 0.5 * std::abs(af - a0) * (af + a0) / j_max);
    // Exact: a_p^2 = 0.5*(a0^2 + af^2) + j*dv
    const double ap2 = std::max(0.0, 0.5 * (a0 * a0 + af * af) + j_max * dv);
    (void)disc;
    const double ap = std::sqrt(ap2);
    if (ap <= a_max + 1e-12) {
      const double a_peak = std::min(ap, a_max);
      r.dt[0] = std::max(0.0, (a_peak - a0) / j_max);
      r.dt[1] = 0.0;
      r.dt[2] = std::max(0.0, (a_peak - af) / j_max);
    } else {
      r.dt[0] = std::max(0.0, (a_max - a0) / j_max);
      r.dt[2] = std::max(0.0, (a_max - af) / j_max);
      const double dv_ramps = (2.0 * a_max * a_max - a0 * a0 - af * af) / (2.0 * j_max);
      r.dt[1] = std::max(0.0, (dv - dv_ramps) / a_max);
    }
    r.j = {{j_max, 0.0, -j_max}};
  } else {
    // DOWN-HOLD-UP: (-j, 0 at a_min, +j)
    const double ap2 = std::max(0.0, 0.5 * (a0 * a0 + af * af) - j_max * dv);
    const double ap = -std::sqrt(ap2);
    if (ap >= a_min - 1e-12) {
      const double a_peak = std::max(ap, a_min);
      r.dt[0] = std::max(0.0, (a0 - a_peak) / j_max);
      r.dt[1] = 0.0;
      r.dt[2] = std::max(0.0, (af - a_peak) / j_max);
    } else {
      r.dt[0] = std::max(0.0, (a0 - a_min) / j_max);
      r.dt[2] = std::max(0.0, (af - a_min) / j_max);
      const double dv_ramps = -(2.0 * a_min * a_min - a0 * a0 - af * af) / (2.0 * j_max);
      r.dt[1] = std::max(0.0, (dv - dv_ramps) / a_min);  // both negative -> positive
    }
    r.j = {{-j_max, 0.0, j_max}};
  }

  double q = 0.0, v = v0, a = a0;
  r.v_min_seen = v0;
  r.v_max_seen = v0;
  r.duration = 0.0;
  for (size_t k = 0; k < 3; ++k) {
    const double h = r.dt[k];
    if (h < -1e-11 || !std::isfinite(h)) {
      return r;
    }
    const double jk = r.j[k];
    double q1 = 0.0, v1 = 0.0, a1 = 0.0;
    integrate_step(h, q, v, a, jk, q1, v1, a1);
    if (std::abs(jk) > 1e-14 && ((a > 0.0 && a1 < 0.0) || (a < 0.0 && a1 > 0.0))) {
      const double tz = -a / jk;
      if (tz > 0.0 && tz < h) {
        const double ve = v + tz * (a + 0.5 * jk * tz);
        r.v_min_seen = std::min(r.v_min_seen, ve);
        r.v_max_seen = std::max(r.v_max_seen, ve);
      }
    }
    r.v_min_seen = std::min(r.v_min_seen, v1);
    r.v_max_seen = std::max(r.v_max_seen, v1);
    q = q1;
    v = v1;
    a = a1;
    r.duration += h;
  }
  r.dq = q;
  r.ok = std::abs(v - vf) <= 1e-6 * (1.0 + std::abs(v0) + std::abs(vf)) &&
         std::abs(a - af) <= 1e-6 * (1.0 + std::abs(a0) + std::abs(af));
  return r;
}

// Builds the exact 7-phase cruise profile at constant middle velocity v3 (a3 = 0)
// over exact total duration T (valid whenever T_L(v3) + T_R(v3) <= T).
bool build_v3_cruise_profile(
  const AxisState & s0, const AxisState & sf, const AxisLimits & lim,
  double T, double v3,
  double & out_dq, AxisProfile * out_prof)
{
  const VelRamp3 left = compute_vel_ramp_3phase(
    s0.v, s0.a, v3, 0.0, lim.a_min, lim.a_max, lim.j_max);
  const VelRamp3 right = compute_vel_ramp_3phase(
    v3, 0.0, sf.v, sf.a, lim.a_min, lim.a_max, lim.j_max);
  if (!left.ok || !right.ok) {
    return false;
  }
  const double v_tol = 1e-6 * (1.0 + std::max(std::abs(lim.v_min), std::abs(lim.v_max)));
  if (left.v_min_seen < lim.v_min - v_tol || left.v_max_seen > lim.v_max + v_tol ||
      right.v_min_seen < lim.v_min - v_tol || right.v_max_seen > lim.v_max + v_tol)
  {
    return false;
  }
  const double t3 = T - left.duration - right.duration;
  if (t3 < -1e-9) {
    return false;
  }
  const double t3_pos = std::max(0.0, t3);
  out_dq = left.dq + v3 * t3_pos + right.dq;
  if (out_prof != nullptr) {
    std::array<double, 7> dt{{
      left.dt[0], left.dt[1], left.dt[2], t3_pos, right.dt[0], right.dt[1], right.dt[2]}};
    std::array<double, 7> j{{
      left.j[0], left.j[1], left.j[2], 0.0, right.j[0], right.j[1], right.j[2]}};
    AxisState sf_eff = sf;
    sf_eff.q = s0.q + out_dq;
    if (!build_and_check_7phase(s0, sf_eff, lim, dt, j, T, *out_prof)) {
      return false;
    }
  }
  return true;
}

// ---------------- Exact Closed-Form Max Displacement at Fixed Duration T (UDDU) ----------------

// Computes the exact maximum displacement dp_max(T) and its achieving UDDU 7-phase profile
// for fixed duration T >= T_min_vel under asymmetric [v_min, v_max], [a_min, a_max], j_max.
bool solve_max_disp_uddu(
  const AxisState & s0, const AxisState & sf, const AxisLimits & lim,
  double T, double & out_dp_max, AxisProfile & out_prof)
{
  const double j = lim.j_max;
  const double aU = lim.a_max;
  const double aL = lim.a_min;
  const double vU = lim.v_max;
  const double a0 = s0.a, af = sf.a;
  const double dv = sf.v - s0.v;

  bool found = false;
  double best_dq = -std::numeric_limits<double>::infinity();
  AxisProfile best_prof;

  auto consider_7 = [&](const std::array<double, 7> & dt, const std::array<double, 7> & jr) {
    // Check duration, integrate directly, validate velocity & acceleration & final (v, a).
    double sum_t = 0.0;
    for (double d : dt) {
      if (!std::isfinite(d) || d < -1e-10) {
        return;
      }
      sum_t += std::max(0.0, d);
    }
    if (std::abs(sum_t - T) > 1e-6 * (1.0 + T)) {
      return;
    }
    double q = s0.q, v = s0.v, a = s0.a;
    for (size_t k = 0; k < 7; ++k) {
      const double h = std::max(0.0, dt[k]);
      integrate_step(h, q, v, a, jr[k], q, v, a);
    }
    AxisState sf_cand = sf;
    sf_cand.q = q;
    AxisProfile cand;
    if (!build_and_check_7phase(s0, sf_cand, lim, dt, jr, T, cand)) {
      return;
    }
    const double dq = q - s0.q;
    if (!found || dq > best_dq) {
      found = true;
      best_dq = dq;
      best_prof = std::move(cand);
    }
  };

  // Case 1: Velocity plateau at v3 = v_max (highest possible velocity cruise).
  {
    double dq_v = 0.0;
    AxisProfile pv;
    if (build_v3_cruise_profile(s0, sf, lim, T, vU, dq_v, &pv)) {
      out_dp_max = dq_v;
      out_prof = std::move(pv);
      return true;
    }
  }

  // Case 0: Direct single-ramp or constant-accel-only when T == T_min_vel.
  {
    const VelRamp3 vmin = compute_vel_ramp_3phase(s0.v, s0.a, sf.v, sf.a, aL, aU, j);
    if (vmin.ok && std::abs(vmin.duration - T) <= 1e-6 * (1.0 + T)) {
      consider_7(
        {{vmin.dt[0], vmin.dt[1], vmin.dt[2], 0.0, 0.0, 0.0, 0.0}},
        {{vmin.j[0], vmin.j[1], vmin.j[2], 0.0, 0.0, 0.0, 0.0}});
    }
  }

  // Cases 2a-2d: UDDU with t3 = 0: (+j, 0 at a1, -j, 0, -j, 0 at a5, +j).
  const double H = j * T + a0 - af;
  const double K = j * dv + 0.5 * (a0 * a0 - af * af);
  const std::array<double, 7> j_uddu{{j, 0.0, -j, 0.0, -j, 0.0, j}};

  if (H > 1e-12) {
    // Case 2a: t1 = 0, t5 = 0 (pure U-D-U)
    {
      const double d = 0.5 * H;
      const double s = K / d;
      const double a1 = 0.5 * (d + s);
      const double a5 = 0.5 * (s - d);
      const double t0 = (a1 - a0) / j;
      const double tau = d / j;
      const double t6 = (af - a5) / j;
      consider_7({{t0, 0.0, tau, 0.0, 0.0, 0.0, t6}}, j_uddu);
    }

    // Case 2b: t1 >= 0 (a1 = aU), t5 = 0
    {
      const double rad = aU * H - K;
      if (rad >= -1e-12 * (1.0 + std::abs(aU * H) + std::abs(K))) {
        const double sr = std::sqrt(std::max(0.0, rad));
        const double a5 = aU - sr;
        const double t0 = (aU - a0) / j;
        const double tau = sr / j;
        const double t6 = (af - a5) / j;
        const double t1 = (H - 2.0 * sr) / j;
        consider_7({{t0, t1, tau, 0.0, 0.0, 0.0, t6}}, j_uddu);
      }
    }

    // Case 2c: t1 = 0, t5 >= 0 (a5 = aL)
    {
      const double rad = K - aL * H;
      if (rad >= -1e-12 * (1.0 + std::abs(aL * H) + std::abs(K))) {
        const double sr = std::sqrt(std::max(0.0, rad));
        const double a1 = aL + sr;
        const double t0 = (a1 - a0) / j;
        const double tau = sr / j;
        const double t6 = (af - aL) / j;
        const double t5 = (H - 2.0 * sr) / j;
        consider_7({{t0, 0.0, tau, 0.0, 0.0, t5, t6}}, j_uddu);
      }
    }

    // Case 2d: t1 >= 0 (a1 = aU), t5 >= 0 (a5 = aL)
    {
      const double t0 = (aU - a0) / j;
      const double tau = (aU - aL) / j;
      const double t6 = (af - aL) / j;
      const double St = (H - 2.0 * (aU - aL)) / j;
      const double Sv = (K - (aU * aU - aL * aL)) / j;
      const double t1 = (Sv - aL * St) / (aU - aL);
      const double t5 = (aU * St - Sv) / (aU - aL);
      consider_7({{t0, t1, tau, 0.0, 0.0, t5, t6}}, j_uddu);
    }
  }

  // Case 2e: Edge case where maximizing displacement momentarily touches lower velocity bound v_min
  // (only possible if T is long enough to cruise at v_min while a0, af push downward).
  if (!found) {
    double dq_vl = 0.0;
    AxisProfile pvl;
    if (build_v3_cruise_profile(s0, sf, lim, T, lim.v_min, dq_vl, &pvl)) {
      found = true;
      best_dq = dq_vl;
      best_prof = std::move(pvl);
    }
  }

  if (found) {
    out_dp_max = best_dq;
    out_prof = std::move(best_prof);
  }
  return found;
}

// Exact convex combination of two feasible AxisProfiles of identical duration T:
// q(t) = (1 - alpha) * pA(t) + alpha * pB(t).
// By convexity of box bounds on (v, a, j), guaranteed valid when durations and endpoints match;
// validates endpoint state and kinematic bounds explicitly to guard against duration mismatch drift.
bool blend_profiles_convex(
  const AxisProfile & pA, const AxisProfile & pB, double alpha,
  const AxisState & sf, const AxisLimits & lim,
  AxisProfile & out_prof)
{
  alpha = std::clamp(alpha, 0.0, 1.0);
  const double T = pA.duration;
  if (std::abs(pB.duration - T) > 1e-6 * (1.0 + T)) {
    return false;
  }

  std::vector<double> knots;
  knots.reserve(pA.t_cum.size() + pB.t_cum.size());
  for (double t : pA.t_cum) {
    knots.push_back(t);
  }
  for (double t : pB.t_cum) {
    knots.push_back(std::clamp(t, 0.0, T));
  }
  std::sort(knots.begin(), knots.end());
  knots.erase(
    std::unique(knots.begin(), knots.end(), [](double x, double y) { return std::abs(x - y) <= 1e-11; }),
    knots.end());
  if (knots.size() < 2) {
    out_prof = pA;
  } else {
    knots.front() = 0.0;
    knots.back() = T;

    double qA0 = 0.0, vA0 = 0.0, aA0 = 0.0, jA0 = 0.0;
    double qB0 = 0.0, vB0 = 0.0, aB0 = 0.0, jB0 = 0.0;
    pA.eval(0.0, qA0, vA0, aA0, jA0);
    pB.eval(0.0, qB0, vB0, aB0, jB0);

    double q = (1.0 - alpha) * qA0 + alpha * qB0;
    double v = (1.0 - alpha) * vA0 + alpha * vB0;
    double a = (1.0 - alpha) * aA0 + alpha * aB0;

    out_prof.clear();
    out_prof.phases.reserve(knots.size());
    for (size_t i = 0; i + 1 < knots.size(); ++i) {
      const double dt = knots[i + 1] - knots[i];
      if (dt <= 1e-13) {
        continue;
      }
      const double t_mid = 0.5 * (knots[i] + knots[i + 1]);
      double qa = 0.0, va = 0.0, aa = 0.0, ja = 0.0;
      double qb = 0.0, vb = 0.0, ab = 0.0, jb = 0.0;
      pA.eval(t_mid, qa, va, aa, ja);
      pB.eval(t_mid, qb, vb, ab, jb);
      const double j_mix = (1.0 - alpha) * ja + alpha * jb;
      out_prof.append_phase(dt, q, v, a, j_mix);
      integrate_step(dt, q, v, a, j_mix, q, v, a);
    }
    out_prof.finalize();
  }

  // Re-validate terminal (q, v, a) against sf and kinematic bounds across every phase.
  const double q_scale = 1.0 + std::abs(sf.q) + (out_prof.phases.empty() ? 0.0 : std::abs(out_prof.phases.front().q0));
  const double v_scale = 1.0 + std::max(std::abs(lim.v_min), std::abs(lim.v_max));
  const double a_scale = 1.0 + std::max(std::abs(lim.a_min), std::abs(lim.a_max));
  if (!std::isfinite(out_prof.qf) || !std::isfinite(out_prof.vf) || !std::isfinite(out_prof.af) ||
      std::abs(out_prof.qf - sf.q) > 1e-5 * q_scale ||
      std::abs(out_prof.vf - sf.v) > 1e-5 * v_scale ||
      std::abs(out_prof.af - sf.a) > 1e-5 * a_scale)
  {
    return false;
  }
  const double v_tol = 1e-5 * v_scale;
  const double a_tol = 1e-5 * a_scale;
  const double j_tol = 1e-5 * (1.0 + lim.j_max);
  for (const auto & ph : out_prof.phases) {
    if (std::abs(ph.j) > lim.j_max + j_tol) {
      return false;
    }
    double q1 = 0.0, v1 = 0.0, a1 = 0.0;
    integrate_step(ph.dt, ph.q0, ph.v0, ph.a0, ph.j, q1, v1, a1);
    if (ph.v0 < lim.v_min - v_tol || ph.v0 > lim.v_max + v_tol ||
        v1 < lim.v_min - v_tol || v1 > lim.v_max + v_tol ||
        ph.a0 < lim.a_min - a_tol || ph.a0 > lim.a_max + a_tol ||
        a1 < lim.a_min - a_tol || a1 > lim.a_max + a_tol)
    {
      return false;
    }
    if (std::abs(ph.j) > 1e-12) {
      const double tau_ext = -ph.a0 / ph.j;
      if (tau_ext > 0.0 && tau_ext < ph.dt) {
        const double v_ext = ph.v0 + ph.a0 * tau_ext + 0.5 * ph.j * tau_ext * tau_ext;
        if (v_ext < lim.v_min - v_tol || v_ext > lim.v_max + v_tol) {
          return false;
        }
      }
    }
  }
  return true;
}

// ---------------- Step 1: Exact Unconstrained 1-DOF Extremal Enumeration ----------------

// Enumerates all valid UDDU Step 1 extremal solutions for the '+' branch and appends
// them to candidates.
void enumerate_step1_uddu(
  const AxisState & s0, const AxisState & sf, const AxisLimits & lim,
  std::vector<AxisProfile> & candidates)
{
  const double j = lim.j_max;
  const double aU = lim.a_max;
  const double aL = lim.a_min;
  const double vU = lim.v_max;
  const double p0 = s0.q, v0 = s0.v, a0 = s0.a;
  const double pf = sf.q, vf = sf.v, af = sf.a;
  const double pd = pf - p0;
  const double dv = vf - v0;
  const std::array<double, 7> j_uddu{{j, 0.0, -j, 0.0, -j, 0.0, j}};

  auto try_add = [&](const std::array<double, 7> & dt) {
    AxisProfile cand;
    if (build_and_check_7phase(s0, sf, lim, dt, j_uddu, std::nullopt, cand)) {
      candidates.push_back(std::move(cand));
    }
  };

  // 1. Velocity-cruise families: ACC0_ACC1_VEL, ACC0_VEL, ACC1_VEL, VEL.
  //    Left half (v0, a0) -> (vU, 0) and right half (vU, 0) -> (vf, af) are decoupled.
  {
    const VelRamp3 left = compute_vel_ramp_3phase(v0, a0, vU, 0.0, aL, aU, j);
    const VelRamp3 right = compute_vel_ramp_3phase(vU, 0.0, vf, af, aL, aU, j);
    if (left.ok && right.ok && std::abs(vU) > 1e-12) {
      const double t3 = (pd - left.dq - right.dq) / vU;
      if (t3 >= -1e-10) {
        std::array<double, 7> dt{{
          left.dt[0], left.dt[1], left.dt[2], std::max(0.0, t3),
          right.dt[0], right.dt[1], right.dt[2]}};
        std::array<double, 7> jr{{
          left.j[0], left.j[1], left.j[2], 0.0,
          right.j[0], right.j[1], right.j[2]}};
        AxisProfile cand;
        if (build_and_check_7phase(s0, sf, lim, dt, jr, std::nullopt, cand)) {
          candidates.push_back(std::move(cand));
        }
      }
    }
  }

  // 2. ACC0_ACC1 (t3 = 0, t1 >= 0, t5 >= 0, a1 = aU, a5 = aL): exact quadratic in t1.
  {
    const double t0 = (aU - a0) / j;
    const double tau = (aU - aL) / j;
    const double t6 = (af - aL) / j;
    if (t0 >= -1e-11 && tau >= -1e-11 && t6 >= -1e-11) {
      const double t0c = std::max(0.0, t0);
      const double tauc = std::max(0.0, tau);
      const double t6c = std::max(0.0, t6);
      double p1 = 0.0, v1 = 0.0, a1_chk = 0.0;
      integrate_step(t0c, p0, v0, a0, j, p1, v1, a1_chk);

      const double dv_ramps = (aU * aU - a0 * a0 + aU * aU - aL * aL + af * af - aL * aL) / (2.0 * j);
      const double dv_rem = dv - dv_ramps;
      // aU * t1 + aL * t5 = dv_rem  =>  t5 = (dv_rem - aU * t1) / aL
      const Poly<2> pt1(0.0, 1.0);
      const Poly<2> pt5(dv_rem / aL, -aU / aL);
      const Poly<2> pv2 = Poly<2>(v1) + aU * pt1;
      const Poly<2> pv5 = pv2 + Poly<2>((aU * aU - aL * aL) / (2.0 * j));
      const Poly<2> pv6 = pv5 + aL * pt5;

      const Poly<2> dp1 = v1 * pt1 + (0.5 * aU) * (pt1 * pt1);
      const Poly<2> dp24 = tauc * pv2 + Poly<2>(0.5 * aU * tauc * tauc - (j / 6.0) * tauc * tauc * tauc);
      const Poly<2> dp5 = pv5 * pt5 + (0.5 * aL) * (pt5 * pt5);
      const Poly<2> dp6 = t6c * pv6 + Poly<2>(0.5 * aL * t6c * t6c + (j / 6.0) * t6c * t6c * t6c);

      const Poly<2> eq = dp1 + dp24 + dp5 + dp6 - Poly<2>(pf - p1);
      RootList roots;
      solve_quadratic(eq.c[2], eq.c[1], eq.c[0], roots);
      for (int i = 0; i < roots.n; ++i) {
        const double t1 = roots.r[i];
        const double t5 = (dv_rem - aU * t1) / aL;
        try_add({{t0c, t1, tauc, 0.0, 0.0, t5, t6c}});
      }
    }
  }

  // 3. ACC0 (t3 = 0, t5 = 0, t1 >= 0, a1 = aU, unknown x = a5): exact quartic in x.
  {
    const double t0 = (aU - a0) / j;
    if (t0 >= -1e-11) {
      const double t0c = std::max(0.0, t0);
      double p1 = 0.0, v1 = 0.0, a1_chk = 0.0;
      integrate_step(t0c, p0, v0, a0, j, p1, v1, a1_chk);

      const double C1 = (vf - v1) / aU - (aU * aU + af * af) / (2.0 * j * aU);
      const Poly<4> px(0.0, 1.0);
      const Poly<4> pt1(C1, 0.0, 1.0 / (j * aU));
      const Poly<4> ptau(aU / j, -1.0 / j);
      const Poly<4> pt6(af / j, -1.0 / j);
      const Poly<4> pv2 = Poly<4>(v1) + aU * pt1;
      const Poly<4> pv5(vf - af * af / (2.0 * j), 0.0, 1.0 / (2.0 * j));

      const Poly<4> dp1 = v1 * pt1 + (0.5 * aU) * (pt1 * pt1);
      const Poly<4> dp24 = pv2 * ptau + (0.5 * aU) * (ptau * ptau) - (j / 6.0) * (ptau * ptau * ptau);
      const Poly<4> dp6 = pv5 * pt6 + 0.5 * (px * pt6 * pt6) + (j / 6.0) * (pt6 * pt6 * pt6);

      const Poly<4> eq = dp1 + dp24 + dp6 - Poly<4>(pf - p1);
      RootList roots;
      solve_quartic(eq.c[4], eq.c[3], eq.c[2], eq.c[1], eq.c[0], roots);
      for (int i = 0; i < roots.n; ++i) {
        const double a5 = roots.r[i];
        const double t1 = C1 + (a5 * a5) / (j * aU);
        const double tau = (aU - a5) / j;
        const double t6 = (af - a5) / j;
        try_add({{t0c, t1, tau, 0.0, 0.0, 0.0, t6}});
      }
    }
  }

  // 4. NONE (t1 = t3 = t5 = 0, pure U-D-U): exact depressed quartic in d = a1 - a5 >= 0.
  {
    const double K = j * dv + 0.5 * (a0 * a0 - af * af);
    // Degenerate d = 0 (single +j ramp from a0 to af)
    if (af >= a0 - 1e-11) {
      const double t0 = std::max(0.0, (af - a0) / j);
      try_add({{t0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}});
    }
    const double c2 = 4.0 * j * (v0 + vf) - 2.0 * (a0 * a0 + af * af);
    const double c1 = (2.0 / 3.0) * (
      6.0 * j * (vf * af - v0 * a0) + 2.0 * (a0 * a0 * a0 - af * af * af) - 6.0 * j * j * pd);
    const double c0 = -K * K;
    RootList roots;
    solve_quartic(1.0, 0.0, c2, c1, c0, roots);
    for (int i = 0; i < roots.n; ++i) {
      const double d = roots.r[i];
      if (d > 1e-11) {
        const double s = K / d;
        const double a1 = 0.5 * (d + s);
        const double a5 = 0.5 * (s - d);
        const double t0 = (a1 - a0) / j;
        const double tau = d / j;
        const double t6 = (af - a5) / j;
        try_add({{t0, 0.0, tau, 0.0, 0.0, 0.0, t6}});
      }
    }
  }
}

// Helper for ACC1 via exact time-reversal + negation of ACC0 only (avoids re-doing VEL/ACC0_ACC1/NONE).
void enumerate_step1_uddu_acc1(
  const AxisState & s0, const AxisState & sf, const AxisLimits & lim,
  std::vector<AxisProfile> & candidates)
{
  AxisState r0, rf;
  AxisLimits rlim;
  time_reverse_negate_problem(s0, sf, lim, r0, rf, rlim);

  const double j = rlim.j_max;
  const double aU = rlim.a_max;
  const double p0 = r0.q, v0 = r0.v, a0 = r0.a;
  const double pf = rf.q, vf = rf.v, af = rf.a;
  const double t0 = (aU - a0) / j;
  if (t0 < -1e-11) {
    return;
  }
  const double t0c = std::max(0.0, t0);
  double p1 = 0.0, v1 = 0.0, a1_chk = 0.0;
  integrate_step(t0c, p0, v0, a0, j, p1, v1, a1_chk);

  const double C1 = (vf - v1) / aU - (aU * aU + af * af) / (2.0 * j * aU);
  const Poly<4> px(0.0, 1.0);
  const Poly<4> pt1(C1, 0.0, 1.0 / (j * aU));
  const Poly<4> ptau(aU / j, -1.0 / j);
  const Poly<4> pt6(af / j, -1.0 / j);
  const Poly<4> pv2 = Poly<4>(v1) + aU * pt1;
  const Poly<4> pv5(vf - af * af / (2.0 * j), 0.0, 1.0 / (2.0 * j));

  const Poly<4> dp1 = v1 * pt1 + (0.5 * aU) * (pt1 * pt1);
  const Poly<4> dp24 = pv2 * ptau + (0.5 * aU) * (ptau * ptau) - (j / 6.0) * (ptau * ptau * ptau);
  const Poly<4> dp6 = pv5 * pt6 + 0.5 * (px * pt6 * pt6) + (j / 6.0) * (pt6 * pt6 * pt6);

  const Poly<4> eq = dp1 + dp24 + dp6 - Poly<4>(pf - p1);
  RootList roots;
  solve_quartic(eq.c[4], eq.c[3], eq.c[2], eq.c[1], eq.c[0], roots);
  const std::array<double, 7> j_uddu{{j, 0.0, -j, 0.0, -j, 0.0, j}};
  for (int i = 0; i < roots.n; ++i) {
    const double a5 = roots.r[i];
    const double t1 = C1 + (a5 * a5) / (j * aU);
    const double tau = (aU - a5) / j;
    const double t6 = (af - a5) / j;
    AxisProfile rev_cand;
    if (build_and_check_7phase(
          r0, rf, rlim, {{t0c, t1, tau, 0.0, 0.0, 0.0, t6}}, j_uddu, std::nullopt, rev_cand))
    {
      candidates.push_back(time_reverse_negate_profile(rev_cand));
    }
  }
}

// ---------------- Step 2: Exact Unconstrained 1-DOF Fixed-Duration Families ----------------

// Family A: Reduced-jerk U-D-U (t1 = t3 = t5 = 0, free j_f in (0, j_max]) on '+' branch.
void try_step2_reduced_jerk_none_uddu(
  const AxisState & s0, const AxisState & sf, const AxisLimits & lim, double T,
  std::vector<AxisProfile> & out_cands)
{
  const double pd = sf.q - s0.q;
  const double dv = sf.v - s0.v;
  const double da = sf.a - s0.a;
  const double W_num = 2.0 * dv - (s0.a + sf.a) * T;

  auto test_jf = [&](double jf) {
    if (!(jf > 1e-9) || jf > lim.j_max * (1.0 + 1e-6)) {
      return;
    }
    const double tau = 0.5 * (T - da / jf);
    const double s = 0.5 * (T + da / jf);
    if (tau < -1e-10 || s < -1e-10) {
      return;
    }
    const double w = (std::abs(2.0 * jf * tau) > 1e-14) ? (W_num / (2.0 * jf * tau)) : 0.0;
    const double t0 = 0.5 * (s + w);
    const double t6 = 0.5 * (s - w);
    AxisProfile cand;
    if (build_and_check_7phase(
          s0, sf, lim, {{t0, 0.0, tau, 0.0, 0.0, 0.0, t6}},
          {{jf, 0.0, -jf, 0.0, -jf, 0.0, jf}}, T, cand))
    {
      out_cands.push_back(std::move(cand));
    }
  };

  if (std::abs(da) <= 1e-12) {
    // tau = T/2, s = T/2, w = (W_num / T) / jf = W_j * x where x = 1/jf.
    const double Wj = W_num / T;
    const Poly<3> pt0(0.25 * T, 0.5 * Wj);
    const Poly<3> pt6(0.25 * T, -0.5 * Wj);
    const Poly<3> px(0.0, 1.0);
    // Multiply dp(x) - pd by x:
    //   x * a1 = x * s0.a + 0.5 * Wj * x + 0.25 * T
    //   x * (dp0 + dp24 + dp6) computed cleanly via x-scaled polynomials.
    const Poly<3> x_a1(0.25 * T, s0.a + 0.5 * Wj);
    const Poly<3> x_a5(-0.25 * T, s0.a + 0.5 * Wj);
    // dp0 = v0*t0 + 0.5*a0*t0^2 + (1/(6x))*t0^3  => x*dp0 = x*v0*t0 + 0.5*a0*x*t0^2 + (1/6)*t0^3
    const Poly<3> x_dp0 = s0.v * (px * pt0) + (0.5 * s0.a) * (px * pt0 * pt0) +
      (1.0 / 6.0) * (pt0 * pt0 * pt0);
    // v1 = v0 + a0*t0 + (1/(2x))*t0^2 => x*v1 = x*v0 + a0*x*t0 + 0.5*t0^2
    const Poly<3> x_v1 = s0.v * px + s0.a * (px * pt0) + 0.5 * (pt0 * pt0);
    const double tau = 0.5 * T;
    // x*dp24 = tau*(x*v1) + 0.5*tau^2*(x*a1) - (1/6)*tau^3
    const Poly<3> x_dp24 = tau * x_v1 + (0.5 * tau * tau) * x_a1 - Poly<3>((1.0 / 6.0) * tau * tau * tau);
    // x*v5 = x*v1 + tau*(x*a1) - 0.5*tau^2
    const Poly<3> x_v5 = x_v1 + tau * x_a1 - Poly<3>(0.5 * tau * tau);
    // x*dp6 = (x*v5)*t6 + 0.5*(x*a5)*t6^2 + (1/6)*t6^3
    const Poly<3> x_dp6 = x_v5 * pt6 + 0.5 * (x_a5 * pt6 * pt6) + (1.0 / 6.0) * (pt6 * pt6 * pt6);

    const Poly<3> eq = x_dp0 + x_dp24 + x_dp6 - pd * px;
    RootList roots;
    solve_cubic(eq.c[3], eq.c[2], eq.c[1], eq.c[0], roots);
    for (int i = 0; i < roots.n; ++i) {
      if (roots.r[i] > 1e-12) {
        test_jf(1.0 / roots.r[i]);
      }
    }
  } else {
    // Parameterize by tau in (0, T): jf = da / (T - 2*tau), s = T - tau,
    // w = W_num * (T - 2*tau) / (2 * da * tau).
    // Evaluate rational residual directly on valid subintervals of tau where 0 < jf <= j_max.
    auto eval_res = [&](double tau, double & jf_out) -> double {
      const double denom_j = T - 2.0 * tau;
      if (std::abs(denom_j) < 1e-13 || tau <= 1e-12) {
        jf_out = 0.0;
        return std::numeric_limits<double>::quiet_NaN();
      }
      jf_out = da / denom_j;
      const double s = T - tau;
      const double w = W_num * denom_j / (2.0 * da * tau);
      const double t0 = 0.5 * (s + w);
      const double t6 = 0.5 * (s - w);
      double q = s0.q, v = s0.v, a = s0.a;
      integrate_step(t0, q, v, a, jf_out, q, v, a);
      integrate_step(tau, q, v, a, -jf_out, q, v, a);
      integrate_step(t6, q, v, a, jf_out, q, v, a);
      return q - sf.q;
    };

    // Valid tau domain where jf > 0 and jf <= j_max:
    // If da > 0: 0 < T - 2*tau and da/(T - 2*tau) <= j_max => tau <= 0.5*(T - da/j_max).
    // If da < 0: T - 2*tau < 0 and da/(T - 2*tau) <= j_max => tau >= 0.5*(T - da/j_max).
    double tau_lo = 1e-9, tau_hi = T - 1e-9;
    if (da > 0.0) {
      tau_hi = 0.5 * (T - da / lim.j_max);
    } else {
      tau_lo = 0.5 * (T - da / lim.j_max);
    }
    if (tau_hi > tau_lo + 1e-11) {
      constexpr int kScan = 24;
      double prev_tau = tau_lo;
      double jf_tmp = 0.0;
      double prev_f = eval_res(prev_tau, jf_tmp);
      if (std::isfinite(prev_f) && std::abs(prev_f) <= 1e-10) {
        test_jf(jf_tmp);
      }
      for (int k = 1; k <= kScan; ++k) {
        const double curr_tau = tau_lo + (tau_hi - tau_lo) * static_cast<double>(k) / kScan;
        double curr_jf = 0.0;
        const double curr_f = eval_res(curr_tau, curr_jf);
        if (std::isfinite(curr_f) && std::abs(curr_f) <= 1e-10) {
          test_jf(curr_jf);
        }
        if (std::isfinite(prev_f) && std::isfinite(curr_f) &&
            ((prev_f < 0.0 && curr_f > 0.0) || (prev_f > 0.0 && curr_f < 0.0)))
        {
          const double r_tau = brent_root(
            [&](double t_try) {
              double j_dummy = 0.0;
              return eval_res(t_try, j_dummy);
            },
            prev_tau, curr_tau, prev_f, curr_f);
          double r_jf = 0.0;
          eval_res(r_tau, r_jf);
          test_jf(r_jf);
        }
        prev_tau = curr_tau;
        prev_f = curr_f;
      }
    }
  }
}

// Family B: Full-jerk velocity-plateau sweep v3 in [v_min, v_max] covering UDDU, UDUD, DUDU, DUUD.
void try_step2_v3_cruise_sweep(
  const AxisState & s0, const AxisState & sf, const AxisLimits & lim, double T,
  std::vector<AxisProfile> & out_cands)
{
  const double pd = sf.q - s0.q;

  auto slack_at = [&](double v3, double & out_dq) -> double {
    const VelRamp3 left = compute_vel_ramp_3phase(s0.v, s0.a, v3, 0.0, lim.a_min, lim.a_max, lim.j_max);
    const VelRamp3 right = compute_vel_ramp_3phase(v3, 0.0, sf.v, sf.a, lim.a_min, lim.a_max, lim.j_max);
    if (!left.ok || !right.ok) {
      out_dq = 0.0;
      return -1.0;
    }
    const double v_tol = 1e-6 * (1.0 + std::max(std::abs(lim.v_min), std::abs(lim.v_max)));
    if (left.v_min_seen < lim.v_min - v_tol || left.v_max_seen > lim.v_max + v_tol ||
        right.v_min_seen < lim.v_min - v_tol || right.v_max_seen > lim.v_max + v_tol)
    {
      out_dq = 0.0;
      return -1.0;
    }
    const double t3 = T - left.duration - right.duration;
    out_dq = left.dq + v3 * std::max(0.0, t3) + right.dq;
    return t3;
  };

  // Find feasible interval [v_lo, v_hi] subset [v_min, v_max] where t3(v3) >= 0.
  // Because T_L(v3) + T_R(v3) is unimodal (convex-like with minimum near [v0, vf]),
  // sample a coarse grid + key anchor velocities to find a feasible seed, then bisect edges.
  double v_seed = 0.0;
  bool has_seed = false;
  double best_slack = -std::numeric_limits<double>::infinity();

  auto probe_seed = [&](double v3) {
    v3 = std::clamp(v3, lim.v_min, lim.v_max);
    double dq = 0.0;
    const double sl = slack_at(v3, dq);
    if (sl > best_slack) {
      best_slack = sl;
      v_seed = v3;
      if (sl >= -1e-9) {
        has_seed = true;
      }
    }
  };

  probe_seed(s0.v);
  probe_seed(sf.v);
  probe_seed(0.5 * (s0.v + sf.v));
  probe_seed(pd / std::max(T, 1e-9));
  constexpr int kProbes = 16;
  for (int i = 0; i <= kProbes; ++i) {
    probe_seed(lim.v_min + (lim.v_max - lim.v_min) * static_cast<double>(i) / kProbes);
  }
  if (!has_seed) {
    return;
  }

  // Bisect lower edge [v_min, v_seed] and upper edge [v_seed, v_max].
  double v_lo = lim.v_min;
  {
    double dq_tmp = 0.0;
    if (slack_at(lim.v_min, dq_tmp) < 0.0) {
      double a = lim.v_min, b = v_seed;
      for (int it = 0; it < 44; ++it) {
        const double m = 0.5 * (a + b);
        if (slack_at(m, dq_tmp) >= 0.0) {
          b = m;
        } else {
          a = m;
        }
      }
      v_lo = b;
    }
  }
  double v_hi = lim.v_max;
  {
    double dq_tmp = 0.0;
    if (slack_at(lim.v_max, dq_tmp) < 0.0) {
      double a = v_seed, b = lim.v_max;
      for (int it = 0; it < 44; ++it) {
        const double m = 0.5 * (a + b);
        if (slack_at(m, dq_tmp) >= 0.0) {
          a = m;
        } else {
          b = m;
        }
      }
      v_hi = a;
    }
  }

  double dq_lo = 0.0, dq_hi = 0.0;
  if (!build_v3_cruise_profile(s0, sf, lim, T, v_lo, dq_lo, nullptr) ||
      !build_v3_cruise_profile(s0, sf, lim, T, v_hi, dq_hi, nullptr))
  {
    return;
  }

  const double tol_p = 1e-7 * (1.0 + std::abs(pd) + std::abs(dq_lo) + std::abs(dq_hi));
  if (pd < std::min(dq_lo, dq_hi) - tol_p || pd > std::max(dq_lo, dq_hi) + tol_p) {
    return;
  }

  double v_star = v_lo;
  if (std::abs(dq_lo - pd) <= tol_p) {
    v_star = v_lo;
  } else if (std::abs(dq_hi - pd) <= tol_p) {
    v_star = v_hi;
  } else {
    v_star = brent_root(
      [&](double v3) {
        double dq = 0.0;
        build_v3_cruise_profile(s0, sf, lim, T, v3, dq, nullptr);
        return dq - pd;
      },
      v_lo, v_hi, dq_lo - pd, dq_hi - pd, 1e-12);
  }

  double dq_chk = 0.0;
  AxisProfile prof;
  if (build_v3_cruise_profile(s0, sf, lim, T, v_star, dq_chk, &prof) &&
      std::abs(dq_chk - pd) <= 1e-5 * (1.0 + std::abs(pd) + std::abs(s0.q) + std::abs(sf.q)))
  {
    out_cands.push_back(std::move(prof));
  }
}

// Family C: Full-jerk t3 = 0 Step 2 ACC0 (free a1, t1 >= 0, t5 = 0) on '+' branch.
void try_step2_acc0_uddu(
  const AxisState & s0, const AxisState & sf, const AxisLimits & lim, double T,
  std::vector<AxisProfile> & out_cands)
{
  const double j = lim.j_max;
  const double a0 = s0.a, af = sf.a;
  const double pd = sf.q - s0.q;
  const double dv = sf.v - s0.v;
  const double H = j * T + a0 - af;
  if (H <= 1e-11) {
    return;
  }
  const double K = j * dv + 0.5 * (a0 * a0 - af * af);
  const double invH = 1.0 / H;
  const double invj = 1.0 / j;

  // Exact polynomials in d = a1 - a5 in [0, H/2]:
  // a1(d) = (K + d^2) / H, a5(d) = (K - H*d + d^2) / H
  // t0(d) = (a1(d) - a0)/j, t1(d) = (H - 2d)/j, tau(d) = d/j, t6(d) = (af - a5(d))/j
  const Poly<6> pa1(K * invH, 0.0, invH);
  const Poly<6> pa5(K * invH, -1.0, invH);
  const Poly<6> pt0 = invj * (pa1 - Poly<6>(a0));
  const Poly<6> pt1(H * invj, -2.0 * invj);
  const Poly<6> ptau(0.0, invj);
  const Poly<6> pt6 = invj * (Poly<6>(af) - pa5);

  const Poly<6> pv1 = Poly<6>(s0.v) + (0.5 * invj) * (pa1 * pa1 - Poly<6>(a0 * a0));
  const Poly<6> pv2 = pv1 + pa1 * pt1;
  const Poly<6> pv5 = Poly<6>(sf.v - 0.5 * af * af * invj) + (0.5 * invj) * (pa5 * pa5);

  const Poly<6> dp0 = s0.v * pt0 + (0.5 * a0) * (pt0 * pt0) + (j / 6.0) * (pt0 * pt0 * pt0);
  const Poly<6> dp1 = pv1 * pt1 + 0.5 * (pa1 * pt1 * pt1);
  const Poly<6> dp24 = pv2 * ptau + 0.5 * (pa1 * ptau * ptau) - (j / 6.0) * (ptau * ptau * ptau);
  const Poly<6> dp6 = pv5 * pt6 + 0.5 * (pa5 * pt6 * pt6) + (j / 6.0) * (pt6 * pt6 * pt6);

  const Poly<6> eq = dp0 + dp1 + dp24 + dp6 - Poly<6>(pd);
  RootList roots;
  solve_poly_on_interval<6>(eq, 0.0, 0.5 * H, roots);

  const std::array<double, 7> j_uddu{{j, 0.0, -j, 0.0, -j, 0.0, j}};
  for (int i = 0; i < roots.n; ++i) {
    const double d = roots.r[i];
    const double a1 = (K + d * d) * invH;
    const double a5 = a1 - d;
    const double t0 = (a1 - a0) * invj;
    const double t1 = (H - 2.0 * d) * invj;
    const double tau = d * invj;
    const double t6 = (af - a5) * invj;
    AxisProfile cand;
    if (build_and_check_7phase(s0, sf, lim, {{t0, t1, tau, 0.0, 0.0, 0.0, t6}}, j_uddu, T, cand)) {
      out_cands.push_back(std::move(cand));
    }
  }
}

// Family C2: Full-jerk t3 = 0 Step 2 ACC0_ACC1 (a1 = aU fixed, a5 free) on '+' branch.
void try_step2_acc0_acc1_uddu(
  const AxisState & s0, const AxisState & sf, const AxisLimits & lim, double T,
  std::vector<AxisProfile> & out_cands)
{
  const double j = lim.j_max;
  const double aU = lim.a_max;
  const double aL = lim.a_min;
  const double a0 = s0.a, af = sf.a;
  const double dv = sf.v - s0.v;
  const double H = j * T + a0 - af;
  const double K = j * dv + 0.5 * (a0 * a0 - af * af);
  const double t0 = (aU - a0) / j;
  if (H <= 1e-11 || t0 < -1e-11) {
    return;
  }
  const double t0c = std::max(0.0, t0);
  const double C = aU * H - K;
  if (C <= 1e-12) {
    return;
  }

  // Domain of x = aU - a5 > 0:
  //   t5(x) = (C - x^2) / (j*x) >= 0  =>  x <= sqrt(C)
  //   t1(x) = (H*x - x^2 - C) / (j*x) >= 0  =>  x in roots of x^2 - H*x + C <= 0
  const double disc = H * H - 4.0 * C;
  if (disc < -1e-12 * (1.0 + H * H)) {
    return;
  }
  const double sd = std::sqrt(std::max(0.0, disc));
  double x_lo = std::max({1e-9, 0.5 * (H - sd), aU - af});
  double x_hi = std::min({std::sqrt(C), 0.5 * (H + sd), aU - aL});
  if (x_hi < x_lo - 1e-9) {
    return;
  }
  x_hi = std::max(x_hi, x_lo);

  const std::array<double, 7> j_uddu{{j, 0.0, -j, 0.0, -j, 0.0, j}};
  auto eval_dp = [&](double x) -> double {
    const double a5 = aU - x;
    const double t1 = (H * x - x * x - C) / (j * x);
    const double tau = x / j;
    const double t5 = (C - x * x) / (j * x);
    const double t6 = (af - a5) / j;
    double q = s0.q, v = s0.v, a = s0.a;
    integrate_step(t0c, q, v, a, j, q, v, a);
    integrate_step(t1, q, v, a, 0.0, q, v, a);
    integrate_step(tau, q, v, a, -j, q, v, a);
    integrate_step(t5, q, v, a, 0.0, q, v, a);
    integrate_step(t6, q, v, a, j, q, v, a);
    return q - sf.q;
  };
  auto test_x = [&](double x) {
    const double a5 = aU - x;
    const double t1 = (H * x - x * x - C) / (j * x);
    const double tau = x / j;
    const double t5 = (C - x * x) / (j * x);
    const double t6 = (af - a5) / j;
    AxisProfile cand;
    if (build_and_check_7phase(s0, sf, lim, {{t0c, t1, tau, 0.0, 0.0, t5, t6}}, j_uddu, T, cand)) {
      out_cands.push_back(std::move(cand));
    }
  };

  constexpr int kScan = 16;
  double prev_x = x_lo;
  double prev_f = eval_dp(prev_x);
  if (std::abs(prev_f) <= 1e-9) {
    test_x(prev_x);
  }
  for (int k = 1; k <= kScan; ++k) {
    const double curr_x = x_lo + (x_hi - x_lo) * static_cast<double>(k) / kScan;
    const double curr_f = eval_dp(curr_x);
    if (std::abs(curr_f) <= 1e-9) {
      test_x(curr_x);
    }
    if ((prev_f < 0.0 && curr_f > 0.0) || (prev_f > 0.0 && curr_f < 0.0)) {
      const double xr = brent_root(eval_dp, prev_x, curr_x, prev_f, curr_f);
      test_x(xr);
    }
    prev_x = curr_x;
    prev_f = curr_f;
  }
}

// ---------------- Brent 1D Minimization Helper (for Continuous Pontryagin Bounce) ----------------

template <typename Fn>
double brent_minimize(Fn && f, double ax, double bx, double cx, double tol, double & out_fmin)
{
  constexpr double kCGold = 0.3819660112501051;
  double a = std::min(ax, cx);
  double b = std::max(ax, cx);
  double x = bx, w = bx, v = bx;
  double fx = f(x), fw = fx, fv = fx;
  double d = 0.0, e = 0.0;

  for (int iter = 0; iter < 40; ++iter) {
    const double xm = 0.5 * (a + b);
    const double tol1 = tol * std::abs(x) + 1e-11;
    const double tol2 = 2.0 * tol1;
    if (std::abs(x - xm) <= tol2 - 0.5 * (b - a)) {
      break;
    }
    bool use_golden = true;
    if (std::abs(e) > tol1 && std::isfinite(fx) && std::isfinite(fw) && std::isfinite(fv)) {
      const double r = (x - w) * (fx - fv);
      double q = (x - v) * (fx - fw);
      double p = (x - v) * q - (x - w) * r;
      q = 2.0 * (q - r);
      if (q > 0.0) {
        p = -p;
      }
      q = std::abs(q);
      const double etemp = e;
      e = d;
      if (std::abs(p) < std::abs(0.5 * q * etemp) && p > q * (a - x) && p < q * (b - x)) {
        d = p / q;
        const double u = x + d;
        if (u - a < tol2 || b - u < tol2) {
          d = (xm - x >= 0.0) ? tol1 : -tol1;
        }
        use_golden = false;
      }
    }
    if (use_golden) {
      e = (x >= xm) ? (a - x) : (b - x);
      d = kCGold * e;
    }
    const double u = (std::abs(d) >= tol1) ? (x + d) : (x + (d >= 0.0 ? tol1 : -tol1));
    const double fu = f(u);
    if (fu <= fx) {
      if (u >= x) {
        a = x;
      } else {
        b = x;
      }
      v = w; fv = fw;
      w = x; fw = fx;
      x = u; fx = fu;
    } else {
      if (u < x) {
        a = u;
      } else {
        b = u;
      }
      if (fu <= fw || w == x) {
        v = w; fv = fw;
        w = u; fw = fu;
      } else if (fu <= fv || v == x || v == w) {
        v = u; fv = fu;
      }
    }
  }
  out_fmin = fx;
  return x;
}

}  // namespace

// ======================== AxisProfile Public Methods ========================

void AxisProfile::clear()
{
  phases.clear();
  t_cum.clear();
  duration = 0.0;
  qf = vf = af = 0.0;
  extrema = PositionExtrema{};
}

void AxisProfile::append_phase(double dt, double q0, double v0, double a0, double j)
{
  phases.push_back({std::max(0.0, dt), q0, v0, a0, j});
}

void AxisProfile::append_profile(const AxisProfile & other)
{
  for (const auto & ph : other.phases) {
    phases.push_back(ph);
  }
}

void AxisProfile::finalize()
{
  t_cum.clear();
  t_cum.reserve(phases.size() + 1);
  t_cum.push_back(0.0);
  extrema = PositionExtrema{};
  duration = 0.0;
  if (phases.empty()) {
    qf = vf = af = 0.0;
    return;
  }
  double q = phases.front().q0;
  double v = phases.front().v0;
  double a = phases.front().a0;
  for (const auto & ph : phases) {
    update_phase_extrema(duration, ph.dt, ph.q0, ph.v0, ph.a0, ph.j, extrema);
    integrate_step(ph.dt, ph.q0, ph.v0, ph.a0, ph.j, q, v, a);
    duration += ph.dt;
    t_cum.push_back(duration);
  }
  qf = q;
  vf = v;
  af = a;
}

void AxisProfile::eval(double t, double & q, double & v, double & a, double & j) const
{
  if (phases.empty()) {
    q = v = a = j = 0.0;
    return;
  }
  if (t <= 0.0) {
    const auto & p = phases.front();
    q = p.q0;
    v = p.v0;
    a = p.a0;
    j = p.j;
    return;
  }
  if (t >= duration) {
    q = qf;
    v = vf;
    a = af;
    j = 0.0;
    return;
  }
  const auto it = std::upper_bound(t_cum.begin(), t_cum.end(), t);
  size_t idx = (it == t_cum.begin()) ? 0 : static_cast<size_t>(std::distance(t_cum.begin(), it) - 1);
  idx = std::min(idx, phases.size() - 1);
  const auto & p = phases[idx];
  const double tau = std::clamp(t - t_cum[idx], 0.0, p.dt);
  integrate_step(tau, p.q0, p.v0, p.a0, p.j, q, v, a);
  j = p.j;
}

// ======================== Braking Excursions & State Validation ========================

double min_upper_excursion(double q0, double v0, double a0, double a_min, double j_max)
{
  const double a_brake = std::abs(a_min);
  if (v0 <= 0.0 && a0 <= 0.0) {
    return q0;
  }
  double q = q0;
  double v = v0;
  double a = a0;

  if (a > 0.0) {
    const double t0 = a / j_max;
    const double v_apex = v + 0.5 * a * t0;
    if (v_apex <= 0.0) {
      return q0;
    }
    q += v * t0 + (a * a * a) / (3.0 * j_max * j_max);
    v = v_apex;
    a = 0.0;
  }

  // Now a <= 0 and v > 0; brake downward with j = -j_max down to a = -a_brake.
  // When v0 < 0 and a0 > 0 with v_apex > 0, q first dips below q0 before rising to q_stop,
  // so the maximum excursion over [0, t_stop] is max(q0, q_stop).
  const double t1 = std::max(0.0, (a + a_brake) / j_max);
  const double v1 = v + a * t1 - 0.5 * j_max * t1 * t1;
  if (v1 <= 0.0) {
    const double disc = std::max(0.0, a * a + 2.0 * j_max * v);
    const double t_star = std::clamp((a + std::sqrt(disc)) / j_max, 0.0, t1);
    const double q_stop = q + v * t_star + 0.5 * a * t_star * t_star -
      (1.0 / 6.0) * j_max * t_star * t_star * t_star;
    return std::max(q0, q_stop);
  }
  const double q1 = q + v * t1 + 0.5 * a * t1 * t1 - (1.0 / 6.0) * j_max * t1 * t1 * t1;
  return std::max(q0, q1 + (v1 * v1) / (2.0 * a_brake));
}

double max_lower_excursion(double q0, double v0, double a0, double a_max, double j_max)
{
  return -min_upper_excursion(-q0, -v0, -a0, -std::abs(a_max), j_max);
}

double max_speed_for_distance(double d, double a_brake, double j_max)
{
  if (d <= 0.0) {
    return 0.0;
  }
  const double ab = std::abs(a_brake);
  const double d_crit = (ab * ab * ab) / (j_max * j_max);
  if (d <= d_crit) {
    return std::cbrt(d * d * j_max);
  }
  const double half_b = 0.5 * (ab * ab) / j_max;
  return std::max(0.0, -half_b + std::sqrt(half_b * half_b + 2.0 * ab * d));
}

bool validate_axis_state(
  const AxisState & st,
  const AxisLimits & lim,
  bool check_forward,
  bool check_reverse,
  const char ** out_error_tag)
{
  auto fail = [&](const char * tag) {
    if (out_error_tag != nullptr) {
      *out_error_tag = tag;
    }
    return false;
  };

  if (!std::isfinite(st.q) || !std::isfinite(st.v) || !std::isfinite(st.a)) {
    return fail("NONFINITE");
  }
  if (st.q < lim.q_min - kPosCheckTol || st.q > lim.q_max + kPosCheckTol) {
    return fail("POS_LIMIT");
  }
  if (st.v < lim.v_min * (1.0 + 1e-6) - 1e-9 || st.v > lim.v_max * (1.0 + 1e-6) + 1e-9) {
    return fail("VEL_LIMIT");
  }
  if (st.a < lim.a_min * (1.0 + 1e-6) - 1e-9 || st.a > lim.a_max * (1.0 + 1e-6) + 1e-9) {
    return fail("ACC_LIMIT");
  }
  if (check_forward) {
    if ((st.a > 0.0 && st.v + (st.a * st.a) / (2.0 * lim.j_max) > lim.v_max * (1.0 + 1e-5)) ||
        (st.a < 0.0 && st.v - (st.a * st.a) / (2.0 * lim.j_max) < lim.v_min * (1.0 + 1e-5)))
    {
      return fail("VEL_REACH_FWD");
    }
    if (lim.has_pos_limits()) {
      const double q_hi = min_upper_excursion(st.q, st.v, st.a, lim.a_min, lim.j_max);
      const double q_lo = max_lower_excursion(st.q, st.v, st.a, lim.a_max, lim.j_max);
      if (q_hi > lim.q_max + kPosCheckTol || q_lo < lim.q_min - kPosCheckTol) {
        return fail("POS_BRAKE_FWD");
      }
    }
  }
  if (check_reverse) {
    // Reverse time r(s) = q(T - s): v_rev = -v, a_rev = +a, v_rev in [-v_max, -v_min], a_rev in [a_min, a_max].
    if ((st.a < 0.0 && st.v + (st.a * st.a) / (2.0 * lim.j_max) > lim.v_max * (1.0 + 1e-5)) ||
        (st.a > 0.0 && st.v - (st.a * st.a) / (2.0 * lim.j_max) < lim.v_min * (1.0 + 1e-5)))
    {
      return fail("VEL_REACH_REV");
    }
    if (lim.has_pos_limits()) {
      const double q_hi_rev = min_upper_excursion(st.q, -st.v, st.a, lim.a_min, lim.j_max);
      const double q_lo_rev = max_lower_excursion(st.q, -st.v, st.a, lim.a_max, lim.j_max);
      if (q_hi_rev > lim.q_max + kPosCheckTol || q_lo_rev < lim.q_min - kPosCheckTol) {
        return fail("POS_BRAKE_REV");
      }
    }
  }
  return true;
}

void project_state_feasible(
  double q,
  double & v,
  double & a,
  const AxisLimits & lim,
  double headroom)
{
  const double v_lo = lim.v_min * headroom;
  const double v_hi = lim.v_max * headroom;
  const double a_lo = lim.a_min * headroom;
  const double a_hi = lim.a_max * headroom;
  const double j_w = lim.j_max * headroom;

  a = std::clamp(a, a_lo, a_hi);
  const double dv_a = (a * a) / (2.0 * j_w);
  const double v_min_a = v_lo + dv_a;
  const double v_max_a = v_hi - dv_a;
  if (v_min_a <= v_max_a) {
    v = std::clamp(v, v_min_a, v_max_a);
  } else {
    // Acceleration alone exceeds half velocity span: shrink a.
    const double a_cap = std::sqrt(std::max(0.0, j_w * (v_hi - v_lo)));
    a = std::clamp(a, -a_cap, a_cap);
    v = 0.5 * (v_lo + v_hi);
  }

  if (!lim.has_pos_limits()) {
    return;
  }

  auto braking_ok = [&](double vt, double at) -> bool {
    const double q_hi_f = min_upper_excursion(q, vt, at, a_lo, j_w);
    const double q_lo_f = max_lower_excursion(q, vt, at, a_hi, j_w);
    const double q_hi_r = min_upper_excursion(q, -vt, at, a_lo, j_w);
    const double q_lo_r = max_lower_excursion(q, -vt, at, a_hi, j_w);
    return q_hi_f <= lim.q_max && q_lo_f >= lim.q_min &&
           q_hi_r <= lim.q_max && q_lo_r >= lim.q_min;
  };

  if (braking_ok(v, a)) {
    return;
  }
  // Radial contraction toward (0, 0) (which is always braking-feasible inside [q_min, q_max]).
  double lo = 0.0, hi = 1.0;
  for (int it = 0; it < 24; ++it) {
    const double mid = 0.5 * (lo + hi);
    if (braking_ok(mid * v, mid * a)) {
      lo = mid;
    } else {
      hi = mid;
    }
  }
  v *= lo;
  a *= lo;
}

// ======================== Velocity Interface Solvers ========================

bool solve_axis_velocity_min_time(
  const AxisState & s0, double vf, double af, const AxisLimits & lim,
  AxisProfile & out_profile)
{
  const VelRamp3 r = compute_vel_ramp_3phase(s0.v, s0.a, vf, af, lim.a_min, lim.a_max, lim.j_max);
  if (!r.ok) {
    return false;
  }
  const double v_tol = 1e-6 * (1.0 + std::max(std::abs(lim.v_min), std::abs(lim.v_max)));
  if (r.v_min_seen < lim.v_min - v_tol || r.v_max_seen > lim.v_max + v_tol) {
    return false;
  }
  out_profile.clear();
  double q = s0.q, v = s0.v, a = s0.a;
  for (size_t k = 0; k < 3; ++k) {
    if (r.dt[k] > 1e-13) {
      out_profile.append_phase(r.dt[k], q, v, a, r.j[k]);
      integrate_step(r.dt[k], q, v, a, r.j[k], q, v, a);
    }
  }
  if (out_profile.phases.empty()) {
    out_profile.append_phase(0.0, s0.q, s0.v, s0.a, 0.0);
  }
  out_profile.finalize();
  return true;
}

bool solve_axis_velocity_at_duration(
  const AxisState & s0, double vf, double af, const AxisLimits & lim,
  double T_target, AxisProfile & out_profile)
{
  AxisProfile min_p;
  if (!solve_axis_velocity_min_time(s0, vf, af, lim, min_p)) {
    return false;
  }
  if (T_target < min_p.duration - kTimeTol) {
    return false;
  }
  if (std::abs(T_target - min_p.duration) <= kTimeTol) {
    out_profile = std::move(min_p);
    return true;
  }
  // Exact 3-phase reduced-acceleration profile matching (v0, a0) -> (vf, af) in exact time T_target.
  const double j = lim.j_max;
  const double a0 = s0.a;
  const double dv = vf - s0.v;
  // Try UP-HOLD-DOWN with free plateau a1 in [max(a0, af), a_max]:
  // t0 = (a1 - a0)/j, t2 = (a1 - af)/j, t1 = T - (2*a1 - a0 - af)/j
  // dv = (2*a1^2 - a0^2 - af^2)/(2j) + a1*t1
  //    => a1^2 - (j*T + a0 + af)*a1 + (j*dv + 0.5*(a0^2 + af^2)) = 0
  auto try_dir = [&](double j_sgn) -> bool {
    const double jv = j_sgn * j;
    const double B = -(jv * T_target + a0 + af);
    const double C = jv * dv + 0.5 * (a0 * a0 + af * af);
    RootList roots;
    solve_quadratic(1.0, B, C, roots);
    for (int i = 0; i < roots.n; ++i) {
      const double a1 = roots.r[i];
      if (a1 < lim.a_min - 1e-7 || a1 > lim.a_max + 1e-7) {
        continue;
      }
      const double t0 = (a1 - a0) / jv;
      const double t2 = (a1 - af) / jv;
      const double t1 = T_target - t0 - t2;
      if (t0 >= -1e-10 && t1 >= -1e-10 && t2 >= -1e-10) {
        AxisProfile cand;
        double q = s0.q, v = s0.v, a = s0.a;
        const std::array<double, 3> dts{{std::max(0.0, t0), std::max(0.0, t1), std::max(0.0, t2)}};
        const std::array<double, 3> jks{{jv, 0.0, -jv}};
        const double v_tol = 1e-6 * (1.0 + std::max(std::abs(lim.v_min), std::abs(lim.v_max)));
        bool ok = (v >= lim.v_min - v_tol && v <= lim.v_max + v_tol);
        for (size_t k = 0; k < 3 && ok; ++k) {
          if (dts[k] > 1e-13) {
            if (std::abs(jks[k]) > 1e-12) {
              const double tau_ext = -a / jks[k];
              if (tau_ext > 0.0 && tau_ext < dts[k]) {
                const double v_ext = v + a * tau_ext + 0.5 * jks[k] * tau_ext * tau_ext;
                if (v_ext < lim.v_min - v_tol || v_ext > lim.v_max + v_tol) {
                  ok = false;
                  break;
                }
              }
            }
            cand.append_phase(dts[k], q, v, a, jks[k]);
            integrate_step(dts[k], q, v, a, jks[k], q, v, a);
            if (v < lim.v_min - v_tol || v > lim.v_max + v_tol) {
              ok = false;
            }
          }
        }
        if (ok && std::abs(v - vf) <= 1e-5 && std::abs(a - af) <= 1e-5) {
          cand.finalize();
          out_profile = std::move(cand);
          return true;
        }
      }
    }
    return false;
  };
  if (try_dir(1.0) || try_dir(-1.0)) {
    return true;
  }
  // Fallback: exact velocity cruise at af=0 if reachable, or hold vf after min_p if af==0.
  if (std::abs(af) <= 1e-10) {
    out_profile = min_p;
    out_profile.append_phase(T_target - min_p.duration, min_p.qf, vf, 0.0, 0.0);
    out_profile.finalize();
    return true;
  }
  return false;
}

// ======================== Displacement Bounds & Unconstrained Step 1 / Step 2 ========================

bool compute_displacement_bounds(
  const AxisState & s0, const AxisState & sf, const AxisLimits & lim, double T,
  double & out_dp_min, double & out_dp_max,
  AxisProfile * out_min_prof, AxisProfile * out_max_prof)
{
  if (!(T >= 0.0) || !std::isfinite(T)) {
    return false;
  }
  const VelRamp3 vmin = compute_vel_ramp_3phase(
    s0.v, s0.a, sf.v, sf.a, lim.a_min, lim.a_max, lim.j_max);
  if (!vmin.ok || T < vmin.duration - 1e-7) {
    return false;
  }

  AxisProfile p_max;
  if (!solve_max_disp_uddu(s0, sf, lim, T, out_dp_max, p_max)) {
    return false;
  }

  AxisState n0, nf;
  AxisLimits nlim;
  negate_problem(s0, sf, lim, n0, nf, nlim);
  double neg_dp_max = 0.0;
  AxisProfile p_min_neg;
  if (!solve_max_disp_uddu(n0, nf, nlim, T, neg_dp_max, p_min_neg)) {
    return false;
  }
  out_dp_min = -neg_dp_max;
  if (out_dp_min > out_dp_max + 1e-6) {
    return false;
  }
  if (out_dp_min > out_dp_max) {
    const double mid = 0.5 * (out_dp_min + out_dp_max);
    out_dp_min = out_dp_max = mid;
  }
  if (out_min_prof != nullptr) {
    negate_profile(p_min_neg);
    *out_min_prof = std::move(p_min_neg);
  }
  if (out_max_prof != nullptr) {
    *out_max_prof = std::move(p_max);
  }
  return true;
}

Step1Result solve_axis_unconstrained_step1(
  const AxisState & s0, const AxisState & sf, const AxisLimits & lim)
{
  Step1Result res;
  // Trivial zero-motion stationary case (v0 = vf = 0, a0 = af = 0).
  if (std::abs(sf.q - s0.q) < 1e-11 && std::abs(s0.v) < 1e-10 && std::abs(sf.v) < 1e-10 &&
      std::abs(s0.a) < 1e-10 && std::abs(sf.a) < 1e-10)
  {
    res.ok = true;
    res.t_min = 0.0;
    res.min_profile.append_phase(0.0, s0.q, s0.v, s0.a, 0.0);
    res.min_profile.finalize();
    return res;
  }

  // Build unconstrained limits copy for algebraic Step 1 enumeration.
  AxisLimits ulim = lim;
  ulim.q_min = -std::numeric_limits<double>::infinity();
  ulim.q_max = std::numeric_limits<double>::infinity();

  std::vector<AxisProfile> cands;
  cands.reserve(24);

  // Zero-duration match when s0 == sf with non-zero (v, a): still enumerate non-zero loop
  // roots so the physical blocked interval (0, T_loop) is detected accurately.
  if (std::abs(sf.q - s0.q) < 1e-11 && std::abs(sf.v - s0.v) < 1e-10 &&
      std::abs(sf.a - s0.a) < 1e-10)
  {
    AxisProfile z;
    z.append_phase(0.0, s0.q, s0.v, s0.a, 0.0);
    z.finalize();
    cands.push_back(std::move(z));
  }

  // '+' branch (UDDU)
  enumerate_step1_uddu(s0, sf, ulim, cands);
  enumerate_step1_uddu_acc1(s0, sf, ulim, cands);

  // '-' branch (DUUD) via exact negation symmetry
  {
    AxisState n0, nf;
    AxisLimits nlim;
    negate_problem(s0, sf, ulim, n0, nf, nlim);
    std::vector<AxisProfile> ncands;
    ncands.reserve(16);
    enumerate_step1_uddu(n0, nf, nlim, ncands);
    enumerate_step1_uddu_acc1(n0, nf, nlim, ncands);
    for (auto & p : ncands) {
      negate_profile(p);
      cands.push_back(std::move(p));
    }
  }

  // Also check velocity-minimum-time profile in case pd happens to equal its displacement.
  {
    const VelRamp3 vmin = compute_vel_ramp_3phase(
      s0.v, s0.a, sf.v, sf.a, ulim.a_min, ulim.a_max, ulim.j_max);
    if (vmin.ok && std::abs(vmin.dq - (sf.q - s0.q)) <= 1e-6 * (1.0 + std::abs(sf.q - s0.q))) {
      AxisProfile vp;
      if (build_and_check_7phase(
            s0, sf, ulim,
            {{vmin.dt[0], vmin.dt[1], vmin.dt[2], 0.0, 0.0, 0.0, 0.0}},
            {{vmin.j[0], vmin.j[1], vmin.j[2], 0.0, 0.0, 0.0, 0.0}},
            std::nullopt, vp))
      {
        cands.push_back(std::move(vp));
      }
    }
  }

  // Fallback reachability scan if algebraic enumeration missed a near-degenerate root.
  const double pd = sf.q - s0.q;
  if (cands.empty()) {
    const VelRamp3 vmin = compute_vel_ramp_3phase(
      s0.v, s0.a, sf.v, sf.a, ulim.a_min, ulim.a_max, ulim.j_max);
    const double t0_scan = vmin.ok ? vmin.duration : 0.0;
    double lo = t0_scan;
    double hi = std::max(0.05, t0_scan + 0.05);
    while (hi < kMaxInternalDur) {
      double dp_lo = 0.0, dp_hi = 0.0;
      if (compute_displacement_bounds(s0, sf, ulim, hi, dp_lo, dp_hi) &&
          pd >= dp_lo - 1e-7 && pd <= dp_hi + 1e-7)
      {
        break;
      }
      lo = hi;
      hi *= 1.5;
    }
    if (hi < kMaxInternalDur) {
      for (int it = 0; it < 48; ++it) {
        const double mid = 0.5 * (lo + hi);
        double dp_lo = 0.0, dp_hi = 0.0;
        if (compute_displacement_bounds(s0, sf, ulim, mid, dp_lo, dp_hi) &&
            pd >= dp_lo - 1e-7 && pd <= dp_hi + 1e-7)
        {
          hi = mid;
        } else {
          lo = mid;
        }
      }
      AxisProfile p_sol;
      if (solve_axis_unconstrained_step2(s0, sf, ulim, hi, p_sol)) {
        cands.push_back(std::move(p_sol));
      }
    }
  }

  if (cands.empty()) {
    return res;
  }

  std::sort(
    cands.begin(), cands.end(),
    [](const AxisProfile & a, const AxisProfile & b) { return a.duration < b.duration; });

  res.ok = true;
  res.t_min = cands.front().duration;
  res.min_profile = cands.front();

  // Deduplicate critical extremal durations T_1 < T_2 < ... < T_r.
  std::vector<double> crits;
  crits.reserve(cands.size());
  for (const auto & p : cands) {
    if (crits.empty() || p.duration - crits.back() > 1e-5) {
      crits.push_back(p.duration);
    }
  }

  // Exact blocked-interval classification between consecutive critical durations
  // using exact reachability bounds [dp_min(T_mid), dp_max(T_mid)].
  for (size_t i = 0; i + 1 < crits.size(); ++i) {
    const double t_mid = 0.5 * (crits[i] + crits[i + 1]);
    double dp_lo = 0.0, dp_hi = 0.0;
    const bool reachable =
      compute_displacement_bounds(s0, sf, ulim, t_mid, dp_lo, dp_hi) &&
      pd >= dp_lo - 1e-6 && pd <= dp_hi + 1e-6;
    if (!reachable) {
      if (!res.blocked_intervals.empty() &&
          std::abs(res.blocked_intervals.back().second - crits[i]) <= 1e-5)
      {
        res.blocked_intervals.back().second = crits[i + 1];
      } else {
        res.blocked_intervals.emplace_back(crits[i], crits[i + 1]);
      }
    }
  }

  return res;
}

bool solve_axis_unconstrained_step2(
  const AxisState & s0, const AxisState & sf, const AxisLimits & lim,
  double T_target, AxisProfile & out_profile)
{
  if (!(T_target >= 0.0) || !std::isfinite(T_target) || T_target > kMaxInternalDur) {
    return false;
  }
  if (std::abs(sf.q - s0.q) < 1e-11 && std::abs(s0.v) < 1e-10 && std::abs(sf.v) < 1e-10 &&
      std::abs(s0.a) < 1e-10 && std::abs(sf.a) < 1e-10)
  {
    out_profile.clear();
    out_profile.append_phase(T_target, s0.q, 0.0, 0.0, 0.0);
    out_profile.finalize();
    return true;
  }

  AxisLimits ulim = lim;
  ulim.q_min = -std::numeric_limits<double>::infinity();
  ulim.q_max = std::numeric_limits<double>::infinity();

  // 1. Exact reachability check: pd must lie in [dp_min(T), dp_max(T)].
  const double pd = sf.q - s0.q;
  double dp_min = 0.0, dp_max = 0.0;
  AxisProfile p_min, p_max;
  if (!compute_displacement_bounds(s0, sf, ulim, T_target, dp_min, dp_max, &p_min, &p_max)) {
    return false;
  }
  const double reach_tol = 1e-6 * (1.0 + std::abs(pd) + std::abs(dp_min) + std::abs(dp_max));
  if (pd < dp_min - reach_tol || pd > dp_max + reach_tol) {
    return false;
  }

  // Exact boundary hits: return the extremal profile immediately.
  if (std::abs(pd - dp_max) <= 1e-8 * (1.0 + std::abs(pd))) {
    out_profile = std::move(p_max);
    return true;
  }
  if (std::abs(pd - dp_min) <= 1e-8 * (1.0 + std::abs(pd))) {
    out_profile = std::move(p_min);
    return true;
  }

  std::vector<AxisProfile> cands;
  cands.reserve(12);

  // 2. Family A: Smooth reduced-jerk NONE (both '+' and '-' branches).
  try_step2_reduced_jerk_none_uddu(s0, sf, ulim, T_target, cands);
  {
    AxisState n0, nf;
    AxisLimits nlim;
    negate_problem(s0, sf, ulim, n0, nf, nlim);
    std::vector<AxisProfile> nc;
    try_step2_reduced_jerk_none_uddu(n0, nf, nlim, T_target, nc);
    for (auto & p : nc) {
      negate_profile(p);
      cands.push_back(std::move(p));
    }
  }

  // 3. Family B: Exact full-jerk velocity-plateau sweep (UDDU / UDUD / DUDU / DUUD).
  try_step2_v3_cruise_sweep(s0, sf, ulim, T_target, cands);

  // 4. Family C: Full-jerk t3 = 0 ACC0 / ACC1 / ACC0_ACC1 (when needed or to find
  //    a non-overshooting branch if lim.has_pos_limits() is true).
  auto has_good_in_bounds = [&]() -> bool {
    if (cands.empty()) {
      return false;
    }
    if (!lim.has_pos_limits()) {
      return true;
    }
    for (const auto & c : cands) {
      if (profile_within_pos(c, lim)) {
        return true;
      }
    }
    return false;
  };

  if (!has_good_in_bounds()) {
    // '+' ACC0 and ACC0_ACC1
    try_step2_acc0_uddu(s0, sf, ulim, T_target, cands);
    try_step2_acc0_acc1_uddu(s0, sf, ulim, T_target, cands);
    // '+' ACC1 and ACC1_ACC0 via time-reversal + negation
    {
      AxisState r0, rf;
      AxisLimits rlim;
      time_reverse_negate_problem(s0, sf, ulim, r0, rf, rlim);
      std::vector<AxisProfile> rc;
      try_step2_acc0_uddu(r0, rf, rlim, T_target, rc);
      try_step2_acc0_acc1_uddu(r0, rf, rlim, T_target, rc);
      for (const auto & p : rc) {
        cands.push_back(time_reverse_negate_profile(p));
      }
    }
    // '-' branch via negation
    {
      AxisState n0, nf;
      AxisLimits nlim;
      negate_problem(s0, sf, ulim, n0, nf, nlim);
      std::vector<AxisProfile> nc;
      try_step2_acc0_uddu(n0, nf, nlim, T_target, nc);
      try_step2_acc0_acc1_uddu(n0, nf, nlim, T_target, nc);
      AxisState nr0, nrf;
      AxisLimits nrlim;
      time_reverse_negate_problem(n0, nf, nlim, nr0, nrf, nrlim);
      std::vector<AxisProfile> nrc;
      try_step2_acc0_uddu(nr0, nrf, nrlim, T_target, nrc);
      try_step2_acc0_acc1_uddu(nr0, nrf, nrlim, T_target, nrc);
      for (const auto & p : nrc) {
        nc.push_back(time_reverse_negate_profile(p));
      }
      for (auto & p : nc) {
        negate_profile(p);
        cands.push_back(std::move(p));
      }
    }
  }

  // 5. Exact convex combination completeness fallback between p_min(T) and p_max(T).
  //    Also added if existing candidates overshoot position limits while convex blend does not!
  if (!has_good_in_bounds()) {
    const double span = dp_max - dp_min;
    const double alpha = (span > 1e-14) ? std::clamp((pd - dp_min) / span, 0.0, 1.0) : 0.5;
    AxisProfile blended;
    if (blend_profiles_convex(p_min, p_max, alpha, sf, ulim, blended)) {
      cands.push_back(std::move(blended));
    }
  }

  if (cands.empty()) {
    return false;
  }

  // Select best candidate:
  //   1) Strictly within [q_min, q_max] preferred over overshooting;
  //   2) Among in-bounds candidates, prefer lowest peak jerk (smoothest profile);
  //   3) Among overshooting candidates, prefer smallest position overshoot.
  size_t best_idx = 0;
  bool best_in = !lim.has_pos_limits() || profile_within_pos(cands[0], lim);
  double best_over = lim.has_pos_limits() ? profile_pos_overshoot(cands[0], lim) : 0.0;
  double best_jerk = profile_peak_jerk(cands[0]);

  for (size_t i = 1; i < cands.size(); ++i) {
    const bool in = !lim.has_pos_limits() || profile_within_pos(cands[i], lim);
    const double over = lim.has_pos_limits() ? profile_pos_overshoot(cands[i], lim) : 0.0;
    const double jrk = profile_peak_jerk(cands[i]);
    if (in && !best_in) {
      best_idx = i; best_in = true; best_over = over; best_jerk = jrk;
    } else if (in && best_in) {
      if (jrk < best_jerk - 1e-6) {
        best_idx = i; best_over = over; best_jerk = jrk;
      }
    } else if (!in && !best_in && over < best_over - 1e-9) {
      best_idx = i; best_over = over; best_jerk = jrk;
    }
  }

  out_profile = std::move(cands[best_idx]);
  return true;
}

// ======================== Continuous Pontryagin Position-Constrained 1-DOF ========================

namespace
{

// Evaluates the 2-part bounded minimum-time split through boundary contact state (q_lim, 0, a_b).
bool eval_boundary_split_min_time(
  const AxisState & s0, const AxisState & sf, const AxisLimits & lim,
  double q_lim, double a_b, int depth,
  AxisProfile & out_prof, double & out_dur)
{
  const AxisState mid{q_lim, 0.0, a_b};
  if (!validate_axis_state(mid, lim, true, true)) {
    return false;
  }
  AxisProfile p1, p2;
  bool c1 = false, c2 = false;
  if (!solve_axis_bounded_min_time(s0, mid, lim, p1, &c1, depth + 1)) {
    return false;
  }
  if (!solve_axis_bounded_min_time(mid, sf, lim, p2, &c2, depth + 1)) {
    return false;
  }
  if (!profile_within_pos(p1, lim) || !profile_within_pos(p2, lim)) {
    return false;
  }
  out_prof = std::move(p1);
  out_prof.append_profile(p2);
  out_prof.finalize();
  out_dur = out_prof.duration;
  return true;
}

bool solve_axis_bounded_min_time_impl(
  const AxisState & s0, const AxisState & sf, const AxisLimits & lim,
  AxisProfile & out_profile, bool * out_clamped, double * out_best_ab, int depth)
{
  if (out_clamped != nullptr) {
    *out_clamped = false;
  }
  if (out_best_ab != nullptr) {
    *out_best_ab = 0.0;
  }
  const Step1Result s1 = solve_axis_unconstrained_step1(s0, sf, lim);
  if (!s1.ok) {
    return false;
  }
  if (!lim.has_pos_limits() || profile_within_pos(s1.min_profile, lim)) {
    out_profile = s1.min_profile;
    return true;
  }
  if (depth >= kMaxBoundDepth) {
    return false;
  }

  // Determine first violated boundary in time.
  const bool hi_vio = std::isfinite(lim.q_max) && (s1.min_profile.extrema.max > lim.q_max + kPosCheckTol);
  const bool lo_vio = std::isfinite(lim.q_min) && (s1.min_profile.extrema.min < lim.q_min - kPosCheckTol);
  double q_lim = lim.q_max;
  double a_edge = lim.a_min;  // inward acceleration bound (< 0 at q_max)
  if (hi_vio && lo_vio) {
    if (s1.min_profile.extrema.t_min < s1.min_profile.extrema.t_max) {
      q_lim = lim.q_min;
      a_edge = lim.a_max;
    }
  } else if (lo_vio) {
    q_lim = lim.q_min;
    a_edge = lim.a_max;
  }

  // Continuous 1D Pontryagin optimization over inward contact acceleration a_b in [0, a_edge]
  // (or [a_edge, 0] for q_max). Covers both smooth boundary-arc contact (a_b = 0) and
  // strictly faster touch-and-go bounces (a_b != 0).
  const double a_lo = std::min(0.0, a_edge);
  const double a_hi = std::max(0.0, a_edge);

  constexpr int kGrid = 24;
  std::vector<double> a_pts(static_cast<size_t>(kGrid + 1), 0.0);
  std::vector<double> t_pts(static_cast<size_t>(kGrid + 1), std::numeric_limits<double>::infinity());
  bool any_feas = false;
  int best_i = -1;
  double best_t = std::numeric_limits<double>::infinity();
  double best_ab = 0.0;
  AxisProfile best_prof;

  for (int i = 0; i <= kGrid; ++i) {
    const double ab = a_lo + (a_hi - a_lo) * static_cast<double>(i) / kGrid;
    a_pts[static_cast<size_t>(i)] = ab;
    AxisProfile tmp;
    double dur = 0.0;
    if (eval_boundary_split_min_time(s0, sf, lim, q_lim, ab, depth, tmp, dur)) {
      t_pts[static_cast<size_t>(i)] = dur;
      if (!any_feas || dur < best_t) {
        any_feas = true;
        best_i = i;
        best_t = dur;
        best_ab = ab;
        best_prof = std::move(tmp);
      }
    }
  }

  // Adaptive dense fallback if the coarse grid missed a narrow feasible bounce window.
  if (!any_feas) {
    constexpr int kDenseGrid = 64;
    a_pts.resize(static_cast<size_t>(kDenseGrid + 1));
    t_pts.assign(static_cast<size_t>(kDenseGrid + 1), std::numeric_limits<double>::infinity());
    for (int i = 0; i <= kDenseGrid; ++i) {
      const double ab = a_lo + (a_hi - a_lo) * static_cast<double>(i) / kDenseGrid;
      a_pts[static_cast<size_t>(i)] = ab;
      AxisProfile tmp;
      double dur = 0.0;
      if (eval_boundary_split_min_time(s0, sf, lim, q_lim, ab, depth, tmp, dur)) {
        t_pts[static_cast<size_t>(i)] = dur;
        if (!any_feas || dur < best_t) {
          any_feas = true;
          best_i = i;
          best_t = dur;
          best_ab = ab;
          best_prof = std::move(tmp);
        }
      }
    }
  }

  if (!any_feas) {
    return false;
  }

  // Refine continuous a_b around best_i using Brent's method (and tighten infeasible
  // neighbor edges by bisection so boundary optima are captured accurately).
  const int last_idx = static_cast<int>(a_pts.size()) - 1;
  double b_lo = (best_i > 0) ? a_pts[static_cast<size_t>(best_i - 1)] : a_lo;
  double b_hi = (best_i < last_idx) ? a_pts[static_cast<size_t>(best_i + 1)] : a_hi;
  const double ab_mid = a_pts[static_cast<size_t>(best_i)];

  auto cost_fn = [&](double ab) -> double {
    AxisProfile tmp;
    double dur = 0.0;
    if (!eval_boundary_split_min_time(s0, sf, lim, q_lim, ab, depth, tmp, dur)) {
      return std::numeric_limits<double>::infinity();
    }
    if (dur < best_t) {
      best_t = dur;
      best_ab = ab;
      best_prof = std::move(tmp);
    }
    return dur;
  };

  if (best_i > 0 && !std::isfinite(t_pts[static_cast<size_t>(best_i - 1)])) {
    double lo = b_lo, hi = ab_mid;
    for (int it = 0; it < 12; ++it) {
      const double m = 0.5 * (lo + hi);
      if (std::isfinite(cost_fn(m))) {
        hi = m;
      } else {
        lo = m;
      }
    }
    b_lo = hi;
  }
  if (best_i < last_idx && !std::isfinite(t_pts[static_cast<size_t>(best_i + 1)])) {
    double lo = ab_mid, hi = b_hi;
    for (int it = 0; it < 12; ++it) {
      const double m = 0.5 * (lo + hi);
      if (std::isfinite(cost_fn(m))) {
        lo = m;
      } else {
        hi = m;
      }
    }
    b_hi = lo;
  }

  if (b_hi - b_lo > 1e-6 * std::abs(a_edge)) {
    const double mid = std::clamp(ab_mid, b_lo + 0.1 * (b_hi - b_lo), b_hi - 0.1 * (b_hi - b_lo));
    double fmin_unused = 0.0;
    brent_minimize(cost_fn, b_lo, mid, b_hi, 1e-5, fmin_unused);
  }

  out_profile = std::move(best_prof);
  if (out_clamped != nullptr) {
    *out_clamped = true;
  }
  if (out_best_ab != nullptr) {
    *out_best_ab = best_ab;
  }
  return true;
}

}  // namespace

bool solve_axis_bounded_min_time(
  const AxisState & s0, const AxisState & sf, const AxisLimits & lim,
  AxisProfile & out_profile, bool * out_clamped, int depth)
{
  return solve_axis_bounded_min_time_impl(s0, sf, lim, out_profile, out_clamped, nullptr, depth);
}

bool solve_axis_bounded_at_duration(
  const AxisState & s0, const AxisState & sf, const AxisLimits & lim,
  double T_target, AxisProfile & out_profile, double & out_next_feasible_T, int depth)
{
  out_profile.clear();
  out_next_feasible_T = std::numeric_limits<double>::infinity();

  if (!(T_target >= 0.0) || !std::isfinite(T_target) || T_target > kMaxInternalDur) {
    return false;
  }
  if (std::abs(sf.q - s0.q) < 1e-11 && std::abs(s0.v) < 1e-10 && std::abs(sf.v) < 1e-10 &&
      std::abs(s0.a) < 1e-10 && std::abs(sf.a) < 1e-10)
  {
    out_profile.append_phase(T_target, std::clamp(s0.q, lim.q_min, lim.q_max), 0.0, 0.0, 0.0);
    out_profile.finalize();
    return true;
  }

  // 1. Try unconstrained Step 2 (which already prefers in-bounds profiles among all valid branches).
  AxisProfile unc;
  const bool unc_ok = solve_axis_unconstrained_step2(s0, sf, lim, T_target, unc);
  if (unc_ok && (!lim.has_pos_limits() || profile_within_pos(unc, lim))) {
    out_profile = std::move(unc);
    return true;
  }

  // 2. If unconstrained Step 2 failed, T_target is below T_min or inside a physical blocked interval.
  //    Adding [q_min, q_max] can never make an unconstrained-infeasible T feasible.
  if (!unc_ok) {
    const Step1Result s1 = solve_axis_unconstrained_step1(s0, sf, lim);
    if (!s1.ok) {
      return false;
    }
    if (T_target < s1.t_min - kTimeTol) {
      out_next_feasible_T = s1.t_min;
      return false;
    }
    for (const auto & blk : s1.blocked_intervals) {
      if (T_target > blk.first - kTimeTol && T_target < blk.second - kTimeTol) {
        out_next_feasible_T = blk.second;
        return false;
      }
    }
    return false;
  }

  // 3. Here unc_ok == true, but unconstrained profile overshoots [q_min, q_max].
  if (depth >= kMaxBoundDepth) {
    return false;
  }

  // Compute exact state-constrained minimum-time trajectory first (retaining its contact a_b*).
  AxisProfile min_bounded;
  bool clamped = false;
  double min_best_ab = 0.0;
  if (!solve_axis_bounded_min_time_impl(
        s0, sf, lim, min_bounded, &clamped, &min_best_ab, depth))
  {
    return false;
  }
  if (T_target < min_bounded.duration - kTimeTol) {
    out_next_feasible_T = min_bounded.duration;
    return false;
  }
  if (std::abs(T_target - min_bounded.duration) <= kTimeTol) {
    out_profile = std::move(min_bounded);
    return true;
  }

  // Determine violated boundary from unconstrained overshoot.
  const bool hi_vio = std::isfinite(lim.q_max) && (unc.extrema.max > lim.q_max + kPosCheckTol);
  const bool lo_vio = std::isfinite(lim.q_min) && (unc.extrema.min < lim.q_min - kPosCheckTol);
  double q_lim = lim.q_max;
  double a_edge = lim.a_min;
  if (hi_vio && lo_vio) {
    if (unc.extrema.t_min < unc.extrema.t_max) {
      q_lim = lim.q_min;
      a_edge = lim.a_max;
    }
  } else if (lo_vio) {
    q_lim = lim.q_min;
    a_edge = lim.a_max;
  }

  auto try_extend_through_ab = [&](double ab) -> bool {
    const AxisState mid_st{q_lim, 0.0, ab};
    if (!validate_axis_state(mid_st, lim, true, true)) {
      return false;
    }
    AxisProfile b1, b2;
    bool bc1 = false, bc2 = false;
    if (!solve_axis_bounded_min_time(s0, mid_st, lim, b1, &bc1, depth + 1) ||
        !solve_axis_bounded_min_time(mid_st, sf, lim, b2, &bc2, depth + 1))
    {
      return false;
    }
    const double tb = b1.duration + b2.duration;
    if (tb > T_target + kTimeTol) {
      return false;
    }
    if (std::abs(tb - T_target) <= kTimeTol) {
      out_profile = std::move(b1);
      out_profile.append_profile(b2);
      out_profile.finalize();
      return true;
    }
    const double sl = std::max(0.0, T_target - tb);
    const double prop = (tb > 1e-9) ? (b1.duration / tb) : 0.5;
    for (double f : {prop, 0.5, 0.0, 1.0, 0.25, 0.75}) {
      AxisProfile e1, e2;
      double n1 = 0.0, n2 = 0.0;
      const double T1 = b1.duration + f * sl;
      const double T2 = T_target - T1;
      if (solve_axis_bounded_at_duration(s0, mid_st, lim, T1, e1, n1, depth + 1) &&
          solve_axis_bounded_at_duration(mid_st, sf, lim, T2, e2, n2, depth + 1))
      {
        out_profile = std::move(e1);
        out_profile.append_profile(e2);
        out_profile.finalize();
        return true;
      }
    }
    return false;
  };

  // Check zero-acceleration boundary arc (q_lim, 0, 0).
  const AxisState rail{q_lim, 0.0, 0.0};
  AxisProfile s1_rail, s2_rail;
  bool c1 = false, c2 = false;
  const bool rail_ok =
    validate_axis_state(rail, lim, true, true) &&
    solve_axis_bounded_min_time(s0, rail, lim, s1_rail, &c1, depth + 1) &&
    solve_axis_bounded_min_time(rail, sf, lim, s2_rail, &c2, depth + 1);

  if (rail_ok) {
    const double t_rail = s1_rail.duration + s2_rail.duration;
    if (T_target >= t_rail - kTimeTol) {
      const double slack = std::max(0.0, T_target - t_rail);
      // Try smooth extension of T1 + T2 = T_target without dwell (testing several split fractions
      // to step cleanly around any sub-segment blocked interval).
      if (slack > 1e-6) {
        const double prop = (t_rail > 1e-9) ? (s1_rail.duration / t_rail) : 0.5;
        const std::array<double, 5> fracs{{prop, 0.5, 0.0, 1.0, 0.25}};
        for (double f : fracs) {
          const double T1 = s1_rail.duration + f * slack;
          const double T2 = T_target - T1;
          AxisProfile e1, e2;
          double n1 = 0.0, n2 = 0.0;
          if (solve_axis_bounded_at_duration(s0, rail, lim, T1, e1, n1, depth + 1) &&
              solve_axis_bounded_at_duration(rail, sf, lim, T2, e2, n2, depth + 1))
          {
            out_profile = std::move(e1);
            out_profile.append_profile(e2);
            out_profile.finalize();
            return true;
          }
        }
      }
      // Exact Pontryagin boundary-arc dwell at (q_lim, 0, 0): strictly C^2 with v=a=j=0.
      out_profile = std::move(s1_rail);
      if (slack > 1e-11) {
        out_profile.append_phase(slack, q_lim, 0.0, 0.0, 0.0);
      }
      out_profile.append_profile(s2_rail);
      out_profile.finalize();
      return true;
    }

    // Here min_bounded.duration < T_target < t_rail:
    // Collect feasible samples along [0, a_edge] (including min_best_ab) and locate a tight
    // feasible bracket [lo, hi] with dur(lo) <= T_target <= dur(hi).
    struct AbSample
    {
      double ab;
      double dur;
    };
    constexpr int kScan = 24;
    std::vector<double> scan_abs;
    scan_abs.reserve(static_cast<size_t>(kScan + 2));
    scan_abs.push_back(0.0);
    for (int i = 1; i <= kScan; ++i) {
      scan_abs.push_back(a_edge * static_cast<double>(i) / kScan);
    }
    if (clamped && ((a_edge > 0.0 && min_best_ab >= 0.0 && min_best_ab <= a_edge) ||
                    (a_edge < 0.0 && min_best_ab <= 0.0 && min_best_ab >= a_edge)))
    {
      scan_abs.push_back(min_best_ab);
    }
    std::sort(scan_abs.begin(), scan_abs.end(), [](double x, double y) {
      return std::abs(x) < std::abs(y);
    });
    scan_abs.erase(
      std::unique(scan_abs.begin(), scan_abs.end(), [](double x, double y) {
        return std::abs(x - y) <= 1e-9;
      }),
      scan_abs.end());

    std::vector<AbSample> feas_samples;
    feas_samples.reserve(scan_abs.size());
    feas_samples.push_back({0.0, t_rail});
    for (size_t idx = 1; idx < scan_abs.size(); ++idx) {
      const double ab = scan_abs[idx];
      AxisProfile tmp;
      double dur = 0.0;
      if (eval_boundary_split_min_time(s0, sf, lim, q_lim, ab, depth, tmp, dur)) {
        if (std::abs(dur - T_target) <= kTimeTol) {
          out_profile = std::move(tmp);
          return true;
        }
        feas_samples.push_back({ab, dur});
      }
    }

    bool have_bracket = false;
    double lo = 0.0;  // dur(lo) <= T_target
    double hi = 0.0;  // dur(hi) >= T_target
    double best_span = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i + 1 < feas_samples.size(); ++i) {
      const auto & a0 = feas_samples[i];
      const auto & a1 = feas_samples[i + 1];
      if ((a0.dur <= T_target && a1.dur >= T_target) ||
          (a1.dur <= T_target && a0.dur >= T_target))
      {
        const double span = std::abs(a1.ab - a0.ab);
        if (span < best_span) {
          best_span = span;
          lo = (a0.dur <= a1.dur) ? a0.ab : a1.ab;
          hi = (a0.dur <= a1.dur) ? a1.ab : a0.ab;
          have_bracket = true;
        }
      }
    }

    if (have_bracket) {
      AxisProfile best_match;
      bool matched = false;
      for (int it = 0; it < 36; ++it) {
        if (std::abs(hi - lo) <= 1e-11 * (1.0 + std::abs(a_edge))) {
          break;
        }
        const double mid = 0.5 * (lo + hi);
        AxisProfile tmp;
        double dur = 0.0;
        if (eval_boundary_split_min_time(s0, sf, lim, q_lim, mid, depth, tmp, dur)) {
          if (std::abs(dur - T_target) <= kTimeTol) {
            best_match = std::move(tmp);
            matched = true;
            break;
          }
          if (dur < T_target) {
            lo = mid;
          } else {
            hi = mid;
          }
          continue;
        }
        // Infeasible point at mid: probe both sub-halves symmetrically rather than discarding [mid, hi].
        bool progressed = false;
        const std::array<double, 2> probes{{
          0.75 * lo + 0.25 * hi,
          0.25 * lo + 0.75 * hi
        }};
        for (double p : probes) {
          AxisProfile ptmp;
          double pdur = 0.0;
          if (!eval_boundary_split_min_time(s0, sf, lim, q_lim, p, depth, ptmp, pdur)) {
            continue;
          }
          if (std::abs(pdur - T_target) <= kTimeTol) {
            best_match = std::move(ptmp);
            matched = true;
            break;
          }
          if (pdur < T_target) {
            lo = p;
          } else {
            hi = p;
          }
          progressed = true;
        }
        if (matched || !progressed) {
          break;
        }
      }
      if (matched) {
        out_profile = std::move(best_match);
        return true;
      }
      // If a discontinuous jump crossed T_target, extend cleanly through lo (strictly feasible).
      if (try_extend_through_ab(lo)) {
        return true;
      }
    }

    // Try extending through any feasible bounce acceleration with dur <= T_target.
    for (const auto & s : feas_samples) {
      if (s.dur <= T_target + kTimeTol && std::abs(s.ab) > 1e-12) {
        if (try_extend_through_ab(s.ab)) {
          return true;
        }
      }
    }
    out_next_feasible_T = std::min(out_next_feasible_T, t_rail);
    // Fall through to check rest-endpoint dwell fallback if either endpoint is at rest.
    const double rest_slack_rail = T_target - min_bounded.duration;
    if (rest_slack_rail > 1e-11) {
      if (std::abs(s0.v) < 1e-10 && std::abs(s0.a) < 1e-10) {
        out_profile.clear();
        out_profile.append_phase(rest_slack_rail, s0.q, 0.0, 0.0, 0.0);
        out_profile.append_profile(min_bounded);
        out_profile.finalize();
        return true;
      }
      if (std::abs(sf.v) < 1e-10 && std::abs(sf.a) < 1e-10) {
        out_profile = std::move(min_bounded);
        out_profile.append_phase(rest_slack_rail, sf.q, 0.0, 0.0, 0.0);
        out_profile.finalize();
        return true;
      }
    }
    return false;
  }

  // Case C: Zero-acc rail (q_lim, 0, 0) was unreachable (only inward bounce a_b != 0 is feasible).
  // Try min_best_ab first, then scan inward a_b values across [0, a_edge].
  if (clamped && std::abs(min_best_ab) > 1e-12 && try_extend_through_ab(min_best_ab)) {
    return true;
  }
  constexpr int kCaseCScan = 24;
  for (int i = 1; i <= kCaseCScan; ++i) {
    const double ab = a_edge * static_cast<double>(i) / kCaseCScan;
    if (try_extend_through_ab(ab)) {
      return true;
    }
  }

  // Interior rest-state dwell fallback (still strictly C^2 piecewise-constant-jerk).
  if (std::isfinite(lim.q_min) && std::isfinite(lim.q_max)) {
    const std::array<double, 3> q_rests{{
      0.5 * (lim.q_min + lim.q_max),
      std::clamp(0.5 * (s0.q + sf.q), lim.q_min, lim.q_max),
      std::clamp(q_lim + 0.1 * (0.5 * (lim.q_min + lim.q_max) - q_lim), lim.q_min, lim.q_max)
    }};
    for (double qr : q_rests) {
      const AxisState rest{qr, 0.0, 0.0};
      AxisProfile r1, r2;
      bool rc1 = false, rc2 = false;
      if (solve_axis_bounded_min_time(s0, rest, lim, r1, &rc1, depth + 1) &&
          solve_axis_bounded_min_time(rest, sf, lim, r2, &rc2, depth + 1))
      {
        const double tr = r1.duration + r2.duration;
        if (tr <= T_target + kTimeTol) {
          out_profile = std::move(r1);
          const double sl = std::max(0.0, T_target - tr);
          if (sl > 1e-11) {
            out_profile.append_phase(sl, qr, 0.0, 0.0, 0.0);
          }
          out_profile.append_profile(r2);
          out_profile.finalize();
          return true;
        }
        out_next_feasible_T = std::min(out_next_feasible_T, tr);
      }
    }
  }

  // Exact rest-endpoint dwell fallback: if either endpoint is at rest (v=0, a=0), delaying
  // departure from s0 or dwelling after arrival at sf is strictly C^2 piecewise-constant-jerk
  // and preserves [q_min, q_max] for any T_target >= min_bounded.duration.
  const double rest_slack = T_target - min_bounded.duration;
  if (rest_slack > 1e-11) {
    if (std::abs(s0.v) < 1e-10 && std::abs(s0.a) < 1e-10) {
      out_profile.clear();
      out_profile.append_phase(rest_slack, s0.q, 0.0, 0.0, 0.0);
      out_profile.append_profile(min_bounded);
      out_profile.finalize();
      return true;
    }
    if (std::abs(sf.v) < 1e-10 && std::abs(sf.a) < 1e-10) {
      out_profile = std::move(min_bounded);
      out_profile.append_phase(rest_slack, sf.q, 0.0, 0.0, 0.0);
      out_profile.finalize();
      return true;
    }
  }
  return false;
}

// ======================== Multi-DOF Synchronized Native OTG ========================

MultiDoFResult solve_multidof(const MultiDoFInput & input)
{
  MultiDoFResult res;
  const size_t N = input.current.size();
  if (N == 0 || input.target.size() != N || input.limits.size() != N) {
    return res;
  }

  res.independent_min_durations.assign(N, 0.0);
  res.profiles.resize(N);

  auto quantize_up = [&](double t) -> double {
    if (input.discretization != DurationDiscretization::DISCRETE) {
      return t;
    }
    const double dt = std::max(1e-6, input.dt_step);
    return std::max(dt, std::ceil(t / dt - 1e-9) * dt);
  };

  // Velocity control interface.
  if (input.control_interface == ControlInterface::VELOCITY) {
    double T_max = input.minimum_duration.value_or(0.0);
    for (size_t j = 0; j < N; ++j) {
      if (!solve_axis_velocity_min_time(
            input.current[j], input.target[j].v, input.target[j].a, input.limits[j], res.profiles[j]))
      {
        res.failed_dof = static_cast<int>(j);
        return res;
      }
      res.independent_min_durations[j] = res.profiles[j].duration;
      T_max = std::max(T_max, res.profiles[j].duration);
    }
    if (input.synchronization == Synchronization::NONE) {
      res.ok = true;
      res.duration = T_max;
      return res;
    }
    double T_sync = input.exact_duration.has_value() ? *input.exact_duration : quantize_up(T_max);
    if (T_sync < T_max - kTimeTol || T_sync > input.max_duration) {
      res.next_feasible_duration = T_max;
      return res;
    }
    for (size_t j = 0; j < N; ++j) {
      if (!solve_axis_velocity_at_duration(
            input.current[j], input.target[j].v, input.target[j].a,
            input.limits[j], T_sync, res.profiles[j]))
      {
        res.failed_dof = static_cast<int>(j);
        return res;
      }
    }
    res.ok = true;
    res.duration = T_sync;
    return res;
  }

  // Position control interface: compute exact bounded minimum duration + unconstrained blocked sets.
  std::vector<Step1Result> unc_s1(N);
  std::vector<AxisProfile> min_profs(N);
  double T_min_all = 0.0;
  bool all_zero = true;

  for (size_t j = 0; j < N; ++j) {
    const auto & s0 = input.current[j];
    const auto & sf = input.target[j];
    if (std::abs(sf.q - s0.q) > 1e-11 || std::abs(s0.v) > 1e-10 || std::abs(sf.v) > 1e-10 ||
        std::abs(s0.a) > 1e-10 || std::abs(sf.a) > 1e-10)
    {
      all_zero = false;
    }
    unc_s1[j] = solve_axis_unconstrained_step1(s0, sf, input.limits[j]);
    if (!unc_s1[j].ok) {
      res.failed_dof = static_cast<int>(j);
      return res;
    }
    if (!input.limits[j].has_pos_limits() || profile_within_pos(unc_s1[j].min_profile, input.limits[j])) {
      min_profs[j] = unc_s1[j].min_profile;
    } else {
      if (!solve_axis_bounded_min_time(s0, sf, input.limits[j], min_profs[j])) {
        res.failed_dof = static_cast<int>(j);
        return res;
      }
    }
    res.independent_min_durations[j] = min_profs[j].duration;
    T_min_all = std::max(T_min_all, min_profs[j].duration);
  }

  if (all_zero && !input.exact_duration.has_value() && !input.minimum_duration.has_value()) {
    const double T0 = (input.discretization == DurationDiscretization::DISCRETE) ? input.dt_step : 0.0;
    for (size_t j = 0; j < N; ++j) {
      res.profiles[j].clear();
      const double qc = std::clamp(input.current[j].q, input.limits[j].q_min, input.limits[j].q_max);
      res.profiles[j].append_phase(T0, qc, 0.0, 0.0, 0.0);
      res.profiles[j].finalize();
    }
    res.ok = true;
    res.duration = T0;
    return res;
  }

  if (input.synchronization == Synchronization::NONE && !input.exact_duration.has_value()) {
    res.profiles = std::move(min_profs);
    res.duration = T_min_all;
    res.ok = true;
    return res;
  }

  // Phase synchronization (straight-line collinear motion in joint space when endpoints are collinear).
  if (input.synchronization == Synchronization::PHASE && !input.exact_duration.has_value()) {
    size_t lead = 0;
    double max_dq = 0.0;
    for (size_t j = 0; j < N; ++j) {
      const double d = std::abs(input.target[j].q - input.current[j].q);
      if (d > max_dq) {
        max_dq = d;
        lead = j;
      }
    }
    if (max_dq > 1e-11) {
      const double dq_lead = input.target[lead].q - input.current[lead].q;
      bool collinear = true;
      AxisLimits s_lim;
      s_lim.q_min = -std::numeric_limits<double>::infinity();
      s_lim.q_max = std::numeric_limits<double>::infinity();
      s_lim.v_min = -std::numeric_limits<double>::infinity();
      s_lim.v_max = std::numeric_limits<double>::infinity();
      s_lim.a_min = -std::numeric_limits<double>::infinity();
      s_lim.a_max = std::numeric_limits<double>::infinity();
      s_lim.j_max = std::numeric_limits<double>::infinity();

      const AxisState lead_s0{0.0, input.current[lead].v / dq_lead, input.current[lead].a / dq_lead};
      const AxisState lead_sf{1.0, input.target[lead].v / dq_lead, input.target[lead].a / dq_lead};

      for (size_t j = 0; j < N; ++j) {
        const double dqj = input.target[j].q - input.current[j].q;
        if (std::abs(dqj) < 1e-11) {
          if (std::abs(input.current[j].v) > 1e-9 || std::abs(input.target[j].v) > 1e-9 ||
              std::abs(input.current[j].a) > 1e-9 || std::abs(input.target[j].a) > 1e-9)
          {
            collinear = false;
            break;
          }
          continue;
        }
        if (std::abs(input.current[j].v / dqj - lead_s0.v) > 1e-6 ||
            std::abs(input.target[j].v / dqj - lead_sf.v) > 1e-6 ||
            std::abs(input.current[j].a / dqj - lead_s0.a) > 1e-6 ||
            std::abs(input.target[j].a / dqj - lead_sf.a) > 1e-6)
        {
          collinear = false;
          break;
        }
        const auto & lj = input.limits[j];
        if (dqj > 0.0) {
          s_lim.v_max = std::min(s_lim.v_max, lj.v_max / dqj);
          s_lim.v_min = std::max(s_lim.v_min, lj.v_min / dqj);
          s_lim.a_max = std::min(s_lim.a_max, lj.a_max / dqj);
          s_lim.a_min = std::max(s_lim.a_min, lj.a_min / dqj);
          if (std::isfinite(lj.q_max)) {
            s_lim.q_max = std::min(s_lim.q_max, (lj.q_max - input.current[j].q) / dqj);
          }
          if (std::isfinite(lj.q_min)) {
            s_lim.q_min = std::max(s_lim.q_min, (lj.q_min - input.current[j].q) / dqj);
          }
        } else {
          s_lim.v_max = std::min(s_lim.v_max, lj.v_min / dqj);
          s_lim.v_min = std::max(s_lim.v_min, lj.v_max / dqj);
          s_lim.a_max = std::min(s_lim.a_max, lj.a_min / dqj);
          s_lim.a_min = std::max(s_lim.a_min, lj.a_max / dqj);
          if (std::isfinite(lj.q_min)) {
            s_lim.q_max = std::min(s_lim.q_max, (lj.q_min - input.current[j].q) / dqj);
          }
          if (std::isfinite(lj.q_max)) {
            s_lim.q_min = std::max(s_lim.q_min, (lj.q_max - input.current[j].q) / dqj);
          }
        }
        s_lim.j_max = std::min(s_lim.j_max, lj.j_max / std::abs(dqj));
      }
      if (collinear) {
        AxisProfile s_prof;
        if (solve_axis_bounded_min_time(lead_s0, lead_sf, s_lim, s_prof)) {
          const double T_ph = quantize_up(std::max(s_prof.duration, input.minimum_duration.value_or(0.0)));
          double next_T = 0.0;
          if (std::abs(T_ph - s_prof.duration) <= kTimeTol ||
              solve_axis_bounded_at_duration(lead_s0, lead_sf, s_lim, T_ph, s_prof, next_T))
          {
            for (size_t j = 0; j < N; ++j) {
              const double dqj = input.target[j].q - input.current[j].q;
              res.profiles[j].clear();
              for (const auto & ph : s_prof.phases) {
                res.profiles[j].append_phase(
                  ph.dt, input.current[j].q + dqj * ph.q0, dqj * ph.v0, dqj * ph.a0, dqj * ph.j);
              }
              res.profiles[j].finalize();
            }
            res.duration = s_prof.duration;
            res.ok = true;
            return res;
          }
        }
      }
    }
  }

  // Exact duration mode.
  if (input.exact_duration.has_value()) {
    const double T_target = *input.exact_duration;
    if (T_target > input.max_duration) {
      return res;
    }
    for (size_t j = 0; j < N; ++j) {
      if (min_profs[j].duration > T_target + kTimeTol) {
        res.failed_dof = static_cast<int>(j);
        res.next_feasible_duration = min_profs[j].duration;
        return res;
      }
    }
    for (size_t j = 0; j < N; ++j) {
      double next_T = 0.0;
      if (!solve_axis_bounded_at_duration(
            input.current[j], input.target[j], input.limits[j], T_target, res.profiles[j], next_T))
      {
        res.failed_dof = static_cast<int>(j);
        res.next_feasible_duration = std::isfinite(next_T) ? next_T : min_profs[j].duration;
        return res;
      }
    }
    res.duration = T_target;
    res.ok = true;
    return res;
  }

  // Free-duration Time synchronization: exact blocked-interval union skipping!
  double T_cand = std::max(T_min_all, input.minimum_duration.value_or(0.0));
  T_cand = quantize_up(T_cand);

  for (int iter = 0; iter < 32; ++iter) {
    if (T_cand > input.max_duration) {
      break;
    }
    // Skip any known unconstrained blocked interval across all DoFs immediately.
    bool jumped = true;
    for (int j_it = 0; jumped && j_it < 32; ++j_it) {
      jumped = false;
      for (size_t j = 0; j < N; ++j) {
        for (const auto & blk : unc_s1[j].blocked_intervals) {
          if (T_cand > blk.first + kTimeTol && T_cand < blk.second - kTimeTol) {
            T_cand = quantize_up(blk.second);
            jumped = true;
            break;
          }
        }
      }
    }
    if (T_cand > input.max_duration) {
      break;
    }

    bool all_ok = true;
    double max_next = T_cand;
    int bad_dof = -1;
    for (size_t j = 0; j < N; ++j) {
      // Reuse min_profs[j] directly when T_cand matches its exact duration!
      if (std::abs(T_cand - min_profs[j].duration) <= kTimeTol) {
        res.profiles[j] = min_profs[j];
        continue;
      }
      double next_T = 0.0;
      if (!solve_axis_bounded_at_duration(
            input.current[j], input.target[j], input.limits[j], T_cand, res.profiles[j], next_T))
      {
        all_ok = false;
        if (bad_dof < 0) {
          bad_dof = static_cast<int>(j);
        }
        if (std::isfinite(next_T) && next_T > max_next) {
          max_next = next_T;
        }
      }
    }
    if (all_ok) {
      res.duration = T_cand;
      res.ok = true;
      return res;
    }
    if (max_next <= T_cand + kTimeTol) {
      res.failed_dof = bad_dof;
      break;
    }
    T_cand = quantize_up(max_next);
  }

  return res;
}

}  // namespace otg
}  // namespace lite6_studio_planner
