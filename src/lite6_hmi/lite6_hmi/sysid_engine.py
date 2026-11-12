import os

import coal
import numpy as np
import pinocchio as pin
import yaml
from ament_index_python.packages import get_package_share_directory
from scipy.interpolate import interp1d
from scipy.optimize import least_squares, lsq_linear
from scipy.signal import savgol_filter


class SysIdEngine:
    """Fourier Fv/Fc identification followed by steady-sliding Stribeck fitting."""

    ARMATURE = 0.1  # Known factory inertia [kg m^2], identical on all six axes.
    SAMPLE_PERIOD = 0.001  # Same 1 ms grid as the CTC control loop and planner.
    MIN_FIT_SPEED = 0.05
    TANH_SLOPE = 300.0
    LIMIT_MARGIN = 0.90
    MIN_DURATION = 6.0  # Keep useful recording time after the 1 s end trims.
    MAX_DURATION = 120.0  # Fourier stage; low-speed experiments are appended.
    LOW_SPEEDS = (0.004, 0.008, 0.014, 0.022, 0.034, 0.048)  # rad/s, both signs
    LOW_CRUISE = 2.0
    LOW_SETTLE = 1.0
    LOW_END_GUARD = 0.3
    LOW_DWELL = 0.4
    LOW_PRE_SPEED = 0.08
    LOW_PRE_CRUISE = 0.6
    STAGE_DWELL = 1.0

    def __init__(self):
        desc = get_package_share_directory('lite6_description')
        planner = get_package_share_directory('lite6_planner')
        urdf = os.path.join(desc, 'urdf', 'lite6.urdf')
        self.model = pin.buildModelFromUrdf(urdf)
        self.data = self.model.createData()
        with open(os.path.join(planner, 'config', 'kinematic_limits.yaml')) as stream:
            limits = yaml.safe_load(stream)['joint_limits']
        self.limits = np.array([
            [limits[f'joint{i}'][key] for key in
             ('min_pos', 'max_pos', 'max_vel', 'max_acc', 'max_jerk')]
            for i in range(1, 7)], dtype=float)
        if (not np.isfinite(self.limits).all() or
                np.any(self.limits[:, 0] >= self.limits[:, 1]) or
                np.any(self.limits[:, 2:] <= 0)):
            raise ValueError('Invalid planner joint limits')

        # Same URDF, SRDF and ground geometry as the authoritative planner.
        self.geometry = pin.buildGeomFromUrdf(
            self.model, urdf, pin.COLLISION, package_dirs=[os.path.dirname(desc)])
        self.geometry.addAllCollisionPairs()
        pin.removeCollisionPairs(self.model, self.geometry,
                                 os.path.join(planner, 'config', 'lite6.srdf'))
        with open(os.path.join(planner, 'config', 'ik_config.yaml')) as stream:
            ground = yaml.safe_load(stream)['ground_plane']
        frame = self.model.getFrameId('link_base')
        ground_id = self.geometry.addGeometryObject(pin.GeometryObject(
            'ground_plane', self.model.frames[frame].parentJoint, frame,
            pin.SE3(np.eye(3), np.asarray(ground['position'], dtype=float)),
            coal.Box(*ground['size'])))
        for i in range(ground_id):
            self.geometry.addCollisionPair(pin.CollisionPair(i, ground_id))
        self.geometry_data = self.geometry.createData()

        self.N_f = 7
        self.f0 = 0.1
        self.w = 2 * np.pi * self.f0
        self.T_total = 10.0
        self.a = np.zeros((6, self.N_f))
        self.b = np.zeros((6, self.N_f))
        self.record_data = []
        self.trajectory_summary = ''
        self.high_speed_duration = None
        self.low_segments = []
        self.low_fit_windows = []
        self.fit_diagnostics = []

    @staticmethod
    def _wave(a, b, w, times):
        """Vectorized position offset and its first three derivatives."""
        wl = w * np.arange(1, a.shape[1] + 1)
        phase = np.asarray(times)[:, None] * wl
        sn, cs = np.sin(phase), np.cos(phase)
        return (sn @ (a / wl).T + (1 - cs) @ (b / wl).T,
                cs @ a.T + sn @ b.T,
                -sn @ (a * wl).T + cs @ (b * wl).T,
                -cs @ (a * wl**2).T - sn @ (b * wl**2).T)

    def _in_collision(self, q):
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateGeometryPlacements(self.model, self.data, self.geometry,
                                     self.geometry_data)
        return pin.computeCollisions(self.geometry, self.geometry_data, True)

    def _path_is_clear(self, positions):
        # Match the planner's joint-space subdivision; retain its final check.
        positions = np.asarray(positions)
        subdivisions = np.maximum(
            1, np.ceil(np.max(np.abs(np.diff(positions, axis=0)), axis=1) / 0.04)
        ).astype(int)
        if np.all(subdivisions == 1):
            # Dense references need no interpolation. Check every distinct pose,
            # but only once for exact repeats (dwells and retraced sweeps).
            # No rounding or temporal decimation: nearby poses remain distinct.
            return not any(self._in_collision(q) for q in np.unique(positions, axis=0))
        previous = positions[0]
        if self._in_collision(previous):
            return False
        for q, pieces in zip(positions[1:], subdivisions):
            for part in range(1, pieces + 1):
                if self._in_collision(previous + (q - previous) * (part / pieces)):
                    return False
            previous = q
        return True

    def generate_fourier_coefficients(self, q0, max_pos_limits):
        """Choose an informative, collision-free shape and adapt amplitude/time.

        The UI amplitudes are upper bounds. All candidates satisfy the original
        zero velocity/acceleration endpoint constraints. Sampled extrema are
        padded with analytic derivative bounds before applying a 10% margin.
        """
        self.high_speed_duration = None
        self.low_segments = []
        self.low_fit_windows = []
        q0 = np.asarray(q0, dtype=float)
        amplitudes = np.asarray(max_pos_limits, dtype=float)
        if (q0.shape != (6,) or amplitudes.shape != (6,) or
                not np.isfinite(q0).all() or not np.isfinite(amplitudes).all() or
                np.any(amplitudes <= 0)):
            raise ValueError('SysID needs six finite positions and positive amplitudes')
        room = np.minimum(q0 - self.limits[:, 0], self.limits[:, 1] - q0)
        if np.any(room <= 0):
            raise ValueError('SysID center must be strictly inside every joint limit')
        if self._in_collision(q0):
            raise ValueError('SysID center is in collision')
        amplitudes = np.minimum(amplitudes, room) * self.LIMIT_MARGIN
        rng = np.random.RandomState(42)  # Local, reproducible candidate search.
        base_duration = 10.0
        w = 2 * np.pi / base_duration
        times = np.linspace(0, base_duration, 2049)
        half_step = (times[1] - times[0]) / 2
        wl = w * np.arange(1, self.N_f + 1)
        shapes = []
        for _ in range(16):
            a = np.zeros_like(self.a)
            b = np.zeros_like(self.b)
            for i in range(6):
                a[i, :-1] = rng.uniform(-1, 1, self.N_f - 1)
                b[i, :-2] = rng.uniform(-1, 1, self.N_f - 2)
                a[i, -1] = -a[i, :-1].sum()
                harmonics = np.arange(1, self.N_f - 1)
                b[i, -2:] = np.linalg.solve(
                    [[self.N_f - 1, self.N_f],
                     [1 / (self.N_f - 1), 1 / self.N_f]],
                    [-np.dot(harmonics, b[i, :-2]),
                     -np.sum(b[i, :-2] / harmonics)])
            q, _, _, _ = self._wave(a, b, w, times)
            # |q'| <= sum(hypot(a_l,b_l)): bounds unsampled position extrema.
            peak = np.max(np.abs(q), axis=0) + np.hypot(a, b).sum(axis=1) * half_step
            scale = amplitudes / peak
            shapes.append((a * scale[:, None], b * scale[:, None]))

        # Try full permitted excursions first, then shrink if geometry blocks
        # every candidate. Never obtain feasibility by relaxing robot limits.
        for contraction in (1.0, 0.8, 0.6, 0.4):
            candidates = []
            for base_a, base_b in shapes:
                a, b = base_a * contraction, base_b * contraction
                _, v, acc, jerk = self._wave(a, b, w, times)
                radius = np.hypot(a, b)
                peaks = np.array([
                    np.max(np.abs(v), axis=0) + (radius * wl).sum(axis=1) * half_step,
                    np.max(np.abs(acc), axis=0) + (radius * wl**2).sum(axis=1) * half_step,
                    np.max(np.abs(jerk), axis=0) + (radius * wl**3).sum(axis=1) * half_step])
                ratios = peaks / (self.LIMIT_MARGIN * self.limits[:, 2:].T)
                stretch = max(self.MIN_DURATION / base_duration,
                              ratios[0].max(), np.sqrt(ratios[1].max()),
                              np.cbrt(ratios[2].max()))
                duration = np.ceil(base_duration * stretch / self.SAMPLE_PERIOD - 1e-9) * self.SAMPLE_PERIOD
                if duration > self.MAX_DURATION:
                    continue
                stretch = duration / base_duration
                speed = v / stretch
                # E-optimal score of [velocity, smooth sign], scaled by each
                # axis speed limit. Favor the weakest joint's information.
                score = np.inf
                for i in range(6):
                    valid = ((np.abs(speed[:, i]) >= self.MIN_FIT_SPEED) &
                             (times * stretch > 1) & (times * stretch < duration - 1))
                    vi = speed[valid, i]
                    # At nominal 200 Hz this leaves ~300 useful samples,
                    # above the estimator's 200-sample minimum after trimming.
                    sample_dt = duration / (len(times) - 1)
                    if (len(vi) * sample_dt < 1.5 or
                            np.count_nonzero(vi > 0) * sample_dt < 0.25 or
                            np.count_nonzero(vi < 0) * sample_dt < 0.25):
                        score = -1
                        break
                    design = np.column_stack((vi / self.limits[i, 2],
                                              np.tanh(self.TANH_SLOPE * vi)))
                    information = design.T @ design / len(times)
                    score = min(score, np.linalg.eigvalsh(information)[0])
                if score > 1e-5:
                    candidates.append((score, a / stretch, b / stretch,
                                       w / stretch, duration))
            for _, a, b, frequency, duration in sorted(candidates, key=lambda c: c[0], reverse=True):
                # Integer control ticks avoid linspace silently shortening the period.
                sample_times = self._sample_times(duration)
                offsets, _, _, _ = self._wave(a, b, frequency, sample_times)
                if not self._path_is_clear(q0 + offsets):
                    continue
                self.a, self.b, self.w = a, b, frequency
                self.T_total = duration
                self.f0 = frequency / (2 * np.pi)
                self.record_data.clear()
                self.trajectory_summary = (
                    f'Adaptive SysID: {duration:.2f} s, peak excursions [rad] '
                    f'{np.round(np.max(np.abs(offsets), axis=0), 4).tolist()}')
                return
        raise ValueError('No sufficiently exciting collision-free SysID trajectory within limits; '
                         'choose a different center or larger excitation bounds')

    def generate_identification_trajectory(self, q0, max_pos_limits, progress=None):
        """Keep Fourier excitation intact; append smooth, paired low-speed sweeps.

        One Supervisor-owned trajectory preserves the existing session/mode flow.
        All six axes move together; every speed is visited in both directions and
        each pair returns to the center. No plant friction parameters are read.
        """
        if progress is not None:
            progress('Checking Fourier trajectory collisions...')
        self.generate_fourier_coefficients(q0, max_pos_limits)
        if progress is not None:
            progress('Checking low-speed trajectory collisions...')
        self.high_speed_duration = self.T_total
        q0 = np.asarray(q0, dtype=float)
        room = np.minimum(q0 - self.limits[:, 0], self.limits[:, 1] - q0)
        allowed = self.LIMIT_MARGIN * np.minimum(np.asarray(max_pos_limits), room)
        cursor = self.high_speed_duration + self.STAGE_DWELL
        offset = 0.0
        for speed in self.LOW_SPEEDS:
            peak_speed = max(speed, self.LOW_PRE_SPEED)
            if peak_speed > self.LIMIT_MARGIN * self.limits[:, 2].min():
                raise ValueError('Joint velocity limits do not permit the low-speed experiment')
            # Quintic velocity blend: max first derivative=1.875, second <5.774.
            ramp = max(0.4, 1.875 * peak_speed / (self.LIMIT_MARGIN * self.limits[:, 3].min()),
                       np.sqrt(5.774 * peak_speed / (self.LIMIT_MARGIN * self.limits[:, 4].min())))
            ramp = np.ceil(ramp / self.SAMPLE_PERIOD) * self.SAMPLE_PERIOD
            excursion = (self.LOW_PRE_SPEED * (self.LOW_PRE_CRUISE + ramp) +
                         speed * (self.LOW_CRUISE + ramp))
            if np.any(excursion > allowed):
                raise ValueError(f'Low-speed SysID needs at least {excursion:.4f} rad of '
                                 'per-axis excursion after the 10% margin; '
                                 'increase bounds or move the center')
            for direction in (1.0, -1.0):
                velocity = direction * speed
                preslide = direction * self.LOW_PRE_SPEED
                # With friction compensation disabled, slow reversal alone may
                # never overcome the PD tracking offset within a short plateau.
                # Establish sliding first, then decelerate in the same direction.
                for duration, v0, v1 in ((ramp, 0., preslide),
                                         (self.LOW_PRE_CRUISE, preslide, preslide),
                                         (ramp, preslide, velocity),
                                         (self.LOW_CRUISE, velocity, velocity),
                                         (ramp, velocity, 0.), (self.LOW_DWELL, 0., 0.)):
                    self.low_segments.append((cursor, duration, offset, v0, v1))
                    if v0 == v1 and v0 == velocity:
                        self.low_fit_windows.append((cursor + self.LOW_SETTLE,
                                                     cursor + duration - self.LOW_END_GUARD,
                                                     velocity))
                    offset += 0.5 * (v0 + v1) * duration
                    cursor += duration
            offset = 0.0  # Paired sweeps return analytically to the center.
        self.T_total = cursor
        self._low_starts = np.array([segment[0] for segment in self.low_segments])
        rng = np.random.RandomState(73)
        patterns = [np.ones(6), -np.ones(6)]
        patterns.extend(rng.choice([-1., 1.], size=(14, 6)))
        low_times = self.trajectory_times()
        low_times = low_times[low_times >= self.high_speed_duration]
        scalar = self._low_points(low_times)[0]
        # Positions are monotone within each ramp/cruise: extrema occur at stops.
        for pattern in patterns:
            if self._path_is_clear(q0 + scalar[:, None] * pattern):
                self.low_directions = pattern
                self.trajectory_summary += (
                    f'; low-speed stage {self.T_total - self.high_speed_duration:.2f} s, '
                    f'{len(self.LOW_SPEEDS)} speeds in both directions; total {self.T_total:.2f} s')
                return
        raise ValueError('No collision-free low-speed SysID sweeps at this center')

    def _low_point(self, t):
        if not self.low_segments or t < self.low_segments[0][0]:
            return 0., 0., 0.
        index = min(int(np.searchsorted(self._low_starts, t, side='right')) - 1,
                    len(self.low_segments) - 1)
        start, duration, q0, v0, v1 = self.low_segments[index]
        u = np.clip((t - start) / duration, 0., 1.)
        blend = u**3 * (10. - 15. * u + 6. * u**2)
        integral = u**4 * (2.5 - 3. * u + u**2)
        derivative = 30. * u**2 * (1. - u)**2
        return (q0 + duration * (v0 * u + (v1 - v0) * integral),
                v0 + (v1 - v0) * blend, (v1 - v0) * derivative / duration)

    def get_trajectory_point(self, t, q0):
        if self.high_speed_duration is None or t < self.high_speed_duration:
            return tuple(np.array(x) for x in zip(*[
                self.get_fourier_point(i, t, q0) for i in range(6)]))
        q, v, a = self._low_point(t)
        return (np.asarray(q0) + q * self.low_directions,
                v * self.low_directions, a * self.low_directions)

    @classmethod
    def _sample_times(cls, duration):
        ticks = int(round(duration / cls.SAMPLE_PERIOD))
        if abs(ticks * cls.SAMPLE_PERIOD - duration) > 1e-8:
            raise ValueError('SysID duration must be an integer number of control ticks')
        return np.arange(ticks + 1, dtype=np.int64) * cls.SAMPLE_PERIOD

    def trajectory_times(self):
        return self._sample_times(self.T_total)

    def _low_points(self, times):
        """Vectorized low-speed references, including the inter-stage dwell."""
        indices = np.searchsorted(self._low_starts, times, side='right') - 1
        segments = np.asarray(self.low_segments)[np.clip(indices, 0, len(self.low_segments) - 1)]
        start, duration, q0, v0, v1 = segments.T
        u = np.clip((times - start) / duration, 0., 1.)
        blend = u**3 * (10. - 15. * u + 6. * u**2)
        integral = u**4 * (2.5 - 3. * u + u**2)
        derivative = 30. * u**2 * (1. - u)**2
        return tuple(np.where(indices < 0, 0., value) for value in (
            q0 + duration * (v0 * u + (v1 - v0) * integral),
            v0 + (v1 - v0) * blend, (v1 - v0) * derivative / duration))

    def trajectory_samples(self, q0):
        """Generate all 1 ms references without a Python loop over Fourier harmonics."""
        times = self.trajectory_times()
        high = times < self.high_speed_duration
        q, v, a = (np.empty((len(times), 6)) for _ in range(3))
        offsets, v[high], a[high], _ = self._wave(self.a, self.b, self.w, times[high])
        q[high] = np.asarray(q0) + offsets
        low_q, low_v, low_a = self._low_points(times[~high])
        q[~high] = np.asarray(q0) + low_q[:, None] * self.low_directions
        v[~high] = low_v[:, None] * self.low_directions
        a[~high] = low_a[:, None] * self.low_directions
        return times, q, v, a

    def get_fourier_point(self, i, t, q0):
        wl = self.w * np.arange(1, self.N_f + 1)
        sn, cs = np.sin(wl * t), np.cos(wl * t)
        q = q0[i] + np.sum(self.a[i] / wl * sn + self.b[i] / wl * (1 - cs))
        dq = np.sum(self.a[i] * cs + self.b[i] * sn)
        ddq = np.sum(-self.a[i] * wl * sn + self.b[i] * wl * cs)
        return q, dq, ddq

    def calculate_least_squares(self) -> str:
        """Fit Fv/Fc first; freeze them while estimating Fs-Fc and vs.

        Low-speed plateaus use the physical steady-sliding sign, not tanh(300*v).
        The controller's tanh is a direction regularizer, not a plant parameter.
        """
        if self.high_speed_duration is None or not self.low_fit_windows:
            raise ValueError('Generate the two-stage SysID trajectory before fitting')
        if len(self.record_data) < 5:
            raise ValueError('Insufficient recorded samples for system identification')
        t_seq = np.array([d['t'] for d in self.record_data], dtype=float)
        q_seq = np.array([d['q'] for d in self.record_data], dtype=float)
        dq_seq = np.array([d['dq'] for d in self.record_data], dtype=float)
        tau_seq = np.array([d['tau_cmd'] for d in self.record_data], dtype=float)
        if (not np.isfinite(t_seq).all() or np.any(np.diff(t_seq) <= 0) or
                any(x.shape != (len(t_seq), 6) or not np.isfinite(x).all()
                    for x in (q_seq, dq_seq, tau_seq))):
            raise ValueError('Invalid SysID samples or non-increasing timestamps')
        self.fit_diagnostics = []
        if t_seq[0] > 0.25 or t_seq[-1] < self.T_total - 0.25:
            raise ValueError(f'Incomplete excitation recording: {t_seq[0]:.4f}..{t_seq[-1]:.4f} s; '
                             f'expected 0..{self.T_total:.4f} s on the CTC clock')
        dt = float(np.median(np.diff(t_seq)))
        if np.max(np.diff(t_seq)) > max(0.05, 5.0 * dt):
            raise ValueError('SysID recording has telemetry gaps; refusing to interpolate missing experiments')
        t_uniform = np.arange(t_seq[0], t_seq[-1], dt)
        q, dq, tau = [interp1d(t_seq, values, axis=0)(t_uniform)
                      for values in (q_seq, dq_seq, tau_seq)]
        window = max(5, int(0.1 / dt) | 1)
        if len(t_uniform) < window:
            raise ValueError('Recording is too short for acceleration estimation')
        ddq = savgol_filter(dq, window, polyorder=3, deriv=1, delta=dt, axis=0)
        rigid_torque = np.array([pin.rnea(self.model, self.data, qi, vi, ai).copy()
                                 for qi, vi, ai in zip(q, dq, ddq)])
        residual = tau - rigid_torque - self.ARMATURE * ddq
        friction = np.zeros((6, 2))
        stribeck = np.zeros((6, 2))
        report = []
        failures = []
        for i in range(6):
            try:
                # Retain the original Fourier-stage speed cutoff and linear solver.
                valid = ((t_uniform > 1.0) & (t_uniform < self.high_speed_duration - 1.0) &
                         (np.abs(dq[:, i]) >= self.MIN_FIT_SPEED))
                if np.count_nonzero(valid) < 200:
                    raise ValueError(f'Joint {i+1}: fewer than 200 useful high-speed friction samples')
                velocity = dq[valid, i]
                design = np.column_stack((velocity, np.tanh(self.TANH_SLOPE * velocity)))
                scaled = design / np.linalg.norm(design, axis=0)
                if np.linalg.cond(scaled) > 1000:
                    raise ValueError(f'Joint {i+1}: insufficient velocity variation for Fv/Fc fit')
                result = lsq_linear(design, residual[valid, i], bounds=(0.0, np.inf))
                if not result.success or not np.isfinite(result.x).all():
                    raise ValueError(f'Joint {i+1}: friction fit failed')
                friction[i] = result.x
                high_rmse = np.sqrt(np.mean((design @ result.x - residual[valid, i])**2))
                fs, vs, low_rmse, count = self._fit_stribeck(
                    i, t_uniform, dq[:, i], ddq[:, i], residual[:, i], dt, *result.x)
                stribeck[i] = (fs, vs)
                report.append(f'# joint{i+1}: high-speed RMSE={high_rmse:.6g} Nm '
                              f'({valid.sum()} samples); low-speed RMSE={low_rmse:.6g} Nm '
                              f'({count} stable samples, {len(self.low_fit_windows)} plateaus)')
            except ValueError as exc:
                failures.append(str(exc))
        if failures:
            raise ValueError('Identification quality checks failed:\n' + '\n'.join(failures) +
                             '\nPlateau diagnostics:\n' + '\n'.join(self.fit_diagnostics))

        def format_list(values):
            return '[' + ', '.join(f'{value:.8g}' for value in values) + ']'

        return ('# --- Two-stage identified friction parameters (not automatically applied) ---\n'
                '# Stage 1: Fourier Fv/Fc. Stage 2: fixed Fv/Fc, fit Fs/vs; exponent=2.\n'
                f'friction_v_nominal: {format_list(friction[:, 0])}\n'
                f'friction_c_nominal: {format_list(friction[:, 1])}\n'
                f'friction_s_nominal: {format_list(stribeck[:, 0])}\n'
                f'friction_vs_nominal: {format_list(stribeck[:, 1])}\n'
                + '\n'.join(report) + '\n'
                + '\n'.join('# ' + line for line in self.fit_diagnostics) + '\n'
                )

    def _fit_stribeck(self, joint, times, velocity, acceleration, torque, dt, fv, fc):
        speeds, excesses = [], []
        count = 0
        failures = []
        for start, end, scalar_velocity in self.low_fit_windows:
            reference = scalar_velocity * self.low_directions[joint]
            in_window = (times >= start) & (times <= end)
            speed_ok = np.abs(velocity - reference) <= max(0.0005, 0.15 * abs(reference))
            acceleration_ok = np.abs(acceleration) <= max(0.002, 0.1 * abs(reference))
            mask = in_window & speed_ok & acceleration_ok
            samples = np.count_nonzero(mask)
            available = np.count_nonzero(in_window)
            duration = samples * dt
            speed_rejected = np.count_nonzero(in_window & ~speed_ok) / max(available, 1)
            acceleration_rejected = np.count_nonzero(in_window & ~acceleration_ok) / max(available, 1)
            diagnostic = (f'Joint {joint+1}, {reference:+.4f} rad/s, [{start:.3f}, {end:.3f}] s: '
                          f'{samples}/{available} samples, {duration:.4f} s stable (need 0.3000 s); '
                          f'velocity rejected {speed_rejected:.1%}, '
                          f'acceleration rejected {acceleration_rejected:.1%}')
            self.fit_diagnostics.append(diagnostic)
            if samples < 20 or duration < 0.3:
                failures.append(diagnostic)
                continue
            v = velocity[mask]
            # One robust observation per speed/direction gives equal plateau weight.
            speeds.append(float(np.median(np.abs(v))))
            excesses.append(float(np.median(np.sign(v) * torque[mask] - fv * np.abs(v) - fc)))
            count += samples
        if failures:
            raise ValueError('Insufficient steady low-speed data: ' + '; '.join(failures))
        speeds, excesses = np.asarray(speeds), np.asarray(excesses)
        scale = max(fc, 0.01)
        vs_bounds = (min(self.LOW_SPEEDS) / 4., max(self.LOW_SPEEDS) * 2.)

        def residual(parameters):
            delta, vs = parameters
            return (delta * np.exp(-(speeds / vs)**2) - excesses) / scale
        results = [least_squares(
            residual, [max(float(excesses.max()), 0.01 * scale), initial_vs],
            bounds=([0., vs_bounds[0]], [np.inf, vs_bounds[1]]),
            x_scale=[scale, 0.02], loss='soft_l1', f_scale=0.01, max_nfev=500)
            for initial_vs in (0.006, 0.015, 0.03, 0.06)]
        results = [r for r in results if r.success and np.isfinite(r.x).all()]
        if not results:
            raise ValueError(f'Joint {joint+1}: Stribeck fit did not converge')
        best = min(results, key=lambda r: r.cost)
        delta, vs = best.x
        rmse = float(np.sqrt(np.mean((residual(best.x) * scale)**2)))
        # An apparently successful solver must not manufacture an unobserved curve.
        if delta <= max(1e-4, 3. * rmse):
            raise ValueError(f'Joint {joint+1}: Stribeck excess is not identifiable above residual noise')
        if speeds.min() > 0.8 * vs or speeds.max() < 1.5 * vs:
            raise ValueError(f'Joint {joint+1}: low-speed samples do not bracket fitted vs={vs:.6g} rad/s')
        if rmse > max(0.002, 0.01 * fc, 0.15 * delta):
            raise ValueError(f'Joint {joint+1}: low-speed Stribeck residual too large ({rmse:.6g} Nm)')
        return fc + delta, vs, rmse, count
