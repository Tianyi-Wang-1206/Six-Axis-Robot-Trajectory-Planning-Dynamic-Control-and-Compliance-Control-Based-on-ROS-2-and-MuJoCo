import os
import json
from datetime import datetime, timezone
from pathlib import Path

import coal
import numpy as np
import pinocchio as pin
import yaml
from ament_index_python.packages import get_package_share_directory
from scipy.interpolate import interp1d
from scipy.optimize import lsq_linear
from scipy.signal import savgol_filter


class SysIdEngine:
    """Fourier identification of viscous and Coulomb friction."""

    ARMATURE = 0.1  # Known factory inertia [kg m^2], identical on all six axes.
    SAMPLE_PERIOD = 0.001  # Same 1 ms grid as the CTC control loop and planner.
    MIN_FIT_SPEED = 0.05
    TANH_SLOPE = 300.0
    LIMIT_MARGIN = 0.90
    MIN_DURATION = 6.0  # Keep useful recording time after the 1 s end trims.
    MAX_DURATION = 120.0

    def __init__(self):
        desc = get_package_share_directory('lite6_description')
        planner = get_package_share_directory('lite6_planner')
        # Full dynamics uses measured fingers; preflight uses their complete swept envelope.
        self.dynamics_model = pin.buildModelFromUrdf(os.path.join(desc, 'urdf', 'lite6.urdf'))
        self.dynamics_data = self.dynamics_model.createData()
        if self.dynamics_model.nq != 8 or self.dynamics_model.nv != 8:
            raise ValueError('Expected six arm and two gripper coordinates')
        urdf = os.path.join(desc, 'urdf', 'lite6_arm.urdf')
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
        """Generate the bounded Fourier experiment for Fv/Fc identification."""
        if progress is not None:
            progress('Checking Fourier trajectory collisions...')
        self.generate_fourier_coefficients(q0, max_pos_limits)

    def get_trajectory_point(self, t, q0):
        return tuple(np.array(x) for x in zip(*[
            self.get_fourier_point(i, t, q0) for i in range(6)]))

    @classmethod
    def _sample_times(cls, duration):
        ticks = int(round(duration / cls.SAMPLE_PERIOD))
        if abs(ticks * cls.SAMPLE_PERIOD - duration) > 1e-8:
            raise ValueError('SysID duration must be an integer number of control ticks')
        return np.arange(ticks + 1, dtype=np.int64) * cls.SAMPLE_PERIOD

    def trajectory_times(self):
        return self._sample_times(self.T_total)

    def trajectory_samples(self, q0):
        """Generate all 1 ms references without a Python loop over Fourier harmonics."""
        times = self.trajectory_times()
        offsets, velocity, acceleration, _ = self._wave(self.a, self.b, self.w, times)
        return times, np.asarray(q0) + offsets, velocity, acceleration

    def get_fourier_point(self, i, t, q0):
        wl = self.w * np.arange(1, self.N_f + 1)
        sn, cs = np.sin(wl * t), np.cos(wl * t)
        q = q0[i] + np.sum(self.a[i] / wl * sn + self.b[i] / wl * (1 - cs))
        dq = np.sum(self.a[i] * cs + self.b[i] * sn)
        ddq = np.sum(-self.a[i] * wl * sn + self.b[i] * wl * cs)
        return q, dq, ddq

    def save_recording(self, session, origin, directory=None):
        """Save raw physical snapshots before validation; no pickle is needed to replay."""
        root = Path(directory) if directory is not None else Path(
            os.environ.get('ROS_HOME', str(Path.home() / '.ros'))) / 'lite6_sysid'
        root.mkdir(parents=True, exist_ok=True)
        path = root / (datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ') +
                       f'_{session}.npz')
        keys = ('t', 'read_time', 'simulation_time', 'step', 'command_sequence',
                'control_period', 'physics_lag', 'command_age', 'q', 'dq', 'tau_cmd',
                'gripper_q', 'gripper_dq', 'gripper_ddq')
        arrays = {key: np.asarray([sample[key] for sample in self.record_data],
                                  dtype=np.int64 if key in ('step', 'command_sequence') else float)
                  for key in keys}
        metadata = dict(schema=3, session=str(session), excitation_start_ros=origin,
                        t_is_relative=origin is not None, duration=self.T_total,
                        friction_model='viscous_coulomb', trajectory='fourier')
        # Exclusive creation protects existing experiments even on a repeated ID.
        with path.open('xb') as stream:
            np.savez_compressed(stream, **arrays, metadata=json.dumps(metadata))
        return path

    def _physical_samples(self):
        """Differentiate on physics time; use ROS time only to locate the experiment."""
        samples = self.record_data
        if len(samples) < 5:
            raise ValueError('Insufficient physical snapshots for system identification')
        steps = np.array([d['step'] for d in samples], dtype=np.int64)
        simulation = np.array([d['simulation_time'] for d in samples], dtype=float)
        phase = np.array([d['t'] for d in samples], dtype=float)
        timing = np.array([[d[k] for k in ('control_period', 'physics_lag', 'command_age')]
                           for d in samples], dtype=float)
        if (not np.isfinite(simulation).all() or not np.isfinite(phase).all() or
                not np.isfinite(timing).all() or np.any(timing < 0) or
                np.any(np.diff(steps) < 0) or np.any(np.diff(simulation) < 0)):
            raise ValueError('Invalid or reset physical snapshot clock')
        if not np.allclose(np.diff(simulation), np.diff(steps) * 0.0001,
                           rtol=0, atol=1e-7):
            raise ValueError('Physical time does not match the fixed 0.1 ms step sequence')
        # ROS-to-steady pairing has small measurement jitter; never differentiate
        # using it. A clock jump invalidates the reference-window alignment.
        offset = float(np.median(phase - simulation))
        if np.max(np.abs(phase - simulation - offset)) > 0.005:
            raise ValueError('ROS/physics clock mapping changed by more than 5 ms')
        unique = np.r_[True, np.diff(steps) > 0]
        self.fit_diagnostics.append(
            f'Timing: {len(steps)} observations, {np.count_nonzero(~unique)} repeated steps; '
            f'max physics lag={timing[:, 1].max():.6f} s, '
            f'max control period={timing[:, 0].max():.6f} s, '
            f'max applied command age={timing[:, 2].max():.6f} s')
        return (simulation[unique] + offset, *(
            np.array([d[key] for d in samples], dtype=float)[unique]
            for key in ('q', 'dq', 'tau_cmd', 'gripper_q', 'gripper_dq', 'gripper_ddq')))

    def calculate_least_squares(self) -> str:
        """Fit nonnegative Fv/Fc from measured Fourier excitation data."""
        self.fit_diagnostics = []
        t_seq, q_seq, dq_seq, tau_seq, gq_seq, gv_seq, ga_seq = self._physical_samples()
        if len(t_seq) < 5:
            raise ValueError('Fewer than five distinct physical snapshots')
        if (not np.isfinite(t_seq).all() or np.any(np.diff(t_seq) <= 0) or
                any(x.shape != (len(t_seq), 6) or not np.isfinite(x).all()
                    for x in (q_seq, dq_seq, tau_seq))):
            raise ValueError('Invalid SysID samples or non-increasing timestamps')
        if any(x.shape != (len(t_seq), 2) or not np.isfinite(x).all()
               for x in (gq_seq, gv_seq, ga_seq)):
            raise ValueError('Missing or invalid gripper dynamics feedback')
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
        gq, gv, ga = [interp1d(t_seq, values, axis=0)(t_uniform)
                      for values in (gq_seq, gv_seq, ga_seq)]
        full_q = np.column_stack((q, gq))
        full_v = np.column_stack((dq, gv))
        full_a = np.column_stack((ddq, ga))
        rigid_torque = np.array([
            pin.rnea(self.dynamics_model, self.dynamics_data, qi, vi, ai)[:6].copy()
            for qi, vi, ai in zip(full_q, full_v, full_a)])
        residual = tau - rigid_torque - self.ARMATURE * ddq
        friction = np.zeros((6, 2))
        report = []
        failures = []
        for i in range(6):
            try:
                # Retain the original Fourier-stage speed cutoff and linear solver.
                valid = ((t_uniform > 1.0) & (t_uniform < self.T_total - 1.0) &
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
                report.append(f'# joint{i+1}: RMSE={high_rmse:.6g} Nm '
                              f'({valid.sum()} samples)')
            except ValueError as exc:
                failures.append(str(exc))
        if failures:
            raise ValueError('Identification quality checks failed:\n' + '\n'.join(failures) +
                             '\nTiming diagnostics:\n' + '\n'.join(self.fit_diagnostics))

        def format_list(values):
            return '[' + ', '.join(f'{value:.8g}' for value in values) + ']'

        return ('# --- Identified friction parameters (not automatically applied) ---\n'
                '# Fourier excitation: viscous Fv and Coulomb Fc.\n'
                f'friction_v_nominal: {format_list(friction[:, 0])}\n'
                f'friction_c_nominal: {format_list(friction[:, 1])}\n'
                + '\n'.join(report) + '\n'
                + '\n'.join('# ' + line for line in self.fit_diagnostics) + '\n'
                )
