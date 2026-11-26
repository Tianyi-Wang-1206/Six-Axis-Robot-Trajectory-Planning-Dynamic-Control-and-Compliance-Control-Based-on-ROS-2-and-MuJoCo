import os
import json
from datetime import datetime, timezone
from pathlib import Path

import coal
import numpy as np
import pinocchio as pin
import yaml
from ament_index_python.packages import get_package_share_directory
from scipy.optimize import lsq_linear


class SysIdEngine:
    """Fourier identification of viscous and Coulomb friction via momentum integral."""

    ARMATURE = 0.1  # Known factory inertia [kg m^2], identical on all six axes.
    SAMPLE_PERIOD = 0.001  # Same 1 ms grid as the CTC control loop and planner.
    PHYSICS_STEP = 0.0001  # Fixed 0.1 ms MuJoCo physics step.
    MIN_FIT_SPEED = 0.08  # rad/s; Stribeck excess < 2e-8 above this threshold.
    SETTLE_DISPLACEMENT = 0.008  # rad; unidirectional travel past reversal for LuGre bristle settling.
    REVERSAL_SPEED_EPS = 1e-4  # rad/s; near-zero speed boundary for reversal tracking.
    WINDOW_DURATION = 0.04  # s; momentum-integral regression window width.
    WINDOW_STRIDE = 0.02  # s; step between consecutive regression windows.
    MIN_WINDOW_DURATION = 0.02  # s; minimum contiguous window length inside a settled run.
    MIN_FIT_WINDOWS = 40  # Minimum settled momentum-integral windows per joint.
    TANH_SLOPE = 300.0
    LIMIT_MARGIN = 0.90
    MIN_DURATION = 6.0  # Keep useful recording time after the 1 s end trims.
    MAX_DURATION = 120.0

    def __init__(self):
        desc = get_package_share_directory('lite6_description')
        kinematics = get_package_share_directory('lite6_kinematics')
        # Full dynamics uses measured fingers; preflight uses their complete swept envelope.
        self.dynamics_model = pin.buildModelFromUrdf(os.path.join(desc, 'urdf', 'lite6.urdf'))
        self.dynamics_data = self.dynamics_model.createData()
        if self.dynamics_model.nq != 8 or self.dynamics_model.nv != 8:
            raise ValueError('Expected six arm and two gripper coordinates')
        urdf = os.path.join(desc, 'urdf', 'lite6_arm.urdf')
        self.model = pin.buildModelFromUrdf(urdf)
        self.data = self.model.createData()
        with open(os.path.join(kinematics, 'config', 'kinematic_limits.yaml')) as stream:
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
                                 os.path.join(kinematics, 'config', 'lite6.srdf'))
        with open(os.path.join(kinematics, 'config', 'ik_config.yaml')) as stream:
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

    @classmethod
    def _settled_sliding_mask(cls, positions, velocities):
        """Return mask of samples with speed >= MIN_FIT_SPEED and settled post-reversal travel."""
        n_samples, n_joints = velocities.shape
        mask = np.zeros((n_samples, n_joints), dtype=bool)
        for i in range(n_joints):
            last_rev_q = positions[0, i]
            current_sign = 0
            for k in range(n_samples):
                v = velocities[k, i]
                if abs(v) < cls.REVERSAL_SPEED_EPS:
                    last_rev_q = positions[k, i]
                    current_sign = 0
                else:
                    sign = 1 if v > 0.0 else -1
                    if sign != current_sign:
                        last_rev_q = positions[k, i]
                        current_sign = sign
                    if (abs(v) >= cls.MIN_FIT_SPEED and
                            abs(positions[k, i] - last_rev_q) >= cls.SETTLE_DISPLACEMENT):
                        mask[k, i] = True
        return mask

    def generate_fourier_coefficients(self, q0, max_pos_limits):
        """Choose a collision-free Fourier shape within amplitude caps and rest endpoints.
        Use analytic derivative bounds with a 10% margin.
        """
        q0 = np.asarray(q0, dtype=float)
        amplitudes = np.asarray(max_pos_limits, dtype=float)
        if (q0.shape != (6,) or amplitudes.shape != (6,) or
                not np.isfinite(q0).all() or not np.isfinite(amplitudes).all() or
                np.any(amplitudes <= 0)):
            raise ValueError('Friction identification needs six finite positions and positive amplitudes')
        room = np.minimum(q0 - self.limits[:, 0], self.limits[:, 1] - q0)
        if np.any(room <= 0):
            raise ValueError('Friction identification center must be strictly inside every joint limit')
        if self._in_collision(q0):
            raise ValueError('Friction identification center is in collision')
        amplitudes = np.minimum(amplitudes, room) * self.LIMIT_MARGIN
        rng = np.random.RandomState(42)  # Local, reproducible candidate search.
        base_duration = 10.0
        w = 2 * np.pi / base_duration
        times = np.linspace(0, base_duration, 2049)
        half_step = (times[1] - times[0]) / 2
        wl = w * np.arange(1, self.N_f + 1)
        shapes = []
        # Generate a diverse pool mixing smoother (1/l) and richer (1/sqrt(l)) harmonic
        # spectra so every axis gets clean settled sliding windows across speed magnitudes.
        for candidate_idx in range(64):
            a = np.zeros_like(self.a)
            b = np.zeros_like(self.b)
            decay = 1.0 if candidate_idx < 32 else 0.5
            weights = np.arange(1, self.N_f + 1, dtype=float) ** (-decay)
            for i in range(6):
                a[i, :-1] = rng.uniform(-1, 1, self.N_f - 1) * weights[:-1]
                b[i, :-2] = rng.uniform(-1, 1, self.N_f - 2) * weights[:-2]
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
                q_cand, v, acc, jerk = self._wave(a, b, w, times)
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
                t_scaled = times * stretch
                time_window = (t_scaled > 1.0) & (t_scaled < duration - 1.0)
                settled = self._settled_sliding_mask(q_cand, speed) & time_window[:, None]
                # E-optimal score of [velocity, smooth sign] on settled post-reversal
                # sliding samples, scaled by each axis speed limit.
                score = np.inf
                sample_dt = duration / (len(times) - 1)
                for i in range(6):
                    vi = speed[settled[:, i], i]
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
                    f'Adaptive Friction ID: {duration:.2f} s, peak excursions [rad] '
                    f'{np.round(np.max(np.abs(offsets), axis=0), 4).tolist()}')
                return
        raise ValueError('No sufficiently exciting collision-free friction identification trajectory within limits; '
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
            raise ValueError('Friction identification duration must be an integer number of control ticks')
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
        """Integrate on exact physics time; use ROS time only to locate the experiment."""
        samples = self.record_data
        if len(samples) < 5:
            raise ValueError('Insufficient physical snapshots for friction identification')
        steps = np.array([d['step'] for d in samples], dtype=np.int64)
        command_seq = np.array([d['command_sequence'] for d in samples], dtype=np.int64)
        simulation = np.array([d['simulation_time'] for d in samples], dtype=float)
        phase = np.array([d['t'] for d in samples], dtype=float)
        timing = np.array([[d[k] for k in ('control_period', 'physics_lag', 'command_age')]
                           for d in samples], dtype=float)
        if (not np.isfinite(simulation).all() or not np.isfinite(phase).all() or
                not np.isfinite(timing).all() or np.any(timing < 0) or
                np.any(np.diff(steps) < 0) or np.any(np.diff(simulation) < 0)):
            raise ValueError('Invalid or reset physical snapshot clock')
        if not np.allclose(np.diff(simulation), np.diff(steps) * self.PHYSICS_STEP,
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
        return (simulation[unique] + offset, command_seq[unique], timing[unique, 2], *(
            np.array([d[key] for d in samples], dtype=float)[unique]
            for key in ('q', 'dq', 'tau_cmd', 'gripper_q', 'gripper_dq', 'gripper_ddq')))

    @classmethod
    def _integrate_zoh_torque(cls, dt_seq, cmd_seq, cmd_age, tau_seq):
        """Integrate ZOH torque over physics intervals using command_sequence and
        command_age to locate a single intra-interval transition.
        """
        dseq = np.diff(cmd_seq)
        tau_int = 0.5 * (tau_seq[:-1] + tau_seq[1:]) * dt_seq[:, None]

        same = (dseq == 0)
        if np.any(same):
            tau_int[same] = tau_seq[1:][same] * dt_seq[same, None]

        single_switch = (dseq == 1)
        if np.any(single_switch):
            dt_new = np.clip(
                np.floor(cmd_age[1:][single_switch] / cls.PHYSICS_STEP + 1e-9) * cls.PHYSICS_STEP,
                cls.PHYSICS_STEP,
                dt_seq[single_switch],
            )
            dt_old = dt_seq[single_switch] - dt_new
            tau_int[single_switch] = (
                tau_seq[:-1][single_switch] * dt_old[:, None] +
                tau_seq[1:][single_switch] * dt_new[:, None]
            )
        return tau_int

    def calculate_least_squares(self) -> str:
        """Fit nonnegative Fv/Fc with momentum integrals over settled sliding windows:
        dp/dt = tau_act - tau_f - g + C^T V, p = (M V)[:6] + J_a dq.
        """
        self.fit_diagnostics = []
        (t_seq, cmd_seq, cmd_age, q_seq, dq_seq, tau_seq,
         gq_seq, gv_seq, ga_seq) = self._physical_samples()
        # Trim to the excitation window so any telemetry gap before/after excitation
        # does not reject a lossless identification window.
        if len(t_seq) >= 5:
            win = (t_seq >= -0.05) & (t_seq <= self.T_total + 0.05)
            if np.count_nonzero(win) >= 5:
                t_seq = t_seq[win]
                cmd_seq = cmd_seq[win]
                cmd_age = cmd_age[win]
                q_seq = q_seq[win]
                dq_seq = dq_seq[win]
                tau_seq = tau_seq[win]
                gq_seq = gq_seq[win]
                gv_seq = gv_seq[win]
                ga_seq = ga_seq[win]
        if len(t_seq) < 5:
            raise ValueError('Fewer than five distinct physical snapshots')
        if (not np.isfinite(t_seq).all() or np.any(np.diff(t_seq) <= 0) or
                any(x.shape != (len(t_seq), 6) or not np.isfinite(x).all()
                    for x in (q_seq, dq_seq, tau_seq))):
            raise ValueError('Invalid friction identification samples or non-increasing timestamps')
        if any(x.shape != (len(t_seq), 2) or not np.isfinite(x).all()
               for x in (gq_seq, gv_seq, ga_seq)):
            raise ValueError('Missing or invalid gripper dynamics feedback')
        if t_seq[0] > 0.25 or t_seq[-1] < self.T_total - 0.25:
            raise ValueError(f'Incomplete excitation recording: {t_seq[0]:.4f}..{t_seq[-1]:.4f} s; '
                             f'expected 0..{self.T_total:.4f} s on the CTC clock')
        dt_seq = np.diff(t_seq)
        dt_med = float(np.median(dt_seq))
        if np.max(dt_seq) > max(0.05, 5.0 * dt_med):
            raise ValueError('Friction identification recording has telemetry gaps; refusing to interpolate missing experiments')

        n_samples = len(t_seq)
        full_q = np.column_stack((q_seq, gq_seq))
        full_v = np.column_stack((dq_seq, gv_seq))

        # Compute exact arm generalized momentum p_k and conservative/Coriolis terms h_k = C^T V - g.
        momentum = np.empty((n_samples, 6), dtype=float)
        ctv_minus_g = np.empty((n_samples, 6), dtype=float)
        for k in range(n_samples):
            qi = full_q[k]
            vi = full_v[k]
            mass_mat = pin.crba(self.dynamics_model, self.dynamics_data, qi)
            mass_mat = np.triu(mass_mat) + np.triu(mass_mat, 1).T
            coriolis = pin.computeCoriolisMatrix(self.dynamics_model, self.dynamics_data, qi, vi)
            gravity = pin.computeGeneralizedGravity(self.dynamics_model, self.dynamics_data, qi)
            momentum[k] = (mass_mat @ vi)[:6] + self.ARMATURE * dq_seq[k]
            ctv_minus_g[k] = (coriolis.T @ vi - gravity)[:6]

        # Exact cumulative integrals directly on non-uniform physics timestamps.
        tau_int = self._integrate_zoh_torque(dt_seq, cmd_seq, cmd_age, tau_seq)
        ctv_g_int = 0.5 * (ctv_minus_g[:-1] + ctv_minus_g[1:]) * dt_seq[:, None]
        tanh_vals = np.tanh(self.TANH_SLOPE * dq_seq)
        tanh_int = 0.5 * (tanh_vals[:-1] + tanh_vals[1:]) * dt_seq[:, None]

        cum_rhs = np.vstack((np.zeros((1, 6)), np.cumsum(tau_int + ctv_g_int, axis=0)))
        cum_tanh = np.vstack((np.zeros((1, 6)), np.cumsum(tanh_int, axis=0)))

        time_mask = (t_seq > 1.0) & (t_seq < self.T_total - 1.0)
        settled_mask = self._settled_sliding_mask(q_seq, dq_seq) & time_mask[:, None]
        good_step = dt_seq <= max(0.01, 4.0 * dt_med)

        friction = np.zeros((6, 2))
        report = []
        failures = []
        for i in range(6):
            try:
                sample_count = int(np.count_nonzero(settled_mask[:, i]))
                if sample_count < 200:
                    raise ValueError(
                        f'Joint {i+1}: fewer than 200 settled high-speed friction samples '
                        f'({sample_count})')

                rows_x = []
                rows_y = []
                k = 0
                while k < n_samples:
                    if not settled_mask[k, i]:
                        k += 1
                        continue
                    run_start = k
                    while k + 1 < n_samples and settled_mask[k + 1, i] and good_step[k]:
                        k += 1
                    run_end = k
                    a_idx = run_start
                    while a_idx < run_end:
                        b_idx = a_idx + 1
                        while b_idx < run_end and (t_seq[b_idx] - t_seq[a_idx]) < self.WINDOW_DURATION:
                            b_idx += 1
                        dt_w = t_seq[b_idx] - t_seq[a_idx]
                        if dt_w >= self.MIN_WINDOW_DURATION:
                            mean_v = (q_seq[b_idx, i] - q_seq[a_idx, i]) / dt_w
                            mean_tanh = (cum_tanh[b_idx, i] - cum_tanh[a_idx, i]) / dt_w
                            mean_tau_f = (
                                (cum_rhs[b_idx, i] - cum_rhs[a_idx, i]) -
                                (momentum[b_idx, i] - momentum[a_idx, i])
                            ) / dt_w
                            rows_x.append((mean_v, mean_tanh))
                            rows_y.append(mean_tau_f)
                        next_a = a_idx + 1
                        while next_a < run_end and (t_seq[next_a] - t_seq[a_idx]) < self.WINDOW_STRIDE:
                            next_a += 1
                        a_idx = next_a
                    k += 1

                if len(rows_x) < self.MIN_FIT_WINDOWS:
                    raise ValueError(
                        f'Joint {i+1}: fewer than {self.MIN_FIT_WINDOWS} settled integral windows '
                        f'({len(rows_x)})')

                design = np.asarray(rows_x, dtype=float)
                target = np.asarray(rows_y, dtype=float)
                norms = np.linalg.norm(design, axis=0)
                if np.any(norms <= 0.0):
                    raise ValueError(f'Joint {i+1}: degenerate excitation design matrix')
                scaled = design / norms
                if np.linalg.cond(scaled) > 1000:
                    raise ValueError(f'Joint {i+1}: insufficient velocity variation for Fv/Fc fit')
                result = lsq_linear(design, target, bounds=(0.0, np.inf))
                if not result.success or not np.isfinite(result.x).all():
                    raise ValueError(f'Joint {i+1}: friction fit failed')
                friction[i] = result.x
                high_rmse = np.sqrt(np.mean((design @ result.x - target)**2))
                report.append(f'# joint{i+1}: RMSE={high_rmse:.6g} Nm '
                              f'({len(rows_x)} windows, {sample_count} settled samples)')
            except ValueError as exc:
                failures.append(str(exc))
        if failures:
            raise ValueError('Identification quality checks failed:\n' + '\n'.join(failures) +
                             '\nTiming diagnostics:\n' + '\n'.join(self.fit_diagnostics))

        def format_list(values):
            return '[' + ', '.join(f'{value:.8g}' for value in values) + ']'

        return ('# --- Identified friction parameters (not automatically applied) ---\n'
                '# Fourier excitation: viscous Fv and Coulomb Fc (momentum-integral fit).\n'
                f'friction_v_nominal: {format_list(friction[:, 0])}\n'
                f'friction_c_nominal: {format_list(friction[:, 1])}\n'
                + '\n'.join(report) + '\n'
                + '\n'.join('# ' + line for line in self.fit_diagnostics) + '\n'
                )
