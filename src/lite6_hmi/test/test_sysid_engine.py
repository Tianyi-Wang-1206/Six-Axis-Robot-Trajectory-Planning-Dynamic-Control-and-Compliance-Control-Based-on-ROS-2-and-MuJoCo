"""Offline regression coverage: no ROS nodes or robot commands are created."""
import numpy as np
import pinocchio as pin
import pytest
import yaml
from scipy.interpolate import BPoly

from lite6_hmi.sysid_engine import SysIdEngine


CENTER = np.deg2rad([0, -30, 60, 180, 0, 0])
AMPLITUDES = np.array([0.4, 0.6, 0.5, 1.0, 1.0, 3.0])


@pytest.fixture
def engine():
    return SysIdEngine()


@pytest.mark.parametrize('center,amplitudes', [
    (CENTER, AMPLITUDES),
    (CENTER, AMPLITUDES * 0.25),
    (CENTER, np.full(6, 100.0)),
    (np.deg2rad([10, -25, 70, 160, 40, 20]), AMPLITUDES),
])
def test_adaptive_trajectory_within_limits(engine, center, amplitudes):
    engine.generate_fourier_coefficients(center, amplitudes)
    times = engine.trajectory_times()
    values = np.array([[engine.get_fourier_point(i, t, center) for i in range(6)]
                       for t in times])
    q, v, a = values[:, :, 0], values[:, :, 1], values[:, :, 2]
    np.testing.assert_allclose(q[[0, -1]], np.tile(center, (2, 1)), atol=1e-12)
    np.testing.assert_allclose(v[[0, -1]], 0, atol=1e-12)
    np.testing.assert_allclose(a[[0, -1]], 0, atol=1e-12)
    assert times[-1] == engine.T_total
    assert np.all(np.diff(times) > 0)
    assert np.max(np.diff(times)) <= engine.SAMPLE_PERIOD + 1e-12
    assert engine._path_is_clear(q)

    # Validate the quintic interpolation of transmitted q/v/a, not merely the
    # Fourier sampling grid used to generate them (JTC's interpolation path).
    curve = BPoly.from_derivatives(times, np.transpose(values, (0, 2, 1)))
    dense_times = np.linspace(0, engine.T_total, len(times) * 10)
    dense_q = curve(dense_times)
    assert np.all(dense_q >= engine.limits[:, 0])
    assert np.all(dense_q <= engine.limits[:, 1])
    assert np.all(np.max(np.abs(dense_q - center), axis=0) <= amplitudes)
    for derivative, column in ((1, 2), (2, 3), (3, 4)):
        peak = np.max(np.abs(curve.derivative(derivative)(dense_times)), axis=0)
        assert np.all(peak < engine.limits[:, column])
    # Every axis must contain useful speed variation in both directions.
    retained_v = curve.derivative()(dense_times[(dense_times > 1) &
                                               (dense_times < engine.T_total - 1)])
    for i in range(6):
        useful = retained_v[np.abs(retained_v[:, i]) >= engine.MIN_FIT_SPEED, i]
        assert np.any(useful > 0) and np.any(useful < 0)
        assert np.std(np.abs(useful)) > 0.01


def test_replanning_reproducible_without_global_rng_side_effect(engine):
    np.random.seed(123)
    expected = np.random.random(4)
    np.random.seed(123)
    engine.generate_fourier_coefficients(CENTER, AMPLITUDES)
    original = (engine.a.copy(), engine.b.copy(), engine.T_total)
    engine.generate_fourier_coefficients(CENTER, AMPLITUDES / 4)
    assert engine.T_total < original[2]
    engine.generate_fourier_coefficients(CENTER, AMPLITUDES)
    np.testing.assert_array_equal(engine.a, original[0])
    np.testing.assert_array_equal(engine.b, original[1])
    assert engine.T_total == original[2]
    np.testing.assert_array_equal(np.random.random(4), expected)


@pytest.mark.parametrize('amplitudes', [[0] * 6, [-1] * 6, [np.nan] * 6,
                                      [np.inf] * 6, [1] * 5, [1e-6] * 6])
def test_invalid_or_uninformative_excitation_rejected(engine, amplitudes):
    with pytest.raises(ValueError):
        engine.generate_fourier_coefficients(CENTER, amplitudes)


def test_boundary_center_rejected(engine):
    center = CENTER.copy()
    center[2] = engine.limits[2, 0]
    with pytest.raises(ValueError, match='strictly inside'):
        engine.generate_fourier_coefficients(center, AMPLITUDES)


def test_collision_causes_alternative_search(engine, monkeypatch):
    original = engine._path_is_clear
    calls = []

    def reject_first_shape(positions):
        calls.append(positions)
        return len(calls) > 1 and original(positions)

    monkeypatch.setattr(engine, '_path_is_clear', reject_first_shape)
    engine.generate_fourier_coefficients(CENTER, AMPLITUDES)
    assert len(calls) >= 2


def test_no_collision_free_path_fails_before_submission(engine, monkeypatch):
    monkeypatch.setattr(engine, '_path_is_clear', lambda positions: False)
    with pytest.raises(ValueError, match='collision-free'):
        engine.generate_fourier_coefficients(CENTER, AMPLITUDES)


def test_fit_subtracts_known_armature_and_returns_only_friction(engine):
    # Independent smooth excitation, with nonuniform sampling, rigid-body
    # coupling and known rotor torque. Fit must recover only the friction.
    rng = np.random.default_rng(7)
    times = np.cumsum(rng.uniform(0.0045, 0.0055, 5000))
    phase = times[:, None] * np.array([0.8, 0.6, 0.7, 1.0, 0.9, 1.2])
    frequency = np.array([0.8, 0.6, 0.7, 1.0, 0.9, 1.2])
    q = CENTER + 0.5 * np.sin(phase)
    v = 0.5 * frequency * np.cos(phase)
    a = -0.5 * frequency**2 * np.sin(phase)
    fv = np.array([1, 1, 0.5, 0.2, 0.1, 0.05])
    fc = np.array([1.5, 1.5, 1, 0.5, 0.2, 0.1])
    torque = np.array([pin.rnea(engine.model, engine.data, qi, vi, ai).copy()
                       for qi, vi, ai in zip(q, v, a)])
    torque += 0.1 * a + fv * v + fc * np.tanh(300 * v)
    engine.record_data = [dict(t=t, q=qi, dq=vi, tau_cmd=tau)
                          for t, qi, vi, tau in zip(times, q, v, torque)]
    result = yaml.safe_load(engine.calculate_least_squares())
    assert set(result) == {'friction_v_nominal', 'friction_c_nominal'}
    np.testing.assert_allclose(result['friction_v_nominal'], fv, atol=0.001)
    np.testing.assert_allclose(result['friction_c_nominal'], fc, atol=0.001)


def test_unexcited_joint_cannot_silently_produce_zero_parameters(engine):
    engine.record_data = [dict(t=k * 0.005, q=CENTER, dq=np.zeros(6),
                               tau_cmd=np.zeros(6)) for k in range(500)]
    with pytest.raises(ValueError, match='Joint 1.*200'):
        engine.calculate_least_squares()
