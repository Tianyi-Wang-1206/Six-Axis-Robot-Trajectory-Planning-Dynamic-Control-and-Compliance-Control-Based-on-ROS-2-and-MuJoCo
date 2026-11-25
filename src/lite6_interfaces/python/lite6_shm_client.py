"""Python ctypes client for Lite6 lock-free shared-memory registers."""

import ctypes
import os
from ament_index_python.packages import get_package_prefix


def _load_bridge_lib():
    candidates = []
    try:
        prefix = get_package_prefix("lite6_interfaces")
        candidates.append(os.path.join(prefix, "lib", "liblite6_shm_bridge.so"))
    except Exception:
        pass
    candidates.append("liblite6_shm_bridge.so")
    last_err = None
    for path in candidates:
        try:
            lib = ctypes.CDLL(path)
            return lib
        except OSError as exc:
            last_err = exc
    raise RuntimeError(f"Failed to load liblite6_shm_bridge.so: {last_err}")


class JointStatePod(ctypes.Structure):
    _fields_ = [
        ("position", ctypes.c_double * 6),
        ("velocity", ctypes.c_double * 6),
        ("effort", ctypes.c_double * 6),
        ("gripper_position", ctypes.c_double * 2),
        ("gripper_velocity", ctypes.c_double * 2),
        ("ros_stamp_ns", ctypes.c_int64),
        ("steady_stamp_ns", ctypes.c_int64),
        ("sequence", ctypes.c_uint64),
        ("valid", ctypes.c_uint8),
        ("reserved", ctypes.c_uint8 * 7),
    ]


class SysIdSamplePod(ctypes.Structure):
    _fields_ = [
        ("stamp_ns", ctypes.c_int64),
        ("read_stamp_ns", ctypes.c_int64),
        ("simulation_time", ctypes.c_double),
        ("step", ctypes.c_uint64),
        ("command_sequence", ctypes.c_uint64),
        ("control_period", ctypes.c_double),
        ("physics_lag", ctypes.c_double),
        ("command_age", ctypes.c_double),
        ("position", ctypes.c_double * 6),
        ("velocity", ctypes.c_double * 6),
        ("effort", ctypes.c_double * 6),
        ("gripper_position", ctypes.c_double * 2),
        ("gripper_velocity", ctypes.c_double * 2),
        ("gripper_acceleration", ctypes.c_double * 2),
    ]


SHM_OPEN_OK = 0
SHM_OPEN_NOT_FOUND = 1
SHM_OPEN_INCOMPATIBLE = 2
SHM_OPEN_OS_ERROR = 3

_LIB = _load_bridge_lib()

_LIB.lite6_shm_steady_now_ns.argtypes = []
_LIB.lite6_shm_steady_now_ns.restype = ctypes.c_int64

_LIB.lite6_shm_is_stamp_fresh.argtypes = [ctypes.c_int64, ctypes.c_int64, ctypes.c_int64]
_LIB.lite6_shm_is_stamp_fresh.restype = ctypes.c_int

_LIB.lite6_shm_joint_pod_size.argtypes = []
_LIB.lite6_shm_joint_pod_size.restype = ctypes.c_size_t

_LIB.lite6_shm_sysid_pod_size.argtypes = []
_LIB.lite6_shm_sysid_pod_size.restype = ctypes.c_size_t

_LIB.lite6_shm_state_open_ex.argtypes = [ctypes.c_int, ctypes.POINTER(ctypes.c_int)]
_LIB.lite6_shm_state_open_ex.restype = ctypes.c_void_p

_LIB.lite6_shm_state_open.argtypes = [ctypes.c_int]
_LIB.lite6_shm_state_open.restype = ctypes.c_void_p

_LIB.lite6_shm_state_close.argtypes = [ctypes.c_void_p]
_LIB.lite6_shm_state_close.restype = None

_LIB.lite6_shm_read_joint_state.argtypes = [
    ctypes.c_void_p,
    ctypes.POINTER(JointStatePod),
    ctypes.POINTER(ctypes.c_uint64),
]
_LIB.lite6_shm_read_joint_state.restype = ctypes.c_int

_LIB.lite6_shm_sysid_open_ex.argtypes = [ctypes.c_int, ctypes.POINTER(ctypes.c_int)]
_LIB.lite6_shm_sysid_open_ex.restype = ctypes.c_void_p

_LIB.lite6_shm_sysid_open.argtypes = [ctypes.c_int]
_LIB.lite6_shm_sysid_open.restype = ctypes.c_void_p

_LIB.lite6_shm_sysid_close.argtypes = [ctypes.c_void_p]
_LIB.lite6_shm_sysid_close.restype = None

_LIB.lite6_shm_sysid_set_active.argtypes = [
    ctypes.c_void_p,
    ctypes.c_int,
    ctypes.c_int64,
]
_LIB.lite6_shm_sysid_set_active.restype = None

_LIB.lite6_shm_sysid_discard_all.argtypes = [ctypes.c_void_p]
_LIB.lite6_shm_sysid_discard_all.restype = None

_LIB.lite6_shm_sysid_pop_batch.argtypes = [
    ctypes.c_void_p,
    ctypes.POINTER(SysIdSamplePod),
    ctypes.c_size_t,
]
_LIB.lite6_shm_sysid_pop_batch.restype = ctypes.c_size_t

_LIB.lite6_shm_sysid_dropped.argtypes = [ctypes.c_void_p]
_LIB.lite6_shm_sysid_dropped.restype = ctypes.c_uint64

if ctypes.sizeof(JointStatePod) != _LIB.lite6_shm_joint_pod_size():
    raise RuntimeError("ABI mismatch for JointStatePod")
if ctypes.sizeof(SysIdSamplePod) != _LIB.lite6_shm_sysid_pod_size():
    raise RuntimeError("ABI mismatch for SysIdSamplePod")


def steady_now_ns() -> int:
    """Return current steady clock nanoseconds matching C++ std::chrono::steady_clock."""
    return int(_LIB.lite6_shm_steady_now_ns())


def is_stamp_fresh(stamp_ns: int, now_ns: int, max_age_ns: int) -> bool:
    """Return True iff stamp_ns is positive, not in the future, and within max_age_ns."""
    return bool(_LIB.lite6_shm_is_stamp_fresh(int(stamp_ns), int(now_ns), int(max_age_ns)))


class StateShmReader:
    """Read-only shared-memory reader for the joint state register."""

    def __init__(self):
        self._handle = None
        self._pod = JointStatePod()
        self._seq = ctypes.c_uint64(0)
        self._try_open()

    def _try_open(self) -> bool:
        if self._handle:
            return True
        status = ctypes.c_int(SHM_OPEN_OS_ERROR)
        handle = _LIB.lite6_shm_state_open_ex(0, ctypes.byref(status))
        if handle:
            self._handle = handle
            return True
        if status.value == SHM_OPEN_NOT_FOUND:
            return False
        if status.value == SHM_OPEN_INCOMPATIBLE:
            raise RuntimeError(
                "Incompatible Lite6 state shared memory segment (magic/version/size mismatch)"
            )
        raise RuntimeError(
            f"Failed to open read-only Lite6 state shared memory (status={status.value})"
        )

    def read_joint_state(self):
        """Return (JointStatePod, seq) when valid, or (None, 0)."""
        if not self._handle and not self._try_open():
            return None, 0
        ok = _LIB.lite6_shm_read_joint_state(
            self._handle, ctypes.byref(self._pod), ctypes.byref(self._seq)
        )
        if not ok:
            return None, 0
        return self._pod, int(self._seq.value)

    def close(self):
        if getattr(self, "_handle", None):
            _LIB.lite6_shm_state_close(self._handle)
            self._handle = None

    def __del__(self):
        self.close()


class SysIdShmReader:
    """SPSC shared-memory ring consumer for lossless 1 kHz SysID samples."""

    def __init__(self, batch_size: int = 256):
        status = ctypes.c_int(SHM_OPEN_OS_ERROR)
        self._handle = _LIB.lite6_shm_sysid_open_ex(1, ctypes.byref(status))
        if not self._handle:
            if status.value == SHM_OPEN_INCOMPATIBLE:
                raise RuntimeError(
                    "Incompatible Lite6 SysID shared memory segment (magic/version/size mismatch)"
                )
            raise RuntimeError(
                f"Failed to open Lite6 SysID shared memory (status={status.value})"
            )
        self._batch_size = int(batch_size)
        self._batch_buf = (SysIdSamplePod * self._batch_size)()

    def set_active(self, active: bool):
        if not self._handle:
            return
        _LIB.lite6_shm_sysid_set_active(
            self._handle, 1 if active else 0, steady_now_ns()
        )

    def discard_all(self):
        if self._handle:
            _LIB.lite6_shm_sysid_discard_all(self._handle)

    def pop_all(self):
        """Drain all queued SysID samples and return a list of SysIdSamplePod copies."""
        if not self._handle:
            return []
        out = []
        while True:
            count = int(
                _LIB.lite6_shm_sysid_pop_batch(
                    self._handle, self._batch_buf, self._batch_size
                )
            )
            if count <= 0:
                break
            for i in range(count):
                sample = SysIdSamplePod()
                ctypes.memmove(
                    ctypes.byref(sample),
                    ctypes.byref(self._batch_buf[i]),
                    ctypes.sizeof(SysIdSamplePod),
                )
                out.append(sample)
            if count < self._batch_size:
                break
        return out

    def dropped_count(self) -> int:
        if not self._handle:
            return 0
        return int(_LIB.lite6_shm_sysid_dropped(self._handle))

    def close(self):
        if getattr(self, "_handle", None):
            try:
                _LIB.lite6_shm_sysid_set_active(self._handle, 0, 0)
            finally:
                _LIB.lite6_shm_sysid_close(self._handle)
                self._handle = None

    def __del__(self):
        self.close()
