"""
safety_monitor.py

see readme for in-detail notes
"""

import time
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Optional, Tuple

import numpy as np


class FaultType(Enum):
    """Safety fault types."""

    VELOCITY_TRIP = auto()
    LOOP_OVERRUN = auto()
    EXCEPTION = auto()


@dataclass
class FaultRecord:
    """Record of a single safety trip."""

    fault_type: FaultType
    joint_index: Optional[int]
    measured_value: float
    threshold: float
    message: str
    timestamp: float = field(default_factory=time.time)

    def __str__(self) -> str:
        """Return a human-readable description of the fault."""
        joint = f' joint {self.joint_index}' if self.joint_index is not None else ''
        return (
            f'[{self.fault_type.name}]{joint}  '
            f'measured={self.measured_value:.4f}  '
            f'threshold={self.threshold:.4f}  '
            f't={self.timestamp:.3f}'
        )


class _LoggerAdapter:
    """Adapt rclpy Logger or stdlib logging.Logger to a common interface."""

    def __init__(self, logger: object) -> None:
        """Wrap logger."""
        self._l = logger

    def info(self, msg: str) -> None:
        """Log at INFO level."""
        if hasattr(self._l, 'info'):
            self._l.info(msg)

    def error(self, msg: str) -> None:
        """Log at ERROR level."""
        if hasattr(self._l, 'error'):
            self._l.error(msg)


class SafetyMonitor:
    # see readme

    def __init__(
        self,
        tau_limits: list,
        vel_limits: list,
        max_missed_cycles: int = 10,
        dt_nominal: float = 0.005,
        overrun_factor: float = 3.0,
        logger: object = None,
    ) -> None:
        """Initialise the SafetyMonitor."""
        self._tau_lim = np.array(tau_limits, dtype=float)
        self._vel_lim = np.array(vel_limits, dtype=float)
        self._max_miss = max_missed_cycles
        self._dt_nominal = dt_nominal
        self._overrun_factor = overrun_factor
        self._log = _LoggerAdapter(logger) if logger is not None else _LoggerAdapter(None)

        self._faulted = False
        self._fault_record: Optional[FaultRecord] = None
        self._consecutive_misses = 0

    # ── public interface ──────────────────────────────────────────────────

    @property
    def is_faulted(self) -> bool:
        """Return True if the monitor is in a latched fault state."""
        return self._faulted

    @property
    def fault_record(self) -> Optional[FaultRecord]:
        """Return the most recent FaultRecord, or None if not faulted."""
        return self._fault_record

    def reset(self) -> None:
        """Clear latched fault state; call via the ~/reset_fault service."""
        self._faulted = False
        self._fault_record = None
        self._consecutive_misses = 0
        self._log.info('Safety monitor: fault state cleared.')

    def trip_exception(self, exc: Exception) -> None:
        """Record an exception as an EXCEPTION fault."""
        self._trip(
            FaultType.EXCEPTION,
            joint_index=None,
            measured=0.0,
            threshold=0.0,
            message=f'Unhandled exception in control loop: {exc}',
        )

    def check_and_clamp(
        self,
        tau_cmd: np.ndarray,
        q_dot: np.ndarray,
        dt_actual: float,
    ) -> Tuple[np.ndarray, bool, Optional[str]]:
        # Clamp torques / check for safety violations.

        if self._faulted:
            return np.zeros_like(tau_cmd), True, str(self._fault_record)

        # ── loop-rate watchdog ────────────────────────────────────────────
        if dt_actual > self._dt_nominal * self._overrun_factor:
            self._consecutive_misses += 1
            if self._consecutive_misses >= self._max_miss:
                return self._trip(
                    FaultType.LOOP_OVERRUN,
                    joint_index=None,
                    measured=dt_actual * 1000.0,
                    threshold=self._dt_nominal * self._overrun_factor * 1000.0,
                    message=(
                        f'Control loop overrun: {self._consecutive_misses} '
                        f'consecutive cycles exceeded '
                        f'{self._dt_nominal * self._overrun_factor * 1e3:.1f} ms '
                        f'(last dt={dt_actual * 1e3:.1f} ms)'
                    ),
                    tau_cmd=tau_cmd,
                )
        else:
            self._consecutive_misses = 0

        # ── velocity watchdog ─────────────────────────────────────────────
        for i, (v, lim) in enumerate(zip(q_dot, self._vel_lim)):
            if abs(v) > lim:
                signed_lim = lim if v > 0.0 else -lim
                return self._trip(
                    FaultType.VELOCITY_TRIP,
                    joint_index=i,
                    measured=v,
                    threshold=signed_lim,
                    message=(
                        f'Joint {i} velocity {v:.4f} rad/s '
                        f'exceeds watchdog limit ±{lim:.4f} rad/s'
                    ),
                    tau_cmd=tau_cmd,
                )

        # ── torque saturation ────────────────────────────────────
        clamped = np.clip(tau_cmd, -self._tau_lim, self._tau_lim)
        return clamped, False, None

    # ── private ───────────────────────────────────────────────────────────

    def _trip(
        self,
        fault_type: FaultType,
        joint_index: Optional[int],
        measured: float,
        threshold: float,
        message: str,
        tau_cmd: Optional[np.ndarray] = None,
    ) -> Tuple[np.ndarray, bool, str]:
        """Latch fault state and return zero torques."""
        self._faulted = True
        self._fault_record = FaultRecord(
            fault_type=fault_type,
            joint_index=joint_index,
            measured_value=float(measured),
            threshold=float(threshold),
            message=message,
        )
        self._log.error(f'SAFETY TRIP — {message}')
        zeros = np.zeros_like(tau_cmd) if tau_cmd is not None else np.zeros(7)
        return zeros, True, message
