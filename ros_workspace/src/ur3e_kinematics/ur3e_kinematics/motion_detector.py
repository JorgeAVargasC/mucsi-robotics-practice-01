"""Detect arm motion from joint velocities or consecutive positions."""

from collections.abc import Sequence
from math import isfinite

from ur3e_kinematics.constants import ARM_JOINTS, MOTION_THRESHOLD_RAD_S


class MotionDetector:
    """Keep the previous positions when velocity must be estimated."""

    def __init__(self, joint_names: Sequence[str] = ARM_JOINTS) -> None:
        self._joint_names = tuple(joint_names)
        self._previous_timestamp: float | None = None
        self._previous_positions: dict[str, float] | None = None

    def is_moving(
        self,
        timestamp: float,
        joint_names: Sequence[str],
        positions: Sequence[float],
        velocities: Sequence[float] = (),
    ) -> bool | None:
        """Return None for messages that do not contain all arm joints."""
        if not isfinite(timestamp):
            raise ValueError('Timestamp must be finite.')
        if len(joint_names) != len(positions):
            raise ValueError('Joint names and positions must have equal lengths.')
        if velocities and len(joint_names) != len(velocities):
            raise ValueError('Velocities must be empty or match the joint names.')
        if len(set(joint_names)) != len(joint_names):
            raise ValueError('Message joint names must be unique.')

        position_by_name = dict(zip(joint_names, positions))
        if not all(name in position_by_name for name in self._joint_names):
            return None
        arm_positions = {name: float(position_by_name[name]) for name in self._joint_names}
        if not all(isfinite(value) for value in arm_positions.values()):
            raise ValueError('Position values must be finite.')

        if velocities:
            velocity_by_name = dict(zip(joint_names, velocities))
            arm_velocities = [float(velocity_by_name[name]) for name in self._joint_names]
            if not all(isfinite(value) for value in arm_velocities):
                raise ValueError('Velocity values must be finite.')
        elif self._previous_timestamp is not None and timestamp > self._previous_timestamp:
            elapsed = timestamp - self._previous_timestamp
            arm_velocities = [
                (arm_positions[name] - self._previous_positions[name]) / elapsed
                for name in self._joint_names
            ]
        else:
            arm_velocities = []

        if self._previous_timestamp is None or timestamp > self._previous_timestamp:
            self._previous_timestamp = timestamp
            self._previous_positions = arm_positions

        return any(abs(velocity) > MOTION_THRESHOLD_RAD_S for velocity in arm_velocities)
