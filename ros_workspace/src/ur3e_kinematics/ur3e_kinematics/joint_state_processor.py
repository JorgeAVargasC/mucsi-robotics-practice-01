"""Process timestamped joint-state samples for the UR3e arm."""

from collections.abc import Sequence
from dataclasses import dataclass
from math import isfinite

from ur3e_kinematics.constants import ARM_JOINTS


@dataclass(frozen=True)
class JointStateSample:
    """Hold one processed sample for each configured arm joint."""

    timestamp: float
    positions: dict[str, float]
    velocities: dict[str, float | None]
    accelerations: dict[str, float | None]


class JointStateProcessor:
    """Map joint-state arrays and calculate missing kinematic values."""

    def __init__(self, joint_names: Sequence[str] = ARM_JOINTS) -> None:
        """Initialize the processor with the joints to extract."""
        if not joint_names:
            raise ValueError('At least one joint name is required.')
        if len(set(joint_names)) != len(joint_names):
            raise ValueError('Joint names must be unique.')

        self._joint_names = tuple(joint_names)
        self._previous_timestamp: float | None = None
        self._previous_positions: dict[str, float] | None = None
        self._previous_velocities: dict[str, float | None] | None = None

    def process(
        self,
        timestamp: float,
        joint_names: Sequence[str],
        positions: Sequence[float],
        velocities: Sequence[float] = (),
    ) -> JointStateSample | None:
        """
        Return a sample, or None if the message lacks a configured joint.

        The timestamp is expressed in seconds. Velocities are read from the
        message when present; otherwise they are estimated from positions.
        Accelerations are estimated from consecutive velocity samples.
        """
        sample_time = float(timestamp)
        if not isfinite(sample_time):
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

        arm_positions = {
            name: float(position_by_name[name]) for name in self._joint_names
        }
        if not all(isfinite(value) for value in arm_positions.values()):
            raise ValueError('Position values must be finite.')

        delta_time = self._positive_delta(sample_time)
        velocity_by_name = dict(zip(joint_names, velocities))
        arm_velocities: dict[str, float | None] = {}
        for name in self._joint_names:
            if velocities:
                velocity = float(velocity_by_name[name])
            elif delta_time is not None and self._previous_positions is not None:
                velocity = (
                    arm_positions[name] - self._previous_positions[name]
                ) / delta_time
            else:
                velocity = None
            arm_velocities[name] = velocity

        if not all(
            value is None or isfinite(value)
            for value in arm_velocities.values()
        ):
            raise ValueError('Velocity values must be finite.')

        arm_accelerations: dict[str, float | None] = {}
        for name in self._joint_names:
            previous_velocity = (
                self._previous_velocities[name]
                if self._previous_velocities is not None
                else None
            )
            current_velocity = arm_velocities[name]
            if (
                delta_time is not None
                and previous_velocity is not None
                and current_velocity is not None
            ):
                acceleration = (
                    current_velocity - previous_velocity
                ) / delta_time
            else:
                acceleration = None
            arm_accelerations[name] = acceleration

        if (
            self._previous_timestamp is None
            or sample_time > self._previous_timestamp
        ):
            self._previous_timestamp = sample_time
            self._previous_positions = arm_positions
            self._previous_velocities = arm_velocities

        return JointStateSample(
            timestamp=sample_time,
            positions=arm_positions,
            velocities=arm_velocities,
            accelerations=arm_accelerations,
        )

    def _positive_delta(self, timestamp: float) -> float | None:
        """Return elapsed seconds when the timestamp advances."""
        if self._previous_timestamp is None:
            return None
        delta_time = timestamp - self._previous_timestamp
        return delta_time if delta_time > 0.0 else None
