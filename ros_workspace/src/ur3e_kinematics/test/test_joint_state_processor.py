"""Tests for timestamped joint-state processing."""

import pytest

from ur3e_kinematics.joint_state_processor import JointStateProcessor

JOINTS = ('joint_a', 'joint_b')


def test_maps_values_by_joint_name_and_ignores_extra_joints() -> None:
    """Select configured joints independent of message ordering."""
    processor = JointStateProcessor(JOINTS)

    sample = processor.process(
        timestamp=1.0,
        joint_names=('joint_b', 'gripper', 'joint_a'),
        positions=(2.0, 0.5, 1.0),
        velocities=(0.2, 0.0, 0.1),
    )

    assert sample is not None
    assert sample.positions == {'joint_a': 1.0, 'joint_b': 2.0}
    assert sample.velocities == {'joint_a': 0.1, 'joint_b': 0.2}


def test_calculates_acceleration_from_velocity_samples() -> None:
    """Leave initial acceleration undefined and differentiate later samples."""
    processor = JointStateProcessor(JOINTS)
    first_sample = processor.process(
        timestamp=1.0,
        joint_names=JOINTS,
        positions=(0.0, 0.0),
        velocities=(1.0, 2.0),
    )
    second_sample = processor.process(
        timestamp=1.5,
        joint_names=JOINTS,
        positions=(0.5, 1.0),
        velocities=(2.0, 1.0),
    )

    assert first_sample is not None
    assert first_sample.accelerations == {'joint_a': None, 'joint_b': None}
    assert second_sample is not None
    assert second_sample.accelerations == {'joint_a': 2.0, 'joint_b': -2.0}


def test_estimates_velocity_when_message_velocity_is_empty() -> None:
    """Estimate velocity and acceleration from timestamped positions."""
    processor = JointStateProcessor(JOINTS)
    processor.process(0.0, JOINTS, (0.0, 0.0))
    second_sample = processor.process(1.0, JOINTS, (2.0, 4.0))
    third_sample = processor.process(3.0, JOINTS, (8.0, 16.0))

    assert second_sample is not None
    assert second_sample.velocities == {'joint_a': 2.0, 'joint_b': 4.0}
    assert second_sample.accelerations == {'joint_a': None, 'joint_b': None}
    assert third_sample is not None
    assert third_sample.velocities == {'joint_a': 3.0, 'joint_b': 6.0}
    assert third_sample.accelerations == {'joint_a': 0.5, 'joint_b': 1.0}


def test_returns_none_when_an_arm_joint_is_missing() -> None:
    """Skip messages that do not contain every configured joint."""
    processor = JointStateProcessor(JOINTS)

    sample = processor.process(
        timestamp=1.0,
        joint_names=('joint_a',),
        positions=(0.0,),
        velocities=(0.0,),
    )

    assert sample is None


def test_rejects_mismatched_array_lengths() -> None:
    """Reject malformed state arrays instead of silently truncating them."""
    processor = JointStateProcessor(JOINTS)

    with pytest.raises(ValueError, match='equal lengths'):
        processor.process(1.0, JOINTS, (0.0,))

    with pytest.raises(ValueError, match='match the joint names'):
        processor.process(1.0, JOINTS, (0.0, 0.0), (0.0,))


def test_non_increasing_timestamp_does_not_corrupt_acceleration_history() -> None:
    """Ignore duplicate timestamps when updating the derivative baseline."""
    processor = JointStateProcessor(JOINTS)
    processor.process(1.0, JOINTS, (0.0, 0.0), (1.0, 1.0))
    duplicate_sample = processor.process(
        1.0, JOINTS, (0.0, 0.0), (10.0, 10.0)
    )
    next_sample = processor.process(2.0, JOINTS, (0.0, 0.0), (3.0, 3.0))

    assert duplicate_sample is not None
    assert duplicate_sample.accelerations == {'joint_a': None, 'joint_b': None}
    assert next_sample is not None
    assert next_sample.accelerations == {'joint_a': 2.0, 'joint_b': 2.0}
