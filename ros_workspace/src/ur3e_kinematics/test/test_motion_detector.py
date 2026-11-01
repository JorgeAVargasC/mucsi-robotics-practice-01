"""Tests for motion detection from joint-state samples."""

import pytest

from ur3e_kinematics.motion_detector import MotionDetector

JOINTS = ('joint_a', 'joint_b')


def test_detects_motion_from_arm_velocities_only() -> None:
    """Ignore noise and extra joints, and detect motion in either direction."""
    detector = MotionDetector(JOINTS)
    names = ('joint_b', 'gripper', 'joint_a')
    assert detector.is_moving(1.0, names, (0, 0, 0), (0.0005, 2.0, 0)) is False
    assert detector.is_moving(2.0, names, (0, 0, 0), (0, 0, -0.1)) is True
    assert detector.is_moving(3.0, names, (0, 0, 0), (0, 0, 0)) is False


def test_estimates_motion_from_positions_without_velocities() -> None:
    """Use consecutive timestamps when messages do not provide velocities."""
    detector = MotionDetector(JOINTS)
    assert detector.is_moving(1.0, JOINTS, (0, 0)) is False
    assert detector.is_moving(2.0, JOINTS, (0, 0)) is False
    assert detector.is_moving(3.0, JOINTS, (0, 0.1)) is True
    assert detector.is_moving(4.0, JOINTS, (0, 0.1)) is False


def test_ignores_non_increasing_timestamps_for_position_history() -> None:
    """A repeated timestamp must not replace the position baseline."""
    detector = MotionDetector(JOINTS)
    assert detector.is_moving(1.0, JOINTS, (0, 0)) is False
    assert detector.is_moving(1.0, JOINTS, (10, 10)) is False
    assert detector.is_moving(2.0, JOINTS, (0, 0)) is False


def test_ignores_messages_without_all_arm_joints() -> None:
    """Gripper-only messages do not change the motion state or its baseline."""
    detector = MotionDetector(JOINTS)
    assert detector.is_moving(1.0, JOINTS, (0, 0)) is False
    assert detector.is_moving(2.0, ('gripper',), (1,), (1,)) is None
    assert detector.is_moving(3.0, JOINTS, (0, 0)) is False


def test_rejects_malformed_and_nonfinite_samples() -> None:
    """Do not treat invalid data as valid movement."""
    detector = MotionDetector(JOINTS)
    with pytest.raises(ValueError, match='equal lengths'):
        detector.is_moving(1.0, JOINTS, (0,))
    with pytest.raises(ValueError, match='match the joint names'):
        detector.is_moving(1.0, JOINTS, (0, 0), (0,))
    with pytest.raises(ValueError, match='finite'):
        detector.is_moving(1.0, JOINTS, (0, 0), (float('nan'), 0))
