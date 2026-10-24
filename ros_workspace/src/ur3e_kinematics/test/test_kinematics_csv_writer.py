"""Tests for CSV storage of processed joint-state samples."""

import csv

from ur3e_kinematics.joint_state_processor import JointStateSample
from ur3e_kinematics.kinematics_csv_writer import KinematicsCsvWriter


def test_writes_timestamp_and_kinematics_for_each_joint(tmp_path) -> None:
    """Write all processed values and leave undefined derivatives empty."""
    output_path = tmp_path / 'kinematics.csv'
    joint_names = ('joint_a', 'joint_b')
    sample = JointStateSample(
        timestamp=12.5,
        positions={'joint_a': 0.1, 'joint_b': -0.2},
        velocities={'joint_a': 0.3, 'joint_b': None},
        accelerations={'joint_a': -0.4, 'joint_b': None},
    )
    writer = KinematicsCsvWriter(output_path, joint_names)
    writer.write_sample(sample)
    writer.close()

    with output_path.open(newline='', encoding='utf-8') as csv_file:
        rows = list(csv.DictReader(csv_file))

    assert rows == [
        {
            'timestamp_s': '12.5',
            'joint_a_position_rad': '0.1',
            'joint_a_velocity_rad_s': '0.3',
            'joint_a_acceleration_rad_s2': '-0.4',
            'joint_b_position_rad': '-0.2',
            'joint_b_velocity_rad_s': '',
            'joint_b_acceleration_rad_s2': '',
        }
    ]
