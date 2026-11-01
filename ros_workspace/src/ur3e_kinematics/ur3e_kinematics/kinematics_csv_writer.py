"""Write processed joint-state samples to a CSV file."""

from collections.abc import Sequence
import csv
from pathlib import Path
from typing import TextIO

from ur3e_kinematics.joint_state_processor import JointStateSample


class KinematicsCsvWriter:
    """Write timestamped position, velocity, and acceleration values."""

    def __init__(self, output_path: Path, joint_names: Sequence[str]) -> None:
        """Create a CSV file with columns for the configured joints."""
        if not joint_names:
            raise ValueError('At least one joint name is required.')
        if len(set(joint_names)) != len(joint_names):
            raise ValueError('Joint names must be unique.')

        self._joint_names = tuple(joint_names)
        self.output_path = Path(output_path)
        self._fieldnames = self._create_fieldnames()
        self._file: TextIO = self.output_path.open(
            mode='w', newline='', encoding='utf-8'
        )
        self._writer = csv.DictWriter(self._file, fieldnames=self._fieldnames)
        self._writer.writeheader()

    def write_sample(self, sample: JointStateSample, trajectory: str) -> None:
        """Append one processed sample as a CSV row."""
        row: dict[str, str | float | None] = {
            'trajectory': trajectory, 'timestamp_s': sample.timestamp,
        }
        for joint_name in self._joint_names:
            row[f'{joint_name}_position_rad'] = sample.positions[joint_name]
            row[f'{joint_name}_velocity_rad_s'] = sample.velocities[joint_name]
            row[f'{joint_name}_acceleration_rad_s2'] = (
                sample.accelerations[joint_name]
            )
        self._writer.writerow(row)

    def close(self) -> None:
        """Close the CSV file and flush buffered rows to disk."""
        self._file.close()

    def _create_fieldnames(self) -> list[str]:
        """Return the CSV header in a stable, joint-by-joint order."""
        fieldnames = ['trajectory', 'timestamp_s']
        for joint_name in self._joint_names:
            fieldnames.extend(
                (
                    f'{joint_name}_position_rad',
                    f'{joint_name}_velocity_rad_s',
                    f'{joint_name}_acceleration_rad_s2',
                )
            )
        return fieldnames
