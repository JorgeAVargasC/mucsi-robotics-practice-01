from setuptools import find_packages, setup

package_name = "ur3e_kinematics"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="laboratorio",
    maintainer_email="javargas1209@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "kinematics_recorder = ur3e_kinematics.kinematics_recorder:main",
            "ex_02_trajectory_executor = ur3e_kinematics.ex_02_trajectory_executor:main",
            "ex_01_joint_states_subscriber = ur3e_kinematics.ex_01_joint_states_subscriber:main",
            "ex_04_joint_kinematics_publisher = ur3e_kinematics.ex_04_joint_kinematics_publisher:main",
        ],
    },
)
