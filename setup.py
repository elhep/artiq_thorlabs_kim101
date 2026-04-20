"""
Setup script for KIM101 driver.
"""

from setuptools import setup, find_packages

with open("requirements.txt") as f:
    required = f.read().splitlines()

setup(
    name="kim101-driver",
    install_requires=required,
    packages=find_packages(),
    entry_points={
        "console_scripts": [
            "aqctl_artiq_ablation_camera = artiq_ablation_camera.aqctl_artiq_ablation_camera:main",
        ],
    },
)
