from setuptools import setup
from setuptools import find_packages
import pyrfuniverse

VERSION = pyrfuniverse.__version__

setup(
    name="pyrfuniverse",
    version=VERSION,
    description="RFUniverse python interface",
    url="https://github.com/robotflow-initiative/pyrfuniverse",
    author="RobotFlow AI Team",
    author_email="robotflow@163.com",
    packages=find_packages(),
    install_requires=[
        "numpy>=1.14.1",
        "grpc",
        "protobuf",
        "opencv-contrib-python",
        "requests"
    ],
    python_requires=">=3.10",
    entry_points={
        'console_scripts': [
            'pyrfuniverse=pyrfuniverse.entry_points:pyrfuniverse_entry_points',
        ]
    }
)
