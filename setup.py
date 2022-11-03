from setuptools import setup
from setuptools import find_packages

import pyrfuniverse

VERSION = pyrfuniverse.__version__

setup(
    name="pyrfuniverse",
    version=VERSION,
    description="RFUniverse python interface",
    url="https://github.com/mvig-robotflow/rfuniverse",
    author="Robotflow AI Team",
    author_email="robotflow@163.com",
    packages=find_packages(exclude=["*.tests", "*.tests.*", "tests.*", "tests"]),
    zip_safe=False,
    install_requires=[
        "cloudpickle",
        "grpcio>=1.11.0",
        "numpy>=1.14.1",
        "Pillow>=4.2.1",
        "protobuf==3.19.0",
        "pyyaml>=3.1.0",
        "gym==0.21.0",
        "opencv-python==4.5.4.60",
        "opencv-contrib-python==4.5.4.60",
    ],
    python_requires=">=3.6.1,<3.10",
)


