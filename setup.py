from setuptools import setup
from setuptools import find_packages

import pyrfuniverse

VERSION = pyrfuniverse.__version__

setup(
    name="pyrfuniverse",
    version=VERSION,
    description="RFUniverse python interface",
    url="https://github.com/mvig-robotflow/pyrfuniverse",
    author="Robotflow AI Team",
    author_email="robotflow@163.com",
    packages=find_packages(exclude=["*.tests", "*.tests.*", "tests.*", "tests"]),
    zip_safe=False,
    install_requires=[
        "numpy>=1.14.1",
        "opencv-contrib-python",
    ],
    python_requires=">=3.6.1",
)
