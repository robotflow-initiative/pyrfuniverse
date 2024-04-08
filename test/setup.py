import os
from setuptools import setup
from setuptools import find_packages
import pyrfuniverse_test

VERSION = pyrfuniverse_test.__version__

need_files = []
for root, dirs, files in os.walk(pyrfuniverse_test.path):
  for fn in files:
      fn = root + "/" + fn
      print(fn)
      need_files.append(fn[1 + len(pyrfuniverse_test.path):])

setup(
    name="pyrfuniverse-test",
    version=VERSION,
    description="RFUniverse test scripts",
    url="https://github.com/robotflow-initiative/pyrfuniverse",
    author="RobotFlow AI Team",
    author_email="robotflow@163.com",
    packages=find_packages(),
    package_data={
        'pyrfuniverse_test': need_files,
    },
    install_requires=[
        f"pyrfuniverse=={VERSION}",
    ],
    python_requires=">=3.10",
    entry_points={
        'console_scripts': [
            'pyrfuniverse-test=pyrfuniverse_test.entry_points:pyrfuniverse_test_entry_points',
        ]
    }
)
