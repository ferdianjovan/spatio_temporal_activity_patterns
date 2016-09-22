from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['activity_temporal_patterns'],
    package_dir={'': 'src'}
)

setup(**setup_args)
