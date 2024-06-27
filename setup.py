
m distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
        packages=['auto_control']
        package_dir={'':'src'},
        install_requires=['pynput']
)

setup(**d)
