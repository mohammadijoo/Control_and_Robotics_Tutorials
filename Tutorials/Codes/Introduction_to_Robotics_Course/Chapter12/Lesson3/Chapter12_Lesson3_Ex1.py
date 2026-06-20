from setuptools import setup

package_name = 'my_py_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'simple_subscriber = my_py_pkg.simple_subscriber:main',
        ],
    },
)
      
