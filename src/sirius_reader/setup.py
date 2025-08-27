from setuptools import find_packages, setup
import os

package_name = 'sirius_reader'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    package_data={
        'sirius_reader': ['SiriusCeptionBin.cpython-310-x86_64-linux-gnu.so'],
    },
    data_files=[
        (os.path.join('lib', package_name), ['./SiriusCeptionBin.cpython-310-x86_64-linux-gnu.so']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pbdq',
    maintainer_email='pbdq@todo.todo',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'sirius_reader = sirius_reader.sirius_reader:main'
        ],
    },
)
