from setuptools import find_packages, setup

package_name = 'mobility'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nelson Durrant',
    maintainer_email='snelsondurrant@gmail.com',
    description='Mobility package for the Mars Rover',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arduino_interface = mobility.arduino_interface:main',
        ],
    },
)
