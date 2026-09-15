from setuptools import find_packages, setup

package_name = 'mirs_coverage'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/coverage.launch.py']),
        ('share/' + package_name + '/rviz', ['rviz/cleaning_zone.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='6kam',
    maintainer_email='6kam465@gmail.com',
    description='BLE cleaning-zone polygon to Nav2 coverage waypoints for MIRS',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'coverage_executor = mirs_coverage.coverage_executor:main',
        ],
    },
)
