from setuptools import find_packages, setup

package_name = 'coverage'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/coverage.launch.py', 'launch/production.launch.py']),
        ('share/' + package_name + '/rviz', ['rviz/coverage.rviz']),
        ('share/' + package_name + '/behavior_trees', ['behavior_trees/navigate_coverage_w_recovery.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='6kam',
    maintainer_email='6kam465@gmail.com',
    description='BLE cleaning-zone polygon to Nav2 complete coverage (opennav_coverage bridge)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'coverage_bridge = coverage.coverage_bridge:main',
        ],
    },
)
