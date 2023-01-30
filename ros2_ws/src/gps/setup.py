from setuptools import setup

package_name = 'gps'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wojtek',
    maintainer_email='wojcsiu541@student.polsl.pl',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = gps.position_publisher:main',
            'listener = gps.position_subscriber:main',
            'pseudoTalker = gps.position_pseudo_publisher:main',
            'calculator = gps.calculator:main',
        ],
    },
)
