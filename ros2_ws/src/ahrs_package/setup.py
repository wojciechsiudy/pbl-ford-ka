from setuptools import setup

package_name = 'ahrs_package'

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
    maintainer='Maciej B.',
    maintainer_email='macibol775@student.polsl.pl',
    description='AHRS output string parser and acquisitor',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = ahrs_package.ahrs_publisher:main',
            'listener = ahrs_package.ahrs_subscriber:main',
        ],
    },
)
