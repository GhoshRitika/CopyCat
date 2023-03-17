from setuptools import setup

package_name = 'finger_tracking'

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
    maintainer='ritz',
    maintainer_email='ritikaghosh2023@u.northwestern.edu',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'finger_tracking = finger_tracking.finger_tracking:main'
        ],
    },
)
