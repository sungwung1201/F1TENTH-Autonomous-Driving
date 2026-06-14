from setuptools import find_packages, setup

package_name = 'project_f1_tenth'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yoon',
    maintainer_email='yoon@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'project_f1_tenth = project_f1_tenth.project_f1_tenth:main',
            'make_waypoint = project_f1_tenth.make_waypoint:main',
            'visualizer = project_f1_tenth.visualizer:main',
        ],
    },
)
