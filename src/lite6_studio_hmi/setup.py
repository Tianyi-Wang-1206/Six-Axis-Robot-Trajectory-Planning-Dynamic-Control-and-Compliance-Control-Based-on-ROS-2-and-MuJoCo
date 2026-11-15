from setuptools import find_packages, setup

package_name = 'lite6_studio_hmi'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tianyi',
    maintainer_email='tianyi@todo.todo',
    description='Industrial trajectory programming Studio HMI for UFactory Lite6',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'studio_gui = lite6_studio_hmi.studio_main:main',
        ],
    },
)
