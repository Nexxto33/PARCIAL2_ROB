from setuptools import find_packages, setup

package_name = 'control_py'

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
    maintainer='leyla',
    maintainer_email='leyla@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [

            'control= control_py.control:main',
            'sub_laser= control_py.sub_laser:main',
            'control_distance= control_py.control_distance:main',

            'navegacion= control_py.navegacion:main',
            'marker_v= control_py.marker:main',

            'nav_auto= control_py.nav_auto:main',
            'nav2 = control_py.nav2:main',

            'ejercicio3 = control_py.ejercicio3:main'

        ],
    },
)
