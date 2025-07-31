import glob
from setuptools import find_packages, setup

package_name = 'path_planning'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (f'share/{package_name}/launch', glob.glob('launch/*.launch.py')),
        (f'share/{package_name}/maps', glob.glob('maps/*.pgm')),
        (f'share/{package_name}/params', glob.glob('params/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ming06',
    maintainer_email='ming06@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
         'path_planner = path_planning.path_planner:main',
         "a_star_node = path_planning.a_star_node_inflate:main",
         "d_star_lite_node = path_planning.d_star_lite_node_inflate:main"
        ],
    },
)
