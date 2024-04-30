
from setuptools import setup

package_name = 'agent_package'

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
    maintainer='lukektx',
    maintainer_email='lukektx@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'agent = agent_package.agent:main',
            'drive_controller = agent_package.drive_controller:main',
            'collision_avoidance = agent_package.collision_avoidance:main'
        ],
    },
)
