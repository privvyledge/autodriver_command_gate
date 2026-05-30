from setuptools import setup

package_name = 'command_gate'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='autodriver',
    maintainer_email='autodriver@todo.todo',
    description='ROS 2 safety gate package for F1/10 autonomous racing',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'command_gate_node_py = command_gate.command_gate_node:main',
        ],
    },
)