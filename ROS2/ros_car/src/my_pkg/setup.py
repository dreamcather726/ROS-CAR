from setuptools import find_packages, setup

package_name = 'my_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            'share/' + package_name + '/launch',
            [
                'launch/bringup.launch.py',
                'launch/live_mapping.launch.py',
                'launch/live_navigation.launch.py',
                'launch/navigation.launch.py',
            ],
        ),
        (
            'share/' + package_name + '/config',
            [
                'config/tf_params.yaml',
                'config/nav2_params.yaml',
            ],
        ),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='hyt',
    maintainer_email='hyt@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            (
                'esp32_bridge_node = '
                'my_pkg.esp32_bridge.esp32_bridge_node:main'
            ),
            'tf_tree_node = my_pkg.tf_tree.tf_tree_node:main',
            (
                'keyboard_control_node = '
                'my_pkg.keyboard_control.keyboard_control_node:main'
            ),
        ],
    },
)
