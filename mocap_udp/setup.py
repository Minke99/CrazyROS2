from setuptools import setup

package_name = 'mocap_udp'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/mocap_launch.py']),
        ('share/' + package_name + '/config', ['config/mocap_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your@email.com',
    description='UDP mocap to PoseStamped publisher',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mocap_udp_node = mocap_udp.mocap_udp_node:main',  # Python main code
        ],
    },
)
