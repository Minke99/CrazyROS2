from setuptools import find_packages, setup

package_name = 'cf_ctrl'

setup(
    name=package_name,
    version='0.0.0',
    # packages=find_packages(exclude=['test']),
    packages=find_packages(include=['cf_ctrl', 'cf_ctrl.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/cf_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='minke',
    maintainer_email='r.jia@foxmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cf_ctrl_node = cf_ctrl.cf_ctrl_node:main',
            'mocap_ctrl_node = cf_ctrl.mocap_quad_node:main',
            'joystick_node = cf_ctrl.tools.joystick_node:main',
        ],
    },
)