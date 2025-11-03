from setuptools import setup


package_name = 'teszt1'


setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['teszt1_bringup/launch/teszt1.launch.py']),
        ('share/' + package_name + '/params', ['teszt1_bringup/params/teszt1.yaml']),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='Levente Vass',
    maintainer_email='levente.vass@ddc.sze.hu',
    description='teszt',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'scan_filter = teszt1.scan_filter_node:main',
            'gap_detector = teszt1.gap_detector_node:main',
            'lookahead_planner = teszt1.lookahead_planner_node:main',
            'curvature_controller = teszt1.curvature_controller_node:main',
            'speed_manager = teszt1.speed_manager_node:main',
            'safety_monitor = teszt1.safety_monitor_node:main',
            'cmd_mux = teszt1.cmd_mux_node:main',
            'viz = teszt1.viz_node:main',
        ],
    },
)