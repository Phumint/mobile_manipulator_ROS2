from setuptools import setup

PACKAGE_NAME = 'moma_paper_demo'

setup(
    name=PACKAGE_NAME,
    version='0.1.0',
    packages=[PACKAGE_NAME],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
        ('share/' + PACKAGE_NAME + '/launch', [
            'launch/demo.launch.py',
            'launch/holistic_demo.launch.py',
        ]),
        ('share/' + PACKAGE_NAME + '/config', [
            'config/demo_params.yaml',
            'config/holistic_demo_params.yaml',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='phumint',
    maintainer_email='phumint1969@gmail.com',
    description='Whole-body control demo for MiR + UR10e mobile manipulator',
    entry_points={
        'console_scripts': [
            'controller_node = moma_paper_demo.controller_node:main',
            'sine_wave_base_node = moma_paper_demo.sine_wave_base_node:main',
            'lock_on_arm_node = moma_paper_demo.lock_on_arm_node:main',
        ],
    },
)
