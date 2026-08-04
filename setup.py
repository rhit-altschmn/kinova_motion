from setuptools import find_packages, setup

package_name = 'rec_rep2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/rec_rep2.launch.py',
            'launch/replayer_only.launch.py',
            'launch/fake_hardware.launch.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/compliant_params.yaml',
        ]),
        ('share/' + package_name + '/scripts',
            ['scripts/launch_gui.sh']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='horrorfry',
    maintainer_email='pearlda@oregonstate.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'recorder = rec_rep2.recorder:main',
            'replayer = rec_rep2.replayer:main',
            'gui = rec_rep2.gui:main',
            'fake_joint_states = rec_rep2.fake_joint_states:main',
            'fake_replayer = rec_rep2.fake_replayer:main',
        ],
    },
)
