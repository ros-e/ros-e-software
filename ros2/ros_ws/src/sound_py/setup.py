from setuptools import setup

package_name = 'sound_py'

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
    maintainer='rose',
    maintainer_email='vasc9380@th-wildau.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'microphone = sound_py.microphone:main',
            'reSpeaker = sound_py.reSpeaker:main',
            'speakSvox = sound_py.speak:main',
            'respeakerInterface = sound_py.respeakerInterface:main',
        ],
    },
)
