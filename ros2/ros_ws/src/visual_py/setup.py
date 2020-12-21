from setuptools import setup

package_name = 'visual_py'

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
            'displayControl = visual_py.displayControl:main',
            'eyeControl = visual_py.eyeControl:main',
            'eyeRosPublisher = visual_py.eyeRosPublisher:main',
            'ledControl = visual_py.ledControl:main',
        ],
    },
)
