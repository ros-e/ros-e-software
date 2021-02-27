from setuptools import setup

package_name = 'light_sim_py'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rose',
    maintainer_email='oskar.lorenz@th-wildau.de',
    description='simulated light service',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'service = light_sim_py.service:main',
            'client = light_sim_py.client:main',
        ],
    },
)
