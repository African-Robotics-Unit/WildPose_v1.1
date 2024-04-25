from setuptools import setup

package_name = 'dji_rs3_pkg'
protocol_module = "dji_rs3_pkg/protocol"

setup(
    name=package_name,
    version='0.0.0',
    packages=[
        package_name, protocol_module,
        'dji_rs3_pkg/protocol/connection',
        'dji_rs3_pkg/protocol/sdk',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='naoya',
    maintainer_email='sh.mn.nat@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dji_rs3_node = dji_rs3_pkg.dji_rs3_node:main'
        ],
    },
)
