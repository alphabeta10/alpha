from setuptools import find_packages, setup

package_name = 'service_demo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alpha',
    maintainer_email='2394023336@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'service_adder_client = service_demo.service_demo:client_main',
            'service_adder_server = service_demo.service_demo:srv_main',

            'service_object_client = service_demo.service_demo:object_client_main',
            'service_object_server = service_demo.service_demo:object_server_main',
        ],
    },
)
