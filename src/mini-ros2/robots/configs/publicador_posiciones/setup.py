from setuptools import find_packages, setup

package_name = 'publicador_posiciones'

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
    maintainer='andrius',
    maintainer_email='ajjaya@espol.edu.ec',
    description='Publicador de cmd',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'publicador_cmd=publicador_posiciones.publicador_cmd:main'
        ],
    },
)
