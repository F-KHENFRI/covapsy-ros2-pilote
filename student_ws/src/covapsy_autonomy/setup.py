from glob import glob

from setuptools import find_packages, setup

package_name = 'covapsy_autonomy'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='covapsy',
    maintainer_email='covapsy@example.com',
    description='Infrastructure voiture CoVAPSy : perception, état de course, sécurité',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'scan_processor = covapsy_autonomy.scan_processor:main',
            'race_manager = covapsy_autonomy.race_manager:main',
            'safety_watchdog = covapsy_autonomy.safety_watchdog:main',
        ],
    },
)
